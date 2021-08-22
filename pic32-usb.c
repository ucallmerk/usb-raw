/* PIC32 USB Custom Driver DRIVER */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/mutex.h>

#include <linux/uaccess.h>
#include <linux/input.h>
#include <linux/usb.h>
#include <linux/poll.h>
#include <linux/kfifo.h>
#include <linux/proc_fs.h>

//Bus 001 Device 006: ID 04d8:003f Microchip Technology, Inc.
#define MC_VENDOR_ID		0x04d8
#define PIC_PRODUCT_ID		0x0055
#define USB_PIC32_MINOR_BASE	0
#define PIC32_INT_BUFF_SZ	128
#define USB_CTRL_SET_TIMEOUT	5000


/* FIFO Defines */

static struct kfifo pic32_fifo;

#define FIFO_SIZE	256
#define	PROC_FIFO	"pic32_input"
#define KEY_BUTTON_LOC 	1
#define KEY_BYTES	3
#define NR_BUTTONS 	24

static DEFINE_MUTEX(fread_lock);

static DEFINE_MUTEX(fwrite_lock);


/* interval in micro frames */
static int in_intvl = 8;
char ascii_map_base = 0x41;/* A */
char ascii_buf[NR_BUTTONS];
static bool key_pressed = false;

const struct usb_device_id pic32_id_table[] = {

	{ USB_DEVICE(MC_VENDOR_ID, PIC_PRODUCT_ID) },
	{}
};

struct usb_pic32 {
	struct usb_interface *intf;
	struct usb_endpoint_descriptor *ep_in;
	int		in_pipe;
	int		ep_in_maxp_sz;
	struct usb_endpoint_descriptor *ep_out;
	int		out_pipe;
	int		ep_out_maxp_sz;
	struct urb	*in_urb;
	struct urb	*out_urb;
	char		*in_buff;
	bool		data_in;
	dma_addr_t	in_dma;
	char		*out_buff;
	dma_addr_t	out_dma;

	struct mutex	mutex;
	int		open_count;

	int		int_in_running;
	int		int_in_done;
	int		int_out_running;
	int		int_out_done;
	bool		interrupt_out_busy;
	bool		disconnected;

	wait_queue_head_t	read_wait;
	struct kfifo	*fifo;
};


static void pic32_usb_process_buffer(char* buffer)
{
	int i,j,k;
	j=0;
	for (k = 0; k < KEY_BYTES; k++) {
		for (i=0; i<NR_BUTTONS; i++) {
			if((short)buffer[k + KEY_BUTTON_LOC] & (short)(1<<i)) {
				ascii_buf[j++] = ascii_map_base + i;
				key_pressed = true;
			}
		}
	}
	ascii_buf[j] = "\n";
	kfifo_in(&pic32_fifo, ascii_buf, j);
}

/* Virtual kbd structures */

ssize_t pic32_input_read(struct device *dev, struct device_attribute *attr,  char *buf)
{

	if(key_pressed) {
		key_pressed = 0;
		return sprintf(buf, "%s\n", ascii_buf);
	}
	return 0;
}

ssize_t pic32_input_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	return 0;
}

const struct device_attribute pic32_attr = {
	.attr = {.name = "pic32_input", .mode = 0444},
	.show = pic32_input_read,
	.store = pic32_input_write,
};



static struct usb_driver pic32_usb_driver;

static void pic32_usb_read_callback(struct urb *urb)
{
	struct usb_pic32 *dev = urb->context;
	int status = urb->status;

	if (status && !(status == -ENOENT || status == -ECONNRESET ||
			status == -ESHUTDOWN)) {
		dev_err(&dev->intf->dev,
			"urb=%p read int status received: %d\n", urb, status);
	}

	dev->data_in =1;
	dev->int_in_done = 1;
	pic32_usb_process_buffer(urb->transfer_buffer);
	wake_up_interruptible(&dev->read_wait);

}

static int pic32_usb_open (struct inode *inode, struct file *file)
{
	struct usb_pic32 *dev = file->private_data;
	struct usb_interface *interface;
	struct usb_device *udev;
	struct urb *urb;
	int subminor;
	int retval = 0;


	nonseekable_open(inode, file);
	subminor = iminor(inode);

	interface = usb_find_interface (&pic32_usb_driver, subminor);

	if (!interface) {
		printk(KERN_ERR "%s - error, can't find device for minor %d\n",
		       __func__, subminor);
		return -ENODEV;
	}

	dev = usb_get_intfdata(interface);
	if (!dev) {
		return -ENODEV;
	}

	/* lock this device */
	if (mutex_lock_interruptible(&dev->mutex))
		return -ERESTARTSYS;

	/* allow opening only once */
	if (dev->open_count) {
		retval = -EBUSY;
		goto unlock_exit;
	}
	dev->open_count = 1;

	/* save device in the file's private structure */
	file->private_data = dev;

#if 1
	/* alloc urb, fillit, submit.. don't wait..*/
	urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!urb) {
		retval = -ENOMEM;
		goto unlock_exit;
	}

	udev = interface_to_usbdev(interface);

	usb_fill_int_urb(urb,
			 udev,
			 dev->in_pipe,
			 dev->in_buff,
			 PIC32_INT_BUFF_SZ,,
			 pic32_usb_read_callback,
			 dev,
			 in_intvl);

	urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	urb->transfer_dma = dev->in_dma;

	dev->int_in_running = 1;
	dev->int_in_done =0;
	dev->data_in = 0;
	retval = usb_submit_urb(urb, GFP_KERNEL);
	if (retval) {
		dev->int_in_running = 0;
		dev->int_in_done = 1;
		dev_err(&dev->intf->dev,
			"failed submitting read urb, error %d\n", retval);
		goto error_3;
	}

#endif

error_3:
	usb_free_urb(urb);
unlock_exit:
	mutex_unlock(&dev->mutex);
	return retval;
}

int pic32_usb_release (struct inode *inode, struct file *file)
{
	struct usb_pic32 *dev;
	int retval = 0;
	dev = file->private_data;

	if (dev == NULL) {
		retval = -ENODEV;
		goto exit;
	}

	mutex_lock(&dev->mutex);

	if (dev->open_count != 1) {
		retval = -ENODEV;
		goto unlock_exit;
	}
	dev->open_count = 0;

unlock_exit:
	mutex_unlock(&dev->mutex);

exit:
	return retval;
}

static ssize_t pic32_usb_read(struct file *file, char __user *buffer, size_t count, loff_t *ppos)
{
	int retval = 0;
	size_t bytes_to_read = 0;
	struct usb_pic32 *dev = file->private_data;

	if (count == 0)
		goto exit;

	/* lock this object */
	if (mutex_lock_interruptible(&dev->mutex)) {
		retval = -ERESTARTSYS;
		goto exit;
	}

	/* verify that the device wasn't unplugged */
	if (dev->disconnected) {
		retval = -ENODEV;
		printk(KERN_ERR "pic32 usb: No device or device unplugged %d\n", retval);
		goto unlock_exit;
	}

	while(!dev->data_in) {

		if (file->f_flags & O_NONBLOCK) {
	            retval = -EAGAIN;
                    goto unlock_exit;
		}

		retval = wait_event_interruptible(dev->read_wait, dev->int_in_done);
		if (retval < 0)
			goto unlock_exit;
	}
	bytes_to_read = min(count, (size_t) dev->ep_in_maxp_sz);
	retval = bytes_to_read;

	if( copy_to_user(buffer, dev->in_buff, bytes_to_read))
		retval = -EFAULT;

#if 0 /* Print buffer */
	int i;
	for (i=0; i<bytes_to_read; i++)
		printk("0x%2x ", dev->in_buff[i]);
#endif
unlock_exit:
	mutex_unlock(&dev->mutex);
exit:
	return retval;
}

static ssize_t pic32_usb_write (struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	struct usb_pic32 *dev = file->private_data;
	struct usb_interface *interface = dev->intf;
	struct usb_device *udev = interface_to_usbdev(interface);
	char *buf;
	int retval = 0, ret = 0;
	int skipped_report_id = 0;
	int actual_length = 0;

	if (count == 0) {
		printk(KERN_ERR "PIC32: %s : Cannot write 0 bytes..\n",__func__);
		goto exit;
	}

	/* lock this object */
	if (mutex_lock_interruptible(&dev->mutex)) {
		retval = -ERESTARTSYS;
		goto exit;
	}

	/* verify that the device wasn't unplugged */
	if (dev->disconnected) {
		retval = -ENODEV;
		printk(KERN_ERR "pic32 usb: No device or device unplugged %d\n", retval);
		goto unlock_exit;
	}

	buf = memdup_user(buffer, count);
	if (IS_ERR(buf)) {
		ret = PTR_ERR(buf);
		goto error_2;
	}
	if (buf[0] == 0x0) {
		/* Don't send the Report ID */
		buf++;
		count--;
		skipped_report_id = 1;
	}

	ret = usb_interrupt_msg(udev, dev->out_pipe,
				buf, count, &actual_length,
				USB_CTRL_SET_TIMEOUT);
	/* return the number of bytes transferred */
	if (ret == 0) {
		ret = actual_length;
		/* count also the report id */
		if (skipped_report_id)
			ret++;
	}

unlock_exit:
	mutex_unlock(&dev->mutex);
exit:
	return count;
	kfree(buf);
error_2:
	return retval;

}

static ssize_t fifo_write(struct file *file, const char __user *buf,
						size_t count, loff_t *ppos)
{
	return 0;
}
static ssize_t fifo_read(struct file *file, char __user *buf,
						size_t count, loff_t *ppos)
{
	int ret;
	unsigned int copied;
	if (mutex_lock_interruptible(&fread_lock))
		return -ERESTARTSYS;
	ret = kfifo_to_user(&pic32_fifo, buf, count, &copied);
	mutex_unlock(&fread_lock);
	return ret ? ret : copied;

}

int fifo_open(struct inode *inode, struct file *file)
{
	return 0;
}
#if 0 /* For kernel >5.0 */
static const struct proc_ops fifo_fops = {
	.proc_open		= fifo_open,
	.proc_read		= fifo_read,
	.proc_write		= fifo_write,
	.proc_lseek		= noop_llseek,
};
#endif
static const struct file_operations fifo_fops = {
	.read		= fifo_read,
	.write		= fifo_write,
	.llseek		= noop_llseek,
};

/* file operations needed when we register this driver */
static const struct file_operations pic32_usb_fops = {
	.owner =	THIS_MODULE,
	.open =		pic32_usb_open,
	.release =	pic32_usb_release,
	.read =		pic32_usb_read,
	.write	=	pic32_usb_write,
};

static struct usb_class_driver pic32_usb_class = {
	.name =		"pic32%d",
	.fops =		&pic32_usb_fops,
	.minor_base =	USB_PIC32_MINOR_BASE,
};

void pic32_usb_free( struct usb_pic32 *dev)
{
	struct usb_interface *interface = dev->intf;
	struct usb_device *udev = interface_to_usbdev(interface);

	usb_free_coherent(udev,
		PIC32_INT_BUFF_SZ, dev->in_buff, dev->in_dma);
	usb_free_coherent(udev,
		PIC32_INT_BUFF_SZ, dev->out_buff, dev->out_dma);
}

int pic32_usb_probe(struct usb_interface *intf,
		   const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(intf);
	struct usb_host_interface *interface = NULL;
	struct usb_pic32 *dev = NULL;
	int ret = -ENOMEM;

	interface = intf->cur_altsetting;

	if (interface->desc.bNumEndpoints != 2)
		return -ENODEV;

	/* Allocate local dev sturcture */
	dev = kzalloc(sizeof(struct usb_pic32), GFP_KERNEL);
	if (!dev)
		goto exit;

	dev->intf = intf;
	init_waitqueue_head(&dev->read_wait);

	ret = usb_find_common_endpoints(interface,
			NULL, NULL, &dev->ep_in, &dev->ep_out);

	if (ret) {
		dev_err(&intf->dev, "Could not find both bulk-in and bulk-out endpoints\n");
		goto error;
	}

	dev->in_pipe = usb_rcvintpipe(udev, dev->ep_in->bEndpointAddress);
	dev->ep_in_maxp_sz = usb_endpoint_maxp(dev->ep_in);

	dev->in_buff = usb_alloc_coherent(udev,
				PIC32_INT_BUFF_SZ, GFP_ATOMIC, &dev->in_dma);
	if(!dev->in_buff) {
		ret = -ENOMEM;
		goto error;
	}

	dev->out_pipe = usb_sndintpipe(udev, dev->ep_out->bEndpointAddress);
	dev->ep_out_maxp_sz = usb_endpoint_maxp(dev->ep_out);
	dev->out_buff = usb_alloc_coherent(udev,
				PIC32_INT_BUFF_SZ, GFP_ATOMIC, &dev->out_dma);
	if(!dev->out_buff)
		goto error;

	usb_set_intfdata(intf, dev);

//	ret = device_create_file(&intf->dev, &pic32_attr);

	ret = usb_register_dev(intf, &pic32_usb_class);
	if (ret) {
		/* something prevented us from registering this driver */
		dev_err(&intf->dev, "Not able to get a minor for this device.\n");
		usb_set_intfdata(intf, NULL);
		goto error;
	}

	/* FIFO Setup */
	ret = kfifo_alloc(&pic32_fifo, FIFO_SIZE, GFP_KERNEL);
	if (ret) {
		printk(KERN_ERR "pic32: error kfifo_alloc\n");
		return ret;
	}
	dev->fifo = &pic32_fifo;

	if (proc_create(PROC_FIFO, 0, NULL, &fifo_fops) == NULL) {
	    printk("pic32: could't create fifo entry in procfs\n");
	    return -ENOMEM;
	}

	dev_info(&intf->dev,
		 "PIC32 USB Device #%d now attached to major %d minor %d\n",
		(intf->minor - USB_PIC32_MINOR_BASE), USB_MAJOR, intf->minor);

exit:
error:
	return ret;

}

static void pic32_usb_disconnect(struct usb_interface *intf)
{
	struct usb_pic32 *dev = usb_get_intfdata(intf);
	int minor;

	minor = intf->minor;
	usb_set_intfdata(intf, NULL);

	/* give back our minor */
	usb_deregister_dev(intf, &pic32_usb_class);

	mutex_lock(&dev->mutex);

	/* if the device is not opened, then we clean up right now */
	if (!dev->open_count) {
		pic32_usb_free(dev);
		mutex_unlock(&dev->mutex);
	} else {
		dev->disconnected = 1;
		wake_up_interruptible_all(&dev->read_wait);
		mutex_unlock(&dev->mutex);
	}

	remove_proc_entry(PROC_FIFO, NULL);
	kfifo_free(&pic32_fifo);
	dev_info(&intf->dev, "PIC32 USB Device #%d now disconnected\n",
		 (minor - USB_PIC32_MINOR_BASE));
}

static struct usb_driver pic32_usb_driver = {
	.name = "pic32",
	.probe = pic32_usb_probe,
	.disconnect = pic32_usb_disconnect,
	.id_table = pic32_id_table,
};

module_usb_driver(pic32_usb_driver);

MODULE_DESCRIPTION("USB PIC32");
MODULE_LICENSE("GPL");
MODULE_AUTHOR(" RK ");
MODULE_DEVICE_TABLE(usb, pic32_id_table);
