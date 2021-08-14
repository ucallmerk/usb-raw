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

//Bus 001 Device 006: ID 04d8:003f Microchip Technology, Inc.
#define MC_VENDOR_ID		0x04d8
#define PIC_PRODUCT_ID		0x003f
#define USB_PIC32_MINOR_BASE	0
#define PIC32_INT_BUFF_SZ	64
#define USB_CTRL_SET_TIMEOUT	5000

/* interval in micro frames */
static int in_intvl = 8;

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
	wait_queue_head_t	write_wait;
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
		goto error_1;
	}

	udev = interface_to_usbdev(interface);

	usb_fill_int_urb(urb,
			 udev,
			 dev->in_pipe,
			 dev->in_buff,
			 dev->ep_in_maxp_sz,
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

unlock_exit:
	mutex_unlock(&dev->mutex);
	return retval;
error_3:
	usb_free_urb(urb);
error_1:
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
	struct urb *urb;
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
	//usb_free_coherent(udev, count, buf, urb->transfer_dma);
error_2:
	usb_free_urb(urb);
	return retval;

}

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
	kfree(dev);
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
	init_waitqueue_head(&dev->write_wait);

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

	ret = usb_register_dev(intf, &pic32_usb_class);
	if (ret) {
		/* something prevented us from registering this driver */
		dev_err(&intf->dev, "Not able to get a minor for this device.\n");
		usb_set_intfdata(intf, NULL);
		goto error;
	}

	dev_info(&intf->dev,
		 "PIC32 USB Device #%d now attached to major %d minor %d\n",
		(intf->minor - USB_PIC32_MINOR_BASE), USB_MAJOR, intf->minor);

exit:
error:
	return ret;

}

void pic32_usb_disconnect(struct usb_interface *intf)
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
		mutex_unlock(&dev->mutex);
		/* let us free memory */
		if(dev->in_buff)
			kfree(dev->in_buff);
		if(dev->in_urb)
			usb_free_urb(dev->in_urb);

	} else {
		dev->disconnected = 1;
		/* wake up pollers */
		mutex_unlock(&dev->mutex);
	}

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
