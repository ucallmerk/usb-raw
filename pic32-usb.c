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

#define MC_VENDOR_ID 0x1c4f
#define PIC_PRODUCT_ID 0x0003
#define USB_PIC32_MINOR_BASE 0
#define PIC32_INT_BUFF_SZ 65
#define KEYBOARD_COMMAND_READ_KEY 0x80
#define KEYBOARD_COMMAND_SET_LED 0x81

/* Using mini interval for in an dout transfers */
static int in_intvl = 1;
static int out_intvl = 1;
static char *read_buffer;

const struct usb_device_id pic32_id_table[] = {

	{ USB_DEVICE(MC_VENDOR_ID, PIC_PRODUCT_ID) },
	{}
};


struct usb_pic32 {
	struct usb_interface *intf;
	struct usb_endpoint_descriptor *ep_in;
	int		ep_in_maxp_sz;
	struct usb_endpoint_descriptor *ep_out;
	int		ep_out_maxp_sz;
	struct urb	*in_urb;
	struct urb	*out_urb;
	char		*in_buff;
	dma_addr_t	in_dma;
	char		*out_buff;
	dma_addr_t	out_dma;

	struct mutex	mutex;
	int		open_count;

	int		int_in_running;
	int		int_in_done;
	int		int_out_running;
	int		int_out_done;

	wait_queue_head_t	read_wait;
	wait_queue_head_t	write_wait;
};

static struct usb_driver pic32_usb_driver;

static void pic32_usb_in_callback(struct urb *urb)
{
	struct usb_pic32 *dev = urb->context;
	int status = urb->status;
	int retval = 0;

	dev_info(&dev->intf->dev,"In call back\n");

	if (status) {
		if (status == -ENOENT ||
		    status == -ECONNRESET ||
		    status == -ESHUTDOWN) {
			goto exit;
		} else {
			dev_dbg(&dev->intf->dev,
				"%s: nonzero status received: %d\n", __func__,
				status);
			goto resubmit;
		}
	}

	if (urb->actual_length > 0) {
		dev_info(&dev->intf->dev,
			 "Received %d bytes\n", urb->actual_length);

		memcpy(read_buffer, dev->in_buff, urb->actual_length);
	}

resubmit:
	/* resubmit if we're still running */
	if (dev->int_in_running) {
		retval = usb_submit_urb(dev->in_urb, GFP_ATOMIC);
		if (retval) {
			dev_err(&dev->intf->dev,
				"usb_submit_urb failed (%d)\n", retval);
		}
	}
exit:
	dev->int_in_done = 1;
	wake_up_interruptible(&dev->read_wait);
}

static void pic32_usb_out_callback(struct urb *urb)
{
	struct usb_pic32 *dev = urb->context;
	int status = urb->status;
	int retval = 0;

	dev_info(&dev->intf->dev,"Out call back\n ");

	if (status) {
		if (status == -ENOENT ||
		    status == -ECONNRESET ||
		    status == -ESHUTDOWN) {
			goto exit;
		} else {
			dev_dbg(&dev->intf->dev,
				"%s: nonzero status received: %d\n", __func__,
				status);
			goto resubmit;
		}
	}

resubmit:
	/* resubmit if we're still running */
	if (dev->int_out_running) {
		retval = usb_submit_urb(dev->out_urb, GFP_ATOMIC);
		if (retval) {
			dev_err(&dev->intf->dev,
				"usb_submit_urb failed (%d)\n", retval);
		}
	}
exit:
	dev->int_out_done = 1;
	wake_up_interruptible(&dev->write_wait);
}



static int pic32_usb_open (struct inode *inode, struct file *file)
{
	struct usb_pic32 *dev;
	struct usb_device *udev;
	struct usb_interface *interface;
	struct usb_endpoint_descriptor *endpoint;
	int subminor;
	int retval;

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

	udev = interface_to_usbdev(interface);
	endpoint = dev->ep_in;

	dev->int_out_running = 1;
	dev->int_out_done = 0;
	retval = usb_submit_urb(dev->out_urb, GFP_KERNEL);
	if (retval) {
		dev_err(&interface->dev,
			"Couldn't submit out_urb %d\n", retval);
		dev->int_out_running = 0;
		dev->open_count = 0;
		goto unlock_exit;
	}
	dev_info (&interface->dev, "Submitted out URB\n");

	dev->int_in_running = 1;
	dev->int_in_done = 0;

	retval = usb_submit_urb(dev->in_urb, GFP_KERNEL);
	if (retval) {
		dev_err(&interface->dev,
			"Couldn't submit in_urb %d\n", retval);
		dev->int_in_running = 0;
		dev->open_count = 0;
		goto unlock_exit;
	}
	dev_info (&interface->dev, "Submitted URB for endpoint in dir setup\n");

	read_buffer = kmalloc(PIC32_INT_BUFF_SZ, GFP_KERNEL);
	if(!read_buffer)
		retval = -ENOMEM;

	/* save device in the file's private structure */
	file->private_data = dev;

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
	size_t min = 0;
	min = min(count, sizeof(read_buffer));
	printk(KERN_ERR " ** read buff size %ld\n", sizeof(read_buffer));
	copy_to_user(buffer, read_buffer, count);
	return 0;
}

static ssize_t pic32_usb_write (struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	return 0;
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

int pic32_usb_probe(struct usb_interface *intf,
		   const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(intf);
	struct usb_host_interface *interface = NULL;
	struct usb_endpoint_descriptor *endpoint = NULL;
	struct usb_pic32 *pic_dev = NULL;
	int pipe, maxp;
	int ret = -ENOMEM;

	interface = intf->cur_altsetting;

	if (interface->desc.bNumEndpoints != 2)
		return -ENODEV;

	endpoint = &interface->endpoint[0].desc;
	if (!usb_endpoint_is_int_in(endpoint))
		return -ENODEV;

	maxp = usb_maxpacket(udev, pipe, usb_pipein(pipe));

	/* Allocate local dev sturcture */
	pic_dev = kzalloc(sizeof(struct usb_pic32), GFP_KERNEL);
	if (!pic_dev)
		goto exit;

	pic_dev->intf = intf;
	pic_dev->ep_in = endpoint;
	pic_dev->ep_in_maxp_sz = maxp;

	pic_dev->in_buff = usb_alloc_coherent(udev,
				PIC32_INT_BUFF_SZ, GFP_ATOMIC, &pic_dev->in_dma);
	if(!pic_dev->in_buff)
		goto error;

	pic_dev->in_urb = usb_alloc_urb(0, GFP_KERNEL);
	if(!pic_dev->in_urb)
		goto error;

	pipe = usb_rcvintpipe(udev, endpoint->bEndpointAddress);

	usb_fill_int_urb(pic_dev->in_urb, udev, pipe,
			 pic_dev->in_buff, pic_dev->ep_in_maxp_sz,
			 pic32_usb_in_callback,
			 pic_dev, in_intvl);


	/* Set int out endpoint */
	endpoint = &interface->endpoint[1].desc;
	if (!usb_endpoint_is_int_out(endpoint))
		return -ENODEV;

	maxp = usb_maxpacket(udev, pipe, usb_pipeout(pipe));

	pic_dev->ep_out = endpoint;
	pic_dev->ep_out_maxp_sz = maxp;

	pic_dev->out_buff = usb_alloc_coherent(udev,
				PIC32_INT_BUFF_SZ, GFP_ATOMIC, &pic_dev->out_dma);
	if(!pic_dev->out_buff)
		goto error;

	/* set up out buff */
	pic_dev->out_buff[0] = 0x0;
	pic_dev->out_buff[1] = KEYBOARD_COMMAND_READ_KEY;

	pic_dev->out_urb = usb_alloc_urb(0, GFP_KERNEL);
	if(!pic_dev->out_urb)
		goto error;

	pipe = usb_sndintpipe(udev, endpoint->bEndpointAddress);

	usb_fill_int_urb(pic_dev->out_urb, udev, pipe,
			 pic_dev->out_buff, pic_dev->ep_out_maxp_sz,
			 pic32_usb_out_callback,
			 pic_dev, out_intvl);

	usb_set_intfdata(intf, pic_dev);

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
	return ret;
error:
	/* let us free memory */
	if(pic_dev->in_buff)
		usb_free_coherent(udev,
			PIC32_INT_BUFF_SZ, pic_dev->in_buff, pic_dev->in_dma);
	if(pic_dev->in_urb)
		usb_free_urb(pic_dev->in_urb);
	kfree(pic_dev);
	return ret;

}

void pic32_usb_disconnect(struct usb_interface *intf)
{
	pr_info(" PIC32 USB DRIVER - Device disconnected.. \n");
}

static struct usb_driver pic32_usb_driver = {
	.name = "pic32",
	.probe = pic32_usb_probe,
	.disconnect = pic32_usb_disconnect,
	.id_table = pic32_id_table,
};

module_usb_driver(pic32_usb_driver);

MODULE_DESCRIPTION("USB pic32");
MODULE_LICENSE("GPL");
MODULE_AUTHOR(" RK ");
MODULE_DEVICE_TABLE(usb, pic32_id_table);
