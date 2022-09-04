#include <linux/blk-mq.h>
#include <linux/blk_types.h>
#include <linux/blkdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/genhd.h>
#include <linux/hdreg.h>
#include <linux/in.h>
#include <linux/inet.h>
#include <linux/init.h>
#include <linux/ip.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/netdevice.h>
#include <linux/slab.h>
#include <linux/socket.h>
#include <linux/spinlock.h>
#include <linux/stat.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/vmalloc.h>
#include <uapi/linux/cdrom.h>  //for CDROM_GET_CAPABILITY
#include <uapi/linux/hdreg.h>  //for struct hd_geometry

#define SUCCESS 0

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Charlie Smurthwaite");
MODULE_DESCRIPTION("A network block device");
MODULE_VERSION("0.01");

int _major = 0;

static long loop_control_ioctl(struct file *file, unsigned int cmd,
                               unsigned long parm) {
  return -ENOSYS;
}

static const struct file_operations loop_ctl_fops = {
    .open = nonseekable_open,
    .unlocked_ioctl = loop_control_ioctl,
    .compat_ioctl = loop_control_ioctl,
    .owner = THIS_MODULE,
    .llseek = noop_llseek,
};

static struct miscdevice loop_misc = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "netblock-control",
    .fops = &loop_ctl_fops,
};

MODULE_ALIAS_MISCDEV(LOOP_CTRL_MINOR);
MODULE_ALIAS("devname:netblock-control");

struct netblock_device {
  struct request_queue *queue;
  struct blk_mq_tag_set tag_set;
  struct gendisk *disk;
  int capacity;
  atomic_t open_counter;
  struct socket *socket;
  struct sockaddr_in addr;
};

static void send_udp(struct netblock_device *dev) {
  int retval;
  struct msghdr msg;
  struct kvec iov;

  iov.iov_base = "Hello!";
  iov.iov_len = 6;

  msg.msg_control = NULL;
  msg.msg_controllen = 0;
  msg.msg_flags = 0;
  msg.msg_name = &(dev->addr);
  msg.msg_namelen = sizeof(struct sockaddr_in);

  retval = kernel_sendmsg(dev->socket, &msg, &iov, 1, iov.iov_len);
  // printk(KERN_INFO "sock_sendmsg returned: %d", retval);
}

static int do_simple_request(struct request *rq, unsigned int *nr_bytes) {
  int ret = SUCCESS;

  struct bio_vec bvec;
  struct req_iterator iter;
  struct netblock_device *dev = rq->q->queuedata;

  // loff_t pos = blk_rq_pos(rq) << SECTOR_SHIFT;
  // loff_t dev_size = (loff_t)(dev->capacity << SECTOR_SHIFT);

  printk(KERN_WARNING "netblock: request for sector %lld length %d\n",
         blk_rq_pos(rq), blk_rq_bytes(rq));

  rq_for_each_segment(bvec, rq, iter) {
    unsigned long b_len = bvec.bv_len;
    printk(KERN_WARNING "netblock: segment length %ld\n", b_len);
    //   if ((pos + b_len) > dev_size) b_len = (unsigned long)(dev_size - pos);

    //   //  if (rq_data_dir(rq))  // WRITE
    //   //    memcpy(dev->data + pos, b_buf, b_len);
    //   //  else  // READ
    //   //    memcpy(b_buf, dev->data + pos, b_len);

    //   pos += b_len;
    *nr_bytes += b_len;
  }
  send_udp(dev);
  return ret;
}

static blk_status_t _queue_rq(struct blk_mq_hw_ctx *hctx,
                              const struct blk_mq_queue_data *bd) {
  unsigned int nr_bytes = 0;
  blk_status_t status = BLK_STS_OK;
  struct request *rq = bd->rq;

  // we cannot use any locks that make the thread sleep
  blk_mq_start_request(rq);

  if (do_simple_request(rq, &nr_bytes) != SUCCESS) status = BLK_STS_IOERR;

  // printk(KERN_WARNING "netblock: request process %d bytes\n", nr_bytes);

  if (blk_update_request(rq, status, nr_bytes))  // GPL-only symbol
    BUG();
  __blk_mq_end_request(rq, status);

  return BLK_STS_OK;  // always return ok
}

static const struct blk_mq_ops netblock_mq_ops = {
    .queue_rq = _queue_rq,
};

static int _open(struct block_device *bdev, fmode_t mode) {
  struct netblock_device *dev = bdev->bd_disk->private_data;
  if (dev == NULL) {
    printk(KERN_WARNING "netblock: invalid disk private_data\n");
    return -ENXIO;
  }

  atomic_inc(&dev->open_counter);

  printk(KERN_WARNING "netblock: device was opened\n");

  return SUCCESS;
}
static void _release(struct gendisk *disk, fmode_t mode) {
  struct netblock_device *dev = disk->private_data;
  if (dev) {
    atomic_dec(&dev->open_counter);

    printk(KERN_WARNING "netblock: device was closed\n");
  } else
    printk(KERN_WARNING "netblock: invalid disk private_data\n");
}

static int _ioctl(struct block_device *bdev, fmode_t mode, unsigned int cmd,
                  unsigned long arg) {
  int ret = -ENOTTY;

  return ret;
}
#ifdef CONFIG_COMPAT
static int _compat_ioctl(struct block_device *bdev, fmode_t mode,
                         unsigned int cmd, unsigned long arg) {
  // CONFIG_COMPAT is to allow running 32-bit userspace code on a 64-bit kernel
  return -ENOTTY;  // not supported
}
#endif

static const struct block_device_operations _fops = {
    .owner = THIS_MODULE,
    .open = _open,
    .release = _release,
    .ioctl = _ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl = _compat_ioctl,
#endif
};

static void netblock_add(int i) {
  int ret;
  struct netblock_device *dev;
  dev = kzalloc(sizeof(*dev), GFP_KERNEL);

  dev->capacity = 4194304;
  dev->tag_set.ops = &netblock_mq_ops;
  dev->tag_set.nr_hw_queues = 1;
  dev->tag_set.queue_depth = 128;
  dev->tag_set.numa_node = NUMA_NO_NODE;
  dev->tag_set.cmd_size = 0;
  dev->tag_set.flags = BLK_MQ_F_SHOULD_MERGE;
  dev->tag_set.driver_data = dev;

  ret = blk_mq_alloc_tag_set(&dev->tag_set);
  if (ret) {
    printk(KERN_WARNING "netblock: unable to allocate tag set\n");
    return;
  }

  dev->queue = blk_mq_init_queue(&dev->tag_set);
  if (IS_ERR(dev->queue)) {
    ret = PTR_ERR(dev->queue);
    printk(KERN_WARNING "netblock: Failed to allocate queue\n");
    return;
  }
  dev->queue->queuedata = dev;

  dev->disk = blk_mq_alloc_disk(&dev->tag_set, dev);
  if (dev->disk == NULL) {
    printk(KERN_WARNING "netblock: Failed to allocate disk\n");
    ret = -ENOMEM;
    return;
  }

  dev->disk->flags |= GENHD_FL_NO_PART_SCAN;  // only one partition
  // disk->flags |= GENHD_FL_EXT_DEVT;
  dev->disk->flags |= GENHD_FL_REMOVABLE;

  dev->disk->major = _major;
  dev->disk->first_minor = i;
  dev->disk->minors = 1;
  dev->disk->fops = &_fops;
  dev->disk->private_data = dev;
  dev->disk->queue = dev->queue;
  sprintf(dev->disk->disk_name, "netblock%d", i);
  set_capacity(dev->disk, dev->capacity);
  dev->disk->flags |= GENHD_FL_NO_PART_SCAN;
  dev->disk->flags |= GENHD_FL_EXT_DEVT;
  atomic_set(&dev->open_counter, 0);

  add_disk(dev->disk);

  ret = sock_create_kern(&init_net, AF_INET, SOCK_DGRAM, 0, &(dev->socket));
  printk(KERN_INFO "sock_create_kern returned: %d", ret);
  dev->addr.sin_family = AF_INET;
  dev->addr.sin_addr.s_addr = htonl((10 << 24) | (1 << 16) | (1 << 8) | (3));
  dev->addr.sin_port = htons(50000);
}

static int __init netblock_init(void) {
  _major = register_blkdev(_major, "netblock");
  if (_major <= 0) {
    printk(KERN_ERR "register_blkdev failed!\n");
    return -EBUSY;
  }

  misc_register(&loop_misc);
  netblock_add(0);
  netblock_add(1);
  netblock_add(2);
  printk(KERN_INFO "Success %i!\n", _major);

  return 0;
}

static void __exit netblock_exit(void) {
  if (_major > 0) unregister_blkdev(_major, "netblock");
  misc_deregister(&loop_misc);
  printk(KERN_INFO "Goodbye!\n");
}

module_init(netblock_init);
module_exit(netblock_exit);
