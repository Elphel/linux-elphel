/**
 * @file klogger_393.c
 * @brief Record strings with timestamps (using FPGA time) to a memory buffer (no I/O),
 * Read the buffer as character device
 * @copyright Copyright (C) 2016 Elphel, Inc.
 * @par <b>License</b>
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/errno.h>
#include <linux/platform_device.h> // For sysfs
#include <linux/slab.h> //kzalloc

#include <linux/fs.h>
#include <asm/uaccess.h> // copy_*_user
#include <linux/of.h>  // Device Tree



#include <uapi/elphel/c313a.h> // PARS_FRAMES_MASK
#include <uapi/elphel/x393_devices.h> // For sysfs

#include "x393.h"
#include "x393_fpga_functions.h"

#include "klogger_393.h"

#define KLOGGER_BUFFER_ANY_SIZE 1 // any buffer size, use multiple subtractions to convert file pointer to buffer pointer

//#define DEV393_KLOGGER        ("klogger_393",      "klogger_393",    144,  1, "0666", "c")  ///< kernel event logger to memory (no i/o)

#ifdef LOCK_BH_KLOGGER
    #define FLAGS_KLOGGER_BH
    #define LOCK_KLOGGER_BH(x)   spin_lock_bh(x)
    #define UNLOCK_KLOGGER_BH(x) spin_unlock_bh(x)
#else
    #define FLAGS_KLOGGER_BH     unsigned long flags;
    #define LOCK_KLOGGER_BH(x)   spin_lock_irqsave(x,flags)
    #define UNLOCK_KLOGGER_BH(x) spin_unlock_irqrestore(x,flags)
#endif


static DEFINE_SPINLOCK(klogger_lock);

static struct device *g_dev_ptr=NULL; ///< Global pointer to basic device structure. This pointer is used in debugfs output functions
static u32 buffer_size = 0;
#ifndef KLOGGER_BUFFER_ANY_SIZE
    static u32 buffer_size_order = 0;
#endif


static u32 klog_mode=0xff;
static u32 buffer_wp = 0;
static u32 num_writes = 0;
static loff_t file_size = 0; // write pointer, 64 bits
static char * klog393_buf = NULL;
const char klogger393_of_prop_bufsize_name[] = "klogger-393,buffer_size";
const static u32 max_string_len = PAGE_SIZE; ///< maximal string length

static const char * const klogger_dev = DEV393_DEVNAME(DEV393_KLOGGER);
static const int klogger_major = DEV393_MAJOR(DEV393_KLOGGER);
static const int klogger_minor = DEV393_MINOR(DEV393_KLOGGER);

/** @brief Global device class for sysfs */
static struct class *klogger_dev_class;

int        klogger393_open(struct inode *inode, struct file *filp);
int        klogger393_release(struct inode *inode, struct file *filp);
loff_t     klogger393_llseek(struct file * file, loff_t offset, int orig);
ssize_t    klogger393_read   (struct file * file, char * buf, size_t count, loff_t *off);

//        printk();
int print_klog393(const int mode,       ///< bits 0: timestamp, 1 - file, 2 - function, 3 - line number, 4 - lock, irq & disable. all 0 - disabled
                  const char *file,      /// file path to show
                  const char *function,  /// function name to show
                  const int line,       // line number to show
                  const char *fmt, ...) ///< Format and argumants as in printf

{
    FLAGS_KLOGGER_BH
    char buf[1024];
    const char * cp;
    sec_usec_t ts;
    va_list args;
    int mmode= mode & klog_mode;
    if (!mmode){
        return 0;
    }
    if (mmode & 16){
//        spin_lock_bh(&klogger_lock);
        LOCK_KLOGGER_BH(&klogger_lock);
    }
    if (mmode & 1) {
        get_fpga_rtc(&ts);
        snprintf(buf,sizeof(buf),"%ld.%06ld:",ts.sec,ts.usec);
        klog393_puts(buf);
    }
    if ((mmode & 2) && file) {
        cp = strrchr(file,'/');
        cp = cp? (cp+1):file;
        snprintf(buf,sizeof(buf),"%s:",cp);
        klog393_puts(buf);
    }
    if ((mmode & 4) && function) {
        snprintf(buf,sizeof(buf),"%s:",function);
        klog393_puts(buf);
    }
    if ((mmode & 8) && line) {
        snprintf(buf,sizeof(buf),"%d:",line);
        klog393_puts(buf);
    }
    va_start(args, fmt);
    vsnprintf(buf,sizeof(buf),fmt,args);
    va_end(args);
    klog393_puts(buf);
    if (mmode & 16){
//        spin_unlock_bh(&klogger_lock);
        UNLOCK_KLOGGER_BH(&klogger_lock);
    }

    return 0;
}




/** Put string into the logger buffer*/
int klog393_puts(const char * str) ///< String to log, limited by max_string_len (currently 4096 bytes)
                                   ///< @return 0 - OK,  -EMSGSIZE - string too long
{
    int sl =    strlen(str);
    u32 new_wp= buffer_wp+sl;
    if (!(klog_mode & 0x80)){
        return 0; // DEBUGGING: Do nothing if bit 7 == 0
    }
//    u32 pl;
    if (sl > max_string_len){
        dev_err(g_dev_ptr,"%s: String too long (%d >%d)\n",
                __func__,strlen(str),  max_string_len     );
        return -EMSGSIZE;
    }
    // See if the string fits in the buffer
    if (likely(new_wp < buffer_size)){
        memcpy(klog393_buf+buffer_wp, str,sl);
        buffer_wp = new_wp;
    } else {
        memcpy(klog393_buf+buffer_wp, str, buffer_size - buffer_wp);
        memcpy(klog393_buf, str, sl - (buffer_size - buffer_wp));
        buffer_wp = new_wp - buffer_size;
    }
    file_size += sl;
    num_writes++;
    return 0;
}
/** Put string into the logger buffer, preceded by a timestamp "<sec>.<usec>: "*/
int klog393_ts(const char * str) ///< String to log, limited by max_string_len (currently 4096 bytes)
                                 ///< @return 0 - OK,  -EMSGSIZE - string too long
{
    sec_usec_t ts;
    char buf[20]; // 18 enough
    get_fpga_rtc(&ts);
    sprintf(buf,"%ld.%06ld: ",ts.sec,ts.usec);
    klog393_puts(buf);
    return klog393_puts(str);
}


static struct file_operations framepars_fops = {
        owner:    THIS_MODULE,
        open:     klogger393_open,
        llseek:   klogger393_llseek,
        read:     klogger393_read,
        release:  klogger393_release
};

/** Driver OPEN method */
int        klogger393_open(struct inode *inode, ///< inode pointer
                           struct file *filp)   ///< file pointer
                                                ///< @return OK - 0, -EINVAL for wrong minor
{
    int minor= MINOR(inode->i_rdev);
    filp->private_data = (int *) minor; // store just minor there
    dev_dbg(g_dev_ptr,"minor=0x%x\n", minor);
    switch (minor) {
    case  DEV393_MINOR(DEV393_KLOGGER):
        inode->i_size = file_size; //or return 8 - number of frame pages?
        return 0;
    default:
        return -EINVAL;
    }
}

/** Driver RELEASE method */
int        klogger393_release(struct inode *inode, ///< inode pointer
                              struct file *filp)   ///< file pointer
                                                   ///< @return OK - 0, -EINVAL for wrong minor
{
    int minor= MINOR(inode->i_rdev);
    switch ( minor ) {
    case  DEV393_MINOR(DEV393_KLOGGER):
                    dev_dbg(g_dev_ptr,"Release DONE, minor=0x%x\n", minor);
    break;
    default: return -EINVAL;
    }
    return 0;
}

/** Driver LLSEEK method */
loff_t  klogger393_llseek(struct file * file,  ///< file structure pointer
                       loff_t offset,       ///< 64-bit offset
                       int orig)            ///< Offset origin (SEEK_SET==0, SEEK_CUR==1, SEEK_END==2 - see fs.h)
                                            ///< @return new file position or negative error
{
    int minor=(int)file->private_data;
    dev_dbg(g_dev_ptr,"file=%x, offset=%llx (%d), orig=%x\r\n", (int) file, offset,(int) offset, (int) orig);
    switch (minor) {
    case DEV393_MINOR(DEV393_KLOGGER):
                switch (orig) {
                case SEEK_SET:
                    file->f_pos = offset;
                    break;
                case SEEK_CUR:
                    file->f_pos += offset;
                    break;
                case SEEK_END:
                    //!overload later?
                    if (offset<=0) {
                        file->f_pos = file_size + offset;
                    } else {
                        file->f_pos = file_size + offset; // Overload if needed as usual
                    }
                    break;
                default:
                    dev_err(g_dev_ptr,"lseek: invalid orig=%d\n", orig);
                    return -EINVAL;
                }
    break;
    default:
        dev_err(g_dev_ptr,"lseek: invalid minor=%d\n", minor);
        return -EINVAL;
    }
    /** truncate position */
    if (file->f_pos < 0) {
        dev_err(g_dev_ptr,"negative position: minor=%d, file->f_pos=0x%llx\n", minor, file->f_pos);
        file->f_pos = 0;
        return (-EOVERFLOW);
    }
    // No sense to output overwritten data in the buffer
    if (file->f_pos < (file_size - buffer_size))
        file->f_pos = (file_size - buffer_size);

    // enable seeking beyond buffer - it now is absolute position in the data stream
    if (file->f_pos > file_size) {
        file->f_pos = file_size;
        return (-EOVERFLOW);
    }
    return (file->f_pos);
}

ssize_t klogger393_read   (struct file * file, ///< file structure pointer
                           char * buf,         ///< user buffer to get data
                           size_t count,       ///< number of bytes to read
                           loff_t *off)        ///< start/current position to read from
                                               ///< @return number of bytes placed in the buffer
{
    int minor=(int)file->private_data;
    off_t buffer_rp;
    u32 len;
    u32 not_copied;
    switch (minor) {
    case DEV393_MINOR(DEV393_KLOGGER):
                            // Increase *off if it points to already overwritten data
                            if (*off < (file_size - buffer_size)) {
                                *off = (file_size - buffer_size);
                                // Keep count the same?
                            }
    // Truncate count if it exceeds remaining size in buffer
    if (count > (file_size - *off))
        count =  file_size - *off;
#ifdef KLOGGER_BUFFER_ANY_SIZE
    for (buffer_rp = *off; buffer_rp >= buffer_size;  buffer_rp -= buffer_size);
#else
    buffer_rp = *offt & buffer_size - 1; // buffer_size should be power of 2
#endif
    len = count;
    // first (usually the only) buffer copy
    if (len > (buffer_size - buffer_rp))
        len = (buffer_size - buffer_rp);
    not_copied=copy_to_user(buf, klog393_buf + buffer_rp, len); // returns number of bytes not copied
    if (not_copied) {
        dev_err(g_dev_ptr,"1. tried to copy 0x%x bytes to offset 0x%llx, not copied =0x%x bytes\n", count, *off,not_copied);
        return -EFAULT;
    }
    if (len < count) { // wrapping around the buffer
        not_copied=copy_to_user(buf+len, klog393_buf, count - len); // returns number of bytes not copied
        if (not_copied) {
            dev_err(g_dev_ptr,"2. tried to copy 0x%x bytes to offset 0x%llx, not copied =0x%x bytes\n", count, *off,not_copied);
            return -EFAULT;
        }
    }
    *off += count;
    // should we update file->f_pos=*off here too?)
    return count;
    break;
    default:
        dev_err(g_dev_ptr," Wrong minor=0x%x\n",minor);
        return -EINVAL;
    }
}

// SysFS interface to read/modify video memory map
#define SYSFS_PERMISSIONS           0644 /* default permissions for sysfs files */
#define SYSFS_READONLY              0444
#define SYSFS_WRITEONLY             0222

static ssize_t show_mode(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf,"0x%x\n", klog_mode);
}
static ssize_t store_mode(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    sscanf(buf, "%i", &klog_mode);
    return count;
}
static ssize_t show_stats(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf,"Number of writes:     0x%x\n"
                       "Buffer size:          0x%x (%d)\n"
                       "Buffer write pointer: 0x%x (%d)\n"
                       "File size:            0x%llx (%lld)\n"
                       "Mode:                 0x%x\n",
                       num_writes, buffer_size, buffer_size, buffer_wp, buffer_wp, file_size, file_size, klog_mode);
}
static DEVICE_ATTR(klogger_mode,    SYSFS_PERMISSIONS,     show_mode,                store_mode);
static DEVICE_ATTR(klogger_stats,   SYSFS_READONLY,        show_stats,                NULL);
static struct attribute *root_dev_attrs[] = {
        &dev_attr_klogger_mode.attr,
        &dev_attr_klogger_stats.attr,
        NULL
};

static const struct attribute_group dev_attr_root_group = {
    .attrs = root_dev_attrs,
    .name  = NULL,
};

static int klogger_393_sysfs_register(struct platform_device *pdev)
{
    int retval=0;
    struct device *dev = &pdev->dev;
    if (&dev->kobj) {
        if (((retval = sysfs_create_group(&dev->kobj, &dev_attr_root_group)))<0) return retval;
    }
    return retval;
}




int klogger_393_probe(struct platform_device *pdev)
{
    int res;
    struct device *dev = &pdev->dev;
    const   char * config_string;
    struct device_node *node = pdev->dev.of_node;
    const __be32 *bufsize_be;
    // char device for sensor port
    struct device *chrdev;

    g_dev_ptr = dev;
    buffer_wp = 0;
    if (node) {
#if 0
        if (of_property_read_string(node, klogger393_of_prop_bufsize_name, &config_string)) {
            dev_err(dev,"%s: Device tree has entry for "DEV393_NAME(DEV393_KLOGGER)", but no '%s' property is provided\n",
                    __func__,klogger393_of_prop_bufsize_name);
            return -EINVAL;
        }

        if (!sscanf(config_string,"%i", &buffer_size)){
            dev_err(dev,"%s: Invalid buffer size for "DEV393_NAME(DEV393_KLOGGER)".%s - %s\n",
                    __func__,klogger393_of_prop_bufsize_name,config_string);
            return -EINVAL;
        }
#endif

        bufsize_be = (__be32 *)of_get_property(node, klogger393_of_prop_bufsize_name, NULL);
        if (!bufsize_be) {
            dev_err(dev,"%s: Device tree has entry for "DEV393_NAME(DEV393_KLOGGER)", but no '%s' property is provided\n",
                    __func__,klogger393_of_prop_bufsize_name);
            return -EINVAL;
        }

        buffer_size = be32_to_cpup(bufsize_be);

        //__builtin_clz
#ifndef KLOGGER_BUFFER_ANY_SIZE
        buffer_size_order = 31 - __builtin_clz(buffer_size);
        if (buffer_size & (( 1 << buffer_size_order) -1)){
            buffer_size_order++;
            buffer_size = 1 << buffer_size_order;
            dev_warn(dev,"%s: Increased buffer size to %d (0x%x) to make it a power of 2 bytes\n",
                    __func__,buffer_size,buffer_size);
        }
#endif
        dev_info(dev,"%s: Setting up buffer for logging "DEV393_NAME(DEV393_KLOGGER)" of %d(0x%x) bytes\n",
                __func__,buffer_size,buffer_size);

//        klog393_buf = devm_kzalloc(dev, buffer_size, GFP_KERNEL);
        klog393_buf = kzalloc(buffer_size, GFP_KERNEL);
        if (!klog393_buf){
            buffer_size = 0;
            dev_err(dev,"%s: Failed to create buffer for "DEV393_NAME(DEV393_KLOGGER)" of %d(0x%x) bytes\n",
                    __func__,buffer_size,buffer_size);
            return -ENOMEM;
        }
        dev_info(dev,"%s: Set up buffer for logging "DEV393_NAME(DEV393_KLOGGER)" of %d(0x%x) bytes @0x%08x\n",
                __func__,buffer_size,buffer_size, (int) klog393_buf);
        res = register_chrdev(DEV393_MAJOR(DEV393_KLOGGER), DEV393_NAME(DEV393_KLOGGER), &framepars_fops);
        if (res < 0) {
            dev_err(dev, "klogger_393_probe: couldn't get a major number %d (DEV393_MAJOR(DEV393_KLOGGER)).\n",
                    DEV393_MAJOR(DEV393_KLOGGER));
            return res;
        }

    	// create device class
    	klogger_dev_class = class_create(THIS_MODULE, DEV393_NAME(DEV393_KLOGGER));
    	if (IS_ERR(klogger_dev_class)) {
    		pr_err("Cannot create \"%s\" class", DEV393_NAME(DEV393_KLOGGER));
    		return PTR_ERR(klogger_dev_class);
    	}

    	// create device
		chrdev = device_create(
				  klogger_dev_class,
				  &pdev->dev,
				  MKDEV(klogger_major, klogger_minor),
				  NULL,
				  "%s",klogger_dev);
		if(IS_ERR(chrdev)){
			pr_err("Failed to create a device (klogger). Error code: %ld\n",PTR_ERR(chrdev));
		}

    } else {
        dev_warn(dev,"%s: No entry for "DEV393_NAME(DEV393_KLOGGER)", in Device Tree, logger is disabled\n", __func__);

    }

    res = klogger_393_sysfs_register(pdev);
    dev_info(dev, DEV393_NAME(DEV393_KLOGGER)": registered sysfs, result = %d\n", res);
    return 0;
}

int klogger_393_remove(struct platform_device *pdev)
{
    if (klog393_buf) {
//        devm_kfree(&pdev->dev, klog393_buf); // actually not needed
    }

    device_destroy(klogger_dev_class, MKDEV(klogger_major, klogger_minor));

    unregister_chrdev(DEV393_MAJOR(DEV393_KLOGGER), DEV393_NAME(DEV393_KLOGGER));
    return 0;
}

static const struct of_device_id klogger_393_of_match[] = {
        { .compatible = "elphel,klogger-393-1.00" },
        { /* end of list */ }
};
MODULE_DEVICE_TABLE(of, klogger_393_of_match);

static struct platform_driver klogger_393 = {
        .probe          =klogger_393_probe,
        .remove         =klogger_393_remove,
        .driver         = {
                .name       = DEV393_NAME(DEV393_KLOGGER),
                .of_match_table = klogger_393_of_match,
        },
};

module_platform_driver(klogger_393);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andrey Filippov <andrey@elphel.com>.");
MODULE_DESCRIPTION("Record/playback strings with FPGA timestams");

