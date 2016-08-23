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
#include <linux/io.h>
#include <linux/errno.h>
#include <linux/platform_device.h> // For sysfs
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


static struct device *g_dev_ptr=NULL; ///< Global pointer to basic device structure. This pointer is used in debugfs output functions
static u32 buffer_size = 0;
#ifndef KLOGGER_BUFFER_ANY_SIZE
    static u32 buffer_size_order = 0;
#endif

static u32 buffer_wp = 0;
static loff_t file_size = 0; // write pointer, 64 bits
static char * klog393_buf = NULL;
const char klogger393_of_prop_name[] = "klogger-393,buffer_size";
const static u32 max_string_len = PAGE_SIZE; ///< maximal string length

int        klogger393_open(struct inode *inode, struct file *filp);
int        klogger393_release(struct inode *inode, struct file *filp);
loff_t     klogger393_llseek(struct file * file, loff_t offset, int orig);
ssize_t    klogger393_read   (struct file * file, char * buf, size_t count, loff_t *off);
/** Put string into the logger buffer*/
int klog393_puts(const char * str) ///< String to log, limited by max_string_len (currently 4096 bytes)
                                   ///< @return 0 - OK,  -EMSGSIZE - string too long
{
    int sl =    strlen(str);
    u32 new_wp= buffer_wp+sl;
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


int klogger_393_probe(struct platform_device *pdev)
{
    int res;
    struct device *dev = &pdev->dev;
    const   char * config_string;
    struct device_node *node = pdev->dev.of_node;
    g_dev_ptr = dev;
    buffer_wp = 0;
    if (node) {
        if (of_property_read_string(node, klogger393_of_prop_name, &config_string)) {
            dev_err(dev,"%s: Device tree has entry for "DEV393_NAME(DEV393_KLOGGER)", but no '%s' property is provided\n",
                    __func__,klogger393_of_prop_name);
            return -EINVAL;
        }
        if (!sscanf(config_string,"%i", &buffer_size)){
            dev_err(dev,"%s: Invalid buffer size for "DEV393_NAME(DEV393_KLOGGER)".%s - %s\n",
                    __func__,klogger393_of_prop_name,config_string);
            return -EINVAL;
        }
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
        klog393_buf = devm_kzalloc(dev, buffer_size, GFP_KERNEL);
        if (!klog393_buf){
            buffer_size = 0;
            dev_err(dev,"%s: Failed to create buffer for "DEV393_NAME(DEV393_KLOGGER)" of %d(0x%x) bytes\n",
                    __func__,buffer_size,buffer_size);
            return -ENOMEM;
        }
        res = register_chrdev(DEV393_MAJOR(DEV393_FRAMEPARS0), DEV393_NAME(DEV393_FRAMEPARS0), &framepars_fops);
        if (res < 0) {
            dev_err(dev, "framepars_init: couldn't get a major number %d (DEV393_MAJOR(DEV393_FRAMEPARS0)).\n",
                    DEV393_MAJOR(DEV393_FRAMEPARS0));
            return res;
        }


    } else {
        dev_warn(dev,"%s: No entry for "DEV393_NAME(DEV393_KLOGGER)", in Device Tree, logger is disabled\n", __func__);

    }



    //    klogger_393_sysfs_register(pdev);
//    dev_info(dev, DEV393_NAME(DEV393_KLOGGER)": registered sysfs\n");
    return 0;
}

int klogger_393_remove(struct platform_device *pdev)
{
    if (klog393_buf)
        devm_kfree(&pdev->dev, klog393_buf);

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

