/*******************************************************************************
* FILE NAME  : clock10359.c
* DESCRIPTION: Control of the CY22393 clock on the 10359 multiplexer connected
* to the sensor port
* Copyright 2002-2016 (C) Elphel, Inc.
* -----------------------------------------------------------------------------*
*
*  This program is free software: you can redistribute it and/or modify
*  it under the terms of the GNU General Public License as published by
*  the Free Software Foundation, either version 2 of the License, or
*  (at your option) any later version.
*
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
*
*  You should have received a copy of the GNU General Public License
*  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*******************************************************************************/
/****************** INCLUDE FILES SECTION ***********************************/
#define DEBUG /* should be before linux/module.h - enables dev_dbg at boot in this file */

#include <linux/module.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/page.h>
#include <linux/io.h>

#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_net.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>

#include "x393.h"
#include "sensor_i2c.h"
#include "clock10359.h"


#define SYSFS_PERMISSIONS         0644 /* default permissions for sysfs files */
#define SYSFS_READONLY            0444
#define SYSFS_WRITEONLY           0222

#define DRV_NAME "elphel_clock10359"

static int clock_frequency[16]; // in Hz per port, per clock
static struct device *sdev = NULL; // store this device here
const char CLOCK_NAME[] = "cy22393";

typedef struct {
  unsigned int p :   11; // p - 16..1600
  unsigned int q :   8;  // q - 0.. 255
  unsigned int dv:   7;  // dv - 1.. 127
  unsigned int corr: 3;  // 0/1/2/3/4
  unsigned int rslt:   3;  // 0 - OK, 1 - frequency low, 2 - frequency high, 3 - other
} t_pll_params;

// in Hz
#define CY22393_SCALE 100 // precision will be 100Hz
#define CY22393_PLLMIN (100000000/CY22393_SCALE)
#define CY22393_PLLMAX (400000000/CY22393_SCALE)
#define CY22393_XTAL   ( 12000000/CY22393_SCALE)
#define CY22393_OUTMAX (166000000/CY22393_SCALE)
#define CY22393_PMIN 16
#define CY22393_PMAX 1600


int calc_pll_params (unsigned int f, t_pll_params * pars); // f -in Hz

int setCYField (int sensor_port, int addr, int mask, int value); /// bus==1 - FPGA (sensor bus through 10359), 0 - CPU bus
int setClockFreq(int nclock, int freq); // freq now in Hz

int calc_pll_params (unsigned int f, t_pll_params * pars) { // f -in Hz
//   t_pll_params pars;
	unsigned int f0= CY22393_XTAL;
	unsigned int f0_2= CY22393_XTAL/2;
	unsigned int fpmn= CY22393_XTAL * (CY22393_PMIN + 6);
	unsigned int fpmx= CY22393_XTAL * (CY22393_PMAX + 6);
	int divmn, divmx, err1,err, div,q,qmn,qmx,qmx1,fdv,p, e,fdvq;
	pars->rslt=3; // other error
	dev_dbg(sdev, "f0=%d,f=%d, CY22393_OUTMAX=%d\r\n",f0,f,CY22393_OUTMAX);

	f/=CY22393_SCALE; // to fit into 32-bit calculations
	dev_dbg(sdev, "f0=%d,f=%d, CY22393_OUTMAX=%d\r\n",f0,f,CY22393_OUTMAX);
	if (f>CY22393_OUTMAX) {
	 pars->rslt=2;
		dev_dbg(sdev, "f0=%d,f=%d, CY22393_OUTMAX=%d\r\n",f0,f,CY22393_OUTMAX);
	 return pars->rslt;
	}
	if (f <=0 ) {
	 pars->rslt=1;
		dev_dbg(sdev, "f0=%d,f=%d, CY22393_OUTMAX=%d\r\n",f0,f,CY22393_OUTMAX);
	 return pars->rslt;
	}
	divmx=CY22393_PLLMAX/f; if (divmx > 127) divmx=127; // could not be <1
	divmn=CY22393_PLLMIN/f; if (divmn < 1)   divmn=1;
	if (divmn >127) {
	 pars->rslt=1;
		dev_dbg(sdev, "f0=%d,f=%d, CY22393_OUTMAX=%d, divmn=%d\r\n",f0,f,CY22393_OUTMAX,divmn);
	 return pars->rslt;
	}
	err1=f;
	qmx1=0;
	for (div=divmn;div<=divmx;div++) {
		err=err1*div;
		fdv=f*div;
		qmn=fpmn/fdv-2; if (qmn < 0) qmn=0;
		qmx=fpmx/fdv-2; if (qmx >255) qmx=255;
		// recalculate qmn to avoid same div*qmn as already tried with lover div
		dev_dbg(sdev, "div=%d,qmn=%d, qmx=%d\r\n",div,qmn,qmx);
		if (div==1) qmx1=qmx;
		else if ((qmn*div) < qmx1) qmn=qmx1/div;
		for (q=qmn+2;q<=qmx+2; q++) {
			fdvq=fdv*q;
			p= (fdvq+f0_2)/f0; // can p be out of range here?
			e= fdvq-f0*p; if (e<0) e=-e;
			if (e< (err*q)) { // better solution found
				pars->rslt=0;
				pars->p=p-6;
				pars->q=q-2;
				pars->dv=div;
				err1=e/q/div;
				err=err1*div;
				dev_dbg(sdev, "f=%d, div=%d, p=%d,q=%d, err1=%d\r\n", (f0*p)/q/div, div,p, q, err1);
				if (err1==0) {
					pars->corr=(pars->p<226)?0:((pars->p<621)?1:((pars->p<829)?2:((pars->p<1038)?3:4)));
					dev_dbg(sdev, "f=%d, div=%d, p=%d,q=%d, err1=%d, rslt=%d\r\n",
									(f0*(pars->p+6))/(pars->q+2)/pars->dv,
									pars->dv,
									(pars->p+6),
									(pars->q+2),
									err1,
									pars->rslt);
					return pars->rslt;
				}
			}
		}
	}
	dev_dbg(sdev, "f=%d, div=%d, p=%d,q=%d, err1=%d, rslt=%d\r\n",
				   (f0*(pars->p+6))/(pars->q+2)/pars->dv,
				   pars->dv,
				   (pars->p+6),
				   (pars->q+2),
				   err1,
				   pars->rslt);
	pars->corr=(pars->p<226)?0:((pars->p<621)?1:((pars->p<829)?2:((pars->p<1038)?3:4)));
	return pars->rslt;
}

int setCYField (int sensor_port, int reg_addr, int mask, int value) {
	int error;
	int reg_data;
	if ((error = x393_xi2c_read_reg(CLOCK_NAME,       // device class name
									sensor_port,      // sensor port number
				                    0,                // slave address (7-bit) offset from the class defined slave address
									reg_addr,         // register address (width is defined by class)
									&reg_data)) <0) { // pointer to a data receiver (read data width is defined by class)
		dev_err(sdev,"setCYField(%d, 0x%x, 0x%x,0x%x) failed reading i2c register\n",
				sensor_port, reg_addr, mask, value);
		return error;
	}
	reg_data ^= (reg_data ^ value) & mask;
	if ((error = x393_xi2c_write_reg(CLOCK_NAME,       // device class name
									 sensor_port,      // sensor port number
				                     0,                // slave address (7-bit) offset from the class defined slave address
									 reg_addr,         // register address (width is defined by class)
									 reg_data)) <0)  { // data to write (width is defined by class)
		dev_err(sdev,"setCYField(%d, 0x%x, 0x%x, 0x%x) failed writing 0x%x to i2c register\n",
				sensor_port, reg_addr, mask, value,reg_data);
		return error;
	}
}

int x393_getClockFreq(int sensor_port, int nclock) {
  if ((sensor_port < 0) || (sensor_port > 3) || (nclock < 0) || (nclock > 3))return -EINVAL; // bad clock number
  else {
    return clock_frequency[(sensor_port << 2) || nclock];
  }
}
EXPORT_SYMBOL_GPL(x393_getClockFreq);


int x393_setClockFreq(int sensor_port, int nclock, int freq) { // freq now in Hz
	int err=0;
	sensor_port &= 3;
	nclock &= 3;
	t_pll_params  pll_params;
	int i,bp,bq,bdiv,pllc,fact;
	bp=0; bq=0; bdiv=0; pllc= 0; // just to make gcc happy
	fact=0;
	dev_dbg(sdev, "setClockFreq(%d,%d,%d)\r\n",sensor_port,nclock,freq);
	if ((freq!=0) && (nclock!=3) ){
		if ( (i=calc_pll_params (freq, &pll_params)) !=0) {
			dev_err(sdev, "bad frequency for clock %d - %d Hz, err=%d\n",nclock,freq,i);
			return -EINVAL;
		}
		fact=CY22393_SCALE*(CY22393_XTAL*(pll_params.p+6)/(pll_params.q+2)/pll_params.dv);
		bp=  pll_params.p;    // (freqtab[freq]>>20)&0x7ff;
		bq=  pll_params.q;    // (freqtab[freq]>>12)&0xff;
		bdiv=pll_params.dv;   // (freqtab[freq]>> 4)&0xff;
		pllc=pll_params.corr; // freqtab[freq]     &0x0f;
	}
	switch (nclock) {
		case 0:
			if (freq==0) {
				err |= setCYField (sensor_port,0x16, 0x40, 0x00); // turn off pll3
				err |= setCYField (sensor_port,0x09, 0x7f, 0x00); // turn off divider- A
				err |= setCYField (sensor_port,0x08, 0x7f, 0x00); // turn off divider- A
			} else {
				err |= setCYField (sensor_port,0x16, 0x7f, 0x40+(pllc<<3)+((bp & 1)<<2)+((bp & 0x600)>>9) );
				err |= setCYField (sensor_port,0x15, 0xff, ((bp & 0x1fe)>>1) );
				err |= setCYField (sensor_port,0x14, 0xff, bq );
				err |= setCYField (sensor_port,0x09, 0x7f, bdiv); // set divider- A
				err |= setCYField (sensor_port,0x08, 0x7f, bdiv); // set divider- A
				err |= setCYField (sensor_port,0x0e, 0x03, 0x03); // set PLL3 as source for ClkA
			}
		//          fpga_state |= FPGA_STATE_CLOCKS;
			break;
		case 1:
			if (freq==0) {
				err |= setCYField (sensor_port,0x0b, 0x7f, 0x00); // turn off divider- B
				err |= setCYField (sensor_port,0x0a, 0x7f, 0x00); // turn off divider- B
				for (i=0;i<24;i+=3) err |= setCYField (sensor_port,0x42+i, 0x40, 0x00);  // turn off pll1
			} else {
				// progam all variants
				for (i=0;i<24;i+=3) {
				  err |= setCYField (sensor_port,0x42+i, 0x7f, 0x40+(pllc<<3)+((bp & 1)<<2)+((bp & 0x600)>>9) );
				  err |= setCYField (sensor_port,0x41+i, 0xff, ((bp & 0x1fe)>>1) );
				  err |= setCYField (sensor_port,0x40+i, 0xff, bq );
				}
				err |= setCYField (sensor_port,0x0b, 0x7f, bdiv); // set divider- B
				err |= setCYField (sensor_port,0x0a, 0x7f, bdiv); // set divider- B
				err |= setCYField (sensor_port,0x0e, 0x0c, 0x04); // set PLL1 as source for ClkB
			}
			break;
		case 2:
			if (freq==0) {
				err |= setCYField (sensor_port,0x13, 0x40, 0x00); // turn off pll2
				err |= setCYField (sensor_port,0x0d, 0x7f, 0x00); // turn off divider- D
			} else {
				err |= setCYField (sensor_port,0x13, 0x7f, 0x40+(pllc<<3)+((bp & 1)<<2)+((bp & 0x600)>>9) );
				err |= setCYField (sensor_port,0x12, 0xff, ((bp & 0x1fe)>>1) );
				err |= setCYField (sensor_port,0x11, 0xff, bq );
				err |= setCYField (sensor_port,0x0d, 0x7f, bdiv); // set divider- D
				err |= setCYField (sensor_port,0x0e, 0xc0, 0x80); // set PLL2 as source for ClkD
			}
			break;
		case 3:
			if ((freq!=0) && (freq!=CY22393_SCALE*CY22393_XTAL)) {
				dev_err(sdev, "Only frequency 0 (off) and %d Hz (xtal) are allowed for channel 3\r\n",CY22393_SCALE*CY22393_XTAL);
				return -EINVAL;
			} else {
			//  int setCYField (sensor_port,int devfd, int addr, int mask, int value) O_RDWR
				if (freq==0) {
				  err |= setCYField (sensor_port,0x0f, 0x04, 0x00);
				} else {
				  err |= setCYField (sensor_port,0x0f, 0x04, 0x04);
				  fact= CY22393_SCALE*CY22393_XTAL;
				}
			}
			break;
		default: return -1; // wrong clock number
	}
	dev_dbg(sdev, "nclock=%d fact=%d\n",nclock,fact);
	if (err != 0) {
		dev_err(sdev, "Error programming clock %d fact=%d, err=0x%x\n",nclock,fact, err);
		return err;
	}
	clock_frequency[(sensor_port << 2) + nclock] = fact;
	return fact;
}
EXPORT_SYMBOL_GPL(x393_setClockFreq);






//	dev_warn(dev,"i2c_member_store(): not implemented\n");

/* =========================== sysfs functionality ============================== */

/* Get channel port and channel number from the last 2 character of the attribute name*/
static int get_port_channel_from_name(struct device_attribute *attr){
	int reg = 0;
	int sport,nclock;
	sscanf(attr->attr.name + (strlen(attr->attr.name)-2), "%du", &reg);
	sport = reg / 10;
	nclock = reg - 10 * sport;
	return (nclock & 3) + ((sport &3) <<2);
}

static ssize_t clock_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int chn = get_port_channel_from_name(attr) ;
	int sensor_port = chn >> 2;
	int nclock = chn & 3;
	int ni, freq, err;
	ni = sscanf(buf,"%i", &freq);
    if (ni < 1) {
    	dev_err(dev, "Requires clock frequency in Hz");
        return -EINVAL;
    }
	if ((err = x393_setClockFreq(sensor_port, nclock, freq)))
		return err;
    return count;
}

static ssize_t clock_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int chn = get_port_channel_from_name(attr);
	int sensor_port = chn >> 2;
	int nclock = chn & 3;
	int freq = x393_getClockFreq(sensor_port, nclock);
	if (freq < 0)
		return freq;
	return sprintf(buf,"%d\n",freq);
}


// Get i2c read data from fifo
static ssize_t clock_help_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf,"Numeric suffix in file names selects sensor port/clock number\n"
					   "clock<p><c>:  read - get programmed clock frequency in Hz, write: set clock frequency\n"
			);
}

//==================================

static DEVICE_ATTR(clock00  ,       SYSFS_PERMISSIONS                 ,    clock_show ,     clock_store);
static DEVICE_ATTR(clock01  ,       SYSFS_PERMISSIONS                 ,    clock_show ,     clock_store);
static DEVICE_ATTR(clock02  ,       SYSFS_PERMISSIONS                 ,    clock_show ,     clock_store);
static DEVICE_ATTR(clock03  ,       SYSFS_PERMISSIONS                 ,    clock_show ,     clock_store);
static DEVICE_ATTR(clock10  ,       SYSFS_PERMISSIONS                 ,    clock_show ,     clock_store);
static DEVICE_ATTR(clock11  ,       SYSFS_PERMISSIONS                 ,    clock_show ,     clock_store);
static DEVICE_ATTR(clock12  ,       SYSFS_PERMISSIONS                 ,    clock_show ,     clock_store);
static DEVICE_ATTR(clock13  ,       SYSFS_PERMISSIONS                 ,    clock_show ,     clock_store);
static DEVICE_ATTR(clock20  ,       SYSFS_PERMISSIONS                 ,    clock_show ,     clock_store);
static DEVICE_ATTR(clock21  ,       SYSFS_PERMISSIONS                 ,    clock_show ,     clock_store);
static DEVICE_ATTR(clock22  ,       SYSFS_PERMISSIONS                 ,    clock_show ,     clock_store);
static DEVICE_ATTR(clock23  ,       SYSFS_PERMISSIONS                 ,    clock_show ,     clock_store);
static DEVICE_ATTR(clock30  ,       SYSFS_PERMISSIONS                 ,    clock_show ,     clock_store);
static DEVICE_ATTR(clock31  ,       SYSFS_PERMISSIONS                 ,    clock_show ,     clock_store);
static DEVICE_ATTR(clock32  ,       SYSFS_PERMISSIONS                 ,    clock_show ,     clock_store);
static DEVICE_ATTR(clock33  ,       SYSFS_PERMISSIONS                 ,    clock_show ,     clock_store);
static DEVICE_ATTR(help,            SYSFS_PERMISSIONS & SYSFS_READONLY,    clock_help_show, NULL);

static struct attribute *root_dev_attrs[] = {
		&dev_attr_clock00.attr,
		&dev_attr_clock01.attr,
		&dev_attr_clock02.attr,
		&dev_attr_clock03.attr,
		&dev_attr_clock10.attr,
		&dev_attr_clock11.attr,
		&dev_attr_clock12.attr,
		&dev_attr_clock13.attr,
		&dev_attr_clock20.attr,
		&dev_attr_clock21.attr,
		&dev_attr_clock22.attr,
		&dev_attr_clock23.attr,
		&dev_attr_clock30.attr,
		&dev_attr_clock31.attr,
		&dev_attr_clock32.attr,
		&dev_attr_clock33.attr,
        &dev_attr_help.attr,
        NULL
};

static const struct attribute_group dev_attr_root_group = {
	.attrs = root_dev_attrs,
	.name  = NULL,
};

static int elphel393_clock10359_sysfs_register(struct platform_device *pdev)
{
	int retval=0;
	struct device *dev = &pdev->dev;
	if (&dev->kobj) {
		if (((retval = sysfs_create_group(&dev->kobj, &dev_attr_root_group)))<0) return retval;
		dev_dbg(dev,"sysfs_create_group(dev_attr_root_group) done \n");
	}
	return retval;
}
// =======================================
static void elphel393_clock10359_init_of(struct platform_device *pdev)
{
	pr_info("elphel393_clock10359_init_of()\n");
}


static int elphel393_clock10359_probe(struct platform_device *pdev)
{
	sdev =&pdev->dev;
	dev_dbg(&pdev->dev,"Probing elphel_clock10359\n");

	elphel393_clock10359_sysfs_register(pdev);
	dev_dbg(&pdev->dev,"elphel393_clock10359_sysfs_register() done\n");

	elphel393_clock10359_init_of(pdev);
	dev_dbg(&pdev->dev,"done probing elphel393_clock10359_probe\n");

	return 0;
}

static int elphel393_clock10359_remove(struct platform_device *pdev)
{
	dev_info(&pdev->dev,"Removing elphel393-sensor-i2c");
	return 0;
}

static struct of_device_id elphel393_clock10359_of_match[] = {
	{ .compatible = "elphel,elphel393-sensor-i2c-1.00", },
	{ /* end of table */}
};

MODULE_DEVICE_TABLE(of, elphel393_clock10359_of_match);

static struct platform_driver elphel393_clock10359 = {
	.probe   = elphel393_clock10359_probe,
	.remove  = elphel393_clock10359_remove,
	.driver  = {
		.name  = "elphel393-sensor-i2c",
		.owner = THIS_MODULE,
		.of_match_table = elphel393_clock10359_of_match,
		.pm = NULL, /* power management */
	},
};

module_platform_driver(elphel393_clock10359);

MODULE_AUTHOR("Andrey Filippov  <andrey@elphel.com>");
MODULE_DESCRIPTION("Elphel 10393 sensor ports i2c");
MODULE_LICENSE("GPL");


