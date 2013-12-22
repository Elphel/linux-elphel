/*!***************************************************************************
 *! FILE NAME  : elphel393-pwr.c
 *! DESCRIPTION: power supplies control on Elphel 10393 board 
 *! Copyright (C) 2013 Elphel, Inc.
 *! -----------------------------------------------------------------------------**
 *!
 *!  This program is free software: you can redistribute it and/or modify
 *!  it under the terms of the GNU General Public License as published by
 *!  the Free Software Foundation, either version 3 of the License, or
 *!  (at your option) any later version.
 *!
 *!  This program is distributed in the hope that it will be useful,
 *!  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *!  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *!  GNU General Public License for more details.
 *!
 *!  You should have received a copy of the GNU General Public License
 *!  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/string.h>
#include <linux/of.h>
#include <linux/gpio.h>
#include <linux/i2c/ltc3589.h>

#define DRIVER_DESCRIPTION	"Elphel 10393 power supply control"
#define DRIVER_VERSION		"1.00"

#define SYSFS_PERMISSIONS         0644 /* default permissions for sysfs files */
#define SYSFS_READONLY            0444
#define SYSFS_WRITEONLY           0222


#define GPIO_CHIP1_ADDR 0x20
#define GPIO_CHIP2_ADDR 0x21
#define LTC3589_ADDR    0x34



struct elphel393_pwr_data_t {
	int chip_i2c_addr[3];
	int simulate; /* do not perform actual i2c writes */
	struct mutex lock;
};

struct pwr_gpio_t {
	const char * label;
	int pin;
};


static struct pwr_gpio_t pwr_gpio [16]={
/* 0x20: */
		{"PWR_MGB1",    0}, /* 1.8V margining magnitude (0 - 5%, 1 - 10%, float - 15%) */
		{"PWR_MG1",     1}, /* 1.8V margining enable 0 - negative margining, 1 - positive margining, float - no margining */
		{"PWR_MGB0",    2}, /* 1.0V margining magnitude (0 - 5%, 1 - 10%, float - 15%) */
		{"PWR_MG0",     3},  /* 1.0V margining enable 0 - negative margining, 1 - positive margining, float - no margining */
		{"PWR_FQ0",     4}, /*   float - nominal frequency (should float for SS), 0 - 0.67 nominal frequency, 1 - 1.5 nominal frequency */ 
		{"PWR_SS",      5}, /*  Spread spectrum, 0 or float - spread spectrum disabled  */
		{"SENSPWREN0",  6}, /*  1 - enable 3.3 power to sensor connectors J6 and J7 (0 or float - disable) */
		{"SENSPWREN1",  7}, /*  1 - enable 3.3 power to sensor connectors J8 and J9 (0 or float - disable) */
/* 0x21: */
		{"NSHUTDOWN",   8}, /*  (pulled up). 0 - shutdown, 1 normal */
		{"DIS_POR",     9}, /* (pulled down). 0 - normal, 1 - disable POR generation on PGOOD deassertion (needed whil changing voltages) */
		{ NULL,        10}, /* Not connected */
		{ NULL,        11}, /* Not connected */
		{ NULL,        12}, /* Not connected */
		{ NULL,        13}, /* Not connected */
		{"MGTAVTTGOOD",14}, /*  (input) 1.2V linear regulator status (generated from 1.8V) */
		{"PGOOD18",    15} /*  (input). Combines other voltages, can be monitored when DIS_POR is activated */
};

static ssize_t simulate_show (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t simulate_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);


/* root directory */
//static DEVICE_ATTR(outputs,   SYSFS_PERMISSIONS & SYSFS_READONLY,  output_description_show, NULL);
//static DEVICE_ATTR(status,    SYSFS_PERMISSIONS & SYSFS_READONLY,  status_show, NULL);
static DEVICE_ATTR(simulate,    SYSFS_PERMISSIONS,                     simulate_show,       simulate_store);

static struct attribute *root_dev_attrs[] = {
		&dev_attr_simulate.attr,
	    NULL
};
static const struct attribute_group dev_attr_root_group = {
	.attrs = root_dev_attrs,
	.name  = NULL,
};

static ssize_t simulate_show (struct device *dev, struct device_attribute *attr, char *buf)
{
	struct elphel393_pwr_data_t *clientdata=platform_get_drvdata(to_platform_device(dev));
    return sprintf(buf, "%d\n",clientdata->simulate);
}
static ssize_t simulate_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct elphel393_pwr_data_t *clientdata=platform_get_drvdata(to_platform_device(dev));
    sscanf(buf, "%du", &clientdata->simulate);
    return count;
}

static int elphel393_pwr_sysfs_register(struct platform_device *pdev)
{
	int retval=0;
	struct device *dev = &pdev->dev;
	if (&dev->kobj) {
		if (((retval = sysfs_create_group(&dev->kobj, &dev_attr_root_group)))<0) return retval;
	}
	return retval;
}
static void elphel393_pwr_init_of(struct platform_device *pdev)
{
	const __be32 * config_data;
//	struct device *dev = &pdev->dev;
	struct device_node *node = pdev->dev.of_node;
	struct elphel393_pwr_data_t *clientdata = platform_get_drvdata(pdev);
	int len,i;
	if (node) {
		config_data = of_get_property(node, "elphel393_pwr,i2c_chips", &len);
		if (config_data){
			len /= sizeof(*config_data);
			dev_info(&pdev->dev,"Found %d items in 'elphel393_pwr,i2c_chips' in the Device Tree\n",len);
			if (len!= ARRAY_SIZE(clientdata->chip_i2c_addr)){
				dev_err(&pdev->dev,"Got %d items in 'elphel393_pwr,i2c_chips', expected %d\n",len,ARRAY_SIZE(clientdata->chip_i2c_addr));
				return;
			}
			for (i=0;i<len;i++)	clientdata->chip_i2c_addr[i]=be32_to_cpup(&config_data[i]);
		}
	}	
}

static int device_by_i2c_addr_match(struct device *dev, void *data)
{
	struct i2c_client *client = to_i2c_client(dev);
	int *addr = (int *)data;
	dev_info(dev,"addr_given=0x%02x, addr found=0x%02x\n",addr[0],(int) client->addr);
	return i2c_verify_client(dev) && (client->addr==addr[0]);
}

static struct device * find_device_by_i2c_addr(int address)
{
	return bus_find_device(&i2c_bus_type, NULL, &address, device_by_i2c_addr_match);
}

static int i2c_addr_gpiochip_match(struct gpio_chip *chip, void *data)
{
	struct i2c_client *client = to_i2c_client(chip->dev);
	int *addr = (int *)data;
	dev_info(chip->dev,"addr_given=0x%02x, addr found=0x%02x\n",addr[0],(int) client->addr);
	return i2c_verify_client(chip->dev) && (client->addr==addr[0]);
}

static int elphel393_pwr_probe(struct platform_device *pdev)
{
	struct gpio_chip *chip;
	struct device * ltc3489_dev;
	int i,rc;
	int base[2];
	struct i2c_client *ltc3589_client;
	struct elphel393_pwr_data_t *clientdata = NULL;

	dev_info(&pdev->dev,"+++ Probing elphel393-pwr +++");
	clientdata = devm_kzalloc(&pdev->dev, sizeof(*clientdata), GFP_KERNEL);
	clientdata->chip_i2c_addr[0]=0x20;
	clientdata->chip_i2c_addr[1]=0x21;
	clientdata->chip_i2c_addr[2]=0x34;
	platform_set_drvdata(pdev, clientdata);

	elphel393_pwr_sysfs_register(pdev);
	elphel393_pwr_init_of(pdev);

	mutex_init(&clientdata->lock);
	
	/* locate GPIO chips by i2c address */
    for (i=0;i<2;i++){
    	chip = gpiochip_find(&clientdata->chip_i2c_addr[i], i2c_addr_gpiochip_match);
    	base[i]=chip->base;
    	dev_info(&pdev->dev,"Found gpio_chip with i2c_addr=0x%02x, label=%s, base=0x%x\n",clientdata->chip_i2c_addr[i],chip->label,base[i]);
    }
    for (i=0;i<ARRAY_SIZE(pwr_gpio);i++) if (pwr_gpio[i].label){
    	pwr_gpio[i].pin=base[i>>3]+(i & 7);
    	rc=gpio_request(pwr_gpio[i].pin, pwr_gpio[i].label);
    	if (rc<0){
    		dev_err(&pdev->dev," Failed to get GPIO[%d] with label %s\n",pwr_gpio[i].pin,pwr_gpio[i].label);
    	} else {
    		dev_info(&pdev->dev,"Confirmed request GPIO[%d] with label %s\n",pwr_gpio[i].pin,pwr_gpio[i].label);
    	}
    }
    /* find ltc3589 */
    ltc3489_dev=find_device_by_i2c_addr(LTC3589_ADDR);
    if (!ltc3489_dev){
		dev_err(&pdev->dev," Failed to find LTC3489 with i2c address 0x%02x\n",LTC3589_ADDR);
		return -EIO;
    }
    ltc3589_client = to_i2c_client(ltc3489_dev);

	dev_info(&pdev->dev,"Located %s with i2c address 0x%02x\n",ltc3589_client->name,LTC3589_ADDR);
	dev_info(&pdev->dev,"LTC3589 status= 0x%02x\n",read_field_ltc3589(ltc3589_client, LTC3589_AWE_PGSTAT));
	
	return 0;
}	


static int elphel393_pwr_remove(struct platform_device *pdev)
{
	dev_info(&pdev->dev,"+++ Removing elphel393-pwr +++");
	return 0;
}



static struct of_device_id elphel393_pwr_of_match[] = {
	{ .compatible = "elphel,elphel393-pwr-1.00", },
	{ /* end of table */}
};
MODULE_DEVICE_TABLE(of, elphel393_pwr_of_match);

static struct platform_driver elphel393_pwr = {
	.probe   = elphel393_pwr_probe,
	.remove  = elphel393_pwr_remove,
	.driver  = {
		.name  = "elphel393-pwr",
		.owner = THIS_MODULE,
		.of_match_table = elphel393_pwr_of_match,
		.pm = NULL, /* power management */
	},
};

module_platform_driver(elphel393_pwr);

MODULE_AUTHOR("Andrey Filippov  <andrey@elphel.com>");
MODULE_DESCRIPTION("Elphel 10393 power supply control");
MODULE_LICENSE("GPL");
