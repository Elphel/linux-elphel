#                                                                                                                                    
# Makefile for Elphel specific devices.                                                                                              
#                                                                                                                                    
                                                                                                                                     
obj-$(CONFIG_ELPHEL393)           += elphel393-pwr.o  
obj-$(CONFIG_ELPHEL393)           += elphel393-mem.o  
obj-$(CONFIG_ELPHELDRVONMICROZED) += elphel393-mem.o
obj-$(CONFIG_ELPHEL393_INIT)      += elphel393-init.o   
obj-$(CONFIG_ELPHEL393)           += x393.o
obj-$(CONFIG_ELPHEL393)           += sensor_i2c.o
fpgajtag-y := fpgajtag353.o x393.o 
obj-$(CONFIG_ELPHEL393_EXTERNAL)  += fpgajtag.o