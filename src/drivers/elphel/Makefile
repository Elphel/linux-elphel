#                                                                                                                                    
# Makefile for Elphel specific devices.                                                                                              
#                                                                                                                                    
                                                                                                                                     
obj-$(CONFIG_ELPHEL393)           += elphel393-pwr.o  
obj-$(CONFIG_ELPHEL393)           += elphel393-mem.o  
obj-$(CONFIG_ELPHELDRVONMICROZED) += elphel393-mem.o
obj-$(CONFIG_ELPHEL393_INIT)      += elphel393-init.o   

obj-$(CONFIG_ELPHEL393)           += framepars.o
obj-$(CONFIG_ELPHEL393)           += sensor_common.o
obj-$(CONFIG_ELPHEL393)           += quantization_tables.o
obj-$(CONFIG_ELPHEL393)           += circbuf.o jpeghead.o

#fpars-y := framepars.o sensor_common.o
#sc-y := sensor_common.o quantization_tables.o framepars.o
#cbuf-y := circbuf.o fpars.o sc.o jpeghead.o
#obj-$(CONFIG_ELPHEL393_EXTERNAL)  += fpars.o
#obj-$(CONFIG_ELPHEL393_EXTERNAL)  += sc.o
#obj-$(CONFIG_ELPHEL393_EXTERNAL)  += cbuf.o
#obj-$(CONFIG_ELPHEL393_EXTERNAL)  += circbuf.o jpeghead.o quantization_tables.o 
