///@file legacy_defines.h
#define X313_MARGINS 4
#define   X3X3_RSTSENSDCM          // FPGA DCM can fail after clock change, needs to be reset
#define   X3X3_SENSDCM_CLK2X_RESET // reset pclk2x DCM also
#define I2C359_CLK_NUMBER 4


#define     CCAM_NEGRST   //set negative MRST polarity
#define     CCAM_TRIG_INT
#define     CCAM_MRST_OFF
#define     CCAM_ARST_OFF
#define     CCAM_RESET_MCONTR_ON  // Set mode that resets memory controller pointers after each frame sync. TODO: Later - make it work without?
#define     CCAM_ENDFRAMES_EN     // Enable ending frame being compressed if no more data will be available (frame ended before specified number of blocks compressed)
