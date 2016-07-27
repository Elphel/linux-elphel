///@file legacy_defines.h
#define X313_MAXWIDTH       65536 // 4096 // multiple of 128
#define X313_MAXHEIGHT      65536 // 16384 // multiple of 16 - unsafe - not enough room for black level subtraction
#define X313_MAXHEIGHT_SAFE 65536 // 4096 // multiple of 16  OK for black level subtraction TODO: disable black level if unsafe
#define X313_MAP_FRAME ((X313_MAP_FPN) + (X313_MAXWIDTH) * (X313_MAXHEIGHT_SAFE))
#define X313_MARGINS 4
#define X313_TILEHOR 16
#define X313_TILEVERT 16
#define   X313_TIMESTAMPLEN 28 // pixels used for timestamp (in linescan mode added after the line)

#define   X3X3_RSTSENSDCM          // FPGA DCM can fail after clock change, needs to be reset
#define   X3X3_SENSDCM_CLK2X_RESET // reset pclk2x DCM also
#define I2C359_CLK_NUMBER 4


#define     CCAM_NEGRST   //set negative MRST polarity
#define     CCAM_TRIG_INT
#define     CCAM_MRST_OFF
#define     CCAM_ARST_OFF
#define     CCAM_RESET_MCONTR_ON  // Set mode that resets memory controller pointers after each frame sync. TODO: Later - make it work without?
#define     CCAM_ENDFRAMES_EN     // Enable ending frame being compressed if no more data will be available (frame ended before specified number of blocks compressed)


#define     CCAM_DCLK_ON
#define     CCAM_CNVEN_OFF
#define     CCAM_MRST_ON
#define     CCAM_EXTERNALTS_EN // Maybe use default as enabled - yes, it will not be active if not available
#define     CCAM_CNVEN_ON
#define     CCAM_POSRST

#define     X3X3_SENSDCM_HACT_ZERO
#define     X3X3_SENSDCM_HACT_LATE90
#define     X3X3_SENSDCM_HACT_EARLY90

#define     X3X3_SENSDCM_INC
#define     X3X3_SENSDCM_DEC
#define     X3X3_SENSDCM_INC90
#define     X3X3_SENSDCM_DEC90


 #define COMPCMD_DEMOS(x) ((1<<13) | (((x) & 0x0f) << 9))
 //  6 blocks output per macroblock:
 #define DEMOS_MONO6     0 // original monochrome YCbCr 4:2:0 with zeroed color components
 #define DEMOS_COLOR18   1 // original color YCbCr 4:2:0, 3x3 demosaic (18x18 tiles)
 #define DEMOS_JP46      2 // original jp4, (4:2:0, zero color), decoded by regular JPEG decoder
 #define DEMOS_JP46DC    3 // dc-improved: same as DEMOS_JP46, but DC difference separate for each component
 #define DEMOS_COLOR20   4 // color YCbCr 4:2:0, 5x5 demosaic (20x20 tiles) - not yet implemented
//  4 blocks output per macroblock:
 #define DEMOS_JP4       5 // similar to DEMOS_JP46 ,   but zero color components are not output
 #define DEMOS_JP4DC     6 // similar to DEMOS_JP46DC , but zero color components are not output
 #define DEMOS_JP4DIFF   7 // differential red := (R-G1), blue:=(B-G1), green=G1, green2 (G2-G1). G1 is defined by Bayer shift, any pixel can be used
 #define DEMOS_JP4HDR    8 // similar to DEMOS_JP4DIFF, but second green (opposite from the reference one) is encoded without subtracting:
                           // red := (R-G1), blue:=(B-G1), green=G1, green2 (high gain)=G2) (G1 and G2 - diagonally opposite)
 #define DEMOS_JP4DIFF2  9 // similar to DEMOS_JP4DIFF, but all differences are divided by 2 to fit into 8 bit range:
                           // red := (R-G1)/2, blue:=(B-G1)/2, green=G1, green2 (G2-G1)/2
 #define DEMOS_JP4HDR2  10 // red := (R-G1)/2, blue:=(B-G1)/2, green=G1, green2 (high gain)=G2),
 #define DEMOS_MONO4    14 // monochrome, but the block scan order is still the same as in YCbCr 4:2:0 (macroblocks in scan order, block in 2x2 macroblock in scan order)

// [8:7] == 0,1 - NOP, 2 - disable, 3 - enable subtracting of average value (DC component), bypassing DCT

 #define COMPCMD_DCSUB(x) ((1<<8) | (((x) & 1) << 7))

// [6] == 1 - enable quantization bank select, 0 - disregard bits [5:3]
// [5:3] = quantization page number (0..7)

 #define COMPCMD_QTAB(x) ((1<<6) | (((x) & 7) << 3))




#define CONFIG_ETRAX_ELPHEL_MT9X001 1

 void x313_dma_stop(){}
 void x313_dma_init(){}
 void reset_compressor(){}

// if ((gtable= get_gamma_fpga(color))) fpga_table_write_nice (CX313_FPGA_TABLES_GAMMA + (color * 256), 256, gtable);



// X3X3_SEQ_SEND1(frame16,  X313_WA_DCR0, X353_DCR0(SENSTRIGEN,async));

#ifdef NC353
 /// IRQ-safe "nice" FPGA table write and histogram read functions - they split the data in chunks of fixed size,
 /// disable IRQ, transfer a chunk, then reenable interrupt before proceedg to the next chunk
 #define FPGA_TABLE_CHUNK 64 // up to 64 words to send to the table/from histogram on a single IRQ-off transfer
 void fpga_table_write_nice (int addr, int len, unsigned long * data) {
   unsigned long flags;
   int l,i;
   MDF12(printk("addr=0x%x, len=0x%x, data=0x%08lx 0x%08lx 0x%08lx 0x%08lx...\n", addr, len, data[0], data[1], data[2], data[3]));
   while (len>0) {
     l=(len < FPGA_TABLE_CHUNK)?len:FPGA_TABLE_CHUNK;
     local_irq_save(flags);
     port_csp0_addr[X313_WA_COMP_TA]=addr; // open fpga for writing table(s)
     for (i=0; i<l; i++) port_csp0_addr[X313_WA_COMP_TD]=data[i]; /// will autoincrement FPGA table address
     local_irq_restore(flags);
     len  -=l;
     addr +=l;
     data +=l;
   }
 }

 ///
 /// reading histograms really does not need disabling IRQs - they only could interfere with other process, reading histograms
 ///
 void fpga_hist_read_nice (int addr, int len, unsigned long * data) {
   unsigned long flags;
   int l,i;
   MDF13(printk("addr=0x%x, len=0x%x, ",addr, len));
   while (len>0) {
     l=(len < FPGA_TABLE_CHUNK)?len:FPGA_TABLE_CHUNK;
     local_irq_save(flags);
 //  #define   X313_WA_HIST_ADDR   0x44
 //  #define   X313_RA_HIST_DATA   0x45  /// use CSP4 with wait cycles to have a pulse
     port_csp0_addr[X313_WA_HIST_ADDR]=addr; /// Write start address, read first word from the memory to the output buffer (will be read out during next read)
     X3X3_AFTERWRITE ; //! needed before reading from FPGA after writing to it (for the writes that influence reads only)
     for (i=0; i<l; i++) data[i]=port_csp4_addr[X313_RA_HIST_DATA]; /// will autoincrement FPGA table address)
     local_irq_restore(flags);
     len  -=l;
     addr +=l;
     data +=l;
   }
   D13(printk("data=0x%08lx 0x%08lx 0x%08lx 0x%08lx...\n", data[0], data[1], data[2], data[3]));
 }

#endif
