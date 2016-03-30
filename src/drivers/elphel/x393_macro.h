/**
 * @file x393_macro.h
 * @brief This file contains various macros used in multiple files.
 */
#ifndef _X393_MACRO
#define _X393_MACRO

/** @brief Number of image channels*/
#define IMAGE_CHN_NUM   4

/* These macro were removed from sensor_common.h*/
#define X313_LENGTH_MASK      0xff000000
#define X313_PADDED_FRAME(x)((((x)+67+CCAM_MMAP_META ) >>2) & 0xfffffff8)
#define X313_BUFFSUB(x,y) (((x)>=(y))? ((x)-(y)) : ((x)+ (CCAM_DMA_SIZE-(y))))
#define X313_BUFFADD(x,y) ((((x) + (y))<=CCAM_DMA_SIZE)? ((x) + (y)) : ((x) - (CCAM_DMA_SIZE-(y))))

#endif /* _X393_MACRO */
