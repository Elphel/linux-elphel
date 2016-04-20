#ifndef __F_EXIF__H_
#define __F_EXIF__H_

extern unsigned char exif_header[];
int exif_header_length(void);

#define EXIF_OFFSET	4

#define EXIF_FIRMWARE		0xC4
#define EXIF_FIRMWARE_LEN	27

//#define EXIF_DATE_TIME		0x7A
#define EXIF_DATE_TIME		0xE0
#define EXIF_DATE_TIME_LEN	20

//#define EXIF_ARTIST		0x8E
#define EXIF_ARTIST		0xF4
#define EXIF_ARTIST_LEN	18

//#define EXIF_DATE_TIME_OR		0xCA
#define EXIF_DATE_TIME_OR		0x0138
#define EXIF_DATE_TIME_OR_LEN	20

//#define EXIF_SUBSEC_OR		0xDE
#define EXIF_SUBSEC_OR		0x014C
#define EXIF_SUBSEC_OR_LEN	7

//#define EXIF_EXP		0xE6
#define EXIF_EXP		0x0130
#define EXIF_EXP_LEN	8

#define EXIF_IMAGE_ID		0x6E
#define EXIF_IMAGE_ID_LEN	64

struct exif_desc_t {
	unsigned char date_time[EXIF_DATE_TIME_LEN];
	unsigned char date_time_or[EXIF_DATE_TIME_OR_LEN];
	unsigned char subsec[EXIF_SUBSEC_OR_LEN];
	unsigned char artist[EXIF_ARTIST_LEN];
	unsigned char firmware[EXIF_FIRMWARE_LEN];
	unsigned long exp[2];
};
extern struct exif_desc_t exif_desc;

#endif //__F_EXIF__H_
