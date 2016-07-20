/*
 exif353.h
 */
#ifndef _EXIF_H
#define _EXIF_H
#if 0
struct exif_time_t {
  char          tomorrow_date[10]; //!"YYYY:MM:DD"
  unsigned long tomorrow_sec;      //!seconds from epoch tomorrow at 00:00
  char          today_date[10];    //!"YYYY:MM:DD"
  unsigned long today_sec;         //!seconds from epoch today at 00:00
} exif_time;

struct exif_datetime_t {
  char          datetime[20];      //!"YYYY:MM:DD HH:MM:SS\0"
  char          subsec[7];         //!ASCII microseconds (0-padded), ."\0"
} now_datetime;
#endif

void exif_invalidate(void);
int exif_rebuild_chn(int sensor_port, int frames);// reallocate meta buffer to store per-frame meta data (later output as Exif)
int exif_rebuild(int frames);                     // rebuild for all sensor ports

int exif_enable_chn(int sensor_port, int en);     // enable/disable Exif processing (both W/R)
int exif_enable(int en);                          // For all sensor ports
int dir_find_tag (unsigned long tag); //!find location of the tag field in meta page using long tag (Exif tag and tag group)

void write_meta_raw_irq(int sensor_port, char * data, int offset, int len); //write data to meta, called from IRQ
int write_meta_irq(int sensor_port, char * data, int * indx,  unsigned long ltag, int len); //write data to meta, called from IRQ(len==0 => use field length)
void putlong_meta_raw_irq(int sensor_port, unsigned long data, int offset); //write data to meta (4 bytes, big endian), called from IRQ
int putlong_meta_irq(int sensor_port, unsigned long data, int * indx,  unsigned long ltag); //write data to meta (4 bytes, big endian), from IRQ
//void write_meta_raw_irq(char * data, int offset, int len); //write data to meta, called from IRQ
//int write_meta_irq(char * data, int * indx,  unsigned long ltag, int len); //write data to meta, called from IRQ(len==0 => use field length). Returns index of the written data, -1 if not written

void write_meta_raw(int sensor_port, char * data, int offset, int len); //write data to meta, called from outside IRQ (atomic)
int write_meta(int sensor_port, char * data, int * indx,  unsigned long ltag, int len); //write data to meta, from outside IRQ (atomic) (len==0 => use field length). Returns index of the written data, -1 if not written
void putlong_meta_raw(int sensor_port, unsigned long data, int offset); //write data to meta (4 bytes, big endian), called from outside IRQ (atomic)
int putlong_meta(int sensor_port, unsigned long data, int * indx,  unsigned long ltag); //write data to meta (4 bytes, big endian), from outside IRQ (atomic). Returns index of the written data, -1 if not written

char * encode_time(char buf[27], unsigned long sec, unsigned long usec);
int store_meta(int sensor_port); //called from IRQ service - put current metadata to meta_buffer, return page index


#endif
