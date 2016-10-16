/*
 exifa.h
*/
#ifndef _ASM_EXIF_H
#define _ASM_EXIF_H

//Major
//// #define X3X3_EXIF 125
//Minors
////#define X3X3_EXIF_EXIF    0 // read encoded Exif data (SEEK_END,
////#define X3X3_EXIF_META    1 // write metadata, concurently opened files. All writes atomic
// control/setup devices
////#define X3X3_EXIF_TEMPLATE 2 // write Exif template
////#define X3X3_EXIF_METADIR  3 // write metadata to Exif header translation (dir_table[MAX_EXIF_FIELDS])
// those 2 files will disable exif_enable and exif_valid, truncate file size to file pointer on release.
////#define X3X3_EXIF_TIME     4 // write today/tomorrow date (YYYY:MM:DD) and number of seconds at today/tomorrow
                             // midnight (00:00:00) in seconds from epoch (long, startting from LSB)

//#define DEV393_EXIF_TEMPLATE  ("exif_template",     "exif_elphel",   125,  2, "0666", "c")  ///< write Exif template
//#define DEV393_EXIF_METADIR   ("exif_metadir",      "exif_elphel",   125,  3, "0666", "c")  ///< write metadata to Exif header translation (dir_table[MAX_EXIF_FIELDS])
//#define DEV393_EXIF_TIME      ("exif_time",         "exif_elphel",   125,  4, "0666", "c")  ///< write today/tomorrow date (YYYY:MM:DD) and number of seconds at today/tomorrow



// commands for the overloaded lseek:

//X3X3_EXIF_TIME
#define EXIF_LSEEK_DISABLE       1 // disable Exif (storing of frame meta, generating Exif)
#define EXIF_LSEEK_ENABLE        2 // enable Exif (build buffer if needed)
#define EXIF_LSEEK_INVALIDATE    3 // invalidate (and disable)
#define EXIF_LSEEK_REBUILD       4 // rebuild buffer

#define EXIF_LSEEK_TOMORROW_DATE 5 // file pointer to YYYY:MM:DD (tomorrow) string
#define EXIF_LSEEK_TOMORROW_SEC  6 // file pointer to unsigned long (little endian) tomorrow seconds from epoch
#define EXIF_LSEEK_TODAY_DATE    7 // file pointer to YYYY:MM:DD (today) string
#define EXIF_LSEEK_TODAY_SEC     8  // file pointer to unsigned long (little endian) today seconds from epoch


/*
Exif data in the images is combined from the "static" structure (template), calculated once at startup, and
variable data stored in the buffer for individual frames in the "Exif" form - converted to ASCII strings
or Rational or else. The generated Exif header copies that variable fileds on top of the Exif template.

The compressed data buffer is stored in "meta pages", one per frame 
*/
struct __attribute__((__packed__)) exif_dir_table_t {
        union {
          unsigned long ltag;// tag group and tag combined
          struct {
            unsigned short tag_group; // tag group: 0 - IFD0, 1 - Exif, 2 - GPS
            unsigned short tag;       // Exif tag as defined in the standard
          };
        };
        unsigned long len; // Number of bytes to be copied from metadata to Exif
        unsigned long src; // offset in meta data page
        unsigned long dst; // offset in output Exif page
};
#define MAX_EXIF_FIELDS  256 // number of Exif tags in the header
#define MAX_EXIF_SIZE   4096 // Exif data size 
//#define MAX_EXIF_FRAMES  512 // number of frames in the buffer
#define MAX_EXIF_FRAMES  2048 // number of frames in the buffer

//Exif Tags - unsigned long, combining actual Exif tags with tag groups (0 - IFD0, 1 - Exif, 2 - GPS)
#define  Exif_Image_ImageDescription   0x0010e
#define  Exif_Image_Make               0x0010f
#define  Exif_Image_Model              0x00110
#define  Exif_Image_Software           0x00131
#define  Exif_Image_DateTime           0x00132
#define  Exif_Image_Artist             0x0013b
#define  Exif_Image_CameraSerialNumber 0x0c62f
#define  Exif_Image_Orientation        0x00112
// used for frame number as defined in Exif specification
#define  Exif_Image_ImageNumber        0x09211
// used for sensor number
#define  Exif_Image_PageNumber         0x00129

#define  Exif_Image_ExifTag            0x08769
#define  Exif_Image_GPSTag             0x08825

//Sub IFD
#define  Exif_Photo_ExposureTime       0x1829a
#define  Exif_Photo_DateTimeOriginal   0x19003
#define  Exif_Photo_MakerNote          0x1927c
#define  Exif_Photo_SubSecTime         0x19290
#define  Exif_Photo_SubSecTimeOriginal 0x19291
//GPSInfo
#define  Exif_GPSInfo_GPSLatitudeRef   0x20001
#define  Exif_GPSInfo_GPSLatitude      0x20002
#define  Exif_GPSInfo_GPSLongitudeRef  0x20003
#define  Exif_GPSInfo_GPSLongitude     0x20004
#define  Exif_GPSInfo_GPSAltitudeRef   0x20005
#define  Exif_GPSInfo_GPSAltitude      0x20006
#define  Exif_GPSInfo_GPSTimeStamp     0x20007
#define  Exif_GPSInfo_GPSMeasureMode   0x2000a
#define  Exif_GPSInfo_GPSDateStamp     0x2001D

/// used for compass module
#define  Exif_GPSInfo_GPSImgDirectionRef  0x20010
#define  Exif_GPSInfo_GPSImgDirection     0x20011
#define  Exif_GPSInfo_GPSDestLatitudeRef  0x20013
#define  Exif_GPSInfo_GPSDestLatitude     0x20014
#define  Exif_GPSInfo_GPSDestLongitudeRef 0x20015
#define  Exif_GPSInfo_GPSDestLongitude    0x20016

#define  Exif_GPSInfo_CompassDirectionRef 0x20010
#define  Exif_GPSInfo_CompassDirection    0x20011
#define  Exif_GPSInfo_CompassPitchRef     0x20013
#define  Exif_GPSInfo_CompassPitch        0x20014
#define  Exif_GPSInfo_CompassRollRef      0x20015
#define  Exif_GPSInfo_CompassRoll         0x20016

//  array(0x9003,2,"2001:06:21 12:00:00","len"=> 20), //date/time original time created, always use 20 bytes (19 ."\0")
//  array(0x9291,2,"0        ") //original time sub-second length=10  9 ."\0"
///move back to interframe_params_t?
struct __attribute__((__packed__)) frame_exif_t {
          unsigned short meta_index;     //! index of the linked meta page
          unsigned short signffff;       //! should be 0xffff - it will be a signature that JPEG data was not overwritten
          unsigned long  frame_length;   //! frame length
};

struct __attribute__((__packed__)) meta_GPSInfo_t {
   unsigned char GPSLatitudeRef; //"N"/"S"
   unsigned long GPSLatitude_deg_nom;
   unsigned long GPSLatitude_deg_denom;
   unsigned long GPSLatitude_min_nom;
   unsigned long GPSLatitude_min_denom;
   unsigned char GPSLongitudeRef; //"E"/"W"
   unsigned long GPSLongitude_deg_nom;
   unsigned long GPSLongitude_deg_denom;
   unsigned long GPSLongitude_min_nom;
   unsigned long GPSLongitude_min_denom;
   unsigned char GPSAltitudeRef; //byte, not ascii 0 - above sea level, 1 - below
   unsigned long GPSAltitude_nom;   //in meters
   unsigned long GPSAltitude_denom;
   unsigned long GPSTimeStamp_hrs_nom;
   unsigned long GPSTimeStamp_hrs_denom;
   unsigned long GPSTimeStamp_min_nom;
   unsigned long GPSTimeStamp_min_denom;
   unsigned long GPSTimeStamp_sec_nom;
   unsigned long GPSTimeStamp_sec_denom;
   unsigned char GPSDateStamp[11]; //includes '\0'
   unsigned char GPSMeasureMode;
};
//hack - use
struct __attribute__((__packed__)) meta_CompassInfo_t {
//   unsigned char GPSImgDirectionRef; //"M"/"T" //0x10
   unsigned long CompassDirection_nom; //0x11
   unsigned long CompassDirection_denom;
   unsigned char CompassPitchRef; //"N"/"S"
   unsigned long CompassPitch_nom;
   unsigned long CompassPitch_denom;
   unsigned char CompassRollRef; //"E"/"W"
   unsigned long CompassRoll_nom;
   unsigned long CompassRoll_denom;
};


#define EXIF_GPS_MIN_DENOM     10000
#define EXIF_GPS_METERS_DENOM     10
#define EXIF_GPS_TIMESEC_DENOM  1000
#define EXIF_GPS_COMPASS_DENOM    10
///hack!
#define EXIF_COMPASS_PITCH_ASCII  "NS" // use for pitch +/-
#define EXIF_COMPASS_ROLL_ASCII   "EW" // use for roll +/-

/// Exif data (variable, stored with each frame) used for KML (not only)
#define  Exif_Image_ImageDescription_Index      0x00
#define  Exif_Photo_DateTimeOriginal_Index      0x01
#define  Exif_Photo_SubSecTimeOriginal_Index    0x02
#define  Exif_Photo_ExposureTime_Index          0x03
#define  Exif_GPSInfo_GPSLatitudeRef_Index      0x04
#define  Exif_GPSInfo_GPSLatitude_Index         0x05
#define  Exif_GPSInfo_GPSLongitudeRef_Index     0x06
#define  Exif_GPSInfo_GPSLongitude_Index        0x07
#define  Exif_GPSInfo_GPSAltitudeRef_Index      0x08
#define  Exif_GPSInfo_GPSAltitude_Index         0x09
#define  Exif_GPSInfo_GPSTimeStamp_Index        0x0a
#define  Exif_GPSInfo_GPSDateStamp_Index        0x0b
#define  Exif_GPSInfo_GPSMeasureMode_Index      0x0c
#define  Exif_GPSInfo_CompassDirectionRef_Index 0x0d
#define  Exif_GPSInfo_CompassDirection_Index    0x0e
#define  Exif_GPSInfo_CompassPitchRef_Index     0x0f
#define  Exif_GPSInfo_CompassPitch_Index        0x10
#define  Exif_GPSInfo_CompassRollRef_Index      0x11
#define  Exif_GPSInfo_CompassRoll_Index         0x12
#define  Exif_Image_ImageNumber_Index           0x13
#define  Exif_Image_Orientation_Index           0x14
#define  Exif_Image_PageNumber_Index            0x15
#define  Exif_Photo_MakerNote_Index             0x16
/// update ExifKmlNumber to be total number of *_Index entries
#define  ExifKmlNumber                          Exif_Photo_MakerNote_Index+1

//#define EXIF_DEV_NAME "/dev/exif_exif"
#define EXIFDIR_DEV_NAME "/dev/exif_metadir"
//#define EXIFMETA_DEV_NAME "/dev/exif_meta"

/**
 * @brief This macro is used to construct file names in user space applications. Example
 * of usage: <e>const char *exif_file_names[SENSOR_PORTS] = { EXIF_DEV_NAMES };</e>. Then the
 * sensor port number can be used to access file name.
 */
#define EXIF_DEV_NAMES		"/dev/exif_exif0", \
							"/dev/exif_exif1", \
							"/dev/exif_exif2", \
							"/dev/exif_exif3"
/**
 * @brief This macro is used to construct file names in user space applications. Example
 * of usage: <e>const char *exifmeta_file_names[SENSOR_PORTS] = { EXIFMETA_DEV_NAMES };</e>. Then
 * the sensor port number can be used to access file name.
 */
#define EXIFMETA_DEV_NAMES	"/dev/exif_meta0", \
							"/dev/exif_meta1", \
							"/dev/exif_meta2", \
							"/dev/exif_meta3"

#endif /* _ASM_EXIF_H */
