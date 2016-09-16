/***************************************************************************//**
* @file      debug393.h
* @brief     Macros for low-overhad logging messages to a large memory buffer
*            using klogger_393
* @copyright Copyright 2016 (C) Elphel, Inc.
* @par <b>License</b>
*  This program is free software: you can redistribute it and/or modify
*  it under the terms of the GNU General Public License as published by
*  the Free Software Foundation, either version 2 of the License, or
*  (at your option) any later version.
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
*  You should have received a copy of the GNU General Public License
*  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*******************************************************************************/


//  Used debug bits:
//  framepars.c
//  0
#define DBGB_FSFA   1 ///< setFrameParsAtomic (kthread)
#define DBGB_FASAP  2 ///< _processParsASAP   (kthread, tasklet)  (first N only)
#define DBGB_FSEQ   3 ///< _processParsSeq    (kthread, tasklet)  (first N only)
#define DBGB_FPPI   4 ///< _processPars       (kthread, tasklet)  (first N only)
#define DBGB_FPPT   5 ///<  processPars        (from tasklets)  (first N only)
#define DBGB_FSFP   6 ///<  setFramePar(), setFramePars() (tasklet)
#define DBGB_FSFV   7 ///<  setFramePar(), setFramePars() - adiitional(verbose) (tasklet)
#define DBGB_FSCF   8 ///<  schedule_pgm_func,schedule_this_pgm_func
#define DBGB_FFOP   9 ///<  file operations
// pgm_functions.c
#define DBGB_PSFN  10 ///< start of each function
#define DBGB_PADD  11 ///< additional details
// x393_vidoemem.c
#define DBGB_VM    12 ///<  vidoemem all debug
#define DBGB_SCRST 13 ///<  vidoemem all debug


// 13
// 14
// 15
// 16
// 17
// 18
// 19
// 20
// 21
// 22
// 23
// 24
// 25
// 26
// 27
// 28
// 29
// 30
// 31

#define DBGB_SFA 1 ///< setFrameParsAtomic (kthread)

#include "klogger_393.h"
#include "framepars.h" // for aglobals
#include "sensor_common.h"// for int getHardFrameNumber(int sensor_port, int use_compressor);

#ifndef ELPHEL_DEBUG393
    #define ELPHEL_DEBUG393 1
    #if ELPHEL_DEBUG393
        static int klog_mode = 0xff; ///< bits specify attributes to log
         /// log unconditionally, for any channel
        #define MDG(...)    print_klog393(klog_mode, __FILE__, __FUNCTION__, __LINE__, __VA_ARGS__);
         /// log only if specified bit in G_DEBUG global parameter for the specified sensor port is set
//        #define MDP(bit,port,fmt,...) { if (GLOBALPARS(port,G_DEBUG) & (1 << bit)) print_klog393(klog_mode, __FILE__, __FUNCTION__, __LINE__,"%d: "fmt,port,__VA_ARGS__); }
        #define MDP(bit,port,fmt,...) { if (GLOBALPARS(port,G_DEBUG) & (1 << bit)) print_klog393(klog_mode, __FILE__, __FUNCTION__, __LINE__,"%d:%d "fmt,port,getHardFrameNumber(port, 0),__VA_ARGS__); }
    #else
        #define MDF(x)
        #define MDP(bit,port,fmt,...)
    #endif
#endif
