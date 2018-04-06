/***************************************************************************//**
* @file      mt9f002.h
* @brief     Handles Micron/Aptina/On Semiconductor MT9F002
* @copyright Copyright 2018 (C) Elphel, Inc.
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
#ifndef _MT9F002_H
#define _MT9F002_H

#define MT9F002_PARTID     0x2E01 ///< MT9F002 PartID register value
#define MT9F002_I2C_ADDR     0x10 ///< MT9P I2C slave address (7 bit)

// bit 9 should have set masking for broken frames
// cleared bit 3 allows writing to some RO registers
//#define MT9F002_RESET_REGISTER_VALUE 0x001c
#define MT9F002_RESET_REGISTER_VALUE 0x0011c
//#define MT9F002_RESET_REGISTER_VALUE 0x0014
// number of lines to sacrifice before generating Frame Valid
#define MT9F002_VACT_DELAY 2

// !!! Unpredicted operation:
// * Value from the datasheet works while image width is above border
//   suspecting sensor's ADCs performance
// * if width < BORDER_WIDTH row_time_in_pixels become 1:1
//   else                    row_time_in_pixels 1:2
#define MIN_LINE_LENGTH_PCK_FROM_DATASHEET 0x4c8
#define X_OUTPUT_BORDER_SIZE               912

// Clocks, this is basis

// from x393_parameters.vh:
// 220MHz -> 22MHz - the real clock is 22MHz - that's why multiplier is 0xb4
// The 3 parameters below are needed to calculate sensor clock
// line 568
#define PXD_CLK_DIV 10
// line 940
#define CLKFBOUT_MULT_PCLK    36, // 880 MHz
// line 941
#define CLKOUT_DIV_PCLK        4, // 220 MHz

// note: there is also DIVCLK_DIVIDE_PCLK

#define MT9F002_IFACE_CLK 24444000*PXD_CLK_DIV/CLKFBOUT_MULT_PCLK/CLKOUT_DIV_PCLK

// External sensor clock before pll

// This is not a constant = 24.444MHz in Hz

#define MT9F002_EXT_CLK MT9F002_IFACE_CLK
// Virtual pixel clock is used as the basis for frame timing equations.
// Constant = 220MHz in Hz
#define MT9F002_VT_PIX_CLK 220000000
// OP_PIX_CLK
#define MT9F002_OP_PIX_CLK 110000000
// Serial output clock
// Constant = 660MHz in Hz
#define MT9F002_OP_SYS_CLK 660000000

// Sensor clock dividers and multiplier
// These should be calculated based on the clocks above

// pll multiplier, default is 0xa5 (165) for 24MHz
// is set to 0xb4 (180) becuase clock is 22MHz (not 24)
#define MT9F002_PLL_MULTIPLIER_VALUE 0xb4
// pre_pll_clk_div (0x0304), default value is 0x6
#define MT9F002_PRE_PLL_CLK_DIV_VALUE 0x6
// vt_pix_clk_div (0x0300), default value is 0x6
#define MT9F002_VT_PIX_CLK_DIV_VALUE 0x6
// vt_sys_clk_div (0x0300), default value is 0x6
#define MT9F002_VT_SYS_CLK_DIV_VALUE 0x1
// shift_vt_pix_clk_div, default value is 0x1
#define MT9F002_SHIFT_VT_PIX_CLK_DIV 0x1
// op_pix_clk divider
#define MT9F002_OP_PIX_CLK_DIV 0xc

// Coarse Integration Time Margin
#define MT9F002_COARSE_EXPOS_MARGIN 0x1

/* ON Semi MT9F002 i2c register addresses */

/*

## FOR INIT, one time?
0x31c0 8db6      # HiSPI timing
0x31c0 8492      # HiSPI timing
0x31c0 8fff      # HiSPI timing
0x31c0 8db6      # HiSPI timing
0x0306 00b4    # SMIA_PLL_MULTIPLIER
0x31c6 8400    # HiSPI control status
0x306e 9280    # DataPath select
0x301a 001c    # RESET register

## FOR WORK,
0x3028 000a    # Analog gain code global
0x302c 000d    # Analog gain code red
0x302e 0010    # Analog gain code blue
0x3012 0060    # Coarse integration time



0x3002 - (0x0020) The first row of visible pixels to be read out (not counting any dark rows that may be read). To move the image
         window, set this register to the starting Y value.
0x3004 - (0x0090) The first column of visible pixels to be read out (not counting any dark columns that may be read). To move the
         image window, set this register to the starting X value.
0x3006 - (0x0CF7) The last row of visible pixels to be read out.
0x3008 - (0x11AF) The last column of visible pixels to be read out.
0x300A - (0x0D6A) The number of complete lines (rows) in the output frame. This includes visible lines and vertical blanking lines.
0x300C - (0x2350) The number of pixel clock periods in one line (row) time. This includes visible pixels and horizontal blanking time.
0x3010 - (0x0128) Fine integration time correction factor. This is an offset that is applied to the programmed value of
                  fine_integration_time such that the actual integration time matches the integration time equation.
                  This register should not be modified under normal operation, but must be modified when binning is enabled or the
                  internal pixel clock divider (pc_speed[2:0]) is used.
0x3012 - (0x0010) Integration time specified in multiples of line_length_pck_.
0x3014 - (0x0524) Integration time specified as a number of pixel clocks.
0x3016 - (0x0111) row speed
0x3018 - extra delay (blanking to get more exact frame rate)
0x301A - reset register
0x301E - Constant offset that is added to the ADC output for all visible pixels in order to set the black level to than 0.
         Read-only. Can be made read/write by clearing R0x301A-B[3].
0x3024 - (0x0) Pixel order: 00 - GRBG, 01 - RGGB, 02 - BGGR, 03 - GBRG. Read only, changes a function of R0x3040[1:0]
0x3028 - (0x000A) gain code global
0x302A - analog gain Gr
0x302C - analog gain R
0x302E - analog gain B
0x3030 - analog gain Gb
0x3032 - digital gain Gr
0x3034 - digital gain R
0x3036 - digital gain B
0x3038 - digital gain Gb
0x303A - SMIA version, 0xA = 1.0
0x303B - frame count
0x3040 - (0x0041) read_mode
0x3046 - flash?!
0x3056 - green1_gain
0x3058 - blue_gain
0x305A - red gain
0x305C - green2 gain
0x305E - global gain
0x306A - datapath status
0x306E - datapath select

0x3070 xxxx    # test pattern mode
       0002    - stripes
       0000    - off

0x3072 - The value for red pixels in the Bayer data used for the solid color test pattern and the test cursors.
0x3074 - The value for green pixels in red/green rows of the Bayer data used for the solid color test pattern and the test cursors.
0x3076 - The value for blue pixels in the Bayer data used for the solid color test pattern and the test cursors.
0x3078 - The value for green pixels in blue/green rows of the Bayer data used for the solid color test pattern and the test
         cursors.

0x307A - test_raw_mode

0x31c0 - for phases: 4 data lanes, 1 clock lane



*/

// #define P_MT9F002_NAME                             ADDR  ///< Bits, Default value, RW, Frame Sync'd, Bad Frame
                                                          ///< Description
/* SMIA Configuration Register List */
#define P_REG_MT9F002_SMIA_MODEL_ID                     0x0000  ///< [15:0],0x2E01,RW,N,N,
                                                            ///< This register is an alias of R0x3000-1. Read-only. Can be made read/write by clearing R0x301A-B[3].
#define P_REG_MT9F002_SMIA_REVISION_NUMBER              0x0002  ///< [ 7:0],  0x00,RW,N,N,
                                                            ///< Aptina-assigned revision number. Read-only. Can be made read/write by clearing R0x301A-B[3].
#define P_REG_MT9F002_SMIA_MANUFACTURER_ID              0x0003  ///< [ 7:0],  0x06,RO,N,N
                                                            ///< Manufacturer ID assigned to Aptina. Read-only. Can be made read/write by clearing R0x301A-B[3].
#define P_REG_MT9F002_SMIA_SMIA_VERSION                 0x0004  ///< [ 7:0],  0x0A,RO,N,N
                                                            ///< This register is an alias of R0x303A. Read-only.
#define P_REG_MT9F002_SMIA_FRAME_COUNT                  0x0005  ///< [ 7:0],  0xFF,RO,Y,N
                                                            ///< This register is an alias of R0x303B. Read-only.
#define P_REG_MT9F002_SMIA_PIXEL_ORDER                  0x0006  ///< [ 7:0],  0x00,RO,N,N
                                                            ///< This register is an alias of R0x3024. Read-only.
#define P_REG_MT9F002_SMIA_DATA_PEDESTAL                0x0008  ///< [15:0],0x00A8,RW,N,Y
                                                            ///< This register is an alias of R0x301E-F. Read-only. Can be made read/write by clearing R0x301A-B[3].
#define P_REG_MT9F002_SMIA_FRAME_FORMAT_MODEL_TYPE      0x0040  ///< [ 7:0],  0x01,RO,N,N
                                                            ///< Type 1. 2-byte Generic Frame Format Description. Read-only.
#define P_REG_MT9F002_SMIA_FRAME_FORMAT_MODEL_SUBTYPE   0x0041

#define P_REG_MT9F002_SMIA_FRAME_FORMAT_DESCRIPTOR_0    0x0042
#define P_REG_MT9F002_SMIA_FRAME_FORMAT_DESCRIPTOR_1    0x0044
#define P_REG_MT9F002_SMIA_FRAME_FORMAT_DESCRIPTOR_2    0x0046
#define P_REG_MT9F002_SMIA_FRAME_FORMAT_DESCRIPTOR_3    0x0048
#define P_REG_MT9F002_SMIA_FRAME_FORMAT_DESCRIPTOR_4    0x004A
#define P_REG_MT9F002_SMIA_FRAME_FORMAT_DESCRIPTOR_5    0x004C
#define P_REG_MT9F002_SMIA_FRAME_FORMAT_DESCRIPTOR_6    0x004E
#define P_REG_MT9F002_SMIA_FRAME_FORMAT_DESCRIPTOR_7    0x0050
#define P_REG_MT9F002_SMIA_FRAME_FORMAT_DESCRIPTOR_8    0x0052
#define P_REG_MT9F002_SMIA_FRAME_FORMAT_DESCRIPTOR_9    0x0054
#define P_REG_MT9F002_SMIA_FRAME_FORMAT_DESCRIPTOR_10   0x0056
#define P_REG_MT9F002_SMIA_FRAME_FORMAT_DESCRIPTOR_11   0x0058
#define P_REG_MT9F002_SMIA_FRAME_FORMAT_DESCRIPTOR_12   0x005A
#define P_REG_MT9F002_SMIA_FRAME_FORMAT_DESCRIPTOR_13   0x005C
#define P_REG_MT9F002_SMIA_FRAME_FORMAT_DESCRIPTOR_14   0x005E

#define P_REG_MT9F002_SMIA_ANALOG_GAIN_CAPABILITY       0x0080
#define P_REG_MT9F002_SMIA_ANALOG_GAIN_CODE_MIN         0x0084
#define P_REG_MT9F002_SMIA_ANALOG_GAIN_CODE_MAX         0x0086
#define P_REG_MT9F002_SMIA_ANALOG_GAIN_CODE_STEP        0x0088
#define P_REG_MT9F002_SMIA_ANALOG_GAIN_TYPE             0x008A
#define P_REG_MT9F002_SMIA_ANALOG_GAIN_M0               0x008C
#define P_REG_MT9F002_SMIA_ANALOG_GAIN_C0               0x008E
#define P_REG_MT9F002_SMIA_ANALOG_GAIN_M1               0x0090
#define P_REG_MT9F002_SMIA_ANALOG_GAIN_C1               0x0092

#define P_REG_MT9F002_SMIA_DATA_FORMAT_MODEL_TYPE       0x00C0
#define P_REG_MT9F002_SMIA_DATA_FORMAT_MODEL_SUBTYPE    0x00C1
#define P_REG_MT9F002_SMIA_DATA_FORMAT_DESCRIPTOR_0     0x00C2
#define P_REG_MT9F002_SMIA_DATA_FORMAT_DESCRIPTOR_1     0x00C4
#define P_REG_MT9F002_SMIA_DATA_FORMAT_DESCRIPTOR_2     0x00C6
#define P_REG_MT9F002_SMIA_DATA_FORMAT_DESCRIPTOR_3     0x00C8
#define P_REG_MT9F002_SMIA_DATA_FORMAT_DESCRIPTOR_4     0x00CA
#define P_REG_MT9F002_SMIA_DATA_FORMAT_DESCRIPTOR_5     0x00CC
#define P_REG_MT9F002_SMIA_DATA_FORMAT_DESCRIPTOR_6     0x00CE


#define P_REG_MT9F002_SMIA_MODE_SELECT                  0x0100
#define P_REG_MT9F002_SMIA_IMAGE_ORIENTATION            0x0101
#define P_REG_MT9F002_SMIA_SOFTWARE_RESET               0x0103
#define P_REG_MT9F002_SMIA_GROUPED_PARAMETER_HOLD       0x0104
#define P_REG_MT9F002_SMIA_MASK_CORRUPTED_FRAMES        0x0105

#define P_REG_MT9F002_SMIA_CCP2_CHANNEL_IDENTIFIER      0x0110
#define P_REG_MT9F002_SMIA_CCP2_SIGNALLING_MODE         0x0111
#define P_REG_MT9F002_SMIA_CCP_DATA_FORMAT              0x0112
#define P_REG_MT9F002_SMIA_GAIN_MODE                    0x0120


#define P_REG_MT9F002_SMIA_FINE_INTEGRATION_TIME        0x0200
#define P_REG_MT9F002_SMIA_COARSE_INTEGRATION_TIME      0x0202
#define P_REG_MT9F002_SMIA_ANALOG_GAIN_CODE_GLOBAL      0x0204
#define P_REG_MT9F002_SMIA_ANALOG_GAIN_CODE_GREENR      0x0206
#define P_REG_MT9F002_SMIA_ANALOG_GAIN_CODE_RED         0x0208
#define P_REG_MT9F002_SMIA_ANALOG_GAIN_CODE_BLUE        0x020A
#define P_REG_MT9F002_SMIA_ANALOG_GAIN_CODE_GREENB      0x020C
#define P_REG_MT9F002_SMIA_DIGITAL_GAIN_GREENR          0x020E
#define P_REG_MT9F002_SMIA_DIGITAL_GAIN_RED             0x0210
#define P_REG_MT9F002_SMIA_DIGITAL_GAIN_BLUE            0x0212
#define P_REG_MT9F002_SMIA_DIGITAL_GAIN_GREENB          0x0214


#define P_REG_MT9F002_SMIA_VT_PIX_CLK_DIV               0x0300
#define P_REG_MT9F002_SMIA_VT_SYS_CLK_DIV               0x0302
#define P_REG_MT9F002_SMIA_PRE_PLL_CLK_DIV              0x0304
#define P_REG_MT9F002_SMIA_PLL_MULTIPLIER               0x0306
#define P_REG_MT9F002_SMIA_OP_PIX_CLK_DIV               0x0308
#define P_REG_MT9F002_SMIA_OP_SYS_CLK_DIV               0x030A

#define P_REG_MT9F002_SMIA_FRAME_LENGTH_LINES           0x0340
#define P_REG_MT9F002_SMIA_LINE_LENGTH_PCK              0x0342
#define P_REG_MT9F002_SMIA_X_ADDR_START                 0x0344
#define P_REG_MT9F002_SMIA_Y_ADDR_START                 0x0346
#define P_REG_MT9F002_SMIA_X_ADDR_END                   0x0348
#define P_REG_MT9F002_SMIA_Y_ADDR_END                   0x034A
#define P_REG_MT9F002_SMIA_X_OUTPUT_SIZE                0x034C
#define P_REG_MT9F002_SMIA_Y_OUTPUT_SIZE                0x034E

#define P_REG_MT9F002_SMIA_X_EVEN_INC                   0x0380
#define P_REG_MT9F002_SMIA_X_ODD_INC                    0x0382
#define P_REG_MT9F002_SMIA_Y_EVEN_INC                   0x0384
#define P_REG_MT9F002_SMIA_Y_ODD_INC                    0x0386


#define P_REG_MT9F002_SMIA_SCALING_MODE                 0x0400
#define P_REG_MT9F002_SMIA_SPATIAL_SAMPLING             0x0402
#define P_REG_MT9F002_SMIA_SCALE_M                      0x0404
#define P_REG_MT9F002_SMIA_SCALE_N                      0x0406


#define P_REG_MT9F002_SMIA_COMPRESSION_MODE             0x0500

#define P_REG_MT9F002_SMIA_TEST_PATTERN_MODE            0x0600
#define P_REG_MT9F002_SMIA_TEST_DATA_RED                0x0602
#define P_REG_MT9F002_SMIA_TEST_DATA_GREENR             0x0604
#define P_REG_MT9F002_SMIA_TEST_DATA_BLUE               0x0606
#define P_REG_MT9F002_SMIA_TEST_DATA_GREENB             0x0608
#define P_REG_MT9F002_SMIA_HORIZONTAL_CURSOR_WIDTH      0x060A
#define P_REG_MT9F002_SMIA_HORIZONTAL_CURSOR_POSITION   0x060C
#define P_REG_MT9F002_SMIA_VERTICAL_CURSOR_WIDTH        0x060E
#define P_REG_MT9F002_SMIA_VERTICAL_CURSOR_POSITION     0x0610

/* SMIA Parameter Limit Register List */
#define P_REG_MT9F002_SMIA_INTEGRATION_TIME_CAPABILITY        0x1000
#define P_REG_MT9F002_SMIA_COARSE_INTEGRATION_TIME_MIN        0x1004
#define P_REG_MT9F002_SMIA_COARSE_INTEGRATION_TIME_MAX_MARGIN 0x1006
#define P_REG_MT9F002_SMIA_FINE_INTEGRATION_TIME_MIN          0x1008
#define P_REG_MT9F002_SMIA_FINE_INTEGRATION_TIME_MAX_MARGIN   0x100A

#define P_REG_MT9F002_SMIA_DIGITAL_GAIN_CAPABILITY            0x1080
#define P_REG_MT9F002_SMIA_DIGITAL_GAIN_MIN                   0x1084
#define P_REG_MT9F002_SMIA_DIGITAL_GAIN_MAX                   0x1086
#define P_REG_MT9F002_SMIA_DIGITAL_GAIN_STEP_SIZE             0x1088

#define P_REG_MT9F002_SMIA_MIN_EXT_CLK_FREQ_MHZ               0x1100
#define P_REG_MT9F002_SMIA_MAX_EXT_CLK_FREQ_MHZ               0x1104
#define P_REG_MT9F002_SMIA_MIN_PRE_PLL_CLK_DIV                0x1108
#define P_REG_MT9F002_SMIA_MAX_PRE_PLL_CLK_DIV                0x110A
#define P_REG_MT9F002_SMIA_MIN_PLL_IP_FREQ_MHZ                0x110C
#define P_REG_MT9F002_SMIA_MAX_PLL_IP_FREQ_MHZ                0x1110
#define P_REG_MT9F002_SMIA_MIN_PLL_MULTIPLIER                 0x1114
#define P_REG_MT9F002_SMIA_MAX_PLL_MULTIPLIER                 0x1116
#define P_REG_MT9F002_SMIA_MIN_PLL_OP_FREQ_MHZ                0x1118
#define P_REG_MT9F002_SMIA_MAX_PLL_OP_FREQ_MHZ                0x111C
#define P_REG_MT9F002_SMIA_MIN_VT_SYS_CLK_DIV                 0x1120
#define P_REG_MT9F002_SMIA_MAX_VT_SYS_CLK_DIV                 0x1122
#define P_REG_MT9F002_SMIA_MIN_VT_SYS_CLK_FREQ_MHZ            0x1124
#define P_REG_MT9F002_SMIA_MAX_VT_SYS_CLK_FREQ_MHZ            0x1128
#define P_REG_MT9F002_SMIA_MIN_VT_PIX_CLK_FREQ_MHZ            0x112C
#define P_REG_MT9F002_SMIA_MAX_VT_PIX_CLK_FREQ_MHZ            0x1130
#define P_REG_MT9F002_SMIA_MIN_VT_PIX_CLK_DIV                 0x1134
#define P_REG_MT9F002_SMIA_MAX_VT_PIX_CLK_DIV                 0x1136
#define P_REG_MT9F002_SMIA_MIN_FRAME_LENGTH_LINES             0x1140
#define P_REG_MT9F002_SMIA_MAX_FRAME_LENGTH_LINES             0x1142
#define P_REG_MT9F002_SMIA_MIN_LINE_LENGTH_PCK                0x1144
#define P_REG_MT9F002_SMIA_MAX_LINE_LENGTH_PCK                0x1146
#define P_REG_MT9F002_SMIA_MIN_LINE_BLANKING_PCK              0x1148
#define P_REG_MT9F002_SMIA_MIN_FRAME_BLANKING_LINES           0x114A
#define P_REG_MT9F002_SMIA_MIN_OP_SYS_CLK_DIV                 0x1160
#define P_REG_MT9F002_SMIA_MAX_OP_SYS_CLK_DIV                 0x1162
#define P_REG_MT9F002_SMIA_MIN_OP_SYS_CLK_FREQ_MHZ            0x1164
#define P_REG_MT9F002_SMIA_MAX_OP_SYS_CLK_FREQ_MHZ            0x1168
#define P_REG_MT9F002_SMIA_MIN_OP_PIX_CLK_DIV                 0x116C
#define P_REG_MT9F002_SMIA_MAX_OP_PIX_CLK_DIV                 0x116E
#define P_REG_MT9F002_SMIA_MIN_OP_PIX_CLK_FREQ_MHZ            0x1170
#define P_REG_MT9F002_SMIA_MAX_OP_PIX_CLK_FREQ_MHZ            0x1174

#define P_REG_MT9F002_SMIA_X_ADDR_MIN                         0x1180
#define P_REG_MT9F002_SMIA_Y_ADDR_MIN                         0x1182
#define P_REG_MT9F002_SMIA_X_ADDR_MAX                         0x1184
#define P_REG_MT9F002_SMIA_Y_ADDR_MAX                         0x1186

#define P_REG_MT9F002_SMIA_MIN_EVEN_INC                       0x11C0
#define P_REG_MT9F002_SMIA_MAX_EVEN_INC                       0x11C2
#define P_REG_MT9F002_SMIA_MIN_ODD_INC                        0x11C4
#define P_REG_MT9F002_SMIA_MAX_ODD_INC                        0x11C6


#define P_REG_MT9F002_SMIA_SCALING_CAPABILITY                 0x1200
#define P_REG_MT9F002_SMIA_SCALER_M_MIN                       0x1204
#define P_REG_MT9F002_SMIA_SCALER_M_MAX                       0x1206
#define P_REG_MT9F002_SMIA_SCALER_N_MIN                       0x1208
#define P_REG_MT9F002_SMIA_SCALER_N_MAX                       0x120A

#define P_REG_MT9F002_SMIA_COMPRESSION_CAPABILITY             0x1300

#define P_REG_MT9F002_SMIA_MATRIX_ELEMENT_REDINRED            0x1400
#define P_REG_MT9F002_SMIA_MATRIX_ELEMENT_GREENINRED          0x1402
#define P_REG_MT9F002_SMIA_MATRIX_ELEMENT_BLUEINRED           0x1404
#define P_REG_MT9F002_SMIA_MATRIX_ELEMENT_REDINGREEN          0x1406
#define P_REG_MT9F002_SMIA_MATRIX_ELEMENT_GREENINGREEN        0x1408
#define P_REG_MT9F002_SMIA_MATRIX_ELEMENT_BLUEINGREEN         0x140A
#define P_REG_MT9F002_SMIA_MATRIX_ELEMENT_REDINBLUE           0x140C
#define P_REG_MT9F002_SMIA_MATRIX_ELEMENT_GREENINBLUE         0x140E
#define P_REG_MT9F002_SMIA_MATRIX_ELEMENT_BLUEINBLUE          0x1410

/* Manufacturer Specific Register List */
#define P_REG_MT9F002_MODEL_ID                      0x3000
#define P_REG_MT9F002_Y_ADDR_START                  0x3002
#define P_REG_MT9F002_X_ADDR_START                  0x3004
#define P_REG_MT9F002_Y_ADDR_END                    0x3006
#define P_REG_MT9F002_X_ADDR_END                    0x3008
#define P_REG_MT9F002_FRAME_LENGTH_LINES            0x300A
#define P_REG_MT9F002_LINE_LENGTH_PCK               0x300C
#define P_REG_MT9F002_FINE_CORRECTION               0x3010
#define P_REG_MT9F002_COARSE_INTEGRATION_TIME       0x3012
#define P_REG_MT9F002_FINE_INTEGRATION_TIME         0x3014
#define P_REG_MT9F002_ROW_SPEED                     0x3016
#define P_REG_MT9F002_EXTRA_DELAY                   0x3018
#define P_REG_MT9F002_RESET_REGISTER                0x301A
#define P_REG_MT9F002_MODE_SELECT                   0x301C
#define P_REG_MT9F002_IMAGE_ORIENTATION             0x301D
#define P_REG_MT9F002_DATA_PEDESTAL                 0x301E
#define P_REG_MT9F002_SOFTWARE_RESET                0x3021
#define P_REG_MT9F002_GROUPED_PARAMETER_HOLD        0x3022
#define P_REG_MT9F002_MASK_CORRUPTED_FRAMES         0x3023
#define P_REG_MT9F002_PIXEL_ORDER                   0x3024
#define P_REG_MT9F002_GPI_STATUS                    0x3026
#define P_REG_MT9F002_ANALOG_GAIN_CODE_GLOBAL       0x3028
#define P_REG_MT9F002_ANALOG_GAIN_CODE_GREENR       0x302A
#define P_REG_MT9F002_ANALOG_GAIN_CODE_RED          0x302C
#define P_REG_MT9F002_ANALOG_GAIN_CODE_BLUE         0x302E
#define P_REG_MT9F002_ANALOG_GAIN_CODE_GREENB       0x3030
#define P_REG_MT9F002_DIGITAL_GAIN_GREENR           0x3032
#define P_REG_MT9F002_DIGITAL_GAIN_RED              0x3034
#define P_REG_MT9F002_DIGITAL_GAIN_BLUE             0x3036
#define P_REG_MT9F002_DIGITAL_GAIN_GREENB           0x3038

#define P_REG_MT9F002_SMIA_VERSION                  0x303A
#define P_REG_MT9F002_FRAME_COUNT                   0x303B
#define P_REG_MT9F002_FRAME_STATUS                  0x303C
#define P_REG_MT9F002_READ_MODE                     0x3040
#define P_REG_MT9F002_FLASH                         0x3046
#define P_REG_MT9F002_FLASH_COUNT                   0x3048
#define P_REG_MT9F002_GREEN1_GAIN                   0x3056
#define P_REG_MT9F002_BLUE_GAIN                     0x3058
#define P_REG_MT9F002_RED_GAIN                      0x305A
#define P_REG_MT9F002_GREEN2_GAIN                   0x305C
#define P_REG_MT9F002_GLOBAL_GAIN                   0x305E

#define P_REG_MT9F002_DATAPATH_STATUS               0x306A
#define P_REG_MT9F002_DATAPATH_SELECT               0x306E

#define P_REG_MT9F002_TEST_PATTERN_MODE             0x3070
#define P_REG_MT9F002_TEST_DATA_RED                 0x3072
#define P_REG_MT9F002_TEST_DATA_GREENR              0x3074
#define P_REG_MT9F002_TEST_DATA_BLUE                0x3076
#define P_REG_MT9F002_TEST_DATA_GREENB              0x3078
#define P_REG_MT9F002_TEST_RAW_MODE                 0x307A

#define P_REG_MT9F002_X_EVEN_INC                    0x30A0
#define P_REG_MT9F002_X_ODD_INC                     0x30A2
#define P_REG_MT9F002_Y_EVEN_INC                    0x30A4
#define P_REG_MT9F002_Y_ODD_INC                     0x30A6
#define P_REG_MT9F002_CALIB_GREEN1_ASC1             0x30A8
#define P_REG_MT9F002_CALIB_BLUE_ASC1               0x30AA
#define P_REG_MT9F002_CALIB_RED_ASC1                0x30AC
#define P_REG_MT9F002_CALIB_GREEN2_ASC1             0x30AE
#define P_REG_MT9F002_CALIB_GLOBAL                  0x30BC

#define P_REG_MT9F002_CALIB_CONTROL                 0x30C0
#define P_REG_MT9F002_CALIB_GREEN1                  0x30C2
#define P_REG_MT9F002_CALIB_BLUE                    0x30C4
#define P_REG_MT9F002_CALIB_RED                     0x30C6
#define P_REG_MT9F002_CALIB_GREEN2                  0x30C8
#define P_REG_MT9F002_CTX_CONTROL_REG               0x30E8
#define P_REG_MT9F002_CTX_WR_DATA_REG               0x30EA
#define P_REG_MT9F002_CTX_RD_DATA_REG               0x30EC
#define P_REG_MT9F002_DARK_CONTROL3                 0x30EE

#define P_REG_MT9F002_OTPM_TCFG_READ_4B             0x3138
#define P_REG_MT9F002_OTPM_CFG                      0x3140
#define P_REG_MT9F002_GLOBAL_FLASH_START            0x315A
#define P_REG_MT9F002_GLOBAL_SEQ_TRIGGER            0x315E
#define P_REG_MT9F002_GLOBAL_RST_END                0x3160
#define P_REG_MT9F002_GLOBAL_SHUTTER_START          0x3162
#define P_REG_MT9F002_GLOBAL_SHUTTER_START2         0x3164
#define P_REG_MT9F002_GLOBAL_READ_START             0x3166
#define P_REG_MT9F002_GLOBAL_READ_START2            0x3168
#define P_REG_MT9F002_DAC_RSTLO                     0x316A
#define P_REG_MT9F002_ANALOG_CONTROL5               0x3178
#define P_REG_MT9F002_SERIAL_FORMAT_DESCRIPTOR_0    0x31A0
#define P_REG_MT9F002_SERIAL_FORMAT_DESCRIPTOR_1    0x31A2
#define P_REG_MT9F002_SERIAL_FORMAT_DESCRIPTOR_2    0x31A4
#define P_REG_MT9F002_SERIAL_FORMAT_DESCRIPTOR_3    0x31A6
#define P_REG_MT9F002_SERIAL_FORMAT_DESCRIPTOR_4    0x31A8
#define P_REG_MT9F002_SERIAL_FORMAT_DESCRIPTOR_5    0x31AA
#define P_REG_MT9F002_SERIAL_FORMAT_DESCRIPTOR_6    0x31AC
#define P_REG_MT9F002_SERIAL_FORMAT                 0x31AE
#define P_REG_MT9F002_FRAME_PREAMBLE                0x31B0
#define P_REG_MT9F002_LINE_PREAMBLE                 0x31B2
#define P_REG_MT9F002_MIPI_TIMING_0                 0x31B4
#define P_REG_MT9F002_MIPI_TIMING_1                 0x31B6
#define P_REG_MT9F002_MIPI_TIMING_2                 0x31B8
#define P_REG_MT9F002_MIPI_TIMING_3                 0x31BA
#define P_REG_MT9F002_MIPI_TIMING_4                 0x31BC
#define P_REG_MT9F002_HISPI_TIMING                  0x31C0
#define P_REG_MT9F002_HISPI_CONTROL_STATUS          0x31C6

#define P_REG_MT9F002_HORIZONTAL_CURSOR_POSITION    0x31E8
#define P_REG_MT9F002_VERTICAL_CURSOR_POSITION      0x31EA
#define P_REG_MT9F002_HORIZONTAL_CURSOR_WIDTH       0x31EC
#define P_REG_MT9F002_VERTICAL_CURSOR_WIDTH         0x31EE

#define P_REG_MT9F002_I2C_IDS_MIPI_DEFAULT          0x31F2
#define P_REG_MT9F002_I2C_IDS                       0x31FC

#define P_REG_MT9F002_P_GR_P0Q0                     0x3600
#define P_REG_MT9F002_P_GR_P0Q1                     0x3602
#define P_REG_MT9F002_P_GR_P0Q2                     0x3604
#define P_REG_MT9F002_P_GR_P0Q3                     0x3606
#define P_REG_MT9F002_P_GR_P0Q4                     0x3608
#define P_REG_MT9F002_P_RD_P0Q0                     0x360A
#define P_REG_MT9F002_P_RD_P0Q1                     0x360C
#define P_REG_MT9F002_P_RD_P0Q2                     0x360E
#define P_REG_MT9F002_P_RD_P0Q3                     0x3610
#define P_REG_MT9F002_P_RD_P0Q4                     0x3612
#define P_REG_MT9F002_P_BL_P0Q0                     0x3614
#define P_REG_MT9F002_P_BL_P0Q1                     0x3616
#define P_REG_MT9F002_P_BL_P0Q2                     0x3618
#define P_REG_MT9F002_P_BL_P0Q3                     0x361A
#define P_REG_MT9F002_P_BL_P0Q4                     0x361C
#define P_REG_MT9F002_P_GB_P0Q0                     0x361E
#define P_REG_MT9F002_P_GB_P0Q1                     0x3620
#define P_REG_MT9F002_P_GB_P0Q2                     0x3622
#define P_REG_MT9F002_P_GB_P0Q3                     0x3624
#define P_REG_MT9F002_P_GB_P0Q4                     0x3626

#define P_REG_MT9F002_P_GR_P1Q0                     0x3640
#define P_REG_MT9F002_P_GR_P1Q1                     0x3642
#define P_REG_MT9F002_P_GR_P1Q2                     0x3644
#define P_REG_MT9F002_P_GR_P1Q3                     0x3646
#define P_REG_MT9F002_P_GR_P1Q4                     0x3648
#define P_REG_MT9F002_P_RD_P1Q0                     0x364A
#define P_REG_MT9F002_P_RD_P1Q1                     0x364C
#define P_REG_MT9F002_P_RD_P1Q2                     0x364E
#define P_REG_MT9F002_P_RD_P1Q3                     0x3650
#define P_REG_MT9F002_P_RD_P1Q4                     0x3652
#define P_REG_MT9F002_P_BL_P1Q0                     0x3654
#define P_REG_MT9F002_P_BL_P1Q1                     0x3656
#define P_REG_MT9F002_P_BL_P1Q2                     0x3658
#define P_REG_MT9F002_P_BL_P1Q3                     0x365A
#define P_REG_MT9F002_P_BL_P1Q4                     0x365C
#define P_REG_MT9F002_P_GB_P1Q0                     0x365E
#define P_REG_MT9F002_P_GB_P1Q1                     0x3660
#define P_REG_MT9F002_P_GB_P1Q2                     0x3662
#define P_REG_MT9F002_P_GB_P1Q3                     0x3664
#define P_REG_MT9F002_P_GB_P1Q4                     0x3666

#define P_REG_MT9F002_P_GR_P2Q0                     0x3680
#define P_REG_MT9F002_P_GR_P2Q1                     0x3682
#define P_REG_MT9F002_P_GR_P2Q2                     0x3684
#define P_REG_MT9F002_P_GR_P2Q3                     0x3686
#define P_REG_MT9F002_P_GR_P2Q4                     0x3688
#define P_REG_MT9F002_P_RD_P2Q0                     0x368A
#define P_REG_MT9F002_P_RD_P2Q1                     0x368C
#define P_REG_MT9F002_P_RD_P2Q2                     0x368E
#define P_REG_MT9F002_P_RD_P2Q3                     0x3690
#define P_REG_MT9F002_P_RD_P2Q4                     0x3692
#define P_REG_MT9F002_P_BL_P2Q0                     0x3694
#define P_REG_MT9F002_P_BL_P2Q1                     0x3696
#define P_REG_MT9F002_P_BL_P2Q2                     0x3698
#define P_REG_MT9F002_P_BL_P2Q3                     0x369A
#define P_REG_MT9F002_P_BL_P2Q4                     0x369C
#define P_REG_MT9F002_P_GB_P2Q0                     0x369E
#define P_REG_MT9F002_P_GB_P2Q1                     0x36A0
#define P_REG_MT9F002_P_GB_P2Q2                     0x36A2
#define P_REG_MT9F002_P_GB_P2Q3                     0x36A4
#define P_REG_MT9F002_P_GB_P2Q4                     0x36A6

#define P_REG_MT9F002_P_GR_P3Q0                     0x36C0
#define P_REG_MT9F002_P_GR_P3Q1                     0x36C2
#define P_REG_MT9F002_P_GR_P3Q2                     0x36C4
#define P_REG_MT9F002_P_GR_P3Q3                     0x36C6
#define P_REG_MT9F002_P_GR_P3Q4                     0x36C8
#define P_REG_MT9F002_P_RD_P3Q0                     0x36CA
#define P_REG_MT9F002_P_RD_P3Q1                     0x36CC
#define P_REG_MT9F002_P_RD_P3Q2                     0x36CE
#define P_REG_MT9F002_P_RD_P3Q3                     0x36D0
#define P_REG_MT9F002_P_RD_P3Q4                     0x36D2
#define P_REG_MT9F002_P_BL_P3Q0                     0x36D4
#define P_REG_MT9F002_P_BL_P3Q1                     0x36D6
#define P_REG_MT9F002_P_BL_P3Q2                     0x36D8
#define P_REG_MT9F002_P_BL_P3Q3                     0x36DA
#define P_REG_MT9F002_P_BL_P3Q4                     0x36DC
#define P_REG_MT9F002_P_GB_P3Q0                     0x36DE
#define P_REG_MT9F002_P_GB_P3Q1                     0x36E0
#define P_REG_MT9F002_P_GB_P3Q2                     0x36E2
#define P_REG_MT9F002_P_GB_P3Q3                     0x36E4
#define P_REG_MT9F002_P_GB_P3Q4                     0x36E6

#define P_REG_MT9F002_P_GR_P4Q0                     0x3700
#define P_REG_MT9F002_P_GR_P4Q1                     0x3702
#define P_REG_MT9F002_P_GR_P4Q2                     0x3704
#define P_REG_MT9F002_P_GR_P4Q3                     0x3706
#define P_REG_MT9F002_P_GR_P4Q4                     0x3708
#define P_REG_MT9F002_P_RD_P4Q0                     0x370A
#define P_REG_MT9F002_P_RD_P4Q1                     0x370C
#define P_REG_MT9F002_P_RD_P4Q2                     0x370E
#define P_REG_MT9F002_P_RD_P4Q3                     0x3710
#define P_REG_MT9F002_P_RD_P4Q4                     0x3712
#define P_REG_MT9F002_P_BL_P4Q0                     0x3714
#define P_REG_MT9F002_P_BL_P4Q1                     0x3716
#define P_REG_MT9F002_P_BL_P4Q2                     0x3718
#define P_REG_MT9F002_P_BL_P4Q3                     0x371A
#define P_REG_MT9F002_P_BL_P4Q4                     0x371C
#define P_REG_MT9F002_P_GB_P4Q0                     0x371E
#define P_REG_MT9F002_P_GB_P4Q1                     0x3720
#define P_REG_MT9F002_P_GB_P4Q2                     0x3722
#define P_REG_MT9F002_P_GB_P4Q3                     0x3724
#define P_REG_MT9F002_P_GB_P4Q4                     0x3726

#define P_REG_MT9F002_POLY_SC_ENABLE                0x3780
#define P_REG_MT9F002_POLY_ORIGIN_C                 0x3782
#define P_REG_MT9F002_POLY_ORIGIN_R                 0x3784
#define P_REG_MT9F002_P_GR_Q5                       0x37C0
#define P_REG_MT9F002_P_RD_Q5                       0x37C2
#define P_REG_MT9F002_P_BL_Q5                       0x37C4
#define P_REG_MT9F002_P_GB_Q5                       0x37C6

#define P_REG_MT9F002_DAC_LD_FBIAS                  0x3EF8

/**
 * SENSOR_REGSxx
 */
#define P_MT9F002_MODEL_ID           0
#define P_MT9F002_RESET              1

#define P_MT9F002_WIDTH              2
#define P_MT9F002_HEIGHT             3

#define P_MT9F002_EXPOS              4

// do not need global gain? why not
#define P_MT9F002_GAIN               5

#define P_MT9F002_GAINGR             6
#define P_MT9F002_GAINR              7
#define P_MT9F002_GAINB              8
#define P_MT9F002_GAINGB             9

// nobody needs to control digital gain

#define P_MT9F002_TEST_PATTERN       10

#define P_MT9F002_HISPI_TIMING            11
#define P_MT9F002_SMIA_PLL_MULTIPLIER     12
#define P_MT9F002_HISPI_CONTROL_STATUS    13
#define P_MT9F002_DATAPATH_SELECT         14
#define P_MT9F002_RESET_REGISTER          15
#define P_MT9F002_ANALOG_GAIN_CODE_GLOBAL 16
#define P_MT9F002_ANALOG_GAIN_CODE_RED    17
#define P_MT9F002_ANALOG_GAIN_CODE_BLUE   18

#define P_MT9F002_COARSE_INTEGRATION_TIME 19
#define P_MT9F002_FINE_INTEGRATION_TIME   20

#define P_MT9F002_Y_ADDR_START       21
#define P_MT9F002_Y_ADDR_END         22
#define P_MT9F002_Y_OUTPUT_SIZE      23
#define P_MT9F002_X_ADDR_START       24
#define P_MT9F002_X_ADDR_END         25
#define P_MT9F002_X_OUTPUT_SIZE      26
#define P_MT9F002_LINE_LENGTH_PCK    27

#define P_MT9F002_X_ODD_INC             28
#define P_MT9F002_MIN_LINE_BLANKING_PCK 29
#define P_MT9F002_MIN_LINE_LENGTH_PCK   30

#define P_MT9F002_FRAME_LENGTH_LINES    31
#define P_MT9F002_MIN_FRAME_BLANKING_LINES  32

#define P_MT9F002_READ_MODE          33

//#define P_REG(x) x

//#define P_MT9F002_MODEL_ID 4
//#define P_MT9F002_FINE_CORRECTION 5

//#define P_REG_HADDR(x)  pi2a[2*(x)+1]>>8 & 0xff
//#define P_REG_LADDR(x)  pi2a[2*(x)+1] & 0xff

/**
LUT to map SENSOR_REGSxxx to internal sensor register addresses
  * needed for any sensor
  * For better manual mapping:
      - even elements are SENSOR_REGSxxx,
      - odd elements are sensor's register addresses.
  * has to be at least 16-bit/entry for 16 bit addresses
  * (for MT9X001 it's a 1-to-1 mapping)
*/
extern const unsigned short mt9f002_par2addr[];
extern const unsigned short mt9f002_pages[];

/** Detect one of Micron/Aptina/On Semiconductor sensors MT9M*, MT9D*,MT9T*, andMT9P* with parallel interface */
int mt9f002_pgm_detectsensor   (int sensor_port,               ///< sensor port number (0..3)
                                struct sensor_t * sensor,      ///< sensor static parameters (capabilities)
                                struct framepars_t * thispars, ///< sensor current parameters
                                struct framepars_t * prevpars, ///< sensor previous parameters (not used here)
                                int frame16)                   ///< 4-bit (hardware) frame number parameters should
                                                               ///< be applied to,  negative - ASAP
                                                               ///< @return 0 - OK, negative - error
;
void mt9f002_set_device(struct device *dev);

#endif
