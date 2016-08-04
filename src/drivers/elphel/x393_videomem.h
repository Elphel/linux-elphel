/***************************************************************************//**
* @file      x393_videomem.h
* @brief     Driver for the external DDR3 memory of x393 (currently 0.5GB)
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
 
struct elphel_video_buf_t
{
    int frame_start[4];      ///< Channel 0 frame start (in bytes)
    int frame_full_width[4]; ///< Channel 0 frame full width (in bytes). 1 memory page is 2048 bytes (128 bursts)
    int frame_height[4];     ///< Channel 0 maximal frame height in pixel lines
    int frames_in_buffer[4]; ///< Number of frames in channel 0 buffer

};
