/*!***************************************************************************
 * @file   elphel393-mem.h
 * @brief 
 * @copyright Copyright (C) 2015 Elphel, Inc.

 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *****************************************************************************/
struct elphel_buf_t
{
	// Coherent DMA buffer
	void      *vaddr;
	dma_addr_t paddr;
	ssize_t    size;

	// Host to device stream DMA buffer
	void      *h2d_vaddr;
	dma_addr_t h2d_paddr;
	ssize_t    h2d_size;

	// Device to host stream DMA buffer
	void      *d2h_vaddr;
	dma_addr_t d2h_paddr;
	ssize_t    d2h_size;

	// Bidirectional stream DMA buffer
	void      *bidir_vaddr;
	dma_addr_t bidir_paddr;
	ssize_t    bidir_size;
};
extern struct elphel_buf_t *pElphel_buf;

