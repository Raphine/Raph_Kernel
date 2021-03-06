/*
 *
 * Copyright (c) 2016 Raphine Project
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * Author: Liva
 * 
 */

#ifndef __RAPH_KERNEL_DEV_FS_FAT_H__
#define __RAPH_KERNEL_DEV_FS_FAT_H__

#include "ff.h"
#include <raph.h>
#include <tty.h>
#include <global.h>

class FatFs {
public:
  FatFs() {
  }
  bool Mount() {
    gtty->Printf(">>>%d\n", f_mount(&_fs, "0:/", 1));
      
      //    return (f_mount(&_fs, "C:/", 1) == FR_OK);
    return false;
  }
private:
  FATFS _fs;
};

#endif // __RAPH_KERNEL_DEV_FS_FAT_H__
