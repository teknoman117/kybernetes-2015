/*
 *  cv.hpp
 *
 *  Copyright (c) 2013 Nathaniel Lewis, Robotics Society at UC Merced
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
 */

#ifndef _kybernetes_cv_cv_h_
#define _kybernetes_cv_cv_h_

#include <cstdint>

// Kybernetes namespace
namespace kybernetes
{
    // controller namespace
    namespace cv
    {
        // Color finding procedures.  Take a YUYV image with a width that is a multiple of 16, and return a bitmap of the pixels within a specified range
        extern "C" void yuv422_bithreshold(void *source, void *destination, uint16_t width, uint16_t height, uint8_t ly, uint8_t lu, uint8_t lv, uint8_t uy, uint8_t uu, uint8_t uv);
    }
}

#endif
