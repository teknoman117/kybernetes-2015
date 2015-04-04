#
#  yuv422_bithreshold.s
#
#  Copyright (c) 2013 Nathaniel Lewis, Robotics Society at UC Merced
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see <http://www.gnu.org/licenses/>.
#

# GNU Assembler requires this for arm
	.syntax unified
	.arch armv7-a
	.fpu neon
	.eabi_attribute 27, 3
	.eabi_attribute 28, 1
	.eabi_attribute 20, 1
	.eabi_attribute 21, 1
	.eabi_attribute 23, 3
	.eabi_attribute 24, 1
	.eabi_attribute 25, 1
	.eabi_attribute 26, 2
	.eabi_attribute 30, 2
	.eabi_attribute 34, 1
	.eabi_attribute 18, 4
	
# Test operation in NEON   
	.text
	.align	2
	.global	yuv422_bithreshold
	.thumb
	.thumb_func
yuv422_bithreshold:
     # Prologue (make stack space, preserve variables)
     push {r4, r5, r6, r7}

     # Create the lower y boundary
     ldrb       r4, [sp, #16]
     vdup.8     q12, r4

     # Create the lower u boundary
     ldrb       r4, [sp, #20]
     vdup.8     d26, r4

     # Create the lower v boundary
     ldrb       r4, [sp, #24]
     vdup.8     d27, r4

     # Create the upper y boundary
     ldrb       r4, [sp, #28]
     vdup.8     q14, r4

     # Create the upper u boundary
     ldrb       r4, [sp, #32]
     vdup.8     d30, r4

     # Create the upper v boundary
     ldrb       r4, [sp, #36]
     vdup.8     d31, r4

# Loop to handle rows of image (reset column counter)
row_loop:
     mov       r4, r2
     
# Loop to handle columns of a row
column_loop:
     # Do a 256 bit vector load from r0 (16 pixels).  This also performs an unzip operation
	vld2.8	{d16-d19}, [r0]!
	
	# Deinterleave the U and V components
	vuzp.8    d18, d19
	
	# Perform unzip on the Y components, this gives use two arrays of y components which align to their u and v
	vuzp.8    d16, d17
	
	# We will compare the Y vectors with boundaries (and'ing both results)
	vcge.u8   q11, q8, q12
	vcle.u8   q0,  q8, q14
	vand.8    q0,  q0, q11
	
	# We will compare the U vectors with boundaries (and'ing both results)
	vcge.u8   d22, d18, d26
	vcle.u8   d23, d18, d30
	vand.8    d2,  d22, d23
	
	# We will compare the V vectors with boundaries (and'ing both results)
	vcge.u8   d22, d19, d27
	vcle.u8   d23, d19, d31
	vand.8    d3,  d22, d23
	
	# We now need to combine the results of all three channel comparisions (and results)
	vand.8    d0, d0, d2
	vand.8    d0, d0, d3
	vand.8    d1, d1, d2
	vand.8    d1, d1, d3
	
	# Since we unzipped the y channels we have to zip back before storing in memory, vst2 zips before store, saving an operation
	vst2.8	{d0-d1}, [r1]!
	
	# Check if we are finished with the row
	subs      r4, r4, #16
	bne       column_loop
	
	# Check if we are finished with the image
	subs      r3, r3, #1
	bne       row_loop 	
	
# Epilogue (restore stack, preserve variables)
	pop {r4, r5, r6, r7}
	bx	lr
