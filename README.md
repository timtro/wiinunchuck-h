wiinunchuck.h
=============
v0.9

A Library for Using the Wii Nunchuk In Arduino Sketches

For full documentation, see

<http://www.timteatro.net/2012/02/10/a-library-for-using-the-wii-nunchuk-in-arduino-sketches/>


This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or (at
your option) any later version.

This program is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

Here is a listing of the file's description:

Library to set up and poll a Wii nunchuk with Arduino. There are
many libraries available to do this, none of which I really liked.
I was fond of Tod Kurt's, but his was incomplete as it did not work
with knockoff nunchuks, it did not consider the least significant
bits of accelerometer data and didn't have any advanced functions
for processing the data such as calculating pitch and roll angles.


Provides functions:
 * void nunchuk_setpowerpins()
 * void nunchuk_init()
 * int nunchuk_get_data()
 * void nunchuk_calibrate_joy()
 * inline unsigned int nunchuk_zbutton()
 * inline unsigned int nunchuk_cbutton()
 * inline int nunchuk_joy_x()
 * inline int nunchuk_cjoy_x()
 * inline int nunchuk_cjoy_y()
 * inline uint16_t nunchuk_accelx()
 * inline uint16_t nunchuk_accely()
 * inline uint16_t nunchuk_accelz()
 * inline int nunchuk_caccelx()
 * inline int nunchuk_caccely()
 * inline int nunchuk_caccelz()
 * inline int nunchuk_joyangle()
 * inline int nunchuk_rollangle()
 * inline int nunchuk_pitchangle()
 * void nunchuk_calibrate_accelxy()
 * void nunchuk_calibrate_accelz()

This library is inspired by the work of Tod E. Kurt,
(http://todbot.com/blog/bionicarduino/)

(c) 2012 by Tim Teatro
