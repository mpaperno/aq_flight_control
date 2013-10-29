/*
    This file is part of AutoQuad.

    AutoQuad is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    AutoQuad is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with AutoQuad.  If not, see <http://www.gnu.org/licenses/>.

    Copyright © 2011, 2012, 2013  Bill Nesbitt
*/

#ifndef _board_dimu_v2_h
#define _board_dimu_v2_h

#define DIMU_HAVE_EEPROM

#define DIMU_HAVE_MPU6000
#define DIMU_HAVE_HMC5983
#define DIMU_HAVE_MS5611

#define DIMU_ORIENT_ACC_X	    (+in[0])
#define DIMU_ORIENT_ACC_Y	    (+in[1])
#define DIMU_ORIENT_ACC_Z	    (+in[2])

#define DIMU_ORIENT_GYO_X	    (-in[0])
#define DIMU_ORIENT_GYO_Y	    (-in[1])
#define DIMU_ORIENT_GYO_Z	    (+in[2])

#define DIMU_ORIENT_MAG_X	    (-in[0])
#define DIMU_ORIENT_MAG_Y	    (-in[1])
#define DIMU_ORIENT_MAG_Z	    (+in[2])

#endif