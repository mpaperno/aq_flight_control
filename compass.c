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

    Copyright © 2011-2014  Bill Nesbitt
*/

#include "aq.h"
#include "compass.h"

float compassNormalize(float heading) {
    while (heading < 0.0f)
	heading += 360.0f;
    while (heading >= 360.0f)
	heading -= 360.0f;

    return heading;
}

// calculate the shortest distance in yaw to get from b => a
float compassDifference(float a, float b) {
    float diff = b - a;

    while (diff > 180.0f)
	diff -= 360.0f;
    while (diff <= -180.0f)
	diff += 360.0f;

    return diff;
}
