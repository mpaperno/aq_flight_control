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

#include "getbuildnum.h"
#include <stdio.h>

unsigned long getBuildNumber(void) {
  return (unsigned)BUILDNUMBER;
}

unsigned long getRevisionNumber(void) {
  unsigned long  rev;

  sscanf(REVISION, "$Revision: %ld", &rev);

  return rev;
}
