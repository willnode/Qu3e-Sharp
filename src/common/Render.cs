//--------------------------------------------------------------------------------------------------
/**
    Qu3e Physics Engine - C# Version 1.01

	Copyright (c) 2014 Randy Gaul http://www.randygaul.net

	This software is provided 'as-is', without any express or implied
	warranty. In no event will the authors be held liable for any damages
	arising from the use of this software.

	Permission is granted to anyone to use this software for any purpose,
	including commercial applications, and to alter it and redistribute it
	freely, subject to the following restrictions:
	  1. The origin of this software must not be misrepresented; you must not
	     claim that you wrote the original software. If you use this software
	     in a product, an acknowledgment in the product documentation would be
	     appreciated but is not required.
	  2. Altered source versions must be plainly marked as such, and must not
	     be misrepresented as being the original software.
	  3. This notice may not be removed or altered from any source distribution.
*/
//--------------------------------------------------------------------------------------------------

using System;
using System.Collections.Generic;
using static Qu3e.Settings;

namespace Qu3e
{
    public abstract class Render
    {
        public abstract void SetPenColor(double r, double g, double b, double a = 1.0f);
        public abstract void SetPenPosition(double x, double y, double z);
        public abstract void SetScale(double sx, double sy, double sz);

        // Render a line from pen position to this point.
        // Sets the pen position to the new point.
        public abstract void Line(double x, double y, double z);

        public abstract void SetTriNormal(double x, double y, double z);

        // Render a triangle with the normal set by SetTriNormal.
        public abstract void Triangle(
            double x1, double y1, double z1,
            double x2, double y2, double z2,
            double x3, double y3, double z3);

        // Draw a point with the scale from SetScale
        public abstract void Point();
    }
}
