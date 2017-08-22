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
    public class Settings
    {
        //--------------------------------------------------------------------------------------------------
        // Internal Implementation const doubleants (do not change unless you know what you're doing)
        //--------------------------------------------------------------------------------------------------
        public const double Q3_SLEEP_LINEAR = 0.01;

        public const double Q3_SLEEP_ANGULAR = (2.0 / 180.0) * Math.PI;

        public const double Q3_SLEEP_TIME = 0.5;

        public const double Q3_BAUMGARTE = 0.2;

        public const double Q3_PENETRATION_SLOP = 0.05;

        public static void Assert(bool condition)
        {
            if (!condition) throw new Exception("Assert results in false condition");
            // System.Diagnostics.Debug.Assert(condition);
        }

        // Restitution mixing. The idea is to use the maximum bounciness, so bouncy
        // objects will never not bounce during collisions.
        public static double MixRestitution(Box A, Box B)
        {
            return Math.Max(A.restitution, B.restitution);
        }

        // Friction mixing. The idea is to allow a very low friction value to
        // drive down the mixing result. Example: anything slides on ice.
        public static double MixFriction(Box A, Box B)
        {
            return Math.Sqrt(A.friction * B.friction);
        }

    }
}
