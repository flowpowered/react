/*
 * This file is part of React.
 *
 * Copyright (c) 2013 Spout LLC <http://www.spout.org/>
 * React is licensed under the Spout License Version 1.
 *
 * React is free software: you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * In addition, 180 days after any changes are published, you can use the
 * software, incorporating those changes, under the terms of the MIT license,
 * as described in the Spout License Version 1.
 *
 * React is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for
 * more details.
 *
 * You should have received a copy of the GNU Lesser General Public License,
 * the MIT license and the Spout License Version 1 along with this program.
 * If not, see <http://www.gnu.org/licenses/> for the GNU Lesser General Public
 * License and see <http://spout.in/licensev1> for the full license, including
 * the MIT license.
 */
package org.spout.physics.math;

import org.spout.physics.ReactDefaults;

/**
 * Various mathematical functions.
 */
public class Mathematics {
    /**
     * Returns true if the values a and b are approximately equal, using {@link ReactDefaults#MACHINE_EPSILON} as the acceptable error. Returns false if the values are not approximately equal.
     *
     * @param a The first value
     * @param b The second value
     * @return Whether or not the values are approximately equal
     */
    public static boolean approxEquals(float a, float b) {
        return approxEquals(a, b, ReactDefaults.MACHINE_EPSILON);
    }

    /**
     * Returns true if the values a and b are approximately equal, using the provided acceptable error. Returns false if the values are not approximately equal.
     *
     * @param a The first value
     * @param b The second value
     * @param epsilon The acceptable error
     * @return Whether or not the values are approximately equal
     */
    public static boolean approxEquals(float a, float b, float epsilon) {
        float difference = a - b;
        return difference < epsilon && difference > -epsilon;
    }
}
