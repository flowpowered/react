/*
 * This file is part of Flow React, licensed under the MIT License (MIT).
 *
 * Copyright (c) 2013 Flow Powered <https://flowpowered.com/>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
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
package com.flowpowered.react;

/**
 * Physics engine constants
 */
public class ReactDefaults {
    /**
     * The machine epsilon. Default: 1.1920929E-7
     */
    public static final float MACHINE_EPSILON = 1.1920929E-7f;
    /**
     * 2 * Pi constant.
     */
    public static final float PI_TIMES_2 = 6.28318530f;
    /**
     * Default internal constant timestep in seconds. Default: 1/60s
     */
    public static final float DEFAULT_TIMESTEP = 1f / 60;
    /**
     * Default restitution coefficient for a rigid body. Default: 0.5
     */
    public static final float DEFAULT_RESTITUTION_COEFFICIENT = 0.5f;
    /**
     * Default friction coefficient for a rigid body. Default: 0.3
     */
    public static final float DEFAULT_FRICTION_COEFFICIENT = 0.3f;
    /**
     * Default bounciness factor for a rigid body. Default: 0.5
     */
    public static final float DEFAULT_BOUNCINESS = 0.5f;
    /**
     * True if the sleeping technique is enabled. Default: true
     */
    public static final boolean SLEEPING_ENABLED = true;
    /**
     * Object margin for collision detection in meters (for the GJK-EPA Algorithm). Default: 0.04
     */
    public static final float OBJECT_MARGIN = 0.04f;
    /**
     * Distance threshold for two contact points for a valid persistent contact. Default: 0.02
     */
    public static final float PERSISTENT_CONTACT_DIST_THRESHOLD = 0.02f;
    /**
     * Velocity threshold for contact velocity restitution. Default: 1
     */
    public static final float RESTITUTION_VELOCITY_THRESHOLD = 1;
    /**
     * Number of iterations when solving the velocity constraints of the Sequential Impulse technique. Default: 10
     */
    public static final int DEFAULT_VELOCITY_SOLVER_NB_ITERATIONS = 10;
    /**
     * Number of iterations when solving the position constraints of the Sequential Impulse technique. Default: 5
     */
    public static final int DEFAULT_POSITION_SOLVER_NB_ITERATIONS = 5;
    /**
     * Time (in seconds) that a body must stay still to be considered sleeping. Default: 1
     */
    public static final float DEFAULT_TIME_BEFORE_SLEEP = 1.0f;
    /**
     * A body with a linear velocity smaller than the sleep linear velocity (in m/s) might enter sleeping mode. Default: 0.02
     */
    public static final float DEFAULT_SLEEP_LINEAR_VELOCITY = 0.02f;
    /**
     * A body with angular velocity smaller than the sleep angular velocity (in rad/s) might enter sleeping mode. Default: 3pi/180
     */
    public static final float DEFAULT_SLEEP_ANGULAR_VELOCITY = (float) (3 * (Math.PI / 180));
    /**
     * The linked phase AABB scaling factor. Default: 2
     */
    public static final float LINKED_PHASE_AABB_SCALING = 2;

    /**
     * Position correction technique used in the constraint solver (for joints). Default: NON_LINEAR_GAUSS_SEIDEL
     * <p/>
     * BAUMGARTE: Faster but can be inaccurate in some situations.
     * <p/>
     * NON_LINEAR_GAUSS_SEIDEL: Slower but more precise. This is the option used by default.
     */
    public static enum JointsPositionCorrectionTechnique {
        /**
         * Faster but can be inaccurate in some situations.
         */
        BAUMGARTE_JOINTS,
        /**
         * Slower but more precise. This is the option used by default.
         */
        NON_LINEAR_GAUSS_SEIDEL
    }

    /**
     * Position correction technique used in the contact solver (for contacts). Default: SPLIT_IMPULSES
     * <p/>
     * BAUMGARTE: Faster but can be inaccurate and can lead to unexpected bounciness in some situations (due to error correction factor being added to the bodies momentum).
     * <p/>
     * SPLIT_IMPULSES: A bit slower but the error correction factor is not added to the bodies momentum. This is the option used by default.
     */
    public static enum ContactsPositionCorrectionTechnique {
        /**
         * Faster but can be inaccurate and can lead to unexpected bounciness in some situations (due to error correction factor being added to the bodies momentum).
         */
        BAUMGARTE_CONTACTS,
        /**
         * A bit slower but the error correction factor is not added to the bodies momentum. This is the option used by default.
         */
        SPLIT_IMPULSES
    }
}
