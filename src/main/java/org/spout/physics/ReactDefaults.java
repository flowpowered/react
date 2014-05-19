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
package org.spout.physics;

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
     * True if the deactivation (sleeping) of inactive bodies is enabled. Default: true
     */
    public static final boolean DEACTIVATION_ENABLED = true;
    /**
     * Object margin for collision detection in cm. Default: 0.04
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
     * Number of iterations when solving the velocity constraints of the Sequential Impulse technique. Default: 15
     */
    public static final int DEFAULT_VELOCITY_SOLVER_NB_ITERATIONS = 15;
    /**
     * Number of iterations when solving the position constraints of the Sequential Impulse technique. Default: 3
     */
    public static final int DEFAULT_POSITION_SOLVER_NB_ITERATIONS = 5;
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
         * Faster but can be inaccurate in some situations. This is the option used by default.
         */
        BAUMGARTE_JOINTS,
        /**
         * Slower but more precise.
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
