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
	 * The machine epsilon.
	 */
	public static final float MACHINE_EPSILON = 1.1920929E-7f;
	/**
	 * Default internal constant timestep in seconds.
	 */
	public static final float DEFAULT_TIMESTEP = 1f / 60;
	/**
	 * Default friction coefficient for a rigid body.
	 */
	public static final float DEFAULT_FRICTION_COEFFICIENT = 0.3f;
	/**
	 * True if the deactivation (sleeping) of inactive bodies is enabled.
	 */
	public static final boolean DEACTIVATION_ENABLED = true;
	/**
	 * Object margin for collision detection in cm.
	 */
	public static final float OBJECT_MARGIN = 0.04f;
	/**
	 * Distance threshold for two contact points for a valid persistent contact.
	 */
	public static final float PERSISTENT_CONTACT_DIST_THRESHOLD = 0.02f;
	/**
	 * Velocity threshold for contact velocity restitution.
	 */
	public static final float RESTITUTION_VELOCITY_THRESHOLD = 1;
	/**
	 * Number of iterations when solving a LCP problem.
	 */
	public static final int DEFAULT_CONSTRAINTS_SOLVER_NB_ITERATIONS = 15;
}
