/*
 * This file is part of JReactPhysics3D.
 *
 * Copyright (c) 2013 Spout LLC <http://www.spout.org/>
 * JReactPhysics3D is licensed under the Spout License Version 1.
 *
 * JReactPhysics3D is free software: you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * In addition, 180 days after any changes are published, you can use the
 * software, incorporating those changes, under the terms of the MIT license,
 * as described in the Spout License Version 1.
 *
 * JReactPhysics3D is distributed in the hope that it will be useful, but WITHOUT ANY
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
package org.spout.jreactphysics3d;

/**
 * Physics engine constants
 */
public class Configuration {
	/**
	 * The machine epsilon
	 */
	public static final float MACHINE_EPSILON = 1.1920929E-7f;
	/**
	 * Default internal constant timestep in seconds
	 */
	public static final float DEFAULT_TIMESTEP = 1.0F / 60.0F;
	/**
	 * True if the deactivation (sleeping) of inactive bodies is enabled
	 */
	public static final boolean DEACTIVATION_ENABLED = true;
	// GJK Algorithm parameters
	/**
	 * Object margin for collision detection in cm
	 */
	public static final float OBJECT_MARGIN = 0.04F;
	// Contact constants
	/**
	 * Friction coefficient
	 */
	public static final float DEFAULT_FRICTION_COEFFICIENT = 0.4F;
	/**
	 * Distance threshold for two contact points for a valid persistent contact
	 */
	public static final float PERSISTENT_CONTACT_DIST_THRESHOLD = 0.02F;
	// Constraint solver constants
	/**
	 * Maximum number of bodies
	 */
	public static final int NB_MAX_BODIES = 100000;
	/**
	 * Maximum number of contacts (for memory pool allocation)
	 */
	public static final int NB_MAX_CONTACTS = 100000;
	/**
	 * Maximum number of constraints
	 */
	public static final int NB_MAX_CONSTRAINTS = 100000;
	/**
	 * Maximum number of collision pairs of bodies (for memory pool allocation)
	 */
	public static final int NB_MAX_COLLISION_PAIRS = 10000;
	// Constraint solver constants
	/**
	 * Number of iterations when solving a LCP problem
	 */
	public static final int DEFAULT_LCP_ITERATIONS = 15;
	/**
	 * Number of iterations when solving a LCP problem for error correction
	 */
	public static final int DEFAULT_LCP_ITERATIONS_ERROR_CORRECTION = 5;
	/**
	 * True if the error correction projection (first order world) is active in the constraint
	 * solver
	 */
	public static final boolean ERROR_CORRECTION_PROJECTION_ENABLED = true;
	/**
	 * Contacts with penetration depth (in meters) larger that this use error correction with
	 * projection
	 */
	public static final float PENETRATION_DEPTH_THRESHOLD_ERROR_CORRECTION = 0.20F;
}
