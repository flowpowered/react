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
package org.spout.physics.body;

import org.spout.physics.collision.shape.CollisionShape;
import org.spout.physics.math.Matrix3x3;
import org.spout.physics.math.Transform;

/**
 * Detector body which only informs callbacks of a collision. This body will not incur any collision adjustments nor will it prevent objects from passing through.
 * <p>
 * Uses for this object are when you want to inform the callbacks of a collision but handle adjustments on your own (ex. Character movement, moving through water, etc.)
 */
public class GhostMobileRigidBody extends MobileRigidBody {
	/**
	 * Constructs a new rigid body from its transform, mass, local inertia tensor, collision shape and ID.
	 *
	 * @param transform The transform (position and orientation)
	 * @param mass The mass
	 * @param inertiaTensorLocal The local inertial tensor
	 * @param collisionShape The collision shape
	 * @param id The ID
	 */
	public GhostMobileRigidBody(Transform transform, float mass, Matrix3x3 inertiaTensorLocal, CollisionShape collisionShape, int id) {
		super(transform, mass, inertiaTensorLocal, collisionShape, id);
	}
}
