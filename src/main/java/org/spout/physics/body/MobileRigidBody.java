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
import org.spout.physics.math.Vector3;

/**
 * Represents a mobile rigid body. Such a body can move and has all of the properties of a normal
 * rigid body.
 */
public class MobileRigidBody extends ImmobileRigidBody {
	private final Vector3 mLinearVelocity = new Vector3();
	private final Vector3 mAngularVelocity = new Vector3();
	private boolean mIsMotionEnabled = true;

	/**
	 * Constructs a new rigid body from its transform, mass, local inertia tensor, collision shape and
	 * ID.
	 *
	 * @param transform The transform (position and orientation)
	 * @param mass The mass
	 * @param inertiaTensorLocal The local inertial tensor
	 * @param collisionShape The collision shape
	 * @param id The ID
	 */
	public MobileRigidBody(Transform transform, float mass, Matrix3x3 inertiaTensorLocal, CollisionShape collisionShape, int id) {
		super(transform, mass, inertiaTensorLocal, collisionShape, id);
	}

	/**
	 * Gets the linear velocity of the body.
	 *
	 * @return The linear velocity
	 */
	@Override
	public Vector3 getLinearVelocity() {
		return mLinearVelocity;
	}

	/**
	 * Set the linear velocity for this body, but only if it can move.
	 *
	 * @param linearVelocity The linear velocity to set
	 * @see #isMotionEnabled()
	 * @see #setMotionEnabled(boolean)
	 */
	public void setLinearVelocity(Vector3 linearVelocity) {
		if (mIsMotionEnabled) {
			mLinearVelocity.set(linearVelocity);
		}
	}

	/**
	 * Gets the angular velocity of the body.
	 *
	 * @return The angular velocity
	 */
	@Override
	public Vector3 getAngularVelocity() {
		return mAngularVelocity;
	}

	/**
	 * Sets the angular velocity of the body.
	 *
	 * @param angularVelocity The angular velocity to set
	 */
	public void setAngularVelocity(Vector3 angularVelocity) {
		mAngularVelocity.set(angularVelocity);
	}

	/**
	 * Returns true if the body can move, false if not.
	 *
	 * @return Whether or not the body can move
	 */
	@Override
	public boolean isMotionEnabled() {
		return mIsMotionEnabled;
	}

	/**
	 * Sets whether or not this body can move. True to allow movement, false to disallow.
	 *
	 * @param isMotionEnabled True if the body should move, false if not
	 */
	public void setMotionEnabled(boolean isMotionEnabled) {
		mIsMotionEnabled = isMotionEnabled;
	}

	@Override
	public String toString() {
		return "MobileRigidBody{id= " + getID() + ", transform= " + getTransform() + ", mass=" + getMass() + ", aabb= " + getAABB() + "}";
	}
}
