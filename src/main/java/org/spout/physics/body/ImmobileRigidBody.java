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
 * Represents an immobile rigid body. Such a body cannot move, but has all of the properties of a normal rigid body, except for velocities. This can be used for scenery, floors, walls, etc.
 */
public class ImmobileRigidBody extends RigidBody {
	private static final Vector3 ZERO = new Vector3(0, 0, 0);
	protected float mMass;
	protected final Matrix3x3 mInertiaTensorLocal = new Matrix3x3();
	protected final Matrix3x3 mInertiaTensorLocalInverse = new Matrix3x3();
	protected final Vector3 mExternalForce = new Vector3();
	protected final Vector3 mExternalTorque = new Vector3();

	/**
	 * Constructs a new rigid body from its transform, mass, local inertia tensor, collision shape and ID.
	 *
	 * @param transform The transform (position and orientation)
	 * @param mass The mass
	 * @param inertiaTensorLocal The local inertial tensor
	 * @param collisionShape The collision shape
	 * @param id The ID
	 */
	public ImmobileRigidBody(Transform transform, float mass, Matrix3x3 inertiaTensorLocal, CollisionShape collisionShape, int id) {
		super(transform, collisionShape, id);
		mMass = mass;
		mInertiaTensorLocal.set(inertiaTensorLocal);
		mInertiaTensorLocalInverse.set(inertiaTensorLocal.getInverse());
	}

	/**
	 * Gets the mass of the body.
	 *
	 * @return The body's mass
	 */
	@Override
	public float getMass() {
		return mMass;
	}

	/**
	 * Gets the inverse of the mass of the body.
	 *
	 * @return The inverse of the mass
	 */
	@Override
	public float getMassInverse() {
		return (1 / mMass);
	}

	/**
	 * Sets the mass of the body.
	 *
	 * @param mass The mass to set
	 */
	public void setMass(float mass) {
		mMass = mass;
	}

	/**
	 * Gets the local inertia tensor of the body (in body coordinates).
	 *
	 * @return The local inertia tensor
	 */
	public Matrix3x3 getInertiaTensorLocal() {
		return mInertiaTensorLocal;
	}

	/**
	 * Sets the local inertia tensor of the body (in body coordinates).
	 *
	 * @param inertiaTensorLocal The local inertia tensor to set
	 */
	public void setInertiaTensorLocal(Matrix3x3 inertiaTensorLocal) {
		mInertiaTensorLocal.set(inertiaTensorLocal);
		mInertiaTensorLocalInverse.set(mInertiaTensorLocal.getInverse());
	}

	/**
	 * Gets the inertia tensor in world coordinates. The inertia tensor I_w in world coordinates is computed with the local inertia tensor I_b in body coordinates by I_w = R * I_b * R^T, where R is the
	 * rotation matrix (and R^T its transpose) of the current orientation quaternion of the body.
	 *
	 * @return The world inertia tensor
	 */
	public Matrix3x3 getInertiaTensorWorld() {
		final Matrix3x3 orientation = mliveTransform.getOrientation().getMatrix();
		return Matrix3x3.multiply(Matrix3x3.multiply(orientation, mInertiaTensorLocal), orientation.getTranspose());
	}

	@Override
	public Matrix3x3 getInertiaTensorInverseWorld() {
		final Matrix3x3 orientation = mliveTransform.getOrientation().getMatrix();
		return Matrix3x3.multiply(Matrix3x3.multiply(orientation, mInertiaTensorLocalInverse), orientation.getTranspose());
	}

	@Override
	public Vector3 getExternalForce() {
		return mExternalForce;
	}

	@Override
	public void setExternalForce(Vector3 force) {
		mExternalForce.set(force);
	}

	@Override
	public Vector3 getExternalTorque() {
		return mExternalTorque;
	}

	@Override
	public void setExternalTorque(Vector3 torque) {
		mExternalTorque.set(torque);
	}

	/**
	 * Always returns the zero vector.
	 *
	 * @return The zero vector
	 */
	@Override
	public Vector3 getLinearVelocity() {
		return new Vector3(ZERO);
	}

	/**
	 * Always returns the zero vector.
	 *
	 * @return The zero vector
	 */
	@Override
	public Vector3 getAngularVelocity() {
		return new Vector3(ZERO);
	}

	/**
	 * Always returns false.
	 *
	 * @return False, always
	 */
	@Override
	public boolean isMotionEnabled() {
		return false;
	}

	@Override
	public String toString() {
		return "ImmobileRigidBody{id= " + getID() + ", transform= " + getTransform() + ", mass=" + getMass() + ", aabb= " + getAABB() + "}";
	}
}
