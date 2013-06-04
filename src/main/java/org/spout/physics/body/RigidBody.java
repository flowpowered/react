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

import org.spout.physics.Configuration;
import org.spout.physics.collision.shape.CollisionShape;
import org.spout.physics.math.Matrix3x3;
import org.spout.physics.math.Transform;
import org.spout.physics.math.Vector3;

/**
 * Represents a rigid body for the physics engine. A rigid body is a non-deformable body that has a
 * constant mass. This class inherits from the CollisionBody class.
 */
public class RigidBody extends CollisionBody {
	// TODO : Remove the mass variable (duplicate with inverseMass)
	protected float mMass;
	protected final Vector3 mLinearVelocity = new Vector3();
	protected final Vector3 mAngularVelocity = new Vector3();
	protected final Vector3 mExternalForce = new Vector3();
	protected final Vector3 mExternalTorque = new Vector3();
	protected final Matrix3x3 mInertiaTensorLocal = new Matrix3x3();
	protected final Matrix3x3 mInertiaTensorLocalInverse = new Matrix3x3();
	protected float mMassInverse;
	protected float mRestitution;
	protected float mFrictionCoefficient;

	/**
	 * Constructs a new rigid body from it's transform, mass, local inertia tensor, collision shape and
	 * tensor.
	 *
	 * @param transform The transform (position and orientation)
	 * @param mass The mass
	 * @param inertiaTensorLocal The local inertial tensor
	 * @param collisionShape The collision shape
	 * @param id The ID
	 */
	public RigidBody(Transform transform, float mass, Matrix3x3 inertiaTensorLocal, CollisionShape collisionShape, int id) {
		super(transform, collisionShape, id);
		if (collisionShape == null) {
			throw new IllegalArgumentException("collisionShape cannot be null");
		}
		mInertiaTensorLocal.set(inertiaTensorLocal);
		mMass = mass;
		mInertiaTensorLocalInverse.set(inertiaTensorLocal.getInverse());
		mMassInverse = 1 / mass;
		mFrictionCoefficient = Configuration.DEFAULT_FRICTION_COEFFICIENT;
		mRestitution = 1;
	}

	/**
	 * Gets the mass of the body.
	 *
	 * @return The body's mass
	 */
	public float getMass() {
		return mMass;
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
	 * Gets the linear velocity of the body.
	 *
	 * @return The linear velocity
	 */
	public Vector3 getLinearVelocity() {
		return mLinearVelocity;
	}

	/**
	 * Gets the angular velocity of the body.
	 *
	 * @return The angular velocity
	 */
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
	 * Sets the inverse of the mass
	 *
	 * @param massInverse The inverse of the mass
	 */
	public void setMassInverse(float massInverse) {
		mMassInverse = massInverse;
	}

	/**
	 * Gets the inverse of the inertia tensor.
	 *
	 * @return The inverse of the inertia tensor
	 */
	public Matrix3x3 getInertiaTensorLocalInverse() {
		return mInertiaTensorLocalInverse;
	}

	/**
	 * Gets the current external force on the body.
	 *
	 * @return The current external force
	 */
	public Vector3 getExternalForce() {
		return mExternalForce;
	}

	/**
	 * Sets the current external force on the body.
	 *
	 * @param force The external force to set
	 */
	public void setExternalForce(Vector3 force) {
		mExternalForce.set(force);
	}

	/**
	 * Gets the current external torque on the body.
	 *
	 * @return The current external torque
	 */
	public Vector3 getExternalTorque() {
		return mExternalTorque;
	}

	/**
	 * Sets the current external torque on the body.
	 *
	 * @param torque The external torque to set
	 */
	public void setExternalTorque(Vector3 torque) {
		mExternalTorque.set(torque);
	}

	/**
	 * Gets the inverse of the mass of the body.
	 *
	 * @return The inverse of the mass
	 */
	public float getMassInverse() {
		return mMassInverse;
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
	}

	/**
	 * Gets the inertia tensor in world coordinates. The inertia tensor I_w in world coordinates is
	 * computed with the local inertia tensor I_b in body coordinates by I_w = R * I_b * R^T, where R
	 * is the rotation matrix (and R^T its transpose) of the current orientation quaternion of the
	 * body.
	 *
	 * @return The world inertia tensor
	 */
	public Matrix3x3 getInertiaTensorWorld() {
		return Matrix3x3.multiply(
				Matrix3x3.multiply(mTransform.getOrientation().getMatrix(), mInertiaTensorLocal),
				mTransform.getOrientation().getMatrix().getTranspose());
	}

	/**
	 * Gets the inverse of the inertia tensor in world coordinates. The inertia tensor I_w in world
	 * coordinates is computed with the local inverse inertia tensor I_b^-1 in body coordinates by I_w
	 * = R * I_b^-1 * R^T, where R is the rotation matrix (and R^T its transpose) of the current
	 * orientation quaternion of the body.
	 *
	 * @return The world inverse inertia tensor
	 */
	public Matrix3x3 getInertiaTensorInverseWorld() {
		return Matrix3x3.multiply(
				Matrix3x3.multiply(mTransform.getOrientation().getMatrix(), mInertiaTensorLocalInverse),
				mTransform.getOrientation().getMatrix().getTranspose());
	}

	/**
	 * Set the linear velocity for this body, but only if it can move.
	 *
	 * @param linearVelocity The linear velocity to set
	 * @see CollisionBody#setIsMotionEnabled(boolean)
	 */
	public void setLinearVelocity(Vector3 linearVelocity) {
		if (mIsMotionEnabled) {
			mLinearVelocity.set(linearVelocity);
		}
	}

	/**
	 * Gets the restitution coefficient for this body.
	 *
	 * @return The restitution coefficient
	 */
	public float getRestitution() {
		return mRestitution;
	}

	/**
	 * Sets the restitution coefficient.
	 *
	 * @param restitution The restitution coefficient to set
	 */
	public void setRestitution(float restitution) {
		assert (restitution >= 0 && restitution <= 1);
		mRestitution = restitution;
	}

	/**
	 * Gets the friction coefficient for this body.
	 *
	 * @return The friction coefficient
	 */
	public float getFrictionCoefficient() {
		return mFrictionCoefficient;
	}

	/**
	 * Sets the friction coefficient for this body.
	 *
	 * @param frictionCoefficient The friction coefficient to set
	 */
	public void setFrictionCoefficient(float frictionCoefficient) {
		mFrictionCoefficient = frictionCoefficient;
	}
}
