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
import org.spout.physics.engine.Material;
import org.spout.physics.math.Matrix3x3;
import org.spout.physics.math.Transform;
import org.spout.physics.math.Vector3;

/**
 * Represents a rigid body for the physics engine. A rigid body is a non-deformable body that has a constant mass. This class inherits from the CollisionBody class.
 */
public class RigidBody extends CollisionBody {
    private static final Material DEFAULT_MATERIAL = Material.asUnmodifiableMaterial(new Material());
    private Material mMaterial = DEFAULT_MATERIAL;
    private boolean mIsGravityEnabled;
    private float mMass;
    private float mMassInverse;
    private final Matrix3x3 mInertiaTensorLocal = new Matrix3x3();
    private final Matrix3x3 mInertiaTensorLocalInverse;
    private final Vector3 mExternalForce = new Vector3();
    private final Vector3 mExternalTorque = new Vector3();
    private final Vector3 mLinearVelocity = new Vector3();
    private final Vector3 mAngularVelocity = new Vector3();

    /**
     * Constructs a new rigid body from its transform, mass, local inertia tensor, collision shape and ID.
     *
     * @param transform The transform (position and orientation)
     * @param mass The mass
     * @param inertiaTensorLocal The local inertial tensor
     * @param collisionShape The collision shape
     * @param id The ID
     */
    public RigidBody(Transform transform, float mass, Matrix3x3 inertiaTensorLocal, CollisionShape collisionShape, int id) {
        super(transform, collisionShape, id);
        mInertiaTensorLocal.set(inertiaTensorLocal);
        mMass = mass;
        mInertiaTensorLocalInverse = inertiaTensorLocal.getInverse();
        mMassInverse = 1 / mass;
        mIsGravityEnabled = true;
    }

    /**
     * Returns true if the gravity needs to be applied to this rigid body.
     *
     * @return Whether or not gravity should be applied to the body
     */
    public boolean isGravityEnabled() {
        return mIsGravityEnabled;
    }

    /**
     * Sets the variable to know if the gravity is applied to this rigid body.
     *
     * @param isEnabled The new gravity state
     */
    public void enableGravity(boolean isEnabled) {
        mIsGravityEnabled = isEnabled;
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
     * Gets the inverse of the mass of the body.
     *
     * @return The inverse of the mass
     */
    public float getMassInverse() {
        return mMassInverse;
    }

    /**
     * Sets the inverse of the mass.
     *
     * @param massInverse The inverse of the mass
     */
    public void setMassInverse(float massInverse) {
        mMassInverse = massInverse;
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
     * Set the linear velocity for this body, but only if it can move.
     *
     * @param linearVelocity The linear velocity to set
     * @see #getIsMotionEnabled()
     * @see #setIsMotionEnabled(boolean)
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
     * Gets the inertia tensor in world coordinates. The inertia tensor I_w in world coordinates is computed with the local inertia tensor I_b in body coordinates by I_w = R * I_b * R^T, where R is
     * the rotation matrix (and R^T its transpose) of the current orientation quaternion of the body.
     *
     * @return The world inertia tensor
     */
    public Matrix3x3 getInertiaTensorWorld() {
        return Matrix3x3.multiply(Matrix3x3.multiply(mTransform.getOrientation().getMatrix(), mInertiaTensorLocal), mTransform.getOrientation().getMatrix().getTranspose());
    }

    /**
     * Gets the inverse of the inertia tensor in world coordinates. The inertia tensor I_w in world coordinates is computed with the local inverse inertia tensor I_b^-1 in body coordinates by I_w = R
     * * I_b^-1 * R^T, where R is the rotation matrix (and R^T its transpose) of the current orientation quaternion of the body.
     *
     * @return The world inverse inertia tensor
     */
    public Matrix3x3 getInertiaTensorInverseWorld() {
        return Matrix3x3.multiply(Matrix3x3.multiply(mTransform.getOrientation().getMatrix(), mInertiaTensorLocalInverse), mTransform.getOrientation().getMatrix().getTranspose());
    }

    /**
     * Sets the rigid body's material.
     *
     * @param material The material to set
     */
    public void setMaterial(Material material) {
        mMaterial = material;
    }

    /**
     * Gets the rigid body's material.
     *
     * @return The material
     */
    public Material getMaterial() {
        return mMaterial;
    }
}
