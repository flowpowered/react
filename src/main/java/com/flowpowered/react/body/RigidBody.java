/*
 * This file is part of React, licensed under the MIT License (MIT).
 *
 * Copyright (c) 2013 Flow Powered <https://flowpowered.com/>
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://danielchappuis.ch>
 * React is re-licensed with permission from ReactPhysics3D author.
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
package com.flowpowered.react.body;

import com.flowpowered.react.collision.shape.CollisionShape;
import com.flowpowered.react.constraint.Joint;
import com.flowpowered.react.constraint.Joint.JointListElement;
import com.flowpowered.react.engine.Material;
import com.flowpowered.react.math.Matrix3x3;
import com.flowpowered.react.math.Transform;
import com.flowpowered.react.math.Vector3;

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
    private float mLinearDamping;
    private float mAngularDamping;
    private JointListElement mJointsList;

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
        mLinearDamping = 0;
        mAngularDamping = 0;
        mJointsList = null;
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
     * @see #isMotionEnabled()
     * @see #enableMotion(boolean)
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

    /**
     * Returns the linear velocity damping factor.
     *
     * @return The linear damping
     */
    public float getLinearDamping() {
        return mLinearDamping;
    }

    /**
     * Sets the linear damping factor.
     *
     * @param linearDamping The liner damping
     */
    public void setLinearDamping(float linearDamping) {
        if (linearDamping < 0) {
            throw new IllegalArgumentException("Linear damping must be greater or equal to 0");
        }
        mLinearDamping = linearDamping;
    }

    /**
     * Returns the angular velocity damping factor.
     *
     * @return The angular damping
     */
    public float getAngularDamping() {
        return mAngularDamping;
    }

    /**
     * Sets the angular damping factor.
     *
     * @param angularDamping The angular damping
     */
    public void setAngularDamping(float angularDamping) {
        if (angularDamping < 0) {
            throw new IllegalArgumentException("Angular damping must be greater or equal to 0");
        }
        mAngularDamping = angularDamping;
    }

    /**
     * Returns the first element of the linked list of joints involving this body.
     *
     * @return The first element of the list
     */
    public JointListElement getJointsList() {
        return mJointsList;
    }

    /**
     * Sets the first element in the joint list, discarding the entire list.
     *
     * @param jointsList The first element in the list
     */
    public void setJointsList(JointListElement jointsList) {
        mJointsList = jointsList;
    }

    /**
     * Removes a joint from the joints list.
     *
     * @param joint The joint to remove
     */
    public void removeJointFromJointsList(Joint joint) {
        if (joint == null) {
            throw new IllegalArgumentException("Joint cannot be null");
        }
        if (mJointsList == null) {
            throw new IllegalStateException("The joint list is already empty");
        }
        if (mJointsList.getJoint() == joint) {
            final JointListElement elementToRemove = mJointsList;
            mJointsList = elementToRemove.getNext();
        } else {
            JointListElement currentElement = mJointsList;
            while (currentElement.getNext() != null) {
                if (currentElement.getNext().getJoint() == joint) {
                    final JointListElement elementToRemove = currentElement.getNext();
                    currentElement.setNext(elementToRemove.getNext());
                    break;
                }
                currentElement = currentElement.getNext();
            }
        }
    }

    @Override
    public void setIsSleeping(boolean isSleeping) {
        if (isSleeping) {
            mLinearVelocity.setToZero();
            mAngularVelocity.setToZero();
            mExternalForce.setToZero();
            mExternalTorque.setToZero();
        }
        super.setIsSleeping(isSleeping);
    }

    /**
     * Applies an external force to the body at its gravity center. If the body is sleeping, calling this method will wake it up. Note that the force will be added to the sum of the applied forces and
     * that this sum will be reset to zero at the end of each call of the {@link com.flowpowered.react.engine.DynamicsWorld#update()} method.
     *
     * @param force The force to apply
     */
    public void applyForceToCenter(Vector3 force) {
        if (!mIsMotionEnabled) {
            return;
        }
        if (mIsSleeping) {
            setIsSleeping(false);
        }
        mExternalForce.add(force);
    }

    /**
     * Applies an external force to the body at a given point (in world-space coordinates). If the point is not at the center of gravity of the body, it will also generate some torque and therefore, change
     * the angular velocity of the body. If the body is sleeping, calling this method will wake it up. Note that the force will be added to the sum of the applied forces and that this sum will be
     * reset to zero at the end of each call of the {@link com.flowpowered.react.engine.DynamicsWorld#update()} method.
     *
     * @param force The force to apply
     * @param point The point to apply the force to
     */
    public void applyForce(Vector3 force, Vector3 point) {
        if (!mIsMotionEnabled) {
            return;
        }
        if (mIsSleeping) {
            setIsSleeping(false);
        }
        mExternalForce.add(force);
        mExternalTorque.add(Vector3.subtract(point, mTransform.getPosition()).cross(force));
    }

    /**
     * Applies an external torque to the body. If the body is sleeping, calling this method will wake it up. Note that the force will be added to the sum of the applied torques and that this sum will
     * be reset to zero at the end of each call of the {@link com.flowpowered.react.engine.DynamicsWorld#update()} method.
     *
     * @param torque The torque to apply
     */
    public void applyTorque(Vector3 torque) {

        // If it is a static body, do not apply any force
        if (!mIsMotionEnabled) {
            return;
        }

        // Awake the body if it was sleeping
        if (mIsSleeping) {
            setIsSleeping(false);
        }

        // Add the torque
        mExternalTorque.add(torque);
    }

    /**
     * Returns the total external force.
     *
     * @return The external force
     */
    public Vector3 getExternalForce() {
        return mExternalForce;
    }

    /**
     * Returns the total external torque.
     *
     * @return The external torque
     */
    public Vector3 getExternalTorque() {
        return mExternalTorque;
    }
}
