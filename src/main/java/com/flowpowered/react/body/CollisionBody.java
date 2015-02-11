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

import com.flowpowered.react.collision.shape.AABB;
import com.flowpowered.react.collision.shape.CollisionShape;
import com.flowpowered.react.engine.ContactManifold.ContactManifoldListElement;
import com.flowpowered.react.math.Transform;

/**
 * Represents a body that is able to collide with others bodies. This class inherits from the Body class.
 */
public class CollisionBody extends Body {
    protected CollisionShape mCollisionShape;
    protected final Transform mTransform;
    protected final Transform mOldTransform;
    protected float mInterpolationFactor;
    protected boolean mIsMotionEnabled;
    protected boolean mIsCollisionEnabled;
    protected final AABB mAabb = new AABB();
    protected boolean mHasMoved;
    protected ContactManifoldListElement mContactManifoldsList;

    /**
     * Constructs a new collision body from its transform, collision shape, and ID.
     *
     * @param transform The transform
     * @param collisionShape The collision shape
     * @param id The ID
     */
    public CollisionBody(Transform transform, CollisionShape collisionShape, int id) {
        super(id);
        if (collisionShape == null) {
            throw new IllegalArgumentException("collisionShape cannot be null");
        }
        mCollisionShape = collisionShape;
        mTransform = transform;
        mHasMoved = false;
        mIsMotionEnabled = true;
        mIsCollisionEnabled = true;
        mInterpolationFactor = 0;
        mOldTransform = transform;
        mCollisionShape.updateAABB(mAabb, transform);
        mContactManifoldsList = null;
    }

    /**
     * Returns true if the body has moved during the last frame, false if not.
     *
     * @return Whether or not the body has moved in the last frame
     */
    public boolean getHasMoved() {
        return mHasMoved;
    }

    /**
     * Set the hasMoved variable (true if the body has moved during the last frame).
     *
     * @param mHasMoved Whether or not the body has moved in the last frame
     */
    public void setHasMoved(boolean mHasMoved) {
        this.mHasMoved = mHasMoved;
    }

    /**
     * Gets the collision shape.
     *
     * @return The collision shape
     */
    public CollisionShape getCollisionShape() {
        return mCollisionShape;
    }

    /**
     * Sets the collision shape
     *
     * @param mCollisionShape The collision shape to set
     */
    public void setCollisionShape(CollisionShape mCollisionShape) {
        this.mCollisionShape = mCollisionShape;
    }

    /**
     * Returns an interpolated body from the old to the current transform, based on this body's interpolation factor.
     *
     * @return A transform interpolated from the old to the current transform based on the interpolation factor
     */
    public Transform getInterpolatedTransform() {
        return Transform.interpolateTransforms(mOldTransform, mTransform, mInterpolationFactor);
    }

    /**
     * Sets the interpolation factor for the transforms of this body.
     *
     * @param factor The interpolation factor
     */
    public void setInterpolationFactor(float factor) {
        mInterpolationFactor = factor;
    }

    /**
     * Gets this body's current position and orientation as a transform.
     *
     * @return The body's transform
     */
    public Transform getTransform() {
        return mTransform;
    }

    /**
     * Sets this body's current position and orientation as a transform.
     *
     * @param transform The transform to set for this body
     */
    public void setTransform(Transform transform) {
        if (!mTransform.equals(transform)) {
            mHasMoved = true;
        }
        mTransform.set(transform);
    }

    /**
     * Gets this body's axis-aligned bounding box (AABB).
     *
     * @return The body's AABB
     */
    public AABB getAABB() {
        return mAabb;
    }

    /**
     * Returns true if the body can move, false if not.
     *
     * @return Whether or not the body can move
     */
    public boolean isMotionEnabled() {
        return mIsMotionEnabled;
    }

    /**
     * Sets whether or not this body can move. True to allow movement, false to disallow.
     *
     * @param isMotionEnabled True if the body should move, false if not
     */
    public void enableMotion(boolean isMotionEnabled) {
        mIsMotionEnabled = isMotionEnabled;
    }

    /**
     * Return true if the body can collide with others bodies.
     *
     * @return Whether or not this body can collide with others
     */
    public boolean isCollisionEnabled() {
        return mIsCollisionEnabled;
    }

    /**
     * Sets whether or not this body can collide. True to allow collisions, false to disallow.
     *
     * @param isCollisionEnabled True if the body should collide, false if not
     */
    public void enableCollision(boolean isCollisionEnabled) {
        mIsCollisionEnabled = isCollisionEnabled;
    }

    /**
     * Update the old transform with the current one. This is used to compute the interpolated position and orientation of the body.
     */
    public void updateOldTransform() {
        mOldTransform.set(mTransform);
    }

    /**
     * Update the rigid body in order to reflect a change in the body state.
     */
    public void updateAABB() {
        if (mHasMoved) {
            mCollisionShape.updateAABB(mAabb, mTransform);
        }
    }

    /**
     * Returns the first element of the linked list of contact manifolds involving this body.
     *
     * @return The first element of the list
     */
    public ContactManifoldListElement getContactManifoldsLists() {
        return mContactManifoldsList;
    }

    /**
     * Sets the first element in the contact element list, discarding the entire list.
     *
     * @param contactManifoldsList The first element in the list
     */
    public void setContactManifoldsList(ContactManifoldListElement contactManifoldsList) {
        mContactManifoldsList = contactManifoldsList;
    }

    /**
     * Resets the contact manifold lists.
     */
    public void resetContactManifoldsList() {
        mContactManifoldsList = null;
    }
}
