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
package org.spout.jreactphysics3d.body;

import org.spout.jreactphysics3d.collision.shape.AABB;
import org.spout.jreactphysics3d.collision.shape.CollisionShape;
import org.spout.jreactphysics3d.mathematics.Transform;

/**
 * Represents a body that is able to collide with others bodies. This class inherits from the Body
 * class.
 */
public class CollisionBody extends Body {
	protected CollisionShape mCollisionShape;
	protected final Transform mTransform = new Transform();
	protected final Transform mOldTransform = new Transform();
	protected float mInterpolationFactor;
	protected boolean mIsActive;
	protected boolean mIsMotionEnabled;
	protected boolean mIsCollisionEnabled;
	protected AABB mAabb;
	protected boolean mHasMoved;

	/**
	 * Constructs a new collision body from it's transform, collision shape, and ID.
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
		mTransform.set(transform);
		mCollisionShape = collisionShape;
		mIsActive = true;
		mHasMoved = false;
		mIsMotionEnabled = true;
		mIsCollisionEnabled = true;
		mInterpolationFactor = 0;
		mOldTransform.set(transform);
		mAabb = new AABB();
		mCollisionShape.updateAABB(mAabb, transform);
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
	 * Returns true if the body is active, false if not.
	 *
	 * @return Whether or not the body is active
	 */
	public boolean getIsActive() {
		return mIsActive;
	}

	/**
	 * Sets the activity for this body. True for active, false for inactive.
	 *
	 * @param mIsActive True if this body is active, false if not
	 */
	public void setIsActive(boolean mIsActive) {
		this.mIsActive = mIsActive;
	}

	/**
	 * Returns an interpolated body from the old to the current transform, based on this body's
	 * interpolation factor.
	 *
	 * @return A transform interpolated from the old to the current transform based on the
	 *         interpolation factor
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
	 * Returns true if the body can move, false if not.
	 *
	 * @return Whether or not the body can move
	 */
	public boolean getIsMotionEnabled() {
		return mIsMotionEnabled;
	}

	/**
	 * Sets whether or not this body can move. True to allow movement, false to disallow.
	 *
	 * @param isMotionEnabled True if the body should move, false if not
	 */
	public void setIsMotionEnabled(boolean isMotionEnabled) {
		mIsMotionEnabled = isMotionEnabled;
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
	 * Return true if the body can collide with others bodies.
	 *
	 * @return Whether or not this body can collide with others
	 */
	public boolean getIsCollisionEnabled() {
		return mIsCollisionEnabled;
	}

	/**
	 * Sets whether or not this body can collide. True to allow collisions, false to disallow.
	 *
	 * @param isCollisionEnabled True if the body should collide, false if not
	 */
	public void setIsCollisionEnabled(boolean isCollisionEnabled) {
		mIsCollisionEnabled = isCollisionEnabled;
	}

	/**
	 * Update the old transform with the current one. This is used to compute the interpolated position
	 * and orientation of the body.
	 */
	public void updateOldTransform() {
		mOldTransform.set(mTransform);
	}

	/**
	 * Update the rigid body in order to reflect a change in the body state.
	 */
	public void updateAABB() {
		// TODO : An AABB should not be updated every frame but only if the body has moved
		mCollisionShape.updateAABB(mAabb, mTransform);
	}
}
