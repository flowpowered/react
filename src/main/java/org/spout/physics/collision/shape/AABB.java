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
package org.spout.physics.collision.shape;

import org.spout.math.vector.Vector3;

/**
 * Represents a bounding volume of type "Axis Aligned Bounding Box". It's a box where all the edges
 * are always aligned with the world coordinate system. The AABB is defined by the minimum and
 * maximum world coordinates of the three axis.
 */
public class AABB {
	private Vector3 mMinCoordinates;
	private Vector3 mMaxCoordinates;

	/**
	 * Default constructor. Min and max are the both zero vector3s.
	 */
	public AABB() {
	}

	/**
	 * Constructs a new AABB with the specified min and max vector3s.
	 *
	 * @param minCoordinates The minimum vector3
	 * @param maxCoordinates The maximum vector3
	 */
	public AABB(Vector3 minCoordinates, Vector3 maxCoordinates) {
		mMinCoordinates = minCoordinates;
		mMaxCoordinates = maxCoordinates;
	}

	/**
	 * Gets the center of this AABB as a new vector3.
	 *
	 * @return The center vector3
	 */
	public Vector3 getCenter() {
		return mMinCoordinates.add(mMaxCoordinates).mul(0.5f);
	}

	/**
	 * Gets the minimum vector3 of this AABB.
	 *
	 * @return The minimum vector3
	 */
	public Vector3 getMin() {
		return mMinCoordinates;
	}

	/**
	 * Gets the maximum vector3 of this AABB.
	 *
	 * @return The maximum vector3
	 */
	public Vector3 getMax() {
		return mMaxCoordinates;
	}

	/**
	 * Sets the minimum vector3 of this AABB to the desired minimum.
	 *
	 * @param min the minimum vector3 to set
	 */
	public void setMin(Vector3 min) {
		mMinCoordinates = min;
	}

	/**
	 * Sets the maximum vector3 of this AABB to the desired maximum.
	 *
	 * @param max the maximum vector3 to set
	 */
	public void setMax(Vector3 max) {
		mMaxCoordinates = max;
	}

	/**
	 * Test whether or not two AABBs are colliding. Returns true if they are colliding, false if not.
	 *
	 * @param aabb The AABB to test for collision
	 * @return True if the AABBs are colliding, false if not
	 */
	public boolean testCollision(AABB aabb) {
		if (mMaxCoordinates.getX() < aabb.getMin().getX()
				|| aabb.getMax().getX() < mMinCoordinates.getX()) {
			return false;
		}
		if (mMaxCoordinates.getY() < aabb.getMin().getY()
				|| aabb.getMax().getY() < mMinCoordinates.getY()) {
			return false;
		}
		if (mMaxCoordinates.getZ() < aabb.getMin().getZ()
				|| aabb.getMax().getZ() < mMinCoordinates.getZ()) {
			return false;
		}
		return true;
	}

	@Override
	public String toString() {
		return "AABB{min= " + mMinCoordinates + ", max= " + mMaxCoordinates + "}";
	}
}
