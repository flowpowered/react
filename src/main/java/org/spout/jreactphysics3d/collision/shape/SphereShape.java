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
package org.spout.jreactphysics3d.collision.shape;

import org.spout.jreactphysics3d.Configuration;
import org.spout.jreactphysics3d.mathematics.Matrix3x3;
import org.spout.jreactphysics3d.mathematics.Transform;
import org.spout.jreactphysics3d.mathematics.Vector3;

/**
 * Represents a sphere collision shape that is centered at the origin and defined by its radius.
 */
public class SphereShape extends CollisionShape {
	private float mRadius;

	/**
	 * Constructs a new sphere from the radius.
	 *
	 * @param radius The radius
	 */
	public SphereShape(float radius) {
		super(CollisionShapeType.SPHERE);
		mRadius = radius;
	}

	/**
	 * Gets the radius.
	 *
	 * @return The radius
	 */
	public float getRadius() {
		return mRadius;
	}

	/**
	 * Sets the radius.
	 *
	 * @param radius The radius
	 */
	public void setRadius(float radius) {
		this.mRadius = radius;
	}

	@Override
	public Vector3 getLocalSupportPointWithMargin(Vector3 direction) {
		final float margin = getMargin();
		if (direction.lengthSquare() >= Configuration.MACHINE_EPSILON * Configuration.MACHINE_EPSILON) {
			return Vector3.multiply(margin, direction.getUnit());
		}
		return new Vector3(0, margin, 0);
	}

	@Override
	public Vector3 getLocalSupportPointWithoutMargin(Vector3 direction) {
		return new Vector3(0, 0, 0);
	}

	@Override
	public Vector3 getLocalExtents(float margin) {
		return new Vector3(mRadius + margin, mRadius + margin, mRadius + margin);
	}

	@Override
	public float getMargin() {
		return mRadius + Configuration.OBJECT_MARGIN;
	}

	@Override
	public void computeLocalInertiaTensor(Matrix3x3 tensor, float mass) {
		final float diag = 0.4f * mass * mRadius * mRadius;
		tensor.setAllValues(
				diag, 0, 0,
				0, diag, 0,
				0, 0, diag);
	}

	@Override
	public void updateAABB(AABB aabb, Transform transform) {
		final Vector3 extents = getLocalExtents(Configuration.OBJECT_MARGIN);
		final Vector3 minCoordinates = Vector3.subtract(transform.getPosition(), extents);
		final Vector3 maxCoordinates = Vector3.add(transform.getPosition(), extents);
		aabb.setMin(minCoordinates);
		aabb.setMax(maxCoordinates);
	}
}
