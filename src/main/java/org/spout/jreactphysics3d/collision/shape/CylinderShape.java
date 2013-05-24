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
import org.spout.jreactphysics3d.mathematics.Vector3;

/**
 * Represents a cylinder collision shape around the Y axis and centered at the origin. The cylinder
 * is defined by its height and the radius of its base. The "transform" of the corresponding rigid
 * body gives an orientation and a position to the cylinder.
 */
public class CylinderShape extends CollisionShape {
	private float mRadius;
	private float mHalfHeight;

	/**
	 * Constructs a new cylinder from the radius of the base and the height.
	 *
	 * @param radius The radius of the base
	 * @param height The height
	 */
	public CylinderShape(float radius, float height) {
		super(CollisionShapeType.CYLINDER);
		mRadius = radius;
		mHalfHeight = height / 2;
	}

	/**
	 * Gets the radius of the base.
	 *
	 * @return The radius
	 */
	public float getRadius() {
		return mRadius;
	}

	/**
	 * Gets the height of the cylinder.
	 *
	 * @return The height
	 */
	public float getHeight() {
		return mHalfHeight * 2;
	}

	/**
	 * Sets the radius of the base.
	 *
	 * @param radius The radius to set
	 */
	public void setRadius(float radius) {
		this.mRadius = radius;
	}

	/**
	 * Sets the height of the cylinder.
	 *
	 * @param height The height of the cylinder to set
	 */
	public void setHeight(float height) {
		this.mHalfHeight = height * 0.5f;
	}

	@Override
	public Vector3 getLocalSupportPointWithMargin(Vector3 direction) {
		final Vector3 supportPoint = getLocalSupportPointWithoutMargin(direction);
		final Vector3 unitVec;
		if (direction.lengthSquare() > Configuration.MACHINE_EPSILON * Configuration.MACHINE_EPSILON) {
			unitVec = direction.getUnit();
		} else {
			unitVec = new Vector3(0, 1, 0);
		}
		supportPoint.add(Vector3.multiply(unitVec, getMargin()));
		return supportPoint;
	}

	@Override
	public Vector3 getLocalSupportPointWithoutMargin(Vector3 direction) {
		final Vector3 supportPoint = new Vector3(0, 0, 0);
		final float uDotv = direction.getY();
		final Vector3 w = new Vector3(direction.getX(), 0, direction.getZ());
		final float lengthW = (float) Math.sqrt(direction.getX() * direction.getX() + direction.getZ() * direction.getZ());
		if (lengthW > Configuration.MACHINE_EPSILON) {
			if (uDotv < 0.0) {
				supportPoint.setY(-mHalfHeight);
			} else {
				supportPoint.setY(mHalfHeight);
				supportPoint.add(Vector3.multiply(mRadius / lengthW, w));
			}
		} else {
			if (uDotv < 0.0) {
				supportPoint.setY(-mHalfHeight);
			} else {
				supportPoint.setY(mHalfHeight);
			}
		}
		return supportPoint;
	}

	@Override
	public Vector3 getLocalExtents(float margin) {
		return new Vector3(mRadius + margin, mHalfHeight + margin, mRadius + margin);
	}

	@Override
	public float getMargin() {
		return Configuration.OBJECT_MARGIN;
	}

	@Override
	public void computeLocalInertiaTensor(Matrix3x3 tensor, float mass) {
		final float height = 2 * mHalfHeight;
		final float diag = (1f / 12) * mass * (3 * mRadius * mRadius + height * height);
		tensor.setAllValues(
				diag, 0, 0,
				0, 0.5f * mass * mRadius * mRadius, 0,
				0, 0, diag);
	}
}
