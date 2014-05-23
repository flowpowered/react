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

import org.spout.physics.ReactDefaults;
import org.spout.physics.math.Matrix3x3;
import org.spout.physics.math.Vector3;

/**
 * Represents a capsule collision shape that is defined around the Y axis. A capsule shape can be seen as the convex hull of two spheres. The capsule shape is defined by its radius (radius of the two
 * spheres of the capsule) and its height (distance between the centers of the two spheres). This collision shape does not have an explicit object margin distance. The margin is implicitly the radius
 * and height of the shape. Therefore, there is no need to specify an object margin for a capsule shape.
 */
public class CapsuleShape extends CollisionShape {
    private final float mRadius;
    private final float mHalfHeight;

    /**
     * Constructs a new capsule shape from its radius and height.
     *
     * @param radius The radius
     * @param height The height
     */
    public CapsuleShape(float radius, float height) {
        super(CollisionShapeType.CAPSULE, radius);
        mRadius = radius;
        mHalfHeight = height * 0.5f;
        if (radius <= 0) {
            throw new IllegalArgumentException("Radius must be greater than zero");
        }
        if (height <= 0) {
            throw new IllegalArgumentException("Height must be greater than zero");
        }
    }

    /**
     * Copy constructor.
     *
     * @param shape The shape to copy
     */
    public CapsuleShape(CapsuleShape shape) {
        super(shape);
        mRadius = shape.mRadius;
        mHalfHeight = shape.mHalfHeight;
    }

    /**
     * Returns the radius of the spherical ends of the capsule.
     *
     * @return The radius of the capsule
     */
    public float getRadius() {
        return mRadius;
    }

    /**
     * Returns the distance between the middle of the two hemispheres.
     *
     * @return The height of the capsule
     */
    public float getHeight() {
        return mHalfHeight + mHalfHeight;
    }

    @Override
    public Vector3 getLocalSupportPointWithMargin(Vector3 direction) {
        if (direction.lengthSquare() >= ReactDefaults.MACHINE_EPSILON * ReactDefaults.MACHINE_EPSILON) {
            final Vector3 unitDirection = direction.getUnit();
            final Vector3 centerTopSphere = new Vector3(0, mHalfHeight, 0);
            final Vector3 topSpherePoint = Vector3.add(centerTopSphere, Vector3.multiply(unitDirection, mRadius));
            final float dotProductTop = topSpherePoint.dot(direction);
            final Vector3 centerBottomSphere = new Vector3(0, -mHalfHeight, 0);
            final Vector3 bottomSpherePoint = Vector3.add(centerBottomSphere, Vector3.multiply(unitDirection, mRadius));
            final float dotProductBottom = bottomSpherePoint.dot(direction);
            if (dotProductTop > dotProductBottom) {
                return topSpherePoint;
            } else {
                return bottomSpherePoint;
            }
        }
        return new Vector3(0, mRadius, 0);
    }

    @Override
    public Vector3 getLocalSupportPointWithoutMargin(Vector3 direction) {
        if (direction.getY() > 0) {
            return new Vector3(0, mHalfHeight, 0);
        } else {
            return new Vector3(0, -mHalfHeight, 0);
        }
    }

    @Override
    public Vector3 getLocalExtents() {
        return new Vector3(mRadius, mHalfHeight + mRadius, mRadius);
    }

    @Override
    public void computeLocalInertiaTensor(Matrix3x3 tensor, float mass) {
        final float height = mHalfHeight + mHalfHeight;
        final float radiusSquare = mRadius * mRadius;
        final float heightSquare = height * height;
        final float radiusSquareDouble = radiusSquare + radiusSquare;
        final float factor1 = 2 * mRadius / (4 * mRadius + 3 * height);
        final float factor2 = 3 * height / (4 * mRadius + 3 * height);
        final float sum1 = 0.4f * radiusSquareDouble;
        final float sum2 = 0.75f * height * mRadius + 0.5f * heightSquare;
        final float sum3 = 0.25f * radiusSquare + 1f / 12 * heightSquare;
        final float IxxAndzz = factor1 * mass * (sum1 + sum2) + factor2 * mass * sum3;
        final float Iyy = factor1 * mass * sum1 + factor2 * mass * 0.25f * radiusSquareDouble;
        tensor.setAllValues(
                IxxAndzz, 0, 0,
                0, Iyy, 0,
                0, 0, IxxAndzz);
    }

    @Override
    public CollisionShape clone() {
        return new CapsuleShape(this);
    }

    @Override
    public boolean isEqualTo(CollisionShape otherCollisionShape) {
        final CapsuleShape otherShape = (CapsuleShape) otherCollisionShape;
        return mRadius == otherShape.mRadius && mHalfHeight == otherShape.mHalfHeight;
    }
}
