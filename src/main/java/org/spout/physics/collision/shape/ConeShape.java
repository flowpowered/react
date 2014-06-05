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
 * Represents a cone collision shape centered at the origin and aligned with the Y axis. The cone is defined by its height and by the radius of its base. The center of the cone is at the half of the
 * height. The "transform" of the corresponding rigid body gives an orientation and a position to the cone. This collision shape uses an extra margin distance around it for collision detection
 * purpose. The default margin is 4cm (if your units are meters, which is recommended). In case, you want to simulate small objects (smaller than the margin distance), you might want to reduce the
 * margin by specifying your own margin distance using the "margin" parameter in the constructor of the cone shape. Otherwise, it is recommended to use the default margin distance by not using the
 * "margin" parameter in the constructor.
 */
public class ConeShape extends CollisionShape {
    private final float mRadius;
    private final float mHalfHeight;
    private final float mSinTheta;

    /**
     * Constructs a new cone shape from the radius of the base and the height.
     *
     * @param radius The radius of the base
     * @param height The height
     */
    public ConeShape(float radius, float height) {
        this(radius, height, ReactDefaults.OBJECT_MARGIN);
    }

    /**
     * Constructs a new cone shape from the radius of the base and the height and the AABB margin.
     *
     * @param radius The radius of the base
     * @param height The height
     * @param margin The margin
     */
    public ConeShape(float radius, float height, float margin) {
        super(CollisionShapeType.CONE, margin);
        mRadius = radius;
        mHalfHeight = height * 0.5f;
        if (mRadius <= 0) {
            throw new IllegalArgumentException("Radius must be greater than zero");
        }
        if (mHalfHeight <= 0) {
            throw new IllegalArgumentException("Height must be greater than zero");
        }
        if (margin <= 0) {
            throw new IllegalArgumentException("Margin must be greater than 0");
        }
        mSinTheta = mRadius / (float) Math.sqrt(mRadius * mRadius + height * height);
    }

    /**
     * Copy constructor.
     *
     * @param shape The shape to copy
     */
    public ConeShape(ConeShape shape) {
        super(shape);
        mRadius = shape.mRadius;
        mHalfHeight = shape.mHalfHeight;
        mSinTheta = shape.mSinTheta;
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
     * Gets the height of the cone.
     *
     * @return The height
     */
    public float getHeight() {
        return mHalfHeight + mHalfHeight;
    }

    @Override
    public Vector3 getLocalSupportPointWithMargin(Vector3 direction) {
        final Vector3 supportPoint = getLocalSupportPointWithoutMargin(direction);
        final Vector3 unitVec;
        if (direction.lengthSquare() > ReactDefaults.MACHINE_EPSILON * ReactDefaults.MACHINE_EPSILON) {
            unitVec = direction.getUnit();
        } else {
            unitVec = new Vector3(0, -1, 0);
        }
        supportPoint.add(Vector3.multiply(unitVec, mMargin));
        return supportPoint;
    }

    @Override
    public Vector3 getLocalSupportPointWithoutMargin(Vector3 direction) {
        final Vector3 v = direction;
        final float sinThetaTimesLengthV = mSinTheta * v.length();
        final Vector3 supportPoint;
        if (v.getY() > sinThetaTimesLengthV) {
            supportPoint = new Vector3(0, mHalfHeight, 0);
        } else {
            final float projectedLength = (float) Math.sqrt(v.getX() * v.getX() + v.getZ() * v.getZ());
            if (projectedLength > ReactDefaults.MACHINE_EPSILON) {
                final float d = mRadius / projectedLength;
                supportPoint = new Vector3(v.getX() * d, -mHalfHeight, v.getZ() * d);
            } else {
                supportPoint = new Vector3(0, -mHalfHeight, 0);
            }
        }
        return supportPoint;
    }

    @Override
    public void getLocalBounds(Vector3 min, Vector3 max) {
        max.setX(mRadius + mMargin);
        max.setY(mHalfHeight + mMargin);
        max.setZ(max.getX());
        min.setX(-max.getX());
        min.setY(-max.getY());
        min.setZ(min.getX());
    }

    @Override
    public void computeLocalInertiaTensor(Matrix3x3 tensor, float mass) {
        final float rSquare = mRadius * mRadius;
        final float diagXZ = 0.15f * mass * (rSquare + mHalfHeight);
        tensor.setAllValues(
                diagXZ, 0, 0,
                0, 0.3f * mass * rSquare, 0,
                0, 0, diagXZ);
    }

    @Override
    public ConeShape clone() {
        return new ConeShape(this);
    }

    @Override
    public boolean isEqualTo(CollisionShape otherCollisionShape) {
        final ConeShape otherShape = (ConeShape) otherCollisionShape;
        return mRadius == otherShape.mRadius && mHalfHeight == otherShape.mHalfHeight;
    }
}
