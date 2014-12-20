/*
 * This file is part of Flow React, licensed under the MIT License (MIT).
 *
 * Copyright (c) 2013 Flow Powered <https://flowpowered.com/>
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
package com.flowpowered.react.collision.shape;

import com.flowpowered.react.ReactDefaults;
import com.flowpowered.react.math.Matrix3x3;
import com.flowpowered.react.math.Transform;
import com.flowpowered.react.math.Vector3;

/**
 * Represents a sphere collision shape that is centered at the origin and defined by its radius. This collision shape does not have an explicit object margin distance. The margin is implicitly the
 * radius of the sphere. Therefore, there is no need to specify an object margin for a sphere shape.
 */
public class SphereShape extends CollisionShape {
    private final float mRadius;

    /**
     * Constructs a new sphere from the radius.
     *
     * @param radius The radius
     */
    public SphereShape(float radius) {
        super(CollisionShapeType.SPHERE, radius);
        mRadius = radius;
        if (radius <= 0) {
            throw new IllegalArgumentException("Radius must be greater than zero");
        }
    }

    /**
     * Copy constructor.
     *
     * @param shape The shape to copy
     */
    public SphereShape(SphereShape shape) {
        super(shape);
        mRadius = shape.mRadius;
    }

    /**
     * Gets the radius.
     *
     * @return The radius
     */
    public float getRadius() {
        return mRadius;
    }

    @Override
    public Vector3 getLocalSupportPointWithMargin(Vector3 direction) {
        if (direction.lengthSquare() >= ReactDefaults.MACHINE_EPSILON * ReactDefaults.MACHINE_EPSILON) {
            return Vector3.multiply(mMargin, direction.getUnit());
        }
        return new Vector3(0, mMargin, 0);
    }

    @Override
    public Vector3 getLocalSupportPointWithoutMargin(Vector3 direction) {
        return new Vector3(0, 0, 0);
    }

    @Override
    public void getLocalBounds(Vector3 min, Vector3 max) {
        max.setX(mRadius);
        max.setY(mRadius);
        max.setZ(mRadius);
        min.setX(-mRadius);
        min.setY(min.getX());
        min.setZ(min.getX());
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
        final Vector3 extents = new Vector3(mRadius, mRadius, mRadius);
        aabb.setMin(Vector3.subtract(transform.getPosition(), extents));
        aabb.setMax(Vector3.add(transform.getPosition(), extents));
    }

    @Override
    public SphereShape clone() {
        return new SphereShape(this);
    }

    @Override
    public boolean isEqualTo(CollisionShape otherCollisionShape) {
        final SphereShape otherShape = (SphereShape) otherCollisionShape;
        return mRadius == otherShape.mRadius;
    }
}
