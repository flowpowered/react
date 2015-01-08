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
import com.flowpowered.react.math.Vector3;

/**
 * Represents a 3D box shape. Those axis are unit length. The three extents are half-lengths of the box along the three x, y, z local axes. The "transform" of the corresponding rigid body will give an
 * orientation and a position to the box. This collision shape uses an extra margin distance around it for collision detection purpose. The default margin is 4cm (if your units are meters, which is
 * recommended). In case, you want to simulate small objects (smaller than the margin distance), you might want to reduce the margin by specifying your own margin distance using the "margin" parameter
 * in the constructor of the box shape. Otherwise, it is recommended to use the default margin distance by not using the "margin" parameter in the constructor.
 */
public class BoxShape extends CollisionShape {
    private final Vector3 mExtent = new Vector3();

    /**
     * Constructs a box shape from the components of its extents which is half the vector between the two opposing corners that are the furthest away.
     *
     * @param x The x extent
     * @param y The y extent
     * @param z The z extent
     */
    public BoxShape(float x, float y, float z) {
        this(new Vector3(x, y, z), ReactDefaults.OBJECT_MARGIN);
    }

    /**
     * Constructs a box shape from its extents which is half the vector between the two opposing corners that are the furthest away.
     *
     * @param extent The extent vector
     */
    public BoxShape(Vector3 extent) {
        this(extent, ReactDefaults.OBJECT_MARGIN);
    }

    /**
     * Constructs a box shape from its extents which is half the vector between the two opposing corners that are the furthest away and the AABB margin.
     *
     * @param extent The extent vector
     * @param margin The margin
     */
    public BoxShape(Vector3 extent, float margin) {
        super(CollisionShapeType.BOX, margin);
        mExtent.set(Vector3.subtract(extent, new Vector3(margin, margin, margin)));
        if (extent.getX() <= 0 || extent.getX() <= margin) {
            throw new IllegalArgumentException("Extent x coordinate must be greater than 0 and the margin");
        }
        if (extent.getY() <= 0 || extent.getY() <= margin) {
            throw new IllegalArgumentException("Extent y coordinate must be greater than 0 and the margin");
        }
        if (extent.getZ() <= 0 || extent.getZ() <= margin) {
            throw new IllegalArgumentException("Extent z coordinate must be greater than 0 and the margin");
        }
        if (margin <= 0) {
            throw new IllegalArgumentException("Margin must be greater than 0");
        }
    }

    /**
     * Copy constructor.
     *
     * @param shape The shape to copy
     */
    public BoxShape(BoxShape shape) {
        super(shape);
        mExtent.set(shape.mExtent);
    }

    /**
     * Gets the extent vector, which is half the vector between the two opposing corners that are the furthest away.
     *
     * @return The extents vector
     */
    public Vector3 getExtent() {
        return Vector3.add(mExtent, new Vector3(mMargin, mMargin, mMargin));
    }

    @Override
    public Vector3 getLocalSupportPointWithMargin(Vector3 direction) {
        if (mMargin < 0) {
            throw new IllegalStateException("margin must be greater than zero");
        }
        return new Vector3(
                direction.getX() < 0 ? -mExtent.getX() - mMargin : mExtent.getX() + mMargin,
                direction.getY() < 0 ? -mExtent.getY() - mMargin : mExtent.getY() + mMargin,
                direction.getZ() < 0 ? -mExtent.getZ() - mMargin : mExtent.getZ() + mMargin);
    }

    @Override
    public Vector3 getLocalSupportPointWithoutMargin(Vector3 direction) {
        return new Vector3(
                direction.getX() < 0 ? -mExtent.getX() : mExtent.getX(),
                direction.getY() < 0 ? -mExtent.getY() : mExtent.getY(),
                direction.getZ() < 0 ? -mExtent.getZ() : mExtent.getZ());
    }

    @Override
    public void getLocalBounds(Vector3 min, Vector3 max) {
        max.set(Vector3.add(mExtent, new Vector3(mMargin, mMargin, mMargin)));
        min.set(Vector3.negate(max));
    }

    @Override
    public void computeLocalInertiaTensor(Matrix3x3 tensor, float mass) {
        final float factor = (1f / 3) * mass;
        Vector3 realExtent = Vector3.add(mExtent, new Vector3(mMargin, mMargin, mMargin));
        final float xSquare = realExtent.getX() * realExtent.getX();
        final float ySquare = realExtent.getY() * realExtent.getY();
        final float zSquare = realExtent.getZ() * realExtent.getZ();
        tensor.setAllValues(
                factor * (ySquare + zSquare), 0, 0,
                0, factor * (xSquare + zSquare), 0,
                0, 0, factor * (xSquare + ySquare));
    }

    @Override
    public BoxShape clone() {
        return new BoxShape(this);
    }

    @Override
    public boolean isEqualTo(CollisionShape otherCollisionShape) {
        final BoxShape otherShape = (BoxShape) otherCollisionShape;
        return mExtent.equals(otherShape.mExtent);
    }
}
