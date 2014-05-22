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
 * Represents a 3D box shape. Those axis are unit length. The three extents are half-lengths of the box along the three x, y, z local axes. The "transform" of the corresponding rigid body gives an
 * orientation and a position to the box.
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
    public Vector3 getLocalExtents() {
        return Vector3.add(mExtent, new Vector3(mMargin, mMargin, mMargin));
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
        return mExtent == otherShape.mExtent;
    }
}
