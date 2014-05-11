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
     * Constructs a box shape from its extents which is half the vector between the two opposing corners that are the furthest away.
     *
     * @param x x extent
     * @param y y extent
     * @param z z extent
     */
    public BoxShape(float x, float y, float z) {
        super(CollisionShapeType.BOX);
        mExtent.setAllValues(x, y, z);
    }

    /**
     * Constructs a box shape from its extents which is half the vector between the two opposing corners that are the furthest away.
     *
     * @param extent The extent vector
     */
    public BoxShape(Vector3 extent) {
        super(CollisionShapeType.BOX);
        mExtent.set(extent);
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
        return mExtent;
    }

    /**
     * Sets the extent vector, which is half the vector between the two opposing corners that are the furthest away.
     *
     * @param extent The extents vector
     */
    public void setExtent(Vector3 extent) {
        mExtent.set(extent);
    }

    @Override
    public Vector3 getLocalSupportPointWithMargin(Vector3 direction) {
        final float margin = getMargin();
        if (margin < 0) {
            throw new IllegalStateException("margin must be greater than zero");
        }
        return new Vector3(
                direction.getX() < 0 ? -mExtent.getX() - margin : mExtent.getX() + margin,
                direction.getY() < 0 ? -mExtent.getY() - margin : mExtent.getY() + margin,
                direction.getZ() < 0 ? -mExtent.getZ() - margin : mExtent.getZ() + margin);
    }

    @Override
    public Vector3 getLocalSupportPointWithoutMargin(Vector3 direction) {
        return new Vector3(
                direction.getX() < 0 ? -mExtent.getX() : mExtent.getX(),
                direction.getY() < 0 ? -mExtent.getY() : mExtent.getY(),
                direction.getZ() < 0 ? -mExtent.getZ() : mExtent.getZ());
    }

    @Override
    public Vector3 getLocalExtents(float margin) {
        return Vector3.add(mExtent, new Vector3(margin, margin, margin));
    }

    @Override
    public float getMargin() {
        return ReactDefaults.OBJECT_MARGIN;
    }

    @Override
    public void computeLocalInertiaTensor(Matrix3x3 tensor, float mass) {
        final float factor = (1f / 3) * mass;
        final float xSquare = mExtent.getX() * mExtent.getX();
        final float ySquare = mExtent.getY() * mExtent.getY();
        final float zSquare = mExtent.getZ() * mExtent.getZ();
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
