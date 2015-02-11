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
package com.flowpowered.react.collision.shape;

import com.flowpowered.react.math.Vector3;

/**
 * Represents a bounding volume of type "Axis Aligned Bounding Box". It's a box where all the edges are always aligned with the world coordinate system. The AABB is defined by the minimum and maximum
 * world coordinates of the three axis.
 */
public class AABB {
    private final Vector3 mMinCoordinates = new Vector3();
    private final Vector3 mMaxCoordinates = new Vector3();

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
        mMinCoordinates.set(minCoordinates);
        mMaxCoordinates.set(maxCoordinates);
    }

    /**
     * Gets the center of this AABB as a new vector3.
     *
     * @return The center vector3
     */
    public Vector3 getCenter() {
        return Vector3.multiply(Vector3.add(mMinCoordinates, mMaxCoordinates), 0.5f);
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
        mMinCoordinates.set(min);
    }

    /**
     * Sets the maximum vector3 of this AABB to the desired maximum.
     *
     * @param max the maximum vector3 to set
     */
    public void setMax(Vector3 max) {
        mMaxCoordinates.set(max);
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
