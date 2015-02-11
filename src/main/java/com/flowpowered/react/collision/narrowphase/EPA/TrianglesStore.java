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
package com.flowpowered.react.collision.narrowphase.EPA;

import com.flowpowered.react.math.Vector3;

/**
 * This class stores several triangles for the polytope in the EPA algorithm. The max number of triangles is {@link #MAX_TRIANGLES}.
 */
public class TrianglesStore {
    public static final int MAX_TRIANGLES = 200;
    private final TriangleEPA[] mTriangles = new TriangleEPA[MAX_TRIANGLES];
    private int mNbTriangles = 0;

    /**
     * Clears all the triangles in the storage.
     */
    public void clear() {
        mNbTriangles = 0;
    }

    /**
     * Gets the number of triangles in the store.
     *
     * @return The number of triangles
     */
    public int getNbTriangles() {
        return mNbTriangles;
    }

    /**
     * Sets the number of triangles back to the specified number. This number should be smaller than the current number.
     *
     * @param backup The new number of triangles.
     */
    public void setNbTriangles(int backup) {
        mNbTriangles = backup;
    }

    /**
     * Gets the last triangle in the store. This will fail if there's no triangles in the store.
     *
     * @return The last triangle if the store
     * @throws IllegalStateException If there's no triangles in the store.
     */
    public TriangleEPA last() {
        if (mNbTriangles <= 0) {
            throw new IllegalStateException("nbTriangles must be greater than zero");
        }
        return mTriangles[mNbTriangles - 1];
    }

    /**
     * Creates and returns a new triangle, keeping it in the store. If the store is full or the triangle is invalid, this will return null.
     *
     * @param vertices The vertices of this triangle (three)
     * @param v0 The index of the first vertex
     * @param v1 The index of the second vertex
     * @param v2 The index of the third vertex
     * @return The new triangle or null if the creation failed or the store is full
     */
    public TriangleEPA newTriangle(Vector3[] vertices, int v0, int v1, int v2) {
        TriangleEPA newTriangle = null;
        if (mNbTriangles != MAX_TRIANGLES) {
            newTriangle = new TriangleEPA(v0, v1, v2);
            mTriangles[mNbTriangles++] = newTriangle;
            if (!newTriangle.computeClosestPoint(vertices)) {
                mNbTriangles--;
                newTriangle = null;
                mTriangles[mNbTriangles] = null;
            }
        }
        return newTriangle;
    }

    /**
     * Gets the triangle at the desired index in the store. This index should be smaller than {@link #MAX_TRIANGLES}.
     *
     * @param index The index
     * @return The triangle, or null if there's not triangle at the index
     */
    public TriangleEPA get(int index) {
        return mTriangles[index];
    }
}
