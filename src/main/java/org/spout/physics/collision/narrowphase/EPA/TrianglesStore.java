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
package org.spout.physics.collision.narrowphase.EPA;

import org.spout.math.vector.Vector3;

/**
 * This class stores several triangles for the polytope in the EPA algorithm. The max number of
 * triangles is {@link #MAX_TRIANGLES}.
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
	 * Sets the number of triangles back to the specified number. This number should be smaller than
	 * the current number.
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
	 * Creates and returns a new triangle, keeping it in the store. If the store is full or the
	 * triangle is invalid, this will return null.
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
	 * Gets the triangle at the desired index in the store. This index should be smaller than {@link
	 * #MAX_TRIANGLES}.
	 *
	 * @param index The index
	 * @return The triangle, or null if there's not triangle at the index
	 */
	public TriangleEPA get(int index) {
		return mTriangles[index];
	}
}
