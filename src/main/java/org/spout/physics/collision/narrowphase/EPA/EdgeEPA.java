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

import org.spout.physics.math.Vector3;

/**
 * Represents an edge for the current polytope in the EPA algorithm.
 */
public class EdgeEPA {
	private TriangleEPA mOwnerTriangle;
	private int mIndex;

	/**
	 * Default constructor. The owner triangle is null and the index is zero.
	 */
	public EdgeEPA() {
		this(null, 0);
	}

	/**
	 * Construct a new edge for the EPA from its owner triangle and index.
	 *
	 * @param ownerTriangle The owner triangle
	 * @param index The index
	 */
	public EdgeEPA(TriangleEPA ownerTriangle, int index) {
		if (index < 0 || index >= 3) {
			throw new IllegalArgumentException("index must be greater or equal to zero and smaller than three");
		}
		mOwnerTriangle = ownerTriangle;
		mIndex = index;
	}

	/**
	 * Copy constructor.
	 *
	 * @param edge The edge to copy
	 */
	public EdgeEPA(EdgeEPA edge) {
		mOwnerTriangle = edge.mOwnerTriangle;
		mIndex = edge.mIndex;
	}

	/**
	 * Gets the owner triangle.
	 *
	 * @return The owner triangle
	 */
	public TriangleEPA getOwnerTriangle() {
		return mOwnerTriangle;
	}

	/**
	 * Gets the edge index.
	 *
	 * @return The edge index
	 */
	public int getIndex() {
		return mIndex;
	}

	/**
	 * Sets the values of this edge to the ones of the provided edge.
	 *
	 * @param edge The edge to copy the values from
	 */
	public void set(EdgeEPA edge) {
		mOwnerTriangle = edge.mOwnerTriangle;
		mIndex = edge.mIndex;
	}

	/**
	 * Gets the index of the source vertex for the edge (vertex starting the edge).
	 *
	 * @return The index of the source vertex
	 */
	public int getSourceVertexIndex() {
		return mOwnerTriangle.get(mIndex);
	}

	/**
	 * Gets the index of the target vertex for the edge (vertex ending the edge).
	 *
	 * @return The index of the target vertex
	 */
	public int getTargetVertexIndex() {
		return mOwnerTriangle.get(indexOfNextCounterClockwiseEdge(mIndex));
	}

	/**
	 * Executes the recursive silhouette algorithm from this edge.
	 *
	 * @param vertices The vertices
	 * @param indexNewVertex The index of the new vertex
	 * @param triangleStore The triangle store
	 * @return True if the owner triangle was obsolete or if a new triangle edge was half liked with
	 *         this one
	 */
	public boolean computeSilhouette(Vector3[] vertices, int indexNewVertex, TrianglesStore triangleStore) {
		if (!mOwnerTriangle.isObsolete()) {
			if (!mOwnerTriangle.isVisibleFromVertex(vertices, indexNewVertex)) {
				TriangleEPA triangle = triangleStore.newTriangle(
						vertices,
						indexNewVertex,
						getTargetVertexIndex(),
						getSourceVertexIndex());
				if (triangle != null) {
					TriangleEPA.halfLink(new EdgeEPA(triangle, 1), this);
					return true;
				}
				return false;
			} else {
				mOwnerTriangle.setObsolete(true);
				int backup = triangleStore.getNbTriangles();
				if (!mOwnerTriangle.getAdjacentEdge(indexOfNextCounterClockwiseEdge(this.mIndex))
						.computeSilhouette(vertices, indexNewVertex, triangleStore)) {
					mOwnerTriangle.setObsolete(false);
					TriangleEPA triangle = triangleStore.newTriangle(
							vertices,
							indexNewVertex,
							getTargetVertexIndex(),
							getSourceVertexIndex());
					if (triangle != null) {
						TriangleEPA.halfLink(new EdgeEPA(triangle, 1), this);
						return true;
					}
					return false;
				} else if (!mOwnerTriangle.getAdjacentEdge(indexOfPreviousCounterClockwiseEdge(this.mIndex))
						.computeSilhouette(vertices, indexNewVertex, triangleStore)) {
					mOwnerTriangle.setObsolete(false);
					triangleStore.setNbTriangles(backup);
					TriangleEPA triangle = triangleStore.newTriangle(
							vertices,
							indexNewVertex,
							getTargetVertexIndex(),
							getSourceVertexIndex());
					if (triangle != null) {
						TriangleEPA.halfLink(new EdgeEPA(triangle, 1), this);
						return true;
					}
					return false;
				}
			}
		}
		return true;
	}

	// Returns the index of the next counter-clockwise edge of the owner triangle.
	private static int indexOfNextCounterClockwiseEdge(int i) {
		return (i + 1) % 3;
	}

	// Returns the index of the previous counter-clockwise edge of the owner triangle.
	private static int indexOfPreviousCounterClockwiseEdge(int i) {
		return (i + 2) % 3;
	}
}
