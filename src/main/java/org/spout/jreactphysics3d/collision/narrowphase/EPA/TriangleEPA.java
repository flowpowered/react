/*
 * This file is part of JReactPhysics3D.
 *
 * Copyright (c) 2013 Spout LLC <http://www.spout.org/>
 * JReactPhysics3D is licensed under the Spout License Version 1.
 *
 * JReactPhysics3D is free software: you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * In addition, 180 days after any changes are published, you can use the
 * software, incorporating those changes, under the terms of the MIT license,
 * as described in the Spout License Version 1.
 *
 * JReactPhysics3D is distributed in the hope that it will be useful, but WITHOUT ANY
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
package org.spout.jreactphysics3d.collision.narrowphase.EPA;

import org.spout.jreactphysics3d.mathematics.Vector3;

/**
 * Represents a triangle face for the current polytope in the EPA algorithm.
 */
public class TriangleEPA {
	private final int[] mIndicesVertices = new int[3];
	private final EdgeEPA[] mAdjacentEdges = new EdgeEPA[3];
	private boolean mIsObsolete;
	private float mDet;
	private final Vector3 mClosestPoint = new Vector3();
	private float mLambda1;
	private float mLambda2;
	private float mDistSquare;

	/**
	 * Default constructor for the triangle. It contains to vertices or edges and the closest point is
	 * the zero vector. The triangle is not flagged as obsolete.
	 */
	public TriangleEPA() {
	}

	/**
	 * Constructs a new triangle from the three vertex indices. This triangle is not flagged as
	 * obsolete.
	 *
	 * @param indexVertex1 The first vertex index
	 * @param indexVertex2 The second vertex index
	 * @param indexVertex3 The third vertex index
	 */
	public TriangleEPA(int indexVertex1, int indexVertex2, int indexVertex3) {
		mIsObsolete = false;
		mIndicesVertices[0] = indexVertex1;
		mIndicesVertices[1] = indexVertex2;
		mIndicesVertices[2] = indexVertex3;
	}

	/**
	 * Gets an edge the triangle at the desired index. The index must be greater or equal to zero and
	 * smaller than three.
	 *
	 * @param index An index in the [0,3[ range
	 * @return The edge at the index
	 */
	public EdgeEPA getAdjacentEdge(int index) {
		if (index < 0 || index >= 3) {
			throw new IllegalArgumentException("index must be greater or equal to zero and smaller than three");
		}
		return mAdjacentEdges[index];
	}

	/**
	 * Sets an adjacent edge of the triangle to the desired value at the specified index. The index
	 * must be greater or equal to zero and smaller than three.
	 *
	 * @param index An index in the [0,3[ range
	 * @param edge The edge to set
	 */
	public void setAdjacentEdge(int index, EdgeEPA edge) {
		if (index < 0 || index >= 3) {
			throw new IllegalArgumentException("index must be greater or equal to zero and smaller than three");
		}
		mAdjacentEdges[index] = edge;
	}

	/**
	 * Gets the square of the distance from the closest point to the origin.
	 *
	 * @return The distance
	 */
	public float getDistSquare() {
		return mDistSquare;
	}

	/**
	 * Sets if this triangle is obsolete.
	 *
	 * @param isObsolete True if the triangle is obsolete, false if not
	 */
	public void setIsObsolete(boolean isObsolete) {
		mIsObsolete = isObsolete;
	}

	/**
	 * Returns true if the triangle face is obsolete, false if not.
	 *
	 * @return True if the triangle is obsolete, false if not
	 */
	public boolean getIsObsolete() {
		return mIsObsolete;
	}

	/**
	 * Gets the point closest to the origin.
	 *
	 * @return The closest point to the origin
	 */
	public Vector3 getClosestPoint() {
		return mClosestPoint;
	}

	/**
	 * Returns true if the closest point on the affine hull is inside the triangle.
	 *
	 * @return True if the point is inside the triangle, false if not
	 */
	public boolean isClosestPointInternalToTriangle() {
		return (mLambda1 >= 0 && mLambda2 >= 0 && (mLambda1 + mLambda2) <= mDet);
	}

	/**
	 * Returns true if the triangle is visible from a given vertex
	 *
	 * @param vertices The array containing the vertex to check
	 * @param index The index of the vertex to check in the array
	 * @return True if the triangle is visible, false if not
	 */
	public boolean isVisibleFromVertex(Vector3[] vertices, int index) {
		final Vector3 closestToVert = Vector3.subtract(vertices[index], mClosestPoint);
		return (mClosestPoint.dot(closestToVert) > 0);
	}

	/**
	 * Computes the point of the object closest to the origin and returns it.
	 *
	 * @param supportPointsOfObject The points of the object
	 * @return The closes point to the origin
	 */
	public Vector3 computeClosestPointOfObject(Vector3[] supportPointsOfObject) {
		final Vector3 p0 = supportPointsOfObject[mIndicesVertices[0]];
		return Vector3.add(p0, Vector3.multiply(1 / mDet, Vector3.add(
				Vector3.multiply(mLambda1, Vector3.subtract(supportPointsOfObject[mIndicesVertices[1]], p0)),
				Vector3.multiply(mLambda2, Vector3.subtract(supportPointsOfObject[mIndicesVertices[2]], p0))
		)));
	}

	/**
	 * Gets the vertex index at the desired index. The index must be greater or equal to zero and
	 * smaller than three.
	 *
	 * @param index An index in the [0,3[ range
	 * @return The vertex index at the index
	 */
	public int get(int index) {
		if (index < 0 || index >= 3) {
			throw new IllegalArgumentException("index must be greater or equal to zero and smaller than three");
		}
		return mIndicesVertices[index];
	}

	/**
	 * Computes the point that is the closest to the origin of this triangle for the given vertices.
	 *
	 * @param vertices The vertices to compute with
	 * @return True if the computation was successful, false if not
	 */
	public boolean computeClosestPoint(Vector3[] vertices) {
		final Vector3 p0 = vertices[mIndicesVertices[0]];
		final Vector3 v1 = Vector3.subtract(vertices[mIndicesVertices[1]], p0);
		final Vector3 v2 = Vector3.subtract(vertices[mIndicesVertices[2]], p0);
		final float v1Dotv1 = v1.dot(v1);
		final float v1Dotv2 = v1.dot(v2);
		final float v2Dotv2 = v2.dot(v2);
		final float p0Dotv1 = p0.dot(v1);
		final float p0Dotv2 = p0.dot(v2);
		mDet = v1Dotv1 * v2Dotv2 - v1Dotv2 * v1Dotv2;
		mLambda1 = p0Dotv2 * v1Dotv2 - p0Dotv1 * v2Dotv2;
		mLambda2 = p0Dotv1 * v1Dotv2 - p0Dotv2 * v1Dotv1;
		if (mDet > 0) {
			mClosestPoint.set(Vector3.add(p0, Vector3.multiply(1 / mDet, Vector3.add(
					Vector3.multiply(mLambda1, v1),
					Vector3.multiply(mLambda2, v2)))));
			mDistSquare = mClosestPoint.dot(mClosestPoint);
			return true;
		}
		return false;
	}

	/**
	 * Executes the recursive silhouette algorithm from this triangle face. The parameter "vertices" is
	 * an array that contains the vertices of the current polytope and the parameter "indexNewVertex"
	 * is the index of the new vertex in this array. The goal of the silhouette algorithm is to add the
	 * new vertex in the polytope by keeping it convex. Therefore, the triangle faces that are visible
	 * from the new vertex must be removed from the polytope. We need to add triangle faces where each
	 * face contains the new vertex and an edge for the silhouette. The silhouette is the connected set
	 * of edges that are part of the border between faces that are seen and not seen from the new
	 * vertex. This method starts from the nearest face of the new vertex, computes the silhouette and
	 * creates the new faces from the new vertex, ensuring that we always have a convex polytope. The
	 * faces visible from the new vertex are flagged as obsolete and will not be considered as being
	 * candidate faces in the future.
	 *
	 * @param vertices The vertices to compute with
	 * @param indexNewVertex The index of the new vertex
	 * @param triangleStore The triangle storage
	 * @return True if the computation succeeded, false if not
	 */
	public boolean computeSilhouette(Vector3[] vertices, int indexNewVertex, TrianglesStore triangleStore) {
		final int first = triangleStore.getNbTriangles();
		setIsObsolete(true);
		final boolean result = mAdjacentEdges[0].computeSilhouette(vertices, indexNewVertex, triangleStore)
				&& mAdjacentEdges[1].computeSilhouette(vertices, indexNewVertex, triangleStore)
				&& mAdjacentEdges[2].computeSilhouette(vertices, indexNewVertex, triangleStore);
		if (result) {
			for (int i = first, j = triangleStore.getNbTriangles() - 1; i != triangleStore.getNbTriangles(); j = i++) {
				final TriangleEPA triangle = triangleStore.get(i);
				halfLink(triangle.getAdjacentEdge(1), new EdgeEPA(triangle, 1));
				if (!link(new EdgeEPA(triangle, 0), new EdgeEPA(triangleStore.get(j), 2))) {
					return false;
				}
			}
		}
		return result;
	}

	/**
	 * Links an edge with another one. This means that the edge of a triangle will be associated with
	 * the edge of another triangle so that both triangles are neighbours along both edges.
	 *
	 * @param edge0 The first edge to link
	 * @param edge1 The second edge to link
	 * @return True if the linking occurred, false if it failed
	 */
	public static boolean link(EdgeEPA edge0, EdgeEPA edge1) {
		final boolean isPossible = (edge0.getSourceVertexIndex() == edge1.getTargetVertexIndex()
				&& edge0.getTargetVertexIndex() == edge1.getSourceVertexIndex());
		if (isPossible) {
			edge0.getOwnerTriangle().mAdjacentEdges[edge0.getIndex()] = edge1;
			edge1.getOwnerTriangle().mAdjacentEdges[edge1.getIndex()] = edge0;
		}
		return isPossible;
	}

	/**
	 * Makes a half link with two edges from two different triangles. An half-link between the edge
	 * "edge0" and the edge "edge1" means that "edge1" is adjacent to "edge0" but not the opposite. The
	 * opposite edge connection is to be made later.
	 *
	 * @param edge0 The first edge to link
	 * @param edge1 The second edge to link
	 */
	public static void halfLink(EdgeEPA edge0, EdgeEPA edge1) {
		if (edge0.getSourceVertexIndex() != edge1.getTargetVertexIndex()
				|| edge0.getTargetVertexIndex() != edge1.getSourceVertexIndex()) {
			throw new IllegalArgumentException("the first edge's source vertex index must be equal to the second edge's target vertex index"
					+ "and the first edge's target vertex index must be equal to the second edge's source vertex index");
		}
		edge0.getOwnerTriangle().mAdjacentEdges[edge0.getIndex()] = edge1;
	}
}
