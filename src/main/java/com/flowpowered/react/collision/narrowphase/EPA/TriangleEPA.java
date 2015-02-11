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
 * Represents a triangle face for the current polytope in the EPA algorithm.
 */
public class TriangleEPA {
    private final int[] mIndicesVertices = new int[3];
    private final EdgeEPA[] mAdjacentEdges = new EdgeEPA[3];
    private boolean mIsObsolete = false;
    private float mDet;
    private final Vector3 mClosestPoint = new Vector3();
    private float mLambda1;
    private float mLambda2;
    private float mDistSquare;

    /**
     * Default constructor for the triangle. It contains no vertices or edges and the closest point is the zero vector. The triangle is not flagged as obsolete.
     */
    public TriangleEPA() {
        this(0, 0, 0);
    }

    /**
     * Constructs a new triangle from the three vertex indices. This triangle is not flagged as obsolete.
     *
     * @param indexVertex1 The first vertex index
     * @param indexVertex2 The second vertex index
     * @param indexVertex3 The third vertex index
     */
    public TriangleEPA(int indexVertex1, int indexVertex2, int indexVertex3) {
        mIndicesVertices[0] = indexVertex1;
        mIndicesVertices[1] = indexVertex2;
        mIndicesVertices[2] = indexVertex3;
    }

    /**
     * Gets an edge the triangle at the desired index. The index must be greater or equal to zero and smaller than three.
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
     * Sets an adjacent edge of the triangle to the desired value at the specified index. The index must be greater or equal to zero and smaller than three.
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
    public void setObsolete(boolean isObsolete) {
        mIsObsolete = isObsolete;
    }

    /**
     * Returns true if the triangle face is obsolete, false if not.
     *
     * @return True if the triangle is obsolete, false if not
     */
    public boolean isObsolete() {
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
        return mLambda1 >= 0 && mLambda2 >= 0 && (mLambda1 + mLambda2) <= mDet;
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
        return mClosestPoint.dot(closestToVert) > 0;
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
     * Gets the vertex index at the desired index. The index must be greater or equal to zero and smaller than three.
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
     * Executes the recursive silhouette algorithm from this triangle face. The parameter "vertices" is an array that contains the vertices of the current polytope and the parameter "indexNewVertex"
     * is the index of the new vertex in this array. The goal of the silhouette algorithm is to add the new vertex in the polytope by keeping it convex. Therefore, the triangle faces that are visible
     * from the new vertex must be removed from the polytope. We need to add triangle faces where each face contains the new vertex and an edge for the silhouette. The silhouette is the connected set
     * of edges that are part of the border between faces that are seen and not seen from the new vertex. This method starts from the nearest face of the new vertex, computes the silhouette and
     * creates the new faces from the new vertex, ensuring that we always have a convex polytope. The faces visible from the new vertex are flagged as obsolete and will not be considered as being
     * candidate faces in the future.
     *
     * @param vertices The vertices to compute with
     * @param indexNewVertex The index of the new vertex
     * @param triangleStore The triangle storage
     * @return True if the computation succeeded, false if not
     */
    public boolean computeSilhouette(Vector3[] vertices, int indexNewVertex, TrianglesStore triangleStore) {
        final int first = triangleStore.getNbTriangles();
        setObsolete(true);
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
     * Links an edge with another one. This means that the edge of a triangle will be associated with the edge of another triangle so that both triangles are neighbours along both edges.
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
     * Makes a half link with two edges from two different triangles. An half-link between the edge "edge0" and the edge "edge1" means that "edge1" is adjacent to "edge0" but not the opposite. The
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
