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

import java.util.Comparator;
import java.util.PriorityQueue;
import java.util.Queue;
import org.spout.math.imaginary.Quaternion;
import org.spout.math.matrix.Matrix3;
import org.spout.math.vector.Vector3;

import org.spout.physics.ReactDefaults;
import org.spout.physics.collision.ContactInfo;
import org.spout.physics.collision.narrowphase.GJK.GJKAlgorithm;
import org.spout.physics.collision.narrowphase.GJK.Simplex;
import org.spout.physics.collision.shape.CollisionShape;
import org.spout.physics.math.Transform;

/**
 * This class is the implementation of the Expanding Polytope Algorithm (EPA). The EPA algorithm
 * computes the penetration depth and contact points between two enlarged objects (with margin)
 * where the original objects (without margin) intersect. The penetration depth of a pair of
 * intersecting objects A and B is the length of a point on the boundary of the Minkowski sum (A-B)
 * closest to the origin. The goal of the EPA algorithm is to start with an initial simplex polytope
 * that contains the origin and expend it in order to find the point on the boundary of (A-B) that
 * is closest to the origin. An initial simplex that contains the origin has been computed with the
 * GJK algorithm. The EPA Algorithm will extend this simplex polytope to find the correct
 * penetration depth. The implementation of the EPA algorithm is based on the book "Collision
 * Detection in 3D Environments".
 */
public class EPAAlgorithm {
	private static final int MAX_SUPPORT_POINTS = 100;
	private static final int MAX_FACETS = 200;

	/**
	 * Computes the penetration depth with the EPA algorithm. This method computes the penetration
	 * depth and contact points between two enlarged objects (with margin) where the original objects
	 * (without margin) intersect. An initial simplex that contains the origin has been computed with
	 * GJK algorithm. The EPA Algorithm will extend this simplex polytope to find the correct
	 * penetration depth. Returns true if the computation was successful, false if not.
	 *
	 * @param simplex The initial simplex
	 * @param collisionShape1 The first collision shape
	 * @param transform1 The transform of the first collision shape
	 * @param collisionShape2 The second collision shape
	 * @param transform2 The transform of the second collision shape
	 * @param v The vector in which to store the closest point
	 * @param contactInfo The contact info in which to store the contact info of the collision
	 * @return Whether or not the computation was successful
	 */
	public boolean computePenetrationDepthAndContactPoints(Simplex simplex,
														   CollisionShape collisionShape1, Transform transform1,
														   CollisionShape collisionShape2, Transform transform2,
														   Vector3 v, ContactInfo contactInfo) {
		final Vector3[] suppPointsA = new Vector3[MAX_SUPPORT_POINTS];
		final Vector3[] suppPointsB = new Vector3[MAX_SUPPORT_POINTS];
		final Vector3[] points = new Vector3[MAX_SUPPORT_POINTS];
		final TrianglesStore triangleStore = new TrianglesStore();
		final Queue<TriangleEPA> triangleHeap = new PriorityQueue<TriangleEPA>(MAX_FACETS, new TriangleComparison());
		final Transform body2Tobody1 = Transform.multiply(transform1.inverse(), transform2);
		final Matrix3 rotateToBody2 = transform2.getOrientation().toMatrix3().transpose().mul(transform1.getOrientation().toMatrix3());
		int nbVertices = simplex.getSimplex(suppPointsA, suppPointsB, points);
		final float tolerance = ReactDefaults.MACHINE_EPSILON * simplex.getMaxLengthSquareOfAPoint();
		int nbTriangles = 0;
		triangleStore.clear();
		switch (nbVertices) {
			case 1:
				return false;
			case 2: {
				final Vector3 d = points[1].sub(points[0].normalize());
				final int minAxis = d.abs().getMinAxis();
				final float sin60 = (float) Math.sqrt(3) * 0.5f;
				final Quaternion rotationQuat = new Quaternion(d.getX() * sin60, d.getY() * sin60, d.getZ() * sin60, 0.5f);
				final Matrix3 rotationMat = rotationQuat.toMatrix3();
				final Vector3 v1 = d.cross(new Vector3(minAxis == 0 ? 1 : 0, minAxis == 1 ? 1 : 0, minAxis == 2 ? 1 : 0));
				final Vector3 v2 = rotationMat.transform(v1);
				final Vector3 v3 = rotationMat.transform(v2);
				suppPointsA[2] = collisionShape1.getLocalSupportPointWithMargin(v1);
				suppPointsB[2] = Transform.multiply(
						body2Tobody1,
						collisionShape2.getLocalSupportPointWithMargin(rotateToBody2.transform(v1.negate())));
				points[2] = suppPointsA[2].sub(suppPointsB[2]);
				suppPointsA[3] = collisionShape1.getLocalSupportPointWithMargin(v2);
				suppPointsB[3] = Transform.multiply(
						body2Tobody1,
						collisionShape2.getLocalSupportPointWithMargin(rotateToBody2.transform(v2.negate())));
				points[3] = suppPointsA[3].sub(suppPointsB[3]);
				suppPointsA[4] = collisionShape1.getLocalSupportPointWithMargin(v3);
				suppPointsB[4] = Transform.multiply(
						body2Tobody1,
						collisionShape2.getLocalSupportPointWithMargin(rotateToBody2.transform(v3.negate())));
				points[4] = suppPointsA[4].sub(suppPointsB[4]);
				if (isOriginInTetrahedron(points[0], points[2], points[3], points[4]) == 0) {
					suppPointsA[1] = suppPointsA[4];
					suppPointsB[1] = suppPointsB[4];
					points[1] = points[4];
				} else if (isOriginInTetrahedron(points[1], points[2], points[3], points[4]) == 0) {
					suppPointsA[0] = suppPointsA[4];
					suppPointsB[0] = suppPointsB[4];
					points[0] = points[4];
				} else {
					return false;
				}
				nbVertices = 4;
			}
			case 4: {
				final int badVertex = isOriginInTetrahedron(points[0], points[1], points[2], points[3]);
				if (badVertex == 0) {
					final TriangleEPA face0 = triangleStore.newTriangle(points, 0, 1, 2);
					final TriangleEPA face1 = triangleStore.newTriangle(points, 0, 3, 1);
					final TriangleEPA face2 = triangleStore.newTriangle(points, 0, 2, 3);
					final TriangleEPA face3 = triangleStore.newTriangle(points, 1, 3, 2);
					if (!(face0 != null && face1 != null && face2 != null && face3 != null
							&& face0.getDistSquare() > 0 && face1.getDistSquare() > 0
							&& face2.getDistSquare() > 0 && face3.getDistSquare() > 0)) {
						return false;
					}
					TriangleEPA.link(new EdgeEPA(face0, 0), new EdgeEPA(face1, 2));
					TriangleEPA.link(new EdgeEPA(face0, 1), new EdgeEPA(face3, 2));
					TriangleEPA.link(new EdgeEPA(face0, 2), new EdgeEPA(face2, 0));
					TriangleEPA.link(new EdgeEPA(face1, 0), new EdgeEPA(face2, 2));
					TriangleEPA.link(new EdgeEPA(face1, 1), new EdgeEPA(face3, 0));
					TriangleEPA.link(new EdgeEPA(face2, 1), new EdgeEPA(face3, 1));
					nbTriangles = addFaceCandidate(face0, triangleHeap, nbTriangles, Float.MAX_VALUE);
					nbTriangles = addFaceCandidate(face1, triangleHeap, nbTriangles, Float.MAX_VALUE);
					nbTriangles = addFaceCandidate(face2, triangleHeap, nbTriangles, Float.MAX_VALUE);
					nbTriangles = addFaceCandidate(face3, triangleHeap, nbTriangles, Float.MAX_VALUE);
					break;
				}
				if (badVertex < 4) {
					suppPointsA[badVertex - 1] = suppPointsA[4];
					suppPointsB[badVertex - 1] = suppPointsB[4];
					points[badVertex - 1] = points[4];
				}
				nbVertices = 3;
			}
			case 3: {
				final Vector3 v1 = points[1].sub(points[0]);
				final Vector3 v2 = points[2].sub(points[0]);
				final Vector3 n = v1.cross(v2);
				suppPointsA[3] = collisionShape1.getLocalSupportPointWithMargin(n);
				suppPointsB[3] = Transform.multiply(
						body2Tobody1,
						collisionShape2.getLocalSupportPointWithMargin(rotateToBody2.transform(n.negate())));
				points[3] = suppPointsA[3].sub(suppPointsB[3]);
				suppPointsA[4] = collisionShape1.getLocalSupportPointWithMargin(n.negate());
				suppPointsB[4] = Transform.multiply(
						body2Tobody1,
						collisionShape2.getLocalSupportPointWithMargin(rotateToBody2.transform(n)));
				points[4] = suppPointsA[4].sub(suppPointsB[4]);
				final TriangleEPA face0 = triangleStore.newTriangle(points, 0, 1, 3);
				final TriangleEPA face1 = triangleStore.newTriangle(points, 1, 2, 3);
				final TriangleEPA face2 = triangleStore.newTriangle(points, 2, 0, 3);
				final TriangleEPA face3 = triangleStore.newTriangle(points, 0, 2, 4);
				final TriangleEPA face4 = triangleStore.newTriangle(points, 2, 1, 4);
				final TriangleEPA face5 = triangleStore.newTriangle(points, 1, 0, 4);
				if (!(face0 != null && face1 != null && face2 != null && face3 != null && face4 != null && face5 != null &&
						face0.getDistSquare() > 0 && face1.getDistSquare() > 0 &&
						face2.getDistSquare() > 0 && face3.getDistSquare() > 0 &&
						face4.getDistSquare() > 0 && face5.getDistSquare() > 0)) {
					return false;
				}
				TriangleEPA.link(new EdgeEPA(face0, 1), new EdgeEPA(face1, 2));
				TriangleEPA.link(new EdgeEPA(face1, 1), new EdgeEPA(face2, 2));
				TriangleEPA.link(new EdgeEPA(face2, 1), new EdgeEPA(face0, 2));
				TriangleEPA.link(new EdgeEPA(face0, 0), new EdgeEPA(face5, 0));
				TriangleEPA.link(new EdgeEPA(face1, 0), new EdgeEPA(face4, 0));
				TriangleEPA.link(new EdgeEPA(face2, 0), new EdgeEPA(face3, 0));
				TriangleEPA.link(new EdgeEPA(face3, 1), new EdgeEPA(face4, 2));
				TriangleEPA.link(new EdgeEPA(face4, 1), new EdgeEPA(face5, 2));
				TriangleEPA.link(new EdgeEPA(face5, 1), new EdgeEPA(face3, 2));
				nbTriangles = addFaceCandidate(face0, triangleHeap, nbTriangles, Float.MAX_VALUE);
				nbTriangles = addFaceCandidate(face1, triangleHeap, nbTriangles, Float.MAX_VALUE);
				nbTriangles = addFaceCandidate(face2, triangleHeap, nbTriangles, Float.MAX_VALUE);
				nbTriangles = addFaceCandidate(face3, triangleHeap, nbTriangles, Float.MAX_VALUE);
				nbTriangles = addFaceCandidate(face4, triangleHeap, nbTriangles, Float.MAX_VALUE);
				nbTriangles = addFaceCandidate(face5, triangleHeap, nbTriangles, Float.MAX_VALUE);
				nbVertices = 5;
			}
			break;
		}
		if (nbTriangles == 0) {
			return false;
		}
		TriangleEPA triangle;
		float upperBoundSquarePenDepth = Float.MAX_VALUE;
		do {
			triangle = triangleHeap.remove();
			nbTriangles--;
			if (!triangle.isObsolete()) {
				if (nbVertices == MAX_SUPPORT_POINTS) {
					break;
				}
				suppPointsA[nbVertices] = collisionShape1.getLocalSupportPointWithMargin(triangle.getClosestPoint());
				suppPointsB[nbVertices] = Transform.multiply(
						body2Tobody1,
						collisionShape2.getLocalSupportPointWithMargin(rotateToBody2.transform(triangle.getClosestPoint().negate())));
				points[nbVertices] = suppPointsA[nbVertices].sub(suppPointsB[nbVertices]);
				final int indexNewVertex = nbVertices;
				nbVertices++;
				final float wDotv = points[indexNewVertex].dot(triangle.getClosestPoint());
				if (wDotv <= 0) {
					throw new IllegalStateException("wDotv must be greater than zero");
				}
				final float wDotVSquare = wDotv * wDotv / triangle.getDistSquare();
				if (wDotVSquare < upperBoundSquarePenDepth) {
					upperBoundSquarePenDepth = wDotVSquare;
				}
				final float error = wDotv - triangle.getDistSquare();
				if (error <= Math.max(tolerance, GJKAlgorithm.REL_ERROR_SQUARE * wDotv)
						|| points[indexNewVertex].equals(points[triangle.get(0)])
						|| points[indexNewVertex].equals(points[triangle.get(1)])
						|| points[indexNewVertex].equals(points[triangle.get(2)])) {
					break;
				}
				int i = triangleStore.getNbTriangles();
				if (!triangle.computeSilhouette(points, indexNewVertex, triangleStore)) {
					break;
				}
				while (i != triangleStore.getNbTriangles()) {
					final TriangleEPA newTriangle = triangleStore.get(i);
					nbTriangles = addFaceCandidate(newTriangle, triangleHeap, nbTriangles, upperBoundSquarePenDepth);
					i++;
				}
			}
		}
		while (nbTriangles > 0 && triangleHeap.element().getDistSquare() <= upperBoundSquarePenDepth);
		v = transform1.getOrientation().toMatrix3().transform(triangle.getClosestPoint());
		final Vector3 pALocal = triangle.computeClosestPointOfObject(suppPointsA);
		final Vector3 pBLocal = Transform.multiply(body2Tobody1.inverse(), triangle.computeClosestPointOfObject(suppPointsB));
		final Vector3 normal = v.normalize();
		final float penetrationDepth = v.length();
		if (penetrationDepth <= 0) {
			throw new IllegalStateException("penetration depth must be greater that zero");
		}
		contactInfo.set(normal, penetrationDepth, pALocal, pBLocal);
		return true;
	}

	// Decides if the origin is in the tetrahedron.
	// Returns 0 if the origin is in the tetrahedron or returns the index (1,2,3 or 4) of the bad
	// vertex if the origin is not in the tetrahedron.
	private static int isOriginInTetrahedron(Vector3 p1, Vector3 p2, Vector3 p3, Vector3 p4) {
		final Vector3 normal1 = p2.sub(p1).cross(p3.sub(p1));
		if (normal1.dot(p1) > 0 == normal1.dot(p4) > 0) {
			return 4;
		}
		final Vector3 normal2 = p4.sub(p2).cross(p3.sub(p2));
		if (normal2.dot(p2) > 0 == normal2.dot(p1) > 0) {
			return 1;
		}
		final Vector3 normal3 = p4.sub(p3).cross(p1.sub(p3));
		if (normal3.dot(p3) > 0 == normal3.dot(p2) > 0) {
			return 2;
		}
		final Vector3 normal4 = p2.sub(p4).cross(p1.sub(p4));
		if (normal4.dot(p4) > 0 == normal4.dot(p3) > 0) {
			return 3;
		}
		return 0;
	}

	// Adds a triangle face in the candidate triangle heap in the EPA algorithm.
	private static int addFaceCandidate(TriangleEPA triangle, Queue<TriangleEPA> heap, int nbTriangles, float upperBoundSquarePenDepth) {
		if (triangle.isClosestPointInternalToTriangle() && triangle.getDistSquare() <= upperBoundSquarePenDepth) {
			heap.add(triangle);
			nbTriangles++;
		}
		return nbTriangles;
	}

	// Compares the EPA triangles in the queue.
	private static class TriangleComparison implements Comparator<TriangleEPA> {
		public int compare(TriangleEPA face1, TriangleEPA face2) {
			final float dist1 = face1.getDistSquare();
			final float dist2 = face2.getDistSquare();
			if (dist1 == dist2) {
				return 0;
			}
			return dist1 > dist2 ? 1 : -1;
		}
	}
}
