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
package org.spout.jreactphysics3d.collision.narrowphase.GJK;

import org.spout.jreactphysics3d.Configuration;
import org.spout.jreactphysics3d.collision.ContactInfo;
import org.spout.jreactphysics3d.collision.narrowphase.EPA.EPAAlgorithm;
import org.spout.jreactphysics3d.collision.narrowphase.NarrowPhaseAlgorithm;
import org.spout.jreactphysics3d.collision.shape.CollisionShape;
import org.spout.jreactphysics3d.mathematics.Matrix3x3;
import org.spout.jreactphysics3d.mathematics.Transform;
import org.spout.jreactphysics3d.mathematics.Vector3;

/**
 * This class implements a narrow-phase collision detection algorithm. This algorithm uses the
 * ISA-GJK algorithm and the EPA algorithm. This implementation is based on the implementation
 * discussed in the book "Collision Detection in 3D Environments". This method implements the Hybrid
 * Technique for calculating the penetration depth. The two objects are enlarged with a small
 * margin. If the object intersect, the penetration depth is quickly computed using the GJK
 * algorithm on the original objects (without margin). If the original objects (without margin)
 * intersect, we run the GJK algorithm again on the enlarged objects (with margin) to compute the
 * simplex polytope that contains the origin and give it to the EPA (Expanding Polytope Algorithm)
 * to compute the correct penetration depth between the enlarged objects.
 */
public class GJKAlgorithm extends NarrowPhaseAlgorithm {
	public static final float REL_ERROR = 1.0e-3f;
	public static final float REL_ERROR_SQUARE = REL_ERROR * REL_ERROR;
	private final EPAAlgorithm mAlgoEPA = new EPAAlgorithm();

	@Override
	public boolean testCollision(CollisionShape collisionShape1, Transform transform1,
								 CollisionShape collisionShape2, Transform transform2,
								 ContactInfo contactInfo) {
		final Vector3 suppA = new Vector3();
		final Vector3 suppB = new Vector3();
		final Vector3 w = new Vector3();
		final Vector3 pA = new Vector3();
		final Vector3 pB = new Vector3();
		float vDotw;
		float prevDistSquare;
		final Transform body2Tobody1 = Transform.multiply(transform1.inverse(), transform2);
		final Matrix3x3 rotateToBody2 = Matrix3x3.multiply(
				transform2.getOrientation().getMatrix().getTranspose(),
				transform1.getOrientation().getMatrix());
		final float margin = collisionShape1.getMargin() + collisionShape2.getMargin();
		final float marginSquare = margin * margin;
		if (margin <= 0) {
			throw new IllegalStateException("margin must be greater than zero");
		}
		final Simplex simplex = new Simplex();
		final Vector3 v = mCurrentOverlappingPair.getPreviousSeparatingAxis();
		float distSquare = Float.MAX_VALUE;
		do {
			suppA.set(collisionShape1.getLocalSupportPointWithoutMargin(Vector3.negate(v)));
			suppB.set(Transform.multiply(body2Tobody1, collisionShape2.getLocalSupportPointWithoutMargin(Matrix3x3.multiply(rotateToBody2, v))));
			w.set(Vector3.subtract(suppA, suppB));
			vDotw = v.dot(w);
			if (vDotw > 0 && vDotw * vDotw > distSquare * marginSquare) {
				mCurrentOverlappingPair.setPreviousSeparatingAxis(v);
				return false;
			}
			if (simplex.isPointInSimplex(w) || distSquare - vDotw <= distSquare * REL_ERROR_SQUARE) {
				simplex.computeClosestPointsOfAandB(pA, pB);
				final float dist = (float) Math.sqrt(distSquare);
				if (dist <= 0) {
					throw new IllegalStateException("dist must be greater than zero");
				}
				pA.set(Vector3.subtract(pA, Vector3.multiply(collisionShape1.getMargin() / dist, v)));
				pB.set(Transform.multiply(body2Tobody1.inverse(), Vector3.add(pB, Vector3.multiply(collisionShape2.getMargin() / dist, v))));
				final Vector3 normal = Matrix3x3.multiply(transform1.getOrientation().getMatrix(), Vector3.negate(v.getUnit()));
				final float penetrationDepth = margin - dist;
				if (penetrationDepth <= 0) {
					return false;
				}
				contactInfo.set(normal, penetrationDepth, pA, pB);
				return true;
			}
			simplex.addPoint(w, suppA, suppB);
			if (simplex.isAffinelyDependent()) {
				simplex.computeClosestPointsOfAandB(pA, pB);
				final float dist = (float) Math.sqrt(distSquare);
				if (dist <= 0) {
					throw new IllegalStateException("dist must be greater than zero");
				}
				pA.set(Vector3.subtract(pA, Vector3.multiply(collisionShape1.getMargin() / dist, v)));
				pB.set(Transform.multiply(body2Tobody1.inverse(), Vector3.add(pB, Vector3.multiply(collisionShape2.getMargin() / dist, v))));
				final Vector3 normal = Matrix3x3.multiply(transform1.getOrientation().getMatrix(), Vector3.negate(v.getUnit()));
				final float penetrationDepth = margin - dist;
				if (penetrationDepth <= 0) {
					return false;
				}
				contactInfo.set(normal, penetrationDepth, pA, pB);
				return true;
			}
			if (!simplex.computeClosestPoint(v)) {
				simplex.computeClosestPointsOfAandB(pA, pB);
				final float dist = (float) Math.sqrt(distSquare);
				if (dist <= 0) {
					throw new IllegalStateException("dist must be greater than zero");
				}
				pA.set(Vector3.subtract(pA, Vector3.multiply(collisionShape1.getMargin() / dist, v)));
				pB.set(Transform.multiply(body2Tobody1.inverse(), Vector3.add(pB, Vector3.multiply(collisionShape2.getMargin() / dist, v))));
				final Vector3 normal = Matrix3x3.multiply(transform1.getOrientation().getMatrix(), Vector3.negate(v.getUnit()));
				final float penetrationDepth = margin - dist;
				if (penetrationDepth <= 0) {
					return false;
				}
				contactInfo.set(normal, penetrationDepth, pA, pB);
				return true;
			}
			prevDistSquare = distSquare;
			distSquare = v.lengthSquare();
			if (prevDistSquare - distSquare <= Configuration.MACHINE_EPSILON * prevDistSquare) {
				simplex.backupClosestPointInSimplex(v);
				distSquare = v.lengthSquare();
				simplex.computeClosestPointsOfAandB(pA, pB);
				final float dist = (float) Math.sqrt(distSquare);
				if (dist <= 0) {
					throw new IllegalStateException("dist must be greater than zero");
				}
				pA.set(Vector3.subtract(pA, Vector3.multiply(collisionShape1.getMargin() / dist, v)));
				pB.set(Transform.multiply(body2Tobody1.inverse(), Vector3.add(pB, Vector3.multiply(collisionShape2.getMargin() / dist, v))));
				final Vector3 normal = Matrix3x3.multiply(transform1.getOrientation().getMatrix(), Vector3.negate(v.getUnit()));
				final float penetrationDepth = margin - dist;
				if (penetrationDepth <= 0) {
					return false;
				}
				contactInfo.set(normal, penetrationDepth, pA, pB);
				return true;
			}
		}
		while (!simplex.isFull() && distSquare > Configuration.MACHINE_EPSILON * simplex.getMaxLengthSquareOfAPoint());
		return computePenetrationDepthForEnlargedObjects(collisionShape1, transform1, collisionShape2, transform2, contactInfo, v);
	}

	// This method runs the GJK algorithm on the two enlarged objects (with margin) to compute a
	// simplex polytope that contains the origin. The two objects are assumed to intersect in the
	// original objects (without margin). Therefore such a polytope must exist. Next we give that
	// polytope to the EPA algorithm to compute the correct penetration depth and contact points of
	// the enlarged objects.
	private boolean computePenetrationDepthForEnlargedObjects(CollisionShape collisionShape1, Transform transform1,
															  CollisionShape collisionShape2, Transform transform2,
															  ContactInfo contactInfo, Vector3 v) {
		final Simplex simplex = new Simplex();
		final Vector3 suppA = new Vector3();
		final Vector3 suppB = new Vector3();
		final Vector3 w = new Vector3();
		float vDotw;
		float distSquare = Float.MAX_VALUE;
		float prevDistSquare;
		final Transform body2ToBody1 = Transform.multiply(transform1.inverse(), transform2);
		final Matrix3x3 rotateToBody2 = Matrix3x3.multiply(
				transform2.getOrientation().getMatrix().getTranspose(),
				transform1.getOrientation().getMatrix());
		do {
			suppA.set(collisionShape1.getLocalSupportPointWithMargin(Vector3.negate(v)));
			suppB.set(Transform.multiply(body2ToBody1, collisionShape2.getLocalSupportPointWithMargin(Matrix3x3.multiply(rotateToBody2, v))));
			w.set(Vector3.subtract(suppA, suppB));
			vDotw = v.dot(w);
			if (vDotw > 0) {
				return false;
			}
			simplex.addPoint(w, suppA, suppB);
			if (simplex.isAffinelyDependent()) {
				return false;
			}

			if (!simplex.computeClosestPoint(v)) {
				return false;
			}
			prevDistSquare = distSquare;
			distSquare = v.lengthSquare();
			if (prevDistSquare - distSquare <= Configuration.MACHINE_EPSILON * prevDistSquare) {
				return false;
			}
		}
		while (!simplex.isFull() && distSquare > Configuration.MACHINE_EPSILON * simplex.getMaxLengthSquareOfAPoint());
		return mAlgoEPA.computePenetrationDepthAndContactPoints(simplex, collisionShape1, transform1, collisionShape2, transform2, v, contactInfo);
	}
}
