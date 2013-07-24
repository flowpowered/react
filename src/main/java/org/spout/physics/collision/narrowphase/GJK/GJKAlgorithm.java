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
package org.spout.physics.collision.narrowphase.GJK;

import org.spout.math.matrix.Matrix3;
import org.spout.math.vector.Vector3;
import org.spout.physics.ReactDefaults;
import org.spout.physics.collision.ContactInfo;
import org.spout.physics.collision.narrowphase.EPA.EPAAlgorithm;
import org.spout.physics.collision.narrowphase.NarrowPhaseAlgorithm;
import org.spout.physics.collision.shape.CollisionShape;
import org.spout.physics.math.Transform;

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
	public static final float REL_ERROR = 1e-3f;
	public static final float REL_ERROR_SQUARE = REL_ERROR * REL_ERROR;
	private final EPAAlgorithm mAlgoEPA = new EPAAlgorithm();

	@Override
	public boolean testCollision(CollisionShape collisionShape1, Transform transform1,
								 CollisionShape collisionShape2, Transform transform2,
								 ContactInfo contactInfo) {
		Vector3 suppA;
		Vector3 suppB;
		Vector3 w;
		Vector3 pA = null;
		Vector3 pB = null;
		float vDotw;
		float prevDistSquare;
		final Transform body2ToBody1 = Transform.multiply(transform1.inverse(), transform2);
		final Matrix3 rotateToBody2 = transform2.getOrientation().toMatrix3().transpose().mul(transform1.getOrientation().toMatrix3());
		final float margin = collisionShape1.getMargin() + collisionShape2.getMargin();
		final float marginSquare = margin * margin;
		if (margin <= 0) {
			throw new IllegalStateException("margin must be greater than zero");
		}
		final Simplex simplex = new Simplex();
		final Vector3 v = mCurrentOverlappingPair.getPreviousSeparatingAxis();
		float distSquare = Float.MAX_VALUE;
		do {
			suppA = collisionShape1.getLocalSupportPointWithoutMargin(v.negate());
			suppB = Transform.multiply(body2ToBody1, collisionShape2.getLocalSupportPointWithoutMargin(rotateToBody2.transform(v)));
			w = suppA.sub(suppB);
			vDotw = v.dot(w);
			if (vDotw > 0 && vDotw * vDotw > distSquare * marginSquare) {
				mCurrentOverlappingPair.setPreviousSeparatingAxis(v);
				return false;
			}
			if (simplex.isPointInSimplex(w) || distSquare - vDotw <= distSquare * REL_ERROR_SQUARE) {
				simplex.computeClosestPointsOfAAndB(pA, pB);
				final float dist = (float) Math.sqrt(distSquare);
				if (dist <= 0) {
					throw new IllegalStateException("dist must be greater than zero");
				}
				pA = pA.sub(v.mul(collisionShape1.getMargin() / dist));
				pB = Transform.multiply(body2ToBody1.inverse(), pB.add(v.mul(collisionShape2.getMargin() / dist)));
				final Vector3 normal = transform1.getOrientation().toMatrix3().transform(v.normalize().negate());
				final float penetrationDepth = margin - dist;
				if (penetrationDepth <= 0) {
					return false;
				}
				contactInfo.set(normal, penetrationDepth, pA, pB);
				return true;
			}
			simplex.addPoint(w, suppA, suppB);
			if (simplex.isAffinelyDependent()) {
				simplex.computeClosestPointsOfAAndB(pA, pB);
				final float dist = (float) Math.sqrt(distSquare);
				if (dist <= 0) {
					throw new IllegalStateException("dist must be greater than zero");
				}
				pA = pA.sub(v.mul(collisionShape1.getMargin() / dist));
				pB = Transform.multiply(body2ToBody1.inverse(), pB.add(v.mul(collisionShape2.getMargin() / dist)));
				final Vector3 normal = transform1.getOrientation().toMatrix3().transform(v.normalize().negate());
				final float penetrationDepth = margin - dist;
				if (penetrationDepth <= 0) {
					return false;
				}
				contactInfo.set(normal, penetrationDepth, pA, pB);
				return true;
			}
			if (!simplex.computeClosestPoint(v)) {
				simplex.computeClosestPointsOfAAndB(pA, pB);
				final float dist = (float) Math.sqrt(distSquare);
				if (dist <= 0) {
					throw new IllegalStateException("dist must be greater than zero");
				}
				pA = pA.sub(v.mul(collisionShape1.getMargin() / dist));
				pB = Transform.multiply(body2ToBody1.inverse(), pB.add(v.mul(collisionShape2.getMargin() / dist)));
				final Vector3 normal = transform1.getOrientation().toMatrix3().transform(v.normalize().negate());
				final float penetrationDepth = margin - dist;
				if (penetrationDepth <= 0) {
					return false;
				}
				contactInfo.set(normal, penetrationDepth, pA, pB);
				return true;
			}
			prevDistSquare = distSquare;
			distSquare = v.lengthSquared();
			if (prevDistSquare - distSquare <= ReactDefaults.MACHINE_EPSILON * prevDistSquare) {
				simplex.backupClosestPointInSimplex(v);
				distSquare = v.lengthSquared();
				simplex.computeClosestPointsOfAAndB(pA, pB);
				final float dist = (float) Math.sqrt(distSquare);
				if (dist <= 0) {
					throw new IllegalStateException("dist must be greater than zero");
				}
				pA = pA.sub(v.mul(collisionShape1.getMargin() / dist));
				pB = Transform.multiply(body2ToBody1.inverse(), pB.add(v.mul(collisionShape2.getMargin() / dist)));
				final Vector3 normal = transform1.getOrientation().toMatrix3().transform(v.normalize().negate());
				final float penetrationDepth = margin - dist;
				if (penetrationDepth <= 0) {
					return false;
				}
				contactInfo.set(normal, penetrationDepth, pA, pB);
				return true;
			}
		}
		while (!simplex.isFull() && distSquare > ReactDefaults.MACHINE_EPSILON * simplex.getMaxLengthSquareOfAPoint());
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
		float vDotw;
		float distSquare = Float.MAX_VALUE;
		float prevDistSquare;
		final Transform body2ToBody1 = Transform.multiply(transform1.inverse(), transform2);
		final Matrix3 rotateToBody2 = transform2.getOrientation().toMatrix3().transpose().mul(transform1.getOrientation().toMatrix3());
		Vector3 suppA;
		Vector3 suppB;
		Vector3 w;
		do {
			suppA = collisionShape1.getLocalSupportPointWithMargin(v.negate());
			suppB = Transform.multiply(body2ToBody1, collisionShape2.getLocalSupportPointWithMargin(rotateToBody2.transform(v)));
			w = suppA.sub(suppB);
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
			distSquare = v.lengthSquared();
			if (prevDistSquare - distSquare <= ReactDefaults.MACHINE_EPSILON * prevDistSquare) {
				return false;
			}
		}
		while (!simplex.isFull() && distSquare > ReactDefaults.MACHINE_EPSILON * simplex.getMaxLengthSquareOfAPoint());
		return mAlgoEPA.computePenetrationDepthAndContactPoints(simplex, collisionShape1, transform1, collisionShape2, transform2, v, contactInfo);
	}
}
