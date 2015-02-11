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
package com.flowpowered.react.collision.narrowphase.GJK;

import com.flowpowered.react.ReactDefaults;
import com.flowpowered.react.collision.narrowphase.EPA.EPAAlgorithm;
import com.flowpowered.react.collision.narrowphase.NarrowPhaseAlgorithm;
import com.flowpowered.react.collision.shape.CollisionShape;
import com.flowpowered.react.constraint.ContactPoint.ContactPointInfo;
import com.flowpowered.react.math.Matrix3x3;
import com.flowpowered.react.math.Transform;
import com.flowpowered.react.math.Vector3;

/**
 * This class implements a narrow-phase collision detection algorithm. This algorithm uses the ISA-GJK algorithm and the EPA algorithm. This implementation is based on the implementation discussed in
 * the book "Collision Detection in Interactive 3D Environments" by Gino van den Bergen. This method implements the Hybrid Technique for calculating the penetration depth. The two objects are enlarged
 * with a small margin. If the objects intersect in their margins, the penetration depth is quickly computed using the GJK algorithm on the original objects (without margin). If the original objects
 * (without margin) intersect, we run again the GJK algorithm again on the enlarged objects (with margin) to compute the simplex polytope that contains the origin and give it to the EPA (Expanding
 * Polytope Algorithm) to compute the correct penetration depth between the enlarged objects.
 */
public class GJKAlgorithm extends NarrowPhaseAlgorithm {
    public static final float REL_ERROR = 1e-3f;
    public static final float REL_ERROR_SQUARE = REL_ERROR * REL_ERROR;
    private final EPAAlgorithm mAlgoEPA = new EPAAlgorithm();

    @Override
    public boolean testCollision(CollisionShape collisionShape1, Transform transform1,
                                 CollisionShape collisionShape2, Transform transform2,
                                 ContactPointInfo contactInfo) {
        final Vector3 suppA = new Vector3();
        final Vector3 suppB = new Vector3();
        final Vector3 w = new Vector3();
        final Vector3 pA = new Vector3();
        final Vector3 pB = new Vector3();
        float vDotw;
        float prevDistSquare;
        final Transform body2ToBody1 = Transform.multiply(transform1.getInverse(), transform2);
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
            suppB.set(Transform.multiply(body2ToBody1, collisionShape2.getLocalSupportPointWithoutMargin(Matrix3x3.multiply(rotateToBody2, v))));
            w.set(Vector3.subtract(suppA, suppB));
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
                pA.set(Vector3.subtract(pA, Vector3.multiply(collisionShape1.getMargin() / dist, v)));
                pB.set(Transform.multiply(body2ToBody1.getInverse(), Vector3.add(pB, Vector3.multiply(collisionShape2.getMargin() / dist, v))));
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
                simplex.computeClosestPointsOfAAndB(pA, pB);
                final float dist = (float) Math.sqrt(distSquare);
                if (dist <= 0) {
                    throw new IllegalStateException("dist must be greater than zero");
                }
                pA.set(Vector3.subtract(pA, Vector3.multiply(collisionShape1.getMargin() / dist, v)));
                pB.set(Transform.multiply(body2ToBody1.getInverse(), Vector3.add(pB, Vector3.multiply(collisionShape2.getMargin() / dist, v))));
                final Vector3 normal = Matrix3x3.multiply(transform1.getOrientation().getMatrix(), Vector3.negate(v.getUnit()));
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
                pA.set(Vector3.subtract(pA, Vector3.multiply(collisionShape1.getMargin() / dist, v)));
                pB.set(Transform.multiply(body2ToBody1.getInverse(), Vector3.add(pB, Vector3.multiply(collisionShape2.getMargin() / dist, v))));
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
            if (prevDistSquare - distSquare <= ReactDefaults.MACHINE_EPSILON * prevDistSquare) {
                simplex.backupClosestPointInSimplex(v);
                distSquare = v.lengthSquare();
                simplex.computeClosestPointsOfAAndB(pA, pB);
                final float dist = (float) Math.sqrt(distSquare);
                if (dist <= 0) {
                    throw new IllegalStateException("dist must be greater than zero");
                }
                pA.set(Vector3.subtract(pA, Vector3.multiply(collisionShape1.getMargin() / dist, v)));
                pB.set(Transform.multiply(body2ToBody1.getInverse(), Vector3.add(pB, Vector3.multiply(collisionShape2.getMargin() / dist, v))));
                final Vector3 normal = Matrix3x3.multiply(transform1.getOrientation().getMatrix(), Vector3.negate(v.getUnit()));
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
                                                              ContactPointInfo contactInfo, Vector3 v) {
        final Simplex simplex = new Simplex();
        final Vector3 suppA = new Vector3();
        final Vector3 suppB = new Vector3();
        final Vector3 w = new Vector3();
        float vDotw;
        float distSquare = Float.MAX_VALUE;
        float prevDistSquare;
        final Transform body2ToBody1 = Transform.multiply(transform1.getInverse(), transform2);
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
            if (prevDistSquare - distSquare <= ReactDefaults.MACHINE_EPSILON * prevDistSquare) {
                return false;
            }
        }
        while (!simplex.isFull() && distSquare > ReactDefaults.MACHINE_EPSILON * simplex.getMaxLengthSquareOfAPoint());
        return mAlgoEPA.computePenetrationDepthAndContactPoints(simplex, collisionShape1, transform1, collisionShape2, transform2, v, contactInfo);
    }
}
