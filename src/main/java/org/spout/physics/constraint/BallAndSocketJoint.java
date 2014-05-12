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
package org.spout.physics.constraint;

import org.spout.physics.constraint.ConstraintSolver.ConstraintSolverData;
import org.spout.physics.math.Matrix3x3;
import org.spout.physics.math.Quaternion;
import org.spout.physics.math.Transform;
import org.spout.physics.math.Vector3;

/**
 * This class represents a ball-and-socket joint that allows arbitrary rotation between two bodies.
 */
public class BallAndSocketJoint extends Constraint {
    private final Vector3 mLocalAnchorPointBody1;
    private final Vector3 mLocalAnchorPointBody2;
    private final Vector3 mU1World = new Vector3();
    private final Vector3 mU2World = new Vector3();
    private final Matrix3x3 mSkewSymmetricMatrixU1World = new Matrix3x3();
    private final Matrix3x3 mSkewSymmetricMatrixU2World = new Matrix3x3();
    private final Matrix3x3 mInverseMassMatrix = new Matrix3x3();
    private final Vector3 mImpulse;

    /**
     * Constructs a new ball and socket joint from provided ball and socket joint info.
     *
     * @param jointInfo The joint info
     */
    public BallAndSocketJoint(BallAndSocketJointInfo jointInfo) {
        super(jointInfo);
        mImpulse = new Vector3(0, 0, 0);
        mLocalAnchorPointBody1 = Transform.multiply(mBody1.getTransform().getInverse(), jointInfo.getAnchorPointWorldSpace());
        mLocalAnchorPointBody2 = Transform.multiply(mBody2.getTransform().getInverse(), jointInfo.getAnchorPointWorldSpace());
    }

    @Override
    public void initBeforeSolve(ConstraintSolverData constraintSolverData) {
        mIndexBody1 = constraintSolverData.getMapBodyToConstrainedVelocityIndex().get(mBody1);
        mIndexBody2 = constraintSolverData.getMapBodyToConstrainedVelocityIndex().get(mBody2);
        final Quaternion orientationBody1 = mBody1.getTransform().getOrientation();
        final Quaternion orientationBody2 = mBody2.getTransform().getOrientation();
        final Matrix3x3 inverseInertiaTensorBody1 = mBody1.getInertiaTensorInverseWorld();
        final Matrix3x3 inverseInertiaTensorBody2 = mBody2.getInertiaTensorInverseWorld();
        mU1World.set(Quaternion.multiply(orientationBody1, mLocalAnchorPointBody1));
        mU2World.set(Quaternion.multiply(orientationBody2, mLocalAnchorPointBody2));
        final Matrix3x3 skewSymmetricMatrixU1 = Matrix3x3.computeSkewSymmetricMatrixForCrossProduct(mU1World);
        final Matrix3x3 skewSymmetricMatrixU2 = Matrix3x3.computeSkewSymmetricMatrixForCrossProduct(mU2World);
        final float inverseMassBodies = mBody1.getMassInverse() + mBody2.getMassInverse();
        final Matrix3x3 massMatrix =
                Matrix3x3.add(
                        new Matrix3x3(
                                inverseMassBodies, 0, 0,
                                0, inverseMassBodies, 0,
                                0, 0, inverseMassBodies),
                        Matrix3x3.add(
                                Matrix3x3.multiply(skewSymmetricMatrixU1, Matrix3x3.multiply(inverseInertiaTensorBody1, skewSymmetricMatrixU1.getTranspose())),
                                Matrix3x3.multiply(skewSymmetricMatrixU2, Matrix3x3.multiply(inverseInertiaTensorBody2, skewSymmetricMatrixU2.getTranspose())))
                );
        mInverseMassMatrix.set(massMatrix.getInverse());
    }

    @Override
    public void solve(ConstraintSolverData constraintSolverData) {
        final Vector3 x1 = mBody1.getTransform().getPosition();
        final Vector3 x2 = mBody2.getTransform().getPosition();
        final Vector3 v1 = constraintSolverData.getLinearVelocities().get(mIndexBody1);
        final Vector3 v2 = constraintSolverData.getLinearVelocities().get(mIndexBody2);
        final Vector3 w1 = constraintSolverData.getAngularVelocities().get(mIndexBody1);
        final Vector3 w2 = constraintSolverData.getAngularVelocities().get(mIndexBody2);
        float inverseMassBody1 = mBody1.getMassInverse();
        float inverseMassBody2 = mBody2.getMassInverse();
        final Matrix3x3 inverseInertiaTensorBody1 = mBody1.getInertiaTensorInverseWorld();
        final Matrix3x3 inverseInertiaTensorBody2 = mBody2.getInertiaTensorInverseWorld();
        final Vector3 Jv = Vector3.add(Vector3.negate(v1), Vector3.add(mU1World.cross(w1), Vector3.subtract(v2, mU2World.cross(w2))));
        final float beta = 0.2f;     // TODO : Use a constant here
        final float biasFactor = beta / constraintSolverData.getTimeStep();
        Vector3 b = Vector3.multiply(biasFactor, Vector3.subtract(Vector3.subtract(Vector3.add(x2, mU2World), x1), mU1World));
        final Vector3 deltaLambda = Matrix3x3.multiply(mInverseMassMatrix, Vector3.subtract(Vector3.negate(Jv), b));
        mImpulse.add(deltaLambda);
        final Vector3 linearImpulseBody1 = Vector3.negate(deltaLambda);
        final Vector3 angularImpulseBody1 = deltaLambda.cross(mU1World);
        final Vector3 linearImpulseBody2 = deltaLambda;
        final Vector3 angularImpulseBody2 = Vector3.negate(deltaLambda.cross(mU2World));
        if (mBody1.isMotionEnabled()) {
            v1.add(Vector3.multiply(inverseMassBody1, linearImpulseBody1));
            w1.add(Matrix3x3.multiply(inverseInertiaTensorBody1, angularImpulseBody1));
        }
        if (mBody2.isMotionEnabled()) {
            v2.add(Vector3.multiply(inverseMassBody2, linearImpulseBody2));
            w2.add(Matrix3x3.multiply(inverseInertiaTensorBody2, angularImpulseBody2));
        }
    }

    /**
     * This structure is used to gather the information needed to create a ball-and-socket joint. This structure will be used to create the actual ball-and-socket joint.
     */
    public static class BallAndSocketJointInfo extends ConstraintInfo {
        private final Vector3 anchorPointWorldSpace = new Vector3();

        public BallAndSocketJointInfo() {
            super(ConstraintType.BALLSOCKETJOINT);
        }

        /**
         * Returns the anchor point in world space.
         *
         * @return The anchor point in world space
         */
        public Vector3 getAnchorPointWorldSpace() {
            return anchorPointWorldSpace;
        }

        /**
         * Sets the anchor point position, in world space.
         *
         * @param anchorPointWorldSpace The anchor point world space position
         */
        public void setAnchorPointWorldSpace(Vector3 anchorPointWorldSpace) {
            this.anchorPointWorldSpace.set(anchorPointWorldSpace);
        }
    }
}
