/*
 * This file is part of Flow React, licensed under the MIT License (MIT).
 *
 * Copyright (c) 2013 Flow Powered <https://flowpowered.com/>
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://danielchappuis.ch>
 * Flow React is re-licensed with permission from ReactPhysics3D author.
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
package com.flowpowered.react.constraint;

import com.flowpowered.react.ReactDefaults.JointsPositionCorrectionTechnique;
import com.flowpowered.react.body.RigidBody;
import com.flowpowered.react.constraint.ConstraintSolver.ConstraintSolverData;
import com.flowpowered.react.math.Matrix3x3;
import com.flowpowered.react.math.Quaternion;
import com.flowpowered.react.math.Transform;
import com.flowpowered.react.math.Vector3;

/**
 * This class represents a ball-and-socket joint that allows arbitrary rotation between two bodies. This joint has three degrees of freedom. It can be used to create a chain of bodies for instance.
 */
public class BallAndSocketJoint extends Joint {
    private static final float BETA = 0.2f;
    private final Vector3 mLocalAnchorPointBody1;
    private final Vector3 mLocalAnchorPointBody2;
    private final Vector3 mR1World = new Vector3();
    private final Vector3 mR2World = new Vector3();
    private final Matrix3x3 mI1 = new Matrix3x3();
    private final Matrix3x3 mI2 = new Matrix3x3();
    private final Vector3 mBiasVector = new Vector3();
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
        final Vector3 x1 = mBody1.getTransform().getPosition();
        final Vector3 x2 = mBody2.getTransform().getPosition();
        final Quaternion orientationBody1 = mBody1.getTransform().getOrientation();
        final Quaternion orientationBody2 = mBody2.getTransform().getOrientation();
        mI1.set(mBody1.getInertiaTensorInverseWorld());
        mI2.set(mBody2.getInertiaTensorInverseWorld());
        mR1World.set(Quaternion.multiply(orientationBody1, mLocalAnchorPointBody1));
        mR2World.set(Quaternion.multiply(orientationBody2, mLocalAnchorPointBody2));
        final Matrix3x3 skewSymmetricMatrixU1 = Matrix3x3.computeSkewSymmetricMatrixForCrossProduct(mR1World);
        final Matrix3x3 skewSymmetricMatrixU2 = Matrix3x3.computeSkewSymmetricMatrixForCrossProduct(mR2World);
        float inverseMassBodies = 0;
        if (mBody1.isMotionEnabled()) {
            inverseMassBodies += mBody1.getMassInverse();
        }
        if (mBody2.isMotionEnabled()) {
            inverseMassBodies += mBody2.getMassInverse();
        }
        final Matrix3x3 massMatrix = new Matrix3x3(
                inverseMassBodies, 0, 0,
                0, inverseMassBodies, 0,
                0, 0, inverseMassBodies);
        if (mBody1.isMotionEnabled()) {
            massMatrix.add(Matrix3x3.multiply(skewSymmetricMatrixU1, Matrix3x3.multiply(mI1, skewSymmetricMatrixU1.getTranspose())));
        }
        if (mBody2.isMotionEnabled()) {
            massMatrix.add(Matrix3x3.multiply(skewSymmetricMatrixU2, Matrix3x3.multiply(mI2, skewSymmetricMatrixU2.getTranspose())));
        }
        mInverseMassMatrix.setToZero();
        if (mBody1.isMotionEnabled() || mBody2.isMotionEnabled()) {
            mInverseMassMatrix.set(massMatrix.getInverse());
        }
        mBiasVector.setToZero();
        if (mPositionCorrectionTechnique == JointsPositionCorrectionTechnique.BAUMGARTE_JOINTS) {
            final float biasFactor = BETA / constraintSolverData.getTimeStep();
            mBiasVector.set(Vector3.multiply(biasFactor, Vector3.subtract(Vector3.subtract(Vector3.add(x2, mR2World), x1), mR1World)));
        }
        if (!constraintSolverData.isWarmStartingActive()) {
            mImpulse.setToZero();
        }
    }

    @Override
    public void warmstart(ConstraintSolverData constraintSolverData) {
        final Vector3 v1 = constraintSolverData.getLinearVelocities()[mIndexBody1];
        final Vector3 v2 = constraintSolverData.getLinearVelocities()[mIndexBody2];
        final Vector3 w1 = constraintSolverData.getAngularVelocities()[mIndexBody1];
        final Vector3 w2 = constraintSolverData.getAngularVelocities()[mIndexBody2];
        final float inverseMassBody1 = mBody1.getMassInverse();
        final float inverseMassBody2 = mBody2.getMassInverse();
        if (mBody1.isMotionEnabled()) {
            final Vector3 linearImpulseBody1 = Vector3.negate(mImpulse);
            final Vector3 angularImpulseBody1 = mImpulse.cross(mR1World);
            v1.add(Vector3.multiply(inverseMassBody1, linearImpulseBody1));
            w1.add(Matrix3x3.multiply(mI1, angularImpulseBody1));
        }
        if (mBody2.isMotionEnabled()) {
            final Vector3 linearImpulseBody2 = mImpulse;
            final Vector3 angularImpulseBody2 = Vector3.negate(mImpulse.cross(mR2World));
            v2.add(Vector3.multiply(inverseMassBody2, linearImpulseBody2));
            w2.add(Matrix3x3.multiply(mI2, angularImpulseBody2));
        }
    }

    @Override
    public void solveVelocityConstraint(ConstraintSolverData constraintSolverData) {
        final Vector3 v1 = constraintSolverData.getLinearVelocities()[mIndexBody1];
        final Vector3 v2 = constraintSolverData.getLinearVelocities()[mIndexBody2];
        final Vector3 w1 = constraintSolverData.getAngularVelocities()[mIndexBody1];
        final Vector3 w2 = constraintSolverData.getAngularVelocities()[mIndexBody2];
        float inverseMassBody1 = mBody1.getMassInverse();
        float inverseMassBody2 = mBody2.getMassInverse();
        final Vector3 Jv = Vector3.subtract(Vector3.subtract(Vector3.add(v2, w2.cross(mR2World)), v1), w1.cross(mR1World));
        final Vector3 deltaLambda = Matrix3x3.multiply(mInverseMassMatrix, Vector3.subtract(Vector3.negate(Jv), mBiasVector));
        mImpulse.add(deltaLambda);
        if (mBody1.isMotionEnabled()) {
            final Vector3 linearImpulseBody1 = Vector3.negate(deltaLambda);
            final Vector3 angularImpulseBody1 = deltaLambda.cross(mR1World);
            v1.add(Vector3.multiply(inverseMassBody1, linearImpulseBody1));
            w1.add(Matrix3x3.multiply(mI1, angularImpulseBody1));
        }
        if (mBody2.isMotionEnabled()) {
            final Vector3 linearImpulseBody2 = deltaLambda;
            final Vector3 angularImpulseBody2 = Vector3.negate(deltaLambda.cross(mR2World));
            v2.add(Vector3.multiply(inverseMassBody2, linearImpulseBody2));
            w2.add(Matrix3x3.multiply(mI2, angularImpulseBody2));
        }
    }

    @Override
    public void solvePositionConstraint(ConstraintSolverData constraintSolverData) {
        if (mPositionCorrectionTechnique != JointsPositionCorrectionTechnique.NON_LINEAR_GAUSS_SEIDEL) {
            return;
        }
        final Vector3 x1 = constraintSolverData.getPositions().get(mIndexBody1);
        final Vector3 x2 = constraintSolverData.getPositions().get(mIndexBody2);
        final Quaternion q1 = constraintSolverData.getOrientations().get(mIndexBody1);
        final Quaternion q2 = constraintSolverData.getOrientations().get(mIndexBody2);
        final float inverseMassBody1 = mBody1.getMassInverse();
        final float inverseMassBody2 = mBody2.getMassInverse();
        mI1.set(mBody1.getInertiaTensorInverseWorld());
        mI2.set(mBody2.getInertiaTensorInverseWorld());
        mR1World.set(Quaternion.multiply(q1, mLocalAnchorPointBody1));
        mR2World.set(Quaternion.multiply(q2, mLocalAnchorPointBody2));
        final Matrix3x3 skewSymmetricMatrixU1 = Matrix3x3.computeSkewSymmetricMatrixForCrossProduct(mR1World);
        final Matrix3x3 skewSymmetricMatrixU2 = Matrix3x3.computeSkewSymmetricMatrixForCrossProduct(mR2World);
        float inverseMassBodies = 0;
        if (mBody1.isMotionEnabled()) {
            inverseMassBodies += inverseMassBody1;
        }
        if (mBody2.isMotionEnabled()) {
            inverseMassBodies += inverseMassBody2;
        }
        final Matrix3x3 massMatrix = new Matrix3x3(
                inverseMassBodies, 0, 0,
                0, inverseMassBodies, 0,
                0, 0, inverseMassBodies);
        if (mBody1.isMotionEnabled()) {
            massMatrix.add(Matrix3x3.multiply(skewSymmetricMatrixU1, Matrix3x3.multiply(mI1, skewSymmetricMatrixU1.getTranspose())));
        }
        if (mBody2.isMotionEnabled()) {
            massMatrix.add(Matrix3x3.multiply(skewSymmetricMatrixU2, Matrix3x3.multiply(mI2, skewSymmetricMatrixU2.getTranspose())));
        }
        mInverseMassMatrix.setToZero();
        if (mBody1.isMotionEnabled() || mBody2.isMotionEnabled()) {
            mInverseMassMatrix.set(massMatrix.getInverse());
        }
        final Vector3 constraintError = Vector3.subtract(Vector3.subtract(Vector3.add(x2, mR2World), x1), mR1World);
        final Vector3 lambda = Matrix3x3.multiply(mInverseMassMatrix, Vector3.negate(constraintError));
        if (mBody1.isMotionEnabled()) {
            final Vector3 linearImpulseBody1 = Vector3.negate(lambda);
            final Vector3 angularImpulseBody1 = lambda.cross(mR1World);
            final Vector3 v1 = Vector3.multiply(inverseMassBody1, linearImpulseBody1);
            final Vector3 w1 = Matrix3x3.multiply(mI1, angularImpulseBody1);
            x1.add(v1);
            q1.add(Quaternion.multiply(Quaternion.multiply(new Quaternion(0, w1), q1), 0.5f));
            q1.normalize();
        }
        if (mBody2.isMotionEnabled()) {
            final Vector3 linearImpulseBody2 = lambda;
            final Vector3 angularImpulseBody2 = Vector3.negate(lambda.cross(mR2World));
            final Vector3 v2 = Vector3.multiply(inverseMassBody2, linearImpulseBody2);
            final Vector3 w2 = Matrix3x3.multiply(mI2, angularImpulseBody2);
            x2.add(v2);
            q2.add(Quaternion.multiply(Quaternion.multiply(new Quaternion(0, w2), q2), 0.5f));
            q2.normalize();
        }
    }

    /**
     * This structure is used to gather the information needed to create a ball-and-socket joint. This structure will be used to create the actual ball-and-socket joint.
     */
    public static class BallAndSocketJointInfo extends JointInfo {
        private final Vector3 anchorPointWorldSpace = new Vector3();

        /**
         * Constructs a new ball and socket joint info from both bodies and the initial anchor point in world space.
         *
         * @param rigidBody1 The first body
         * @param rigidBody2 The second body
         * @param initAnchorPointWorldSpace The anchor point in world space
         */
        public BallAndSocketJointInfo(RigidBody rigidBody1, RigidBody rigidBody2, Vector3 initAnchorPointWorldSpace) {
            super(rigidBody1, rigidBody2, JointType.BALLSOCKETJOINT);
            anchorPointWorldSpace.set(initAnchorPointWorldSpace);
        }

        /**
         * Returns the anchor point in world space.
         *
         * @return The anchor point in world space
         */
        public Vector3 getAnchorPointWorldSpace() {
            return anchorPointWorldSpace;
        }
    }
}
