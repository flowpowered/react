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

import org.spout.physics.ReactDefaults.JointsPositionCorrectionTechnique;
import org.spout.physics.body.RigidBody;
import org.spout.physics.constraint.ConstraintSolver.ConstraintSolverData;
import org.spout.physics.math.Matrix3x3;
import org.spout.physics.math.Quaternion;
import org.spout.physics.math.Transform;
import org.spout.physics.math.Vector3;

/**
 * This class represents a fixed joint that is used to forbid any translation or rotation between two bodies.
 */
public class FixedJoint extends Joint {
    private static final float BETA = 0.2f;
    private final Vector3 mLocalAnchorPointBody1;
    private final Vector3 mLocalAnchorPointBody2;
    private final Vector3 mR1World = new Vector3();
    private final Vector3 mR2World = new Vector3();
    private final Matrix3x3 mI1 = new Matrix3x3();
    private final Matrix3x3 mI2 = new Matrix3x3();
    private final Vector3 mImpulseTranslation;
    private final Vector3 mImpulseRotation;
    private final Matrix3x3 mInverseMassMatrixTranslation = new Matrix3x3();
    private final Matrix3x3 mInverseMassMatrixRotation = new Matrix3x3();
    private final Vector3 mBiasTranslation = new Vector3();
    private final Vector3 mBiasRotation = new Vector3();
    private final Quaternion mInitOrientationDifferenceInv;

    /**
     * Constructs a fixed joint from provided fixed joint info.
     *
     * @param jointInfo The joint info
     */
    public FixedJoint(FixedJointInfo jointInfo) {
        super(jointInfo);
        mImpulseTranslation = new Vector3(0, 0, 0);
        mImpulseRotation = new Vector3(0, 0, 0);
        final Transform transform1 = mBody1.getTransform();
        final Transform transform2 = mBody2.getTransform();
        mLocalAnchorPointBody1 = Transform.multiply(transform1.getInverse(), jointInfo.getAnchorPointWorldSpace());
        mLocalAnchorPointBody2 = Transform.multiply(transform2.getInverse(), jointInfo.getAnchorPointWorldSpace());
        mInitOrientationDifferenceInv = Quaternion.multiply(transform2.getOrientation(), transform1.getOrientation().getInverse());
        mInitOrientationDifferenceInv.normalize();
        mInitOrientationDifferenceInv.inverse();
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
        mInverseMassMatrixTranslation.setToZero();
        if (mBody1.isMotionEnabled() || mBody2.isMotionEnabled()) {
            mInverseMassMatrixTranslation.set(massMatrix.getInverse());
        }
        final float biasFactor = BETA / constraintSolverData.getTimeStep();
        mBiasTranslation.setToZero();
        if (mPositionCorrectionTechnique == JointsPositionCorrectionTechnique.BAUMGARTE_JOINTS) {
            mBiasTranslation.set(Vector3.multiply(biasFactor, Vector3.subtract(Vector3.subtract(Vector3.add(x2, mR2World), x1), mR1World)));
        }
        mInverseMassMatrixRotation.setToZero();
        if (mBody1.isMotionEnabled()) {
            mInverseMassMatrixRotation.add(mI1);
        }
        if (mBody2.isMotionEnabled()) {
            mInverseMassMatrixRotation.add(mI2);
        }
        if (mBody1.isMotionEnabled() || mBody2.isMotionEnabled()) {
            mInverseMassMatrixRotation.set(mInverseMassMatrixRotation.getInverse());
        }
        mBiasRotation.setToZero();
        if (mPositionCorrectionTechnique == JointsPositionCorrectionTechnique.BAUMGARTE_JOINTS) {
            final Quaternion currentOrientationDifference = Quaternion.multiply(orientationBody2, orientationBody1.getInverse());
            currentOrientationDifference.normalize();
            final Quaternion qError = Quaternion.multiply(currentOrientationDifference, mInitOrientationDifferenceInv);
            mBiasRotation.set(Vector3.multiply(biasFactor * 2, qError.getVectorV()));
        }
        if (!constraintSolverData.isWarmStartingActive()) {
            mImpulseTranslation.setToZero();
            mImpulseRotation.setToZero();
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
            final Vector3 linearImpulseBody1 = Vector3.negate(mImpulseTranslation);
            final Vector3 angularImpulseBody1 = mImpulseTranslation.cross(mR1World);
            angularImpulseBody1.add(Vector3.negate(mImpulseRotation));
            v1.add(Vector3.multiply(inverseMassBody1, linearImpulseBody1));
            w1.add(Matrix3x3.multiply(mI1, angularImpulseBody1));
        }
        if (mBody2.isMotionEnabled()) {
            final Vector3 linearImpulseBody2 = mImpulseTranslation;
            final Vector3 angularImpulseBody2 = Vector3.negate(mImpulseTranslation.cross(mR2World));
            angularImpulseBody2.add(mImpulseRotation);
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
        final float inverseMassBody1 = mBody1.getMassInverse();
        final float inverseMassBody2 = mBody2.getMassInverse();
        final Vector3 JvTranslation = Vector3.subtract(Vector3.subtract(Vector3.add(v2, w2.cross(mR2World)), v1), w1.cross(mR1World));
        final Vector3 deltaLambda = Matrix3x3.multiply(mInverseMassMatrixTranslation, Vector3.subtract(Vector3.negate(JvTranslation), mBiasTranslation));
        mImpulseTranslation.add(deltaLambda);
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
        final Vector3 JvRotation = Vector3.subtract(w2, w1);
        final Vector3 deltaLambda2 = Matrix3x3.multiply(mInverseMassMatrixRotation, Vector3.subtract(Vector3.negate(JvRotation), mBiasRotation));
        mImpulseRotation.add(deltaLambda2);
        if (mBody1.isMotionEnabled()) {
            final Vector3 angularImpulseBody1 = Vector3.negate(deltaLambda2);
            w1.add(Matrix3x3.multiply(mI1, angularImpulseBody1));
        }
        if (mBody2.isMotionEnabled()) {
            final Vector3 angularImpulseBody2 = deltaLambda2;
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
        mInverseMassMatrixTranslation.setToZero();
        if (mBody1.isMotionEnabled() || mBody2.isMotionEnabled()) {
            mInverseMassMatrixTranslation.set(massMatrix.getInverse());
        }
        final Vector3 errorTranslation = Vector3.subtract(Vector3.subtract(Vector3.add(x2, mR2World), x1), mR1World);
        final Vector3 lambdaTranslation = Matrix3x3.multiply(mInverseMassMatrixTranslation, Vector3.negate(errorTranslation));
        if (mBody1.isMotionEnabled()) {
            final Vector3 linearImpulseBody1 = Vector3.negate(lambdaTranslation);
            final Vector3 angularImpulseBody1 = lambdaTranslation.cross(mR1World);
            final Vector3 v1 = Vector3.multiply(inverseMassBody1, linearImpulseBody1);
            final Vector3 w1 = Matrix3x3.multiply(mI1, angularImpulseBody1);
            x1.add(v1);
            q1.add(Quaternion.multiply(Quaternion.multiply(new Quaternion(0, w1), q1), 0.5f));
            q1.normalize();
        }
        if (mBody2.isMotionEnabled()) {
            final Vector3 linearImpulseBody2 = lambdaTranslation;
            final Vector3 angularImpulseBody2 = Vector3.negate(lambdaTranslation.cross(mR2World));
            final Vector3 v2 = Vector3.multiply(inverseMassBody2, linearImpulseBody2);
            final Vector3 w2 = Matrix3x3.multiply(mI2, angularImpulseBody2);
            x2.add(v2);
            q2.add(Quaternion.multiply(Quaternion.multiply(new Quaternion(0, w2), q2), 0.5f));
            q2.normalize();
        }
        mInverseMassMatrixRotation.setToZero();
        if (mBody1.isMotionEnabled()) {
            mInverseMassMatrixRotation.add(mI1);
        }
        if (mBody2.isMotionEnabled()) {
            mInverseMassMatrixRotation.add(mI2);
        }
        if (mBody1.isMotionEnabled() || mBody2.isMotionEnabled()) {
            mInverseMassMatrixRotation.set(mInverseMassMatrixRotation.getInverse());
        }
        final Quaternion currentOrientationDifference = Quaternion.multiply(q2, q1.getInverse());
        currentOrientationDifference.normalize();
        final Quaternion qError = Quaternion.multiply(currentOrientationDifference, mInitOrientationDifferenceInv);
        final Vector3 errorRotation = Vector3.multiply(2, qError.getVectorV());
        final Vector3 lambdaRotation = Matrix3x3.multiply(mInverseMassMatrixRotation, Vector3.negate(errorRotation));
        if (mBody1.isMotionEnabled()) {
            final Vector3 angularImpulseBody1 = Vector3.negate(lambdaRotation);
            final Vector3 w1 = Matrix3x3.multiply(mI1, angularImpulseBody1);
            q1.add(Quaternion.multiply(Quaternion.multiply(new Quaternion(0, w1), q1), 0.5f));
            q1.normalize();
        }
        if (mBody2.isMotionEnabled()) {
            final Vector3 angularImpulseBody2 = lambdaRotation;
            final Vector3 w2 = Matrix3x3.multiply(mI2, angularImpulseBody2);
            q2.add(Quaternion.multiply(Quaternion.multiply(new Quaternion(0, w2), q2), 0.5f));
            q2.normalize();
        }
    }

    /**
     * This structure is used to gather the information needed to create a fixed joint. This structure will be used to create the actual fixed joint.
     */
    public static class FixedJointInfo extends JointInfo {
        private final Vector3 anchorPointWorldSpace = new Vector3();

        /**
         * Constructs a new fixed joint info from both bodies and the initial anchor point in world space.
         *
         * @param rigidBody1 The first body
         * @param rigidBody2 The second body
         * @param initAnchorPointWorldSpace The anchor point in world space
         */
        public FixedJointInfo(RigidBody rigidBody1, RigidBody rigidBody2, Vector3 initAnchorPointWorldSpace) {
            super(rigidBody1, rigidBody2, JointType.FIXEDJOINT);
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
