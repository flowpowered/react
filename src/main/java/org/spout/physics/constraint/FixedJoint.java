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
public class FixedJoint extends Constraint {
    private static final float BETA = 0.2f;
    private final Vector3 mLocalAnchorPointBody1;
    private final Vector3 mLocalAnchorPointBody2;
    private final Vector3 mR1World = new Vector3();
    private final Vector3 mR2World = new Vector3();
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
        final Matrix3x3 I1 = mBody1.getInertiaTensorInverseWorld();
        final Matrix3x3 I2 = mBody2.getInertiaTensorInverseWorld();
        mR1World.set(Quaternion.multiply(orientationBody1, mLocalAnchorPointBody1));
        mR2World.set(Quaternion.multiply(orientationBody2, mLocalAnchorPointBody2));
        final Matrix3x3 skewSymmetricMatrixU1 = Matrix3x3.computeSkewSymmetricMatrixForCrossProduct(mR1World);
        final Matrix3x3 skewSymmetricMatrixU2 = Matrix3x3.computeSkewSymmetricMatrixForCrossProduct(mR2World);
        float inverseMassBodies = 0;
        if (mBody1.getIsMotionEnabled()) {
            inverseMassBodies += mBody1.getMassInverse();
        }
        if (mBody2.getIsMotionEnabled()) {
            inverseMassBodies += mBody2.getMassInverse();
        }
        final Matrix3x3 massMatrix = new Matrix3x3(inverseMassBodies, 0, 0,
                0, inverseMassBodies, 0,
                0, 0, inverseMassBodies);
        if (mBody1.getIsMotionEnabled()) {
            massMatrix.add(Matrix3x3.multiply(skewSymmetricMatrixU1, Matrix3x3.multiply(I1, skewSymmetricMatrixU1.getTranspose())));
        }
        if (mBody2.getIsMotionEnabled()) {
            massMatrix.add(Matrix3x3.multiply(skewSymmetricMatrixU2, Matrix3x3.multiply(I2, skewSymmetricMatrixU2.getTranspose())));
        }
        mInverseMassMatrixTranslation.setToZero();
        if (mBody1.getIsMotionEnabled() || mBody2.getIsMotionEnabled()) {
            mInverseMassMatrixTranslation.set(massMatrix.getInverse());
        }
        final float biasFactor = BETA / constraintSolverData.getTimeStep();
        mBiasTranslation.setToZero();
        if (mPositionCorrectionTechnique == JointsPositionCorrectionTechnique.BAUMGARTE_JOINTS) {
            mBiasTranslation.set(Vector3.multiply(biasFactor, Vector3.subtract(Vector3.subtract(Vector3.add(x2, mR2World), x1), mR1World)));
        }
        mInverseMassMatrixRotation.setToZero();
        if (mBody1.getIsMotionEnabled()) {
            mInverseMassMatrixRotation.add(I1);
        }
        if (mBody2.getIsMotionEnabled()) {
            mInverseMassMatrixRotation.add(I2);
        }
        if (mBody1.getIsMotionEnabled() || mBody2.getIsMotionEnabled()) {
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
        final Vector3 v1 = constraintSolverData.getLinearVelocities().get(mIndexBody1);
        final Vector3 v2 = constraintSolverData.getLinearVelocities().get(mIndexBody2);
        final Vector3 w1 = constraintSolverData.getAngularVelocities().get(mIndexBody1);
        final Vector3 w2 = constraintSolverData.getAngularVelocities().get(mIndexBody2);
        final float inverseMassBody1 = mBody1.getMassInverse();
        final float inverseMassBody2 = mBody2.getMassInverse();
        final Matrix3x3 I1 = mBody1.getInertiaTensorInverseWorld();
        final Matrix3x3 I2 = mBody2.getInertiaTensorInverseWorld();
        final Vector3 linearImpulseBody1 = Vector3.negate(mImpulseTranslation);
        final Vector3 angularImpulseBody1 = mImpulseTranslation.cross(mR1World);
        final Vector3 linearImpulseBody2 = mImpulseTranslation;
        final Vector3 angularImpulseBody2 = Vector3.negate(mImpulseTranslation.cross(mR2World));
        angularImpulseBody1.add(Vector3.negate(mImpulseRotation));
        angularImpulseBody2.add(mImpulseRotation);
        if (mBody1.getIsMotionEnabled()) {
            v1.add(Vector3.multiply(inverseMassBody1, linearImpulseBody1));
            w1.add(Matrix3x3.multiply(I1, angularImpulseBody1));
        }
        if (mBody2.getIsMotionEnabled()) {
            v2.add(Vector3.multiply(inverseMassBody2, linearImpulseBody2));
            w2.add(Matrix3x3.multiply(I2, angularImpulseBody2));
        }
    }

    @Override
    public void solveVelocityConstraint(ConstraintSolverData constraintSolverData) {
        final Vector3 v1 = constraintSolverData.getLinearVelocities().get(mIndexBody1);
        final Vector3 v2 = constraintSolverData.getLinearVelocities().get(mIndexBody2);
        final Vector3 w1 = constraintSolverData.getAngularVelocities().get(mIndexBody1);
        final Vector3 w2 = constraintSolverData.getAngularVelocities().get(mIndexBody2);
        final float inverseMassBody1 = mBody1.getMassInverse();
        final float inverseMassBody2 = mBody2.getMassInverse();
        final Matrix3x3 I1 = mBody1.getInertiaTensorInverseWorld();
        final Matrix3x3 I2 = mBody2.getInertiaTensorInverseWorld();
        final Vector3 JvTranslation = Vector3.subtract(Vector3.subtract(Vector3.add(v2, w2.cross(mR2World)), v1), w1.cross(mR1World));
        final Vector3 deltaLambda = Matrix3x3.multiply(mInverseMassMatrixTranslation, Vector3.subtract(Vector3.negate(JvTranslation), mBiasTranslation));
        mImpulseTranslation.add(deltaLambda);
        final Vector3 linearImpulseBody1 = Vector3.negate(deltaLambda);
        final Vector3 angularImpulseBody1 = deltaLambda.cross(mR1World);
        final Vector3 linearImpulseBody2 = deltaLambda;
        final Vector3 angularImpulseBody2 = Vector3.negate(deltaLambda.cross(mR2World));
        if (mBody1.getIsMotionEnabled()) {
            v1.add(Vector3.multiply(inverseMassBody1, linearImpulseBody1));
            w1.add(Matrix3x3.multiply(I1, angularImpulseBody1));
        }
        if (mBody2.getIsMotionEnabled()) {
            v2.add(Vector3.multiply(inverseMassBody2, linearImpulseBody2));
            w2.add(Matrix3x3.multiply(I2, angularImpulseBody2));
        }
        final Vector3 JvRotation = Vector3.subtract(w2, w1);
        final Vector3 deltaLambda2 = Matrix3x3.multiply(mInverseMassMatrixRotation, Vector3.subtract(Vector3.negate(JvRotation), mBiasRotation));
        mImpulseRotation.add(deltaLambda2);
        angularImpulseBody1.set(Vector3.negate(deltaLambda2));
        angularImpulseBody2.set(deltaLambda2);
        if (mBody1.getIsMotionEnabled()) {
            w1.add(Matrix3x3.multiply(I1, angularImpulseBody1));
        }
        if (mBody2.getIsMotionEnabled()) {
            w2.add(Matrix3x3.multiply(I2, angularImpulseBody2));
        }
    }

    @Override
    public void solvePositionConstraint(ConstraintSolverData constraintSolverData) {

    }

    /**
     * This structure is used to gather the information needed to create a fixed joint. This structure will be used to create the actual fixed joint.
     */
    public static class FixedJointInfo extends ConstraintInfo {
        private final Vector3 anchorPointWorldSpace = new Vector3();

        /**
         * Constructs a new fixed joint info from both bodies and the initial anchor point in world space.
         *
         * @param rigidBody1 The first body
         * @param rigidBody2 The second body
         * @param initAnchorPointWorldSpace The anchor point in world space
         */
        public FixedJointInfo(RigidBody rigidBody1, RigidBody rigidBody2, Vector3 initAnchorPointWorldSpace) {
            super(rigidBody1, rigidBody2, ConstraintType.FIXEDJOINT);
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
