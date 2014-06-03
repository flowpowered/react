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
import org.spout.physics.math.Mathematics;
import org.spout.physics.math.Matrix2x2;
import org.spout.physics.math.Matrix3x3;
import org.spout.physics.math.Quaternion;
import org.spout.physics.math.Transform;
import org.spout.physics.math.Vector2;
import org.spout.physics.math.Vector3;

/**
 * This class represents a slider joint.
 */
public class SliderJoint extends Constraint {
    private static final float BETA = 0.2f;
    private final Vector3 mLocalAnchorPointBody1;
    private final Vector3 mLocalAnchorPointBody2;
    private final Vector3 mSliderAxisBody1;
    private final Matrix3x3 mI1 = new Matrix3x3();
    private final Matrix3x3 mI2 = new Matrix3x3();
    private final Quaternion mInitOrientationDifferenceInv;
    private final Vector3 mN1 = new Vector3();
    private final Vector3 mN2 = new Vector3();
    private final Vector3 mR1 = new Vector3();
    private final Vector3 mR2 = new Vector3();
    private final Vector3 mR2CrossN1 = new Vector3();
    private final Vector3 mR2CrossN2 = new Vector3();
    private final Vector3 mR2CrossSliderAxis = new Vector3();
    private final Vector3 mR1PlusUCrossN1 = new Vector3();
    private final Vector3 mR1PlusUCrossN2 = new Vector3();
    private final Vector3 mR1PlusUCrossSliderAxis = new Vector3();
    private final Vector2 mBTranslation = new Vector2();
    private final Vector3 mBRotation = new Vector3();
    private float mBLowerLimit;
    private float mBUpperLimit;
    private final Matrix2x2 mInverseMassMatrixTranslationConstraint = new Matrix2x2();
    private final Matrix3x3 mInverseMassMatrixRotationConstraint = new Matrix3x3();
    private float mInverseMassMatrixLimit;
    private float mInverseMassMatrixMotor;
    private final Vector2 mImpulseTranslation;
    private final Vector3 mImpulseRotation;
    private float mImpulseLowerLimit;
    private float mImpulseUpperLimit;
    private float mImpulseMotor;
    private boolean mIsLimitEnabled;
    private boolean mIsMotorEnabled;
    private final Vector3 mSliderAxisWorld = new Vector3();
    private float mLowerLimit;
    private float mUpperLimit;
    private boolean mIsLowerLimitViolated;
    private boolean mIsUpperLimitViolated;
    private float mMotorSpeed;
    private float mMaxMotorForce;

    /**
     * Constructs a slider joint from provided slider joint info.
     *
     * @param jointInfo The joint info
     */
    public SliderJoint(SliderJointInfo jointInfo) {
        super(jointInfo);
        if (jointInfo.getMaxTranslationLimit() < 0) {
            throw new IllegalArgumentException("Upper limit must be greater or equal to 0");
        }
        if (jointInfo.getMinTranslationLimit() > 0) {
            throw new IllegalArgumentException("Lower limit must be smaller or equal to 0");
        }
        if (jointInfo.getMaxMotorForce() < 0) {
            throw new IllegalArgumentException("Max motor force must be greater or equal to 0");
        }
        mImpulseTranslation = new Vector2(0, 0);
        mImpulseRotation = new Vector3(0, 0, 0);
        mImpulseLowerLimit = 0;
        mImpulseUpperLimit = 0;
        mImpulseMotor = 0;
        mIsLimitEnabled = jointInfo.isLimitEnabled();
        mIsMotorEnabled = jointInfo.isMotorEnabled();
        mLowerLimit = jointInfo.getMinTranslationLimit();
        mUpperLimit = jointInfo.getMaxTranslationLimit();
        mIsLowerLimitViolated = false;
        mIsUpperLimitViolated = false;
        mMotorSpeed = jointInfo.getMotorSpeed();
        mMaxMotorForce = jointInfo.getMaxMotorForce();
        final Transform transform1 = mBody1.getTransform();
        final Transform transform2 = mBody2.getTransform();
        mLocalAnchorPointBody1 = Transform.multiply(transform1.getInverse(), jointInfo.getAnchorPointWorldSpace());
        mLocalAnchorPointBody2 = Transform.multiply(transform2.getInverse(), jointInfo.getAnchorPointWorldSpace());
        mInitOrientationDifferenceInv = Quaternion.multiply(transform2.getOrientation(), transform1.getOrientation().getInverse());
        mInitOrientationDifferenceInv.normalize();
        mInitOrientationDifferenceInv.inverse();
        mSliderAxisBody1 = Quaternion.multiply(mBody1.getTransform().getOrientation().getInverse(), jointInfo.getSliderAxisWorldSpace());
        mSliderAxisBody1.normalize();
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
        mR1.set(Quaternion.multiply(orientationBody1, mLocalAnchorPointBody1));
        mR2.set(Quaternion.multiply(orientationBody2, mLocalAnchorPointBody2));
        final Vector3 u = Vector3.subtract(Vector3.subtract(Vector3.add(x2, mR2), x1), mR1);
        mSliderAxisWorld.set(Quaternion.multiply(orientationBody1, mSliderAxisBody1));
        mSliderAxisWorld.normalize();
        mN1.set(mSliderAxisWorld.getOneUnitOrthogonalVector());
        mN2.set(mSliderAxisWorld.cross(mN1));
        final float uDotSliderAxis = u.dot(mSliderAxisWorld);
        final float lowerLimitError = uDotSliderAxis - mLowerLimit;
        final float upperLimitError = mUpperLimit - uDotSliderAxis;
        final boolean oldIsLowerLimitViolated = mIsLowerLimitViolated;
        mIsLowerLimitViolated = lowerLimitError <= 0;
        if (mIsLowerLimitViolated != oldIsLowerLimitViolated) {
            mImpulseLowerLimit = 0;
        }
        final boolean oldIsUpperLimitViolated = mIsUpperLimitViolated;
        mIsUpperLimitViolated = upperLimitError <= 0;
        if (mIsUpperLimitViolated != oldIsUpperLimitViolated) {
            mImpulseUpperLimit = 0;
        }
        mR2CrossN1.set(mR2.cross(mN1));
        mR2CrossN2.set(mR2.cross(mN2));
        mR2CrossSliderAxis.set(mR2.cross(mSliderAxisWorld));
        final Vector3 r1PlusU = Vector3.add(mR1, u);
        mR1PlusUCrossN1.set(r1PlusU.cross(mN1));
        mR1PlusUCrossN2.set(r1PlusU.cross(mN2));
        mR1PlusUCrossSliderAxis.set(r1PlusU.cross(mSliderAxisWorld));
        float sumInverseMass = 0;
        final Vector3 I1R1PlusUCrossN1 = new Vector3(0, 0, 0);
        final Vector3 I1R1PlusUCrossN2 = new Vector3(0, 0, 0);
        final Vector3 I2R2CrossN1 = new Vector3(0, 0, 0);
        final Vector3 I2R2CrossN2 = new Vector3(0, 0, 0);
        if (mBody1.getIsMotionEnabled()) {
            sumInverseMass += mBody1.getMassInverse();
            I1R1PlusUCrossN1.set(Matrix3x3.multiply(mI1, mR1PlusUCrossN1));
            I1R1PlusUCrossN2.set(Matrix3x3.multiply(mI1, mR1PlusUCrossN2));
        }
        if (mBody2.getIsMotionEnabled()) {
            sumInverseMass += mBody2.getMassInverse();
            I2R2CrossN1.set(Matrix3x3.multiply(mI2, mR2CrossN1));
            I2R2CrossN2.set(Matrix3x3.multiply(mI2, mR2CrossN2));
        }
        final float el11 = sumInverseMass + mR1PlusUCrossN1.dot(I1R1PlusUCrossN1) + mR2CrossN1.dot(I2R2CrossN1);
        final float el12 = mR1PlusUCrossN1.dot(I1R1PlusUCrossN2) + mR2CrossN1.dot(I2R2CrossN2);
        final float el21 = mR1PlusUCrossN2.dot(I1R1PlusUCrossN1) + mR2CrossN2.dot(I2R2CrossN1);
        final float el22 = sumInverseMass + mR1PlusUCrossN2.dot(I1R1PlusUCrossN2) + mR2CrossN2.dot(I2R2CrossN2);
        final Matrix2x2 matrixKTranslation = new Matrix2x2(el11, el12, el21, el22);
        mInverseMassMatrixTranslationConstraint.setToZero();
        if (mBody1.getIsMotionEnabled() || mBody2.getIsMotionEnabled()) {
            mInverseMassMatrixTranslationConstraint.set(matrixKTranslation.getInverse());
        }
        mBTranslation.setToZero();
        final float biasFactor = BETA / constraintSolverData.getTimeStep();
        if (mPositionCorrectionTechnique == JointsPositionCorrectionTechnique.BAUMGARTE_JOINTS) {
            mBTranslation.setX(u.dot(mN1));
            mBTranslation.setY(u.dot(mN2));
            mBTranslation.multiply(biasFactor);
        }
        mInverseMassMatrixRotationConstraint.setToZero();
        if (mBody1.getIsMotionEnabled()) {
            mInverseMassMatrixRotationConstraint.add(mI1);
        }
        if (mBody2.getIsMotionEnabled()) {
            mInverseMassMatrixRotationConstraint.add(mI2);
        }
        if (mBody1.getIsMotionEnabled() || mBody2.getIsMotionEnabled()) {
            mInverseMassMatrixRotationConstraint.set(mInverseMassMatrixRotationConstraint.getInverse());
        }
        mBRotation.setToZero();
        if (mPositionCorrectionTechnique == JointsPositionCorrectionTechnique.BAUMGARTE_JOINTS) {
            final Quaternion currentOrientationDifference = Quaternion.multiply(orientationBody2, orientationBody1.getInverse());
            currentOrientationDifference.normalize();
            final Quaternion qError = Quaternion.multiply(currentOrientationDifference, mInitOrientationDifferenceInv);
            mBRotation.set(Vector3.multiply(biasFactor * 2, qError.getVectorV()));
        }
        if (mIsLimitEnabled && (mIsLowerLimitViolated || mIsUpperLimitViolated)) {
            mInverseMassMatrixLimit = 0;
            if (mBody1.getIsMotionEnabled()) {
                mInverseMassMatrixLimit += mBody1.getMassInverse() + mR1PlusUCrossSliderAxis.dot(Matrix3x3.multiply(mI1, mR1PlusUCrossSliderAxis));
            }
            if (mBody2.getIsMotionEnabled()) {
                mInverseMassMatrixLimit += mBody2.getMassInverse() + mR2CrossSliderAxis.dot(Matrix3x3.multiply(mI2, mR2CrossSliderAxis));
            }
            mInverseMassMatrixLimit = mInverseMassMatrixLimit > 0 ? 1 / mInverseMassMatrixLimit : 0;
            mBLowerLimit = 0;
            if (mPositionCorrectionTechnique == JointsPositionCorrectionTechnique.BAUMGARTE_JOINTS) {
                mBLowerLimit = biasFactor * lowerLimitError;
            }
            mBUpperLimit = 0;
            if (mPositionCorrectionTechnique == JointsPositionCorrectionTechnique.BAUMGARTE_JOINTS) {
                mBUpperLimit = biasFactor * upperLimitError;
            }
        }
        mInverseMassMatrixMotor = 0;
        if (mBody1.getIsMotionEnabled()) {
            mInverseMassMatrixMotor += mBody1.getMassInverse();
        }
        if (mBody2.getIsMotionEnabled()) {
            mInverseMassMatrixMotor += mBody2.getMassInverse();
        }
        mInverseMassMatrixMotor = mInverseMassMatrixMotor > 0 ? 1 / mInverseMassMatrixMotor : 0;
        if (!constraintSolverData.isWarmStartingActive()) {
            mImpulseTranslation.setToZero();
            mImpulseRotation.setToZero();
            mImpulseLowerLimit = 0;
            mImpulseUpperLimit = 0;
            mImpulseMotor = 0;
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
        final float impulseLimits = mImpulseUpperLimit - mImpulseLowerLimit;
        final Vector3 linearImpulseLimits = Vector3.multiply(impulseLimits, mSliderAxisWorld);
        final Vector3 impulseMotor = Vector3.multiply(mImpulseMotor, mSliderAxisWorld);
        if (mBody1.getIsMotionEnabled()) {
            final Vector3 linearImpulseBody1 = Vector3.subtract(Vector3.multiply(Vector3.negate(mN1), mImpulseTranslation.getX()), Vector3.multiply(mN2, mImpulseTranslation.getY()));
            final Vector3 angularImpulseBody1 = Vector3.subtract(Vector3.multiply(Vector3.negate(mR1PlusUCrossN1), mImpulseTranslation.getX()),
                    Vector3.multiply(mR1PlusUCrossN2, mImpulseTranslation.getY()));
            angularImpulseBody1.add(Vector3.negate(mImpulseRotation));
            linearImpulseBody1.add(linearImpulseLimits);
            angularImpulseBody1.add(Vector3.multiply(impulseLimits, mR1PlusUCrossSliderAxis));
            linearImpulseBody1.add(impulseMotor);
            v1.add(Vector3.multiply(inverseMassBody1, linearImpulseBody1));
            w1.add(Matrix3x3.multiply(mI1, angularImpulseBody1));
        }
        if (mBody2.getIsMotionEnabled()) {
            final Vector3 linearImpulseBody2 = Vector3.add(Vector3.multiply(mN1, mImpulseTranslation.getX()), Vector3.multiply(mN2, mImpulseTranslation.getY()));
            final Vector3 angularImpulseBody2 = Vector3.add(Vector3.multiply(mR2CrossN1, mImpulseTranslation.getX()), Vector3.multiply(mR2CrossN2, mImpulseTranslation.getY()));
            angularImpulseBody2.add(mImpulseRotation);
            linearImpulseBody2.add(Vector3.negate(linearImpulseLimits));
            angularImpulseBody2.add(Vector3.multiply(-impulseLimits, mR2CrossSliderAxis));
            linearImpulseBody2.add(Vector3.negate(impulseMotor));
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
        final float el1 = -mN1.dot(v1) - w1.dot(mR1PlusUCrossN1) + mN1.dot(v2) + w2.dot(mR2CrossN1);
        final float el2 = -mN2.dot(v1) - w1.dot(mR1PlusUCrossN2) + mN2.dot(v2) + w2.dot(mR2CrossN2);
        final Vector2 JvTranslation = new Vector2(el1, el2);
        final Vector2 deltaLambda = Matrix2x2.multiply(mInverseMassMatrixTranslationConstraint, Vector2.subtract(Vector2.negate(JvTranslation), mBTranslation));
        mImpulseTranslation.add(deltaLambda);
        if (mBody1.getIsMotionEnabled()) {
            final Vector3 linearImpulseBody1 = Vector3.subtract(Vector3.multiply(Vector3.negate(mN1), deltaLambda.getX()), Vector3.multiply(mN2, deltaLambda.getY()));
            final Vector3 angularImpulseBody1 = Vector3.subtract(Vector3.multiply(Vector3.negate(mR1PlusUCrossN1), deltaLambda.getX()), Vector3.multiply(mR1PlusUCrossN2, deltaLambda.getY()));
            v1.add(Vector3.multiply(inverseMassBody1, linearImpulseBody1));
            w1.add(Matrix3x3.multiply(mI1, angularImpulseBody1));
        }
        if (mBody2.getIsMotionEnabled()) {
            final Vector3 linearImpulseBody2 = Vector3.add(Vector3.multiply(mN1, deltaLambda.getX()), Vector3.multiply(mN2, deltaLambda.getY()));
            final Vector3 angularImpulseBody2 = Vector3.add(Vector3.multiply(mR2CrossN1, deltaLambda.getX()), Vector3.multiply(mR2CrossN2, deltaLambda.getY()));
            v2.add(Vector3.multiply(inverseMassBody2, linearImpulseBody2));
            w2.add(Matrix3x3.multiply(mI2, angularImpulseBody2));
        }
        final Vector3 JvRotation = Vector3.subtract(w2, w1);
        final Vector3 deltaLambda2 = Matrix3x3.multiply(mInverseMassMatrixRotationConstraint, Vector3.subtract(Vector3.negate(JvRotation), mBRotation));
        mImpulseRotation.add(deltaLambda2);
        if (mBody1.getIsMotionEnabled()) {
            final Vector3 angularImpulseBody1 = Vector3.negate(deltaLambda2);
            w1.add(Matrix3x3.multiply(mI1, angularImpulseBody1));
        }
        if (mBody2.getIsMotionEnabled()) {
            final Vector3 angularImpulseBody2 = deltaLambda2;
            w2.add(Matrix3x3.multiply(mI2, angularImpulseBody2));
        }
        if (mIsLimitEnabled) {
            if (mIsLowerLimitViolated) {
                final float JvLowerLimit = mSliderAxisWorld.dot(v2) + mR2CrossSliderAxis.dot(w2) - mSliderAxisWorld.dot(v1) - mR1PlusUCrossSliderAxis.dot(w1);
                float deltaLambdaLower = mInverseMassMatrixLimit * (-JvLowerLimit - mBLowerLimit);
                final float lambdaTemp = mImpulseLowerLimit;
                mImpulseLowerLimit = Math.max(mImpulseLowerLimit + deltaLambdaLower, 0);
                deltaLambdaLower = mImpulseLowerLimit - lambdaTemp;
                if (mBody1.getIsMotionEnabled()) {
                    final Vector3 linearImpulseBody1 = Vector3.multiply(-deltaLambdaLower, mSliderAxisWorld);
                    final Vector3 angularImpulseBody1 = Vector3.multiply(-deltaLambdaLower, mR1PlusUCrossSliderAxis);
                    v1.add(Vector3.multiply(inverseMassBody1, linearImpulseBody1));
                    w1.add(Matrix3x3.multiply(mI1, angularImpulseBody1));
                }
                if (mBody2.getIsMotionEnabled()) {
                    final Vector3 linearImpulseBody2 = Vector3.multiply(deltaLambdaLower, mSliderAxisWorld);
                    final Vector3 angularImpulseBody2 = Vector3.multiply(deltaLambdaLower, mR2CrossSliderAxis);
                    v2.add(Vector3.multiply(inverseMassBody2, linearImpulseBody2));
                    w2.add(Matrix3x3.multiply(mI2, angularImpulseBody2));
                }
            }
            if (mIsUpperLimitViolated) {
                final float JvUpperLimit = mSliderAxisWorld.dot(v1) + mR1PlusUCrossSliderAxis.dot(w1) - mSliderAxisWorld.dot(v2) - mR2CrossSliderAxis.dot(w2);
                float deltaLambdaUpper = mInverseMassMatrixLimit * (-JvUpperLimit - mBUpperLimit);
                final float lambdaTemp = mImpulseUpperLimit;
                mImpulseUpperLimit = Math.max(mImpulseUpperLimit + deltaLambdaUpper, 0);
                deltaLambdaUpper = mImpulseUpperLimit - lambdaTemp;
                if (mBody1.getIsMotionEnabled()) {
                    final Vector3 linearImpulseBody1 = Vector3.multiply(deltaLambdaUpper, mSliderAxisWorld);
                    final Vector3 angularImpulseBody1 = Vector3.multiply(deltaLambdaUpper, mR1PlusUCrossSliderAxis);
                    v1.add(Vector3.multiply(inverseMassBody1, linearImpulseBody1));
                    w1.add(Matrix3x3.multiply(mI1, angularImpulseBody1));
                }
                if (mBody2.getIsMotionEnabled()) {
                    final Vector3 linearImpulseBody2 = Vector3.multiply(-deltaLambdaUpper, mSliderAxisWorld);
                    final Vector3 angularImpulseBody2 = Vector3.multiply(-deltaLambdaUpper, mR2CrossSliderAxis);
                    v2.add(Vector3.multiply(inverseMassBody2, linearImpulseBody2));
                    w2.add(Matrix3x3.multiply(mI2, angularImpulseBody2));
                }
            }
        }
        if (mIsMotorEnabled) {
            final float JvMotor = mSliderAxisWorld.dot(v1) - mSliderAxisWorld.dot(v2);
            final float maxMotorImpulse = mMaxMotorForce * constraintSolverData.getTimeStep();
            float deltaLambdaMotor = mInverseMassMatrixMotor * (-JvMotor - mMotorSpeed);
            final float lambdaTemp = mImpulseMotor;
            mImpulseMotor = Mathematics.clamp(mImpulseMotor + deltaLambdaMotor, -maxMotorImpulse, maxMotorImpulse);
            deltaLambdaMotor = mImpulseMotor - lambdaTemp;
            if (mBody1.getIsMotionEnabled()) {
                final Vector3 linearImpulseBody1 = Vector3.multiply(deltaLambdaMotor, mSliderAxisWorld);
                v1.add(Vector3.multiply(inverseMassBody1, linearImpulseBody1));
            }
            if (mBody2.getIsMotionEnabled()) {
                final Vector3 linearImpulseBody2 = Vector3.multiply(-deltaLambdaMotor, mSliderAxisWorld);
                v2.add(Vector3.multiply(inverseMassBody2, linearImpulseBody2));
            }
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
        mR1.set(Quaternion.multiply(q1, mLocalAnchorPointBody1));
        mR2.set(Quaternion.multiply(q2, mLocalAnchorPointBody2));
        final Vector3 u = Vector3.subtract(Vector3.subtract(Vector3.add(x2, mR2), x1), mR1);
        mSliderAxisWorld.set(Quaternion.multiply(q1, mSliderAxisBody1));
        mSliderAxisWorld.normalize();
        mN1.set(mSliderAxisWorld.getOneUnitOrthogonalVector());
        mN2.set(mSliderAxisWorld.cross(mN1));
        final float uDotSliderAxis = u.dot(mSliderAxisWorld);
        final float lowerLimitError = uDotSliderAxis - mLowerLimit;
        final float upperLimitError = mUpperLimit - uDotSliderAxis;
        mIsLowerLimitViolated = lowerLimitError <= 0;
        mIsUpperLimitViolated = upperLimitError <= 0;
        mR2CrossN1.set(mR2.cross(mN1));
        mR2CrossN2.set(mR2.cross(mN2));
        mR2CrossSliderAxis.set(mR2.cross(mSliderAxisWorld));
        final Vector3 r1PlusU = Vector3.add(mR1, u);
        mR1PlusUCrossN1.set(r1PlusU.cross(mN1));
        mR1PlusUCrossN2.set(r1PlusU.cross(mN2));
        mR1PlusUCrossSliderAxis.set(r1PlusU.cross(mSliderAxisWorld));
        float sumInverseMass = 0;
        final Vector3 I1R1PlusUCrossN1 = new Vector3(0, 0, 0);
        final Vector3 I1R1PlusUCrossN2 = new Vector3(0, 0, 0);
        final Vector3 I2R2CrossN1 = new Vector3(0, 0, 0);
        final Vector3 I2R2CrossN2 = new Vector3(0, 0, 0);
        if (mBody1.getIsMotionEnabled()) {
            sumInverseMass += mBody1.getMassInverse();
            I1R1PlusUCrossN1.set(Matrix3x3.multiply(mI1, mR1PlusUCrossN1));
            I1R1PlusUCrossN2.set(Matrix3x3.multiply(mI1, mR1PlusUCrossN2));
        }
        if (mBody2.getIsMotionEnabled()) {
            sumInverseMass += mBody2.getMassInverse();
            I2R2CrossN1.set(Matrix3x3.multiply(mI2, mR2CrossN1));
            I2R2CrossN2.set(Matrix3x3.multiply(mI2, mR2CrossN2));
        }
        final float el11 = sumInverseMass + mR1PlusUCrossN1.dot(I1R1PlusUCrossN1) + mR2CrossN1.dot(I2R2CrossN1);
        final float el12 = mR1PlusUCrossN1.dot(I1R1PlusUCrossN2) + mR2CrossN1.dot(I2R2CrossN2);
        final float el21 = mR1PlusUCrossN2.dot(I1R1PlusUCrossN1) + mR2CrossN2.dot(I2R2CrossN1);
        final float el22 = sumInverseMass + mR1PlusUCrossN2.dot(I1R1PlusUCrossN2) + mR2CrossN2.dot(I2R2CrossN2);
        final Matrix2x2 matrixKTranslation = new Matrix2x2(el11, el12, el21, el22);
        mInverseMassMatrixTranslationConstraint.setToZero();
        if (mBody1.getIsMotionEnabled() || mBody2.getIsMotionEnabled()) {
            mInverseMassMatrixTranslationConstraint.set(matrixKTranslation.getInverse());
        }
        final Vector2 translationError = new Vector2(u.dot(mN1), u.dot(mN2));
        final Vector2 lambdaTranslation = Matrix2x2.multiply(mInverseMassMatrixTranslationConstraint, Vector2.negate(translationError));
        if (mBody1.getIsMotionEnabled()) {
            final Vector3 linearImpulseBody1 = Vector3.subtract(Vector3.multiply(Vector3.negate(mN1), lambdaTranslation.getX()), Vector3.multiply(mN2, lambdaTranslation.getY()));
            final Vector3 angularImpulseBody1 = Vector3.subtract(Vector3.multiply(Vector3.negate(mR1PlusUCrossN1), lambdaTranslation.getX()),
                    Vector3.multiply(mR1PlusUCrossN2, lambdaTranslation.getY()));
            final Vector3 v1 = Vector3.multiply(inverseMassBody1, linearImpulseBody1);
            final Vector3 w1 = Matrix3x3.multiply(mI1, angularImpulseBody1);
            x1.add(v1);
            q1.add(Quaternion.multiply(Quaternion.multiply(new Quaternion(0, w1), q1), 0.5f));
            q1.normalize();
        }
        if (mBody2.getIsMotionEnabled()) {
            final Vector3 linearImpulseBody2 = Vector3.add(Vector3.multiply(mN1, lambdaTranslation.getX()), Vector3.multiply(mN2, lambdaTranslation.getY()));
            final Vector3 angularImpulseBody2 = Vector3.add(Vector3.multiply(mR2CrossN1, lambdaTranslation.getX()), Vector3.multiply(mR2CrossN2, lambdaTranslation.getY()));
            final Vector3 v2 = Vector3.multiply(inverseMassBody2, linearImpulseBody2);
            final Vector3 w2 = Matrix3x3.multiply(mI2, angularImpulseBody2);
            x2.add(v2);
            q2.add(Quaternion.multiply(Quaternion.multiply(new Quaternion(0, w2), q2), 0.5f));
            q2.normalize();
        }
        mInverseMassMatrixRotationConstraint.setToZero();
        if (mBody1.getIsMotionEnabled()) {
            mInverseMassMatrixRotationConstraint.add(mI1);
        }
        if (mBody2.getIsMotionEnabled()) {
            mInverseMassMatrixRotationConstraint.add(mI2);
        }
        if (mBody1.getIsMotionEnabled() || mBody2.getIsMotionEnabled()) {
            mInverseMassMatrixRotationConstraint.set(mInverseMassMatrixRotationConstraint.getInverse());
        }
        final Quaternion currentOrientationDifference = Quaternion.multiply(q2, q1.getInverse());
        currentOrientationDifference.normalize();
        final Quaternion qError = Quaternion.multiply(currentOrientationDifference, mInitOrientationDifferenceInv);
        final Vector3 errorRotation = Vector3.multiply(2, qError.getVectorV());
        final Vector3 lambdaRotation = Matrix3x3.multiply(mInverseMassMatrixRotationConstraint, Vector3.negate(errorRotation));
        if (mBody1.getIsMotionEnabled()) {
            final Vector3 angularImpulseBody1 = Vector3.negate(lambdaRotation);
            final Vector3 w1 = Matrix3x3.multiply(mI1, angularImpulseBody1);
            q1.add(Quaternion.multiply(Quaternion.multiply(new Quaternion(0, w1), q1), 0.5f));
            q1.normalize();
        }
        if (mBody2.getIsMotionEnabled()) {
            final Vector3 angularImpulseBody2 = lambdaRotation;
            final Vector3 w2 = Matrix3x3.multiply(mI2, angularImpulseBody2);
            q2.add(Quaternion.multiply(Quaternion.multiply(new Quaternion(0, w2), q2), 0.5f));
            q2.normalize();
        }
        if (mIsLimitEnabled) {
            if (mIsLowerLimitViolated || mIsUpperLimitViolated) {
                mInverseMassMatrixLimit = 0;
                if (mBody1.getIsMotionEnabled()) {
                    mInverseMassMatrixLimit += mBody1.getMassInverse() + mR1PlusUCrossSliderAxis.dot(Matrix3x3.multiply(mI1, mR1PlusUCrossSliderAxis));
                }
                if (mBody2.getIsMotionEnabled()) {
                    mInverseMassMatrixLimit += mBody2.getMassInverse() + mR2CrossSliderAxis.dot(Matrix3x3.multiply(mI2, mR2CrossSliderAxis));
                }
                mInverseMassMatrixLimit = mInverseMassMatrixLimit > 0 ? 1 / mInverseMassMatrixLimit : 0;
            }
            if (mIsLowerLimitViolated) {
                final float lambdaLowerLimit = mInverseMassMatrixLimit * -lowerLimitError;
                if (mBody1.getIsMotionEnabled()) {
                    final Vector3 linearImpulseBody1 = Vector3.multiply(-lambdaLowerLimit, mSliderAxisWorld);
                    final Vector3 angularImpulseBody1 = Vector3.multiply(-lambdaLowerLimit, mR1PlusUCrossSliderAxis);
                    final Vector3 v1 = Vector3.multiply(inverseMassBody1, linearImpulseBody1);
                    final Vector3 w1 = Matrix3x3.multiply(mI1, angularImpulseBody1);
                    x1.add(v1);
                    q1.add(Quaternion.multiply(Quaternion.multiply(new Quaternion(0, w1), q1), 0.5f));
                    q1.normalize();
                }
                if (mBody2.getIsMotionEnabled()) {
                    final Vector3 linearImpulseBody2 = Vector3.multiply(lambdaLowerLimit, mSliderAxisWorld);
                    final Vector3 angularImpulseBody2 = Vector3.multiply(lambdaLowerLimit, mR2CrossSliderAxis);
                    final Vector3 v2 = Vector3.multiply(inverseMassBody2, linearImpulseBody2);
                    final Vector3 w2 = Matrix3x3.multiply(mI2, angularImpulseBody2);
                    x2.add(v2);
                    q2.add(Quaternion.multiply(Quaternion.multiply(new Quaternion(0, w2), q2), 0.5f));
                    q2.normalize();
                }
            }
            if (mIsUpperLimitViolated) {
                final float lambdaUpperLimit = mInverseMassMatrixLimit * (-upperLimitError);
                if (mBody1.getIsMotionEnabled()) {
                    final Vector3 linearImpulseBody1 = Vector3.multiply(lambdaUpperLimit, mSliderAxisWorld);
                    final Vector3 angularImpulseBody1 = Vector3.multiply(lambdaUpperLimit, mR1PlusUCrossSliderAxis);
                    final Vector3 v1 = Vector3.multiply(inverseMassBody1, linearImpulseBody1);
                    final Vector3 w1 = Matrix3x3.multiply(mI1, angularImpulseBody1);
                    x1.add(v1);
                    q1.add(Quaternion.multiply(Quaternion.multiply(new Quaternion(0, w1), q1), 0.5f));
                    q1.normalize();
                }
                if (mBody2.getIsMotionEnabled()) {
                    final Vector3 linearImpulseBody2 = Vector3.multiply(-lambdaUpperLimit, mSliderAxisWorld);
                    final Vector3 angularImpulseBody2 = Vector3.multiply(-lambdaUpperLimit, mR2CrossSliderAxis);
                    final Vector3 v2 = Vector3.multiply(inverseMassBody2, linearImpulseBody2);
                    final Vector3 w2 = Matrix3x3.multiply(mI2, angularImpulseBody2);
                    x2.add(v2);
                    q2.add(Quaternion.multiply(Quaternion.multiply(new Quaternion(0, w2), q2), 0.5f));
                    q2.normalize();
                }
            }
        }
    }

    /**
     * Enables or disables the joint limits.
     *
     * @param isLimitEnabled The new state of the joint limits
     */
    public void enableLimit(boolean isLimitEnabled) {
        if (isLimitEnabled != mIsLimitEnabled) {
            mIsLimitEnabled = isLimitEnabled;
            resetLimits();
        }
    }

    /**
     * Enables or disables the joint motor.
     *
     * @param isMotorEnabled The new state of the joint motor
     */
    public void enableMotor(boolean isMotorEnabled) {
        mIsMotorEnabled = isMotorEnabled;
        mImpulseMotor = 0;
        // TODO : Wake up the bodies of the joint here when sleeping is implemented
    }

    /**
     * Sets the minimum translation limit.
     *
     * @param lowerLimit The minimum limit
     */
    public void setMinTranslationLimit(float lowerLimit) {
        if (lowerLimit > mUpperLimit) {
            throw new IllegalArgumentException("Lower limit must be smaller or equal to current upper limit");
        }
        if (lowerLimit != mLowerLimit) {
            mLowerLimit = lowerLimit;
            resetLimits();
        }
    }

    /**
     * Sets the maximum translation limit.
     *
     * @param upperLimit The maximum limit
     */
    public void setMaxTranslationLimit(float upperLimit) {
        if (mLowerLimit > upperLimit) {
            throw new IllegalArgumentException("Current lower limit must be smaller or equal to upper limit");
        }
        if (upperLimit != mUpperLimit) {
            mUpperLimit = upperLimit;
            resetLimits();
        }
    }

    // Resets the limits.
    private void resetLimits() {
        mImpulseLowerLimit = 0;
        mImpulseUpperLimit = 0;
        // TODO : Wake up the bodies of the joint here when sleeping is implemented
    }

    /**
     * Sets the motor speed.
     *
     * @param motorSpeed The motor speed
     */
    public void setMotorSpeed(float motorSpeed) {
        if (motorSpeed != mMotorSpeed) {
            mMotorSpeed = motorSpeed;
            // TODO : Wake up the bodies of the joint here when sleeping is implemented
        }
    }

    /**
     * Sets the maximum motor force.
     *
     * @param maxMotorForce The maximum motor force
     */
    public void setMaxMotorForce(float maxMotorForce) {
        if (mMaxMotorForce < 0) {
            throw new IllegalArgumentException("Max motor force must be greater or equal to 0");
        }
        if (maxMotorForce != mMaxMotorForce) {
            mMaxMotorForce = maxMotorForce;
            // TODO : Wake up the bodies of the joint here when sleeping is implemented
        }
    }

    /**
     * Return true if the limits or the joint are enabled.
     *
     * @return Whether or not the limits are enabled
     */
    public boolean isLimitEnabled() {
        return mIsLimitEnabled;
    }

    /**
     * Returns true if the motor of the joint is enabled.
     *
     * @return Whether or not the motor is enabled
     */
    public boolean isMotorEnabled() {
        return mIsMotorEnabled;
    }

    /**
     * Returns the current translation value of the joint.
     *
     * @return The current translation
     */
    public float getTranslation() {
        final Vector3 x1 = mBody1.getTransform().getPosition();
        final Vector3 x2 = mBody2.getTransform().getPosition();
        final Quaternion q1 = mBody1.getTransform().getOrientation();
        final Quaternion q2 = mBody2.getTransform().getOrientation();
        final Vector3 anchorBody1 = Vector3.add(x1, Quaternion.multiply(q1, mLocalAnchorPointBody1));
        final Vector3 anchorBody2 = Vector3.add(x2, Quaternion.multiply(q2, mLocalAnchorPointBody2));
        final Vector3 u = Vector3.subtract(anchorBody2, anchorBody1);
        final Vector3 sliderAxisWorld = Quaternion.multiply(q1, mSliderAxisBody1);
        sliderAxisWorld.normalize();
        return u.dot(sliderAxisWorld);
    }

    /**
     * Returns the minimum limit.
     *
     * @return The minimum limit
     */
    public float getMinTranslationLimit() {
        return mLowerLimit;
    }

    /**
     * Returns the maximum limit.
     *
     * @return The maximum limit
     */
    public float getMaxTranslationLimit() {
        return mUpperLimit;
    }

    /**
     * Returns the motor speed.
     *
     * @return The motor speed
     */
    public float getMotorSpeed() {
        return mMotorSpeed;
    }

    /**
     * Returns the maximum motor force.
     *
     * @return The maximum motor force
     */
    public float getMaxMotorForce() {
        return mMaxMotorForce;
    }

    /**
     * Returns the intensity of the current force applied for the joint motor.
     *
     * @param timeStep The simulation time step
     * @return The motor force for the time step
     */
    public float getMotorForce(float timeStep) {
        return mImpulseMotor / timeStep;
    }

    /**
     * This structure is used to gather the information needed to create a slider joint. This structure will be used to create the actual slider joint.
     */
    public static class SliderJointInfo extends ConstraintInfo {
        private final Vector3 anchorPointWorldSpace = new Vector3();
        private final Vector3 sliderAxisWorldSpace = new Vector3();
        private final boolean isLimitEnabled;
        private final boolean isMotorEnabled;
        private final float minTranslationLimit;
        private final float maxTranslationLimit;
        private final float motorSpeed;
        private final float maxMotorForce;

        /**
         * Constructs a new unlimited and non-motored slider joint info from the both bodies, the initial anchor point in world space and the init axis, also in world space.
         *
         * @param body1 The first body
         * @param body2 The second body
         * @param initAnchorPointWorldSpace The initial anchor point in world space
         * @param initSliderAxisWorldSpace The initial axis in world space
         */
        public SliderJointInfo(RigidBody body1, RigidBody body2, Vector3 initAnchorPointWorldSpace, Vector3 initSliderAxisWorldSpace) {
            super(body1, body2, ConstraintType.SLIDERJOINT);
            anchorPointWorldSpace.set(initAnchorPointWorldSpace);
            sliderAxisWorldSpace.set(initSliderAxisWorldSpace);
            isLimitEnabled = false;
            isMotorEnabled = false;
            minTranslationLimit = -1;
            maxTranslationLimit = 1;
            motorSpeed = 0;
            maxMotorForce = 0;
        }

        /**
         * Constructs a new limited but non-motored slider joint info from the both bodies, the initial anchor point in world space, the initial axis, also in world space, and the upper and lower
         * limits of the joint.
         *
         * @param body1 The first body
         * @param body2 The second body
         * @param initAnchorPointWorldSpace The initial anchor point in world space
         * @param initSliderAxisWorldSpace The initial axis in world space
         * @param initLowerLimit The initial lower limit
         * @param initUpperLimit The initial upper limit
         */
        public SliderJointInfo(RigidBody body1, RigidBody body2, Vector3 initAnchorPointWorldSpace, Vector3 initSliderAxisWorldSpace, float initLowerLimit, float initUpperLimit) {
            super(body1, body2, ConstraintType.SLIDERJOINT);
            anchorPointWorldSpace.set(initAnchorPointWorldSpace);
            sliderAxisWorldSpace.set(initSliderAxisWorldSpace);
            isLimitEnabled = true;
            isMotorEnabled = false;
            minTranslationLimit = initLowerLimit;
            maxTranslationLimit = initUpperLimit;
            motorSpeed = 0;
            maxMotorForce = 0;
        }

        /**
         * Constructs a new limited and motored slider joint info from the both bodies, the initial anchor point in world space, the initial axis, also in world space, the upper and lower limits of
         * the joint, the motor speed and the maximum motor force.
         *
         * @param body1 The first body
         * @param body2 The second body
         * @param initAnchorPointWorldSpace The initial anchor point in world space
         * @param initSliderAxisWorldSpace The initial axis in world space
         * @param initLowerLimit The initial lower limit
         * @param initUpperLimit The initial upper limit
         * @param initMotorSpeed The initial motor speed
         * @param initMaxMotorForce The initial maximum motor force
         */
        public SliderJointInfo(RigidBody body1, RigidBody body2, Vector3 initAnchorPointWorldSpace, Vector3 initSliderAxisWorldSpace, float initLowerLimit, float initUpperLimit, float initMotorSpeed,
                               float initMaxMotorForce) {
            super(body1, body2, ConstraintType.SLIDERJOINT);
            anchorPointWorldSpace.set(initAnchorPointWorldSpace);
            sliderAxisWorldSpace.set(initSliderAxisWorldSpace);
            isLimitEnabled = true;
            isMotorEnabled = true;
            minTranslationLimit = initLowerLimit;
            maxTranslationLimit = initUpperLimit;
            motorSpeed = initMotorSpeed;
            maxMotorForce = initMaxMotorForce;
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
         * Returns the axis in world space.
         *
         * @return The axis in world space
         */
        public Vector3 getSliderAxisWorldSpace() {
            return sliderAxisWorldSpace;
        }

        /**
         * Returns true if the limits are active.
         *
         * @return Whether or not the limits are active
         */
        public boolean isLimitEnabled() {
            return isLimitEnabled;
        }

        /**
         * Returns the lower limit.
         *
         * @return The lower limit
         */
        public float getMinTranslationLimit() {
            return minTranslationLimit;
        }

        /**
         * Returns the upper limit.
         *
         * @return The upper limit
         */
        public float getMaxTranslationLimit() {
            return maxTranslationLimit;
        }

        /**
         * Returns true if the motor is enabled.
         *
         * @return Whether or not the motor is enabled
         */
        public boolean isMotorEnabled() {
            return isMotorEnabled;
        }

        /**
         * Returns the motor speed.
         *
         * @return The motor speed
         */
        public float getMotorSpeed() {
            return motorSpeed;
        }

        /**
         * Returns the max motor force.
         *
         * @return The motor force
         */
        public float getMaxMotorForce() {
            return maxMotorForce;
        }
    }
}
