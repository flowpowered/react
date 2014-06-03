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

import org.spout.physics.ReactDefaults;
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
 *
 */
public class HingeJoint extends Constraint {
    private static final float BETA = 0.2f;
    private final Vector3 mLocalAnchorPointBody1;
    private final Vector3 mLocalAnchorPointBody2;
    private final Vector3 mHingeLocalAxisBody1;
    private final Vector3 mHingeLocalAxisBody2;
    private final Matrix3x3 mI1 = new Matrix3x3();
    private final Matrix3x3 mI2 = new Matrix3x3();
    private final Vector3 mA1 = new Vector3();
    private final Vector3 mR1World = new Vector3();
    private final Vector3 mR2World = new Vector3();
    private final Vector3 mB2CrossA1 = new Vector3();
    private final Vector3 mC2CrossA1 = new Vector3();
    private final Vector3 mImpulseTranslation;
    private final Vector2 mImpulseRotation;
    private float mImpulseLowerLimit;
    private float mImpulseUpperLimit;
    private float mImpulseMotor;
    private final Matrix3x3 mInverseMassMatrixTranslation = new Matrix3x3();
    private final Matrix2x2 mInverseMassMatrixRotation = new Matrix2x2();
    private float mInverseMassMatrixLimitMotor;
    private float mInverseMassMatrixMotor;
    private final Vector3 mBTranslation = new Vector3();
    private final Vector2 mBRotation = new Vector2();
    private float mBLowerLimit;
    private float mBUpperLimit;
    private final Quaternion mInitOrientationDifferenceInv;
    private boolean mIsLimitEnabled;
    private boolean mIsMotorEnabled;
    private float mLowerLimit;
    private float mUpperLimit;
    private boolean mIsLowerLimitViolated;
    private boolean mIsUpperLimitViolated;
    private float mMotorSpeed;
    private float mMaxMotorTorque;

    public HingeJoint(HingeJointInfo jointInfo) {
        super(jointInfo);
        if (jointInfo.getMinAngleLimit() > 0 || jointInfo.getMinAngleLimit() < -2 * Math.PI) {
            throw new IllegalArgumentException("Lower limit must be smaller or equal to 0 and greater or equal to -2 * PI");
        }
        if (jointInfo.getMaxAngleLimit() < 0 || jointInfo.getMaxAngleLimit() > 2 * Math.PI) {
            throw new IllegalArgumentException("Upper limit must be greater or equal to 0 and smaller or equal to 2 * PI");
        }
        mImpulseTranslation = new Vector3(0, 0, 0);
        mImpulseRotation = new Vector2(0, 0);
        mImpulseLowerLimit = 0;
        mImpulseUpperLimit = 0;
        mImpulseMotor = 0;
        mIsLimitEnabled = jointInfo.isLimitEnabled();
        mIsMotorEnabled = jointInfo.isMotorEnabled();
        mLowerLimit = jointInfo.getMinAngleLimit();
        mUpperLimit = jointInfo.getMaxAngleLimit();
        mIsLowerLimitViolated = false;
        mIsUpperLimitViolated = false;
        mMotorSpeed = jointInfo.getMotorSpeed();
        mMaxMotorTorque = jointInfo.getMaxMotorTorque();
        final Transform transform1 = mBody1.getTransform();
        final Transform transform2 = mBody2.getTransform();
        mLocalAnchorPointBody1 = Transform.multiply(transform1.getInverse(), jointInfo.getAnchorPointWorldSpace());
        mLocalAnchorPointBody2 = Transform.multiply(transform2.getInverse(), jointInfo.getAnchorPointWorldSpace());
        mHingeLocalAxisBody1 = Quaternion.multiply(transform1.getOrientation().getInverse(), jointInfo.getRotationAxisWorld());
        mHingeLocalAxisBody2 = Quaternion.multiply(transform2.getOrientation().getInverse(), jointInfo.getRotationAxisWorld());
        mHingeLocalAxisBody1.normalize();
        mHingeLocalAxisBody2.normalize();
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
        final float hingeAngle = computeCurrentHingeAngle(orientationBody1, orientationBody2);
        final float lowerLimitError = hingeAngle - mLowerLimit;
        final float upperLimitError = mUpperLimit - hingeAngle;
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
        mA1.set(Quaternion.multiply(orientationBody1, mHingeLocalAxisBody1));
        final Vector3 a2 = Quaternion.multiply(orientationBody2, mHingeLocalAxisBody2);
        mA1.normalize();
        a2.normalize();
        final Vector3 b2 = a2.getOneUnitOrthogonalVector();
        final Vector3 c2 = a2.cross(b2);
        mB2CrossA1.set(b2.cross(mA1));
        mC2CrossA1.set(c2.cross(mA1));
        final Matrix3x3 skewSymmetricMatrixU1 = Matrix3x3.computeSkewSymmetricMatrixForCrossProduct(mR1World);
        final Matrix3x3 skewSymmetricMatrixU2 = Matrix3x3.computeSkewSymmetricMatrixForCrossProduct(mR2World);
        float inverseMassBodies = 0;
        if (mBody1.getIsMotionEnabled()) {
            inverseMassBodies += mBody1.getMassInverse();
        }
        if (mBody2.getIsMotionEnabled()) {
            inverseMassBodies += mBody2.getMassInverse();
        }
        final Matrix3x3 massMatrix = new Matrix3x3(
                inverseMassBodies, 0, 0,
                0, inverseMassBodies, 0,
                0, 0, inverseMassBodies);
        if (mBody1.getIsMotionEnabled()) {
            massMatrix.add(Matrix3x3.multiply(skewSymmetricMatrixU1, Matrix3x3.multiply(mI1, skewSymmetricMatrixU1.getTranspose())));
        }
        if (mBody2.getIsMotionEnabled()) {
            massMatrix.add(Matrix3x3.multiply(skewSymmetricMatrixU2, Matrix3x3.multiply(mI2, skewSymmetricMatrixU2.getTranspose())));
        }
        mInverseMassMatrixTranslation.setToZero();
        if (mBody1.getIsMotionEnabled() || mBody2.getIsMotionEnabled()) {
            mInverseMassMatrixTranslation.set(massMatrix.getInverse());
        }
        mBTranslation.setToZero();
        final float biasFactor = BETA / constraintSolverData.getTimeStep();
        if (mPositionCorrectionTechnique == JointsPositionCorrectionTechnique.BAUMGARTE_JOINTS) {
            mBTranslation.set(Vector3.multiply(biasFactor, Vector3.subtract(Vector3.subtract(Vector3.add(x2, mR2World), x1), mR1World)));
        }
        final Vector3 I1B2CrossA1 = new Vector3(0, 0, 0);
        final Vector3 I1C2CrossA1 = new Vector3(0, 0, 0);
        final Vector3 I2B2CrossA1 = new Vector3(0, 0, 0);
        final Vector3 I2C2CrossA1 = new Vector3(0, 0, 0);
        if (mBody1.getIsMotionEnabled()) {
            I1B2CrossA1.set(Matrix3x3.multiply(mI1, mB2CrossA1));
            I1C2CrossA1.set(Matrix3x3.multiply(mI1, mC2CrossA1));
        }
        if (mBody2.getIsMotionEnabled()) {
            I2B2CrossA1.set(Matrix3x3.multiply(mI2, mB2CrossA1));
            I2C2CrossA1.set(Matrix3x3.multiply(mI2, mC2CrossA1));
        }
        final float el11 = mB2CrossA1.dot(I1B2CrossA1) + mB2CrossA1.dot(I2B2CrossA1);
        final float el12 = mB2CrossA1.dot(I1C2CrossA1) + mB2CrossA1.dot(I2C2CrossA1);
        final float el21 = mC2CrossA1.dot(I1B2CrossA1) + mC2CrossA1.dot(I2B2CrossA1);
        final float el22 = mC2CrossA1.dot(I1C2CrossA1) + mC2CrossA1.dot(I2C2CrossA1);
        final Matrix2x2 matrixKRotation = new Matrix2x2(el11, el12, el21, el22);
        mInverseMassMatrixRotation.setToZero();
        if (mBody1.getIsMotionEnabled() || mBody2.getIsMotionEnabled()) {
            mInverseMassMatrixRotation.set(matrixKRotation.getInverse());
        }
        mBRotation.setToZero();
        if (mPositionCorrectionTechnique == JointsPositionCorrectionTechnique.BAUMGARTE_JOINTS) {
            mBRotation.set(Vector2.multiply(biasFactor, new Vector2(mA1.dot(b2), mA1.dot(c2))));
        }
        if (!constraintSolverData.isWarmStartingActive()) {
            mImpulseTranslation.setToZero();
            mImpulseRotation.setToZero();
            mImpulseLowerLimit = 0;
            mImpulseUpperLimit = 0;
            mImpulseMotor = 0;
        }
        if (mIsLimitEnabled && (mIsLowerLimitViolated || mIsUpperLimitViolated)) {
            mInverseMassMatrixLimitMotor = 0;
            if (mBody1.getIsMotionEnabled()) {
                mInverseMassMatrixLimitMotor += mA1.dot(Matrix3x3.multiply(mI1, mA1));
            }
            if (mBody2.getIsMotionEnabled()) {
                mInverseMassMatrixLimitMotor += mA1.dot(Matrix3x3.multiply(mI2, mA1));
            }
            mInverseMassMatrixLimitMotor = mInverseMassMatrixLimitMotor > 0 ? 1 / mInverseMassMatrixLimitMotor : 0;
            mBLowerLimit = 0;
            if (mPositionCorrectionTechnique == JointsPositionCorrectionTechnique.BAUMGARTE_JOINTS) {
                mBLowerLimit = biasFactor * lowerLimitError;
            }
            mBUpperLimit = 0;
            if (mPositionCorrectionTechnique == JointsPositionCorrectionTechnique.BAUMGARTE_JOINTS) {
                mBUpperLimit = biasFactor * upperLimitError;
            }
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
        final Vector3 rotationImpulse = Vector3.subtract(Vector3.multiply(Vector3.negate(mB2CrossA1), mImpulseRotation.getX()), Vector3.multiply(mC2CrossA1, mImpulseRotation.getY()));
        final Vector3 limitsImpulse = Vector3.multiply(mImpulseUpperLimit - mImpulseLowerLimit, mA1);
        final Vector3 motorImpulse = Vector3.multiply(-mImpulseMotor, mA1);
        if (mBody1.getIsMotionEnabled()) {
            final Vector3 linearImpulseBody1 = Vector3.negate(mImpulseTranslation);
            final Vector3 angularImpulseBody1 = mImpulseTranslation.cross(mR1World);
            angularImpulseBody1.add(rotationImpulse);
            angularImpulseBody1.add(limitsImpulse);
            angularImpulseBody1.add(motorImpulse);
            v1.add(Vector3.multiply(inverseMassBody1, linearImpulseBody1));
            w1.add(Matrix3x3.multiply(mI1, angularImpulseBody1));
        }
        if (mBody2.getIsMotionEnabled()) {
            final Vector3 linearImpulseBody2 = mImpulseTranslation;
            final Vector3 angularImpulseBody2 = Vector3.negate(mImpulseTranslation.cross(mR2World));
            angularImpulseBody2.add(Vector3.negate(rotationImpulse));
            angularImpulseBody2.add(Vector3.negate(limitsImpulse));
            angularImpulseBody2.add(Vector3.negate(motorImpulse));
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
        final Vector3 deltaLambdaTranslation = Matrix3x3.multiply(mInverseMassMatrixTranslation, Vector3.subtract(Vector3.negate(JvTranslation), mBTranslation));
        mImpulseTranslation.add(deltaLambdaTranslation);
        if (mBody1.getIsMotionEnabled()) {
            final Vector3 linearImpulseBody1 = Vector3.negate(deltaLambdaTranslation);
            final Vector3 angularImpulseBody1 = deltaLambdaTranslation.cross(mR1World);
            v1.add(Vector3.multiply(inverseMassBody1, linearImpulseBody1));
            w1.add(Matrix3x3.multiply(mI1, angularImpulseBody1));
        }
        if (mBody2.getIsMotionEnabled()) {
            final Vector3 linearImpulseBody2 = deltaLambdaTranslation;
            final Vector3 angularImpulseBody2 = Vector3.negate(deltaLambdaTranslation.cross(mR2World));
            v2.add(Vector3.multiply(inverseMassBody2, linearImpulseBody2));
            w2.add(Matrix3x3.multiply(mI2, angularImpulseBody2));
        }
        final Vector2 JvRotation = new Vector2(-mB2CrossA1.dot(w1) + mB2CrossA1.dot(w2), -mC2CrossA1.dot(w1) + mC2CrossA1.dot(w2));
        final Vector2 deltaLambdaRotation = Matrix2x2.multiply(mInverseMassMatrixRotation, Vector2.subtract(Vector2.negate(JvRotation), mBRotation));
        mImpulseRotation.add(deltaLambdaRotation);
        if (mBody1.getIsMotionEnabled()) {
            final Vector3 angularImpulseBody1 = Vector3.subtract(Vector3.multiply(Vector3.negate(mB2CrossA1), deltaLambdaRotation.getX()), Vector3.multiply(mC2CrossA1, deltaLambdaRotation.getY()));
            w1.add(Matrix3x3.multiply(mI1, angularImpulseBody1));
        }
        if (mBody2.getIsMotionEnabled()) {
            final Vector3 angularImpulseBody2 = Vector3.add(Vector3.multiply(mB2CrossA1, deltaLambdaRotation.getX()), Vector3.multiply(mC2CrossA1, deltaLambdaRotation.getY()));
            w2.add(Matrix3x3.multiply(mI2, angularImpulseBody2));
        }
        if (mIsLimitEnabled) {
            if (mIsLowerLimitViolated) {
                final float JvLowerLimit = Vector3.subtract(w2, w1).dot(mA1);
                float deltaLambdaLower = mInverseMassMatrixLimitMotor * (-JvLowerLimit - mBLowerLimit);
                final float lambdaTemp = mImpulseLowerLimit;
                mImpulseLowerLimit = Math.max(mImpulseLowerLimit + deltaLambdaLower, 0);
                deltaLambdaLower = mImpulseLowerLimit - lambdaTemp;
                if (mBody1.getIsMotionEnabled()) {
                    final Vector3 angularImpulseBody1 = Vector3.multiply(-deltaLambdaLower, mA1);
                    w1.add(Matrix3x3.multiply(mI1, angularImpulseBody1));
                }
                if (mBody2.getIsMotionEnabled()) {
                    final Vector3 angularImpulseBody2 = Vector3.multiply(deltaLambdaLower, mA1);
                    w2.add(Matrix3x3.multiply(mI2, angularImpulseBody2));
                }
            }
            if (mIsUpperLimitViolated) {
                final float JvUpperLimit = -Vector3.subtract(w2, w1).dot(mA1);
                float deltaLambdaUpper = mInverseMassMatrixLimitMotor * (-JvUpperLimit - mBUpperLimit);
                final float lambdaTemp = mImpulseUpperLimit;
                mImpulseUpperLimit = Math.max(mImpulseUpperLimit + deltaLambdaUpper, 0);
                deltaLambdaUpper = mImpulseUpperLimit - lambdaTemp;
                if (mBody1.getIsMotionEnabled()) {
                    final Vector3 angularImpulseBody1 = Vector3.multiply(deltaLambdaUpper, mA1);
                    w1.add(Matrix3x3.multiply(mI1, angularImpulseBody1));
                }
                if (mBody2.getIsMotionEnabled()) {
                    final Vector3 angularImpulseBody2 = Vector3.multiply(-deltaLambdaUpper, mA1);
                    w2.add(Matrix3x3.multiply(mI2, angularImpulseBody2));
                }
            }
        }
        if (mIsMotorEnabled) {
            final float JvMotor = mA1.dot(Vector3.subtract(w1, w2));
            final float maxMotorImpulse = mMaxMotorTorque * constraintSolverData.getTimeStep();
            float deltaLambdaMotor = mInverseMassMatrixLimitMotor * (-JvMotor - mMotorSpeed);
            final float lambdaTemp = mImpulseMotor;
            mImpulseMotor = Mathematics.clamp(mImpulseMotor + deltaLambdaMotor, -maxMotorImpulse, maxMotorImpulse);
            deltaLambdaMotor = mImpulseMotor - lambdaTemp;
            if (mBody1.getIsMotionEnabled()) {
                final Vector3 angularImpulseBody1 = Vector3.multiply(-deltaLambdaMotor, mA1);
                w1.add(Matrix3x3.multiply(mI1, angularImpulseBody1));
            }
            if (mBody2.getIsMotionEnabled()) {
                final Vector3 angularImpulseBody2 = Vector3.multiply(deltaLambdaMotor, mA1);
                w2.add(Matrix3x3.multiply(mI2, angularImpulseBody2));
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
        mR1World.set(Quaternion.multiply(q1, mLocalAnchorPointBody1));
        mR2World.set(Quaternion.multiply(q2, mLocalAnchorPointBody2));
        final float hingeAngle = computeCurrentHingeAngle(q1, q2);
        final float lowerLimitError = hingeAngle - mLowerLimit;
        final float upperLimitError = mUpperLimit - hingeAngle;
        mIsLowerLimitViolated = lowerLimitError <= 0;
        mIsUpperLimitViolated = upperLimitError <= 0;
        mA1.set(Quaternion.multiply(q1, mHingeLocalAxisBody1));
        final Vector3 a2 = Quaternion.multiply(q2, mHingeLocalAxisBody2);
        mA1.normalize();
        a2.normalize();
        final Vector3 b2 = a2.getOneUnitOrthogonalVector();
        final Vector3 c2 = a2.cross(b2);
        mB2CrossA1.set(b2.cross(mA1));
        mC2CrossA1.set(c2.cross(mA1));
        final Matrix3x3 skewSymmetricMatrixU1 = Matrix3x3.computeSkewSymmetricMatrixForCrossProduct(mR1World);
        final Matrix3x3 skewSymmetricMatrixU2 = Matrix3x3.computeSkewSymmetricMatrixForCrossProduct(mR2World);
        float inverseMassBodies = 0;
        if (mBody1.getIsMotionEnabled()) {
            inverseMassBodies += mBody1.getMassInverse();
        }
        if (mBody2.getIsMotionEnabled()) {
            inverseMassBodies += mBody2.getMassInverse();
        }
        final Matrix3x3 massMatrix = new Matrix3x3(
                inverseMassBodies, 0, 0,
                0, inverseMassBodies, 0,
                0, 0, inverseMassBodies);
        if (mBody1.getIsMotionEnabled()) {
            massMatrix.add(Matrix3x3.multiply(skewSymmetricMatrixU1, Matrix3x3.multiply(mI1, skewSymmetricMatrixU1.getTranspose())));
        }
        if (mBody2.getIsMotionEnabled()) {
            massMatrix.add(Matrix3x3.multiply(skewSymmetricMatrixU2, Matrix3x3.multiply(mI2, skewSymmetricMatrixU2.getTranspose())));
        }
        mInverseMassMatrixTranslation.setToZero();
        if (mBody1.getIsMotionEnabled() || mBody2.getIsMotionEnabled()) {
            mInverseMassMatrixTranslation.set(massMatrix.getInverse());
        }
        final Vector3 errorTranslation = Vector3.subtract(Vector3.subtract(Vector3.add(x2, mR2World), x1), mR1World);
        final Vector3 lambdaTranslation = Matrix3x3.multiply(mInverseMassMatrixTranslation, Vector3.negate(errorTranslation));
        if (mBody1.getIsMotionEnabled()) {
            final Vector3 linearImpulseBody1 = Vector3.negate(lambdaTranslation);
            final Vector3 angularImpulseBody1 = lambdaTranslation.cross(mR1World);
            final Vector3 v1 = Vector3.multiply(inverseMassBody1, linearImpulseBody1);
            final Vector3 w1 = Matrix3x3.multiply(mI1, angularImpulseBody1);
            x1.add(v1);
            q1.add(Quaternion.multiply(Quaternion.multiply(new Quaternion(0, w1), q1), 0.5f));
            q1.normalize();
        }
        if (mBody2.getIsMotionEnabled()) {
            final Vector3 linearImpulseBody2 = lambdaTranslation;
            final Vector3 angularImpulseBody2 = Vector3.negate(lambdaTranslation.cross(mR2World));
            final Vector3 v2 = Vector3.multiply(inverseMassBody2, linearImpulseBody2);
            final Vector3 w2 = Matrix3x3.multiply(mI2, angularImpulseBody2);
            x2.add(v2);
            q2.add(Quaternion.multiply(Quaternion.multiply(new Quaternion(0, w2), q2), 0.5f));
            q2.normalize();
        }
        final Vector3 I1B2CrossA1 = new Vector3(0, 0, 0);
        final Vector3 I1C2CrossA1 = new Vector3(0, 0, 0);
        final Vector3 I2B2CrossA1 = new Vector3(0, 0, 0);
        final Vector3 I2C2CrossA1 = new Vector3(0, 0, 0);
        if (mBody1.getIsMotionEnabled()) {
            I1B2CrossA1.set(Matrix3x3.multiply(mI1, mB2CrossA1));
            I1C2CrossA1.set(Matrix3x3.multiply(mI1, mC2CrossA1));
        }
        if (mBody2.getIsMotionEnabled()) {
            I2B2CrossA1.set(Matrix3x3.multiply(mI2, mB2CrossA1));
            I2C2CrossA1.set(Matrix3x3.multiply(mI2, mC2CrossA1));
        }
        final float el11 = mB2CrossA1.dot(I1B2CrossA1) + mB2CrossA1.dot(I2B2CrossA1);
        final float el12 = mB2CrossA1.dot(I1C2CrossA1) + mB2CrossA1.dot(I2C2CrossA1);
        final float el21 = mC2CrossA1.dot(I1B2CrossA1) + mC2CrossA1.dot(I2B2CrossA1);
        final float el22 = mC2CrossA1.dot(I1C2CrossA1) + mC2CrossA1.dot(I2C2CrossA1);
        final Matrix2x2 matrixKRotation = new Matrix2x2(el11, el12, el21, el22);
        mInverseMassMatrixRotation.setToZero();
        if (mBody1.getIsMotionEnabled() || mBody2.getIsMotionEnabled()) {
            mInverseMassMatrixRotation.set(matrixKRotation.getInverse());
        }
        final Vector2 errorRotation = new Vector2(mA1.dot(b2), mA1.dot(c2));
        final Vector2 lambdaRotation = Matrix2x2.multiply(mInverseMassMatrixRotation, Vector2.negate(errorRotation));
        if (mBody1.getIsMotionEnabled()) {
            final Vector3 angularImpulseBody1 = Vector3.subtract(Vector3.multiply(Vector3.negate(mB2CrossA1), lambdaRotation.getX()), Vector3.multiply(mC2CrossA1, lambdaRotation.getY()));
            final Vector3 w1 = Matrix3x3.multiply(mI1, angularImpulseBody1);
            q1.add(Quaternion.multiply(Quaternion.multiply(new Quaternion(0, w1), q1), 0.5f));
            q1.normalize();
        }
        if (mBody2.getIsMotionEnabled()) {
            final Vector3 angularImpulseBody2 = Vector3.add(Vector3.multiply(mB2CrossA1, lambdaRotation.getX()), Vector3.multiply(mC2CrossA1, lambdaRotation.getY()));
            final Vector3 w2 = Matrix3x3.multiply(mI2, angularImpulseBody2);
            q2.add(Quaternion.multiply(Quaternion.multiply(new Quaternion(0, w2), q2), 0.5f));
            q2.normalize();
        }
        if (mIsLimitEnabled) {
            if (mIsLowerLimitViolated || mIsUpperLimitViolated) {
                mInverseMassMatrixLimitMotor = 0;
                if (mBody1.getIsMotionEnabled()) {
                    mInverseMassMatrixLimitMotor += mA1.dot(Matrix3x3.multiply(mI1, mA1));
                }
                if (mBody2.getIsMotionEnabled()) {
                    mInverseMassMatrixLimitMotor += mA1.dot(Matrix3x3.multiply(mI2, mA1));
                }
                mInverseMassMatrixLimitMotor = mInverseMassMatrixLimitMotor > 0 ? 1 / mInverseMassMatrixLimitMotor : 0;
            }
            if (mIsLowerLimitViolated) {
                final float lambdaLowerLimit = mInverseMassMatrixLimitMotor * -lowerLimitError;
                if (mBody1.getIsMotionEnabled()) {
                    final Vector3 angularImpulseBody1 = Vector3.multiply(-lambdaLowerLimit, mA1);
                    final Vector3 w1 = Matrix3x3.multiply(mI1, angularImpulseBody1);
                    q1.add(Quaternion.multiply(Quaternion.multiply(new Quaternion(0, w1), q1), 0.5f));
                    q1.normalize();
                }
                if (mBody2.getIsMotionEnabled()) {
                    final Vector3 angularImpulseBody2 = Vector3.multiply(lambdaLowerLimit, mA1);
                    final Vector3 w2 = Matrix3x3.multiply(mI2, angularImpulseBody2);
                    q2.add(Quaternion.multiply(Quaternion.multiply(new Quaternion(0, w2), q2), 0.5f));
                    q2.normalize();
                }
            }
            if (mIsUpperLimitViolated) {
                final float lambdaUpperLimit = mInverseMassMatrixLimitMotor * -upperLimitError;
                if (mBody1.getIsMotionEnabled()) {
                    final Vector3 angularImpulseBody1 = Vector3.multiply(lambdaUpperLimit, mA1);
                    final Vector3 w1 = Matrix3x3.multiply(mI1, angularImpulseBody1);
                    q1.add(Quaternion.multiply(Quaternion.multiply(new Quaternion(0, w1), q1), 0.5f));
                    q1.normalize();
                }
                if (mBody2.getIsMotionEnabled()) {
                    final Vector3 angularImpulseBody2 = Vector3.multiply(-lambdaUpperLimit, mA1);
                    final Vector3 w2 = Matrix3x3.multiply(mI2, angularImpulseBody2);
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
     * Sets the minimum angle limit.
     *
     * @param lowerLimit The minimum limit
     */
    public void setMinAngleLimit(float lowerLimit) {
        if (lowerLimit > 0 || lowerLimit < -2 * Math.PI) {
            throw new IllegalArgumentException("Lower limit must be smaller or equal to 0 and greater or equal to -2 * PI");
        }
        if (lowerLimit != mLowerLimit) {
            mLowerLimit = lowerLimit;
            resetLimits();
        }
    }

    /**
     * Sets the maximum angle limit.
     *
     * @param upperLimit The maximum limit
     */
    public void setMaxAngleLimit(float upperLimit) {
        if (upperLimit < 0 || upperLimit > 2 * Math.PI) {
            throw new IllegalArgumentException("Upper limit must be greater or equal to 0 and smaller or equal to 2 * PI");
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
     * Sets the maximum motor torque.
     *
     * @param maxMotorForce The maximum motor torque
     */
    public void setMaxMotorForce(float maxMotorForce) {
        if (mMaxMotorTorque < 0) {
            throw new IllegalArgumentException("Max motor torque must be greater or equal to 0");
        }
        if (maxMotorForce != mMaxMotorTorque) {
            mMaxMotorTorque = maxMotorForce;
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
     * Returns the minimum limit.
     *
     * @return The minimum limit
     */
    public float getMinAngleLimit() {
        return mLowerLimit;
    }

    /**
     * Returns the maximum limit.
     *
     * @return The maximum limit
     */
    public float getMaxAngleLimit() {
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
     * Returns the maximum motor torque.
     *
     * @return The maximum motor torque
     */
    public float getMaxMotorTorque() {
        return mMaxMotorTorque;
    }

    /**
     * Returns the intensity of the current torque applied for the joint motor.
     *
     * @param timeStep The simulation time step
     * @return The motor torque for the time step
     */
    public float getMotorTorque(float timeStep) {
        return mImpulseMotor / timeStep;
    }

    // Given an angle in radian, this method returns the corresponding angle in the range [-pi; pi].
    private float computeNormalizedAngle(float angle) {
        angle = angle % ReactDefaults.PI_TIMES_2;
        if (angle < -Math.PI) {
            return angle + ReactDefaults.PI_TIMES_2;
        } else if (angle > Math.PI) {
            return angle - ReactDefaults.PI_TIMES_2;
        } else {
            return angle;
        }
    }

    // Given an "inputAngle" in the range [-pi, pi], this method returns an
    // angle (modulo 2 * pi) in the range [-2 * pi; 2 * pi] that is closest to one of the two angle limits in arguments.
    private float computeCorrespondingAngleNearLimits(float inputAngle, float lowerLimitAngle, float upperLimitAngle) {
        if (upperLimitAngle <= lowerLimitAngle) {
            return inputAngle;
        } else if (inputAngle > upperLimitAngle) {
            final float diffToUpperLimit = Math.abs(computeNormalizedAngle(inputAngle - upperLimitAngle));
            final float diffToLowerLimit = Math.abs(computeNormalizedAngle(inputAngle - lowerLimitAngle));
            return diffToUpperLimit > diffToLowerLimit ? inputAngle - ReactDefaults.PI_TIMES_2 : inputAngle;
        } else if (inputAngle < lowerLimitAngle) {
            final float diffToUpperLimit = Math.abs(computeNormalizedAngle(upperLimitAngle - inputAngle));
            final float diffToLowerLimit = Math.abs(computeNormalizedAngle(lowerLimitAngle - inputAngle));
            return diffToUpperLimit > diffToLowerLimit ? inputAngle : inputAngle + ReactDefaults.PI_TIMES_2;
        } else {
            return inputAngle;
        }
    }

    // Computes the current angle around the hinge axis.
    private float computeCurrentHingeAngle(Quaternion orientationBody1, Quaternion orientationBody2) {
        float hingeAngle;
        final Quaternion currentOrientationDiff = Quaternion.multiply(orientationBody2, orientationBody1.getInverse());
        currentOrientationDiff.normalize();
        final Quaternion relativeRotation = Quaternion.multiply(currentOrientationDiff, mInitOrientationDifferenceInv);
        relativeRotation.normalize();
        final float cosHalfAngle = relativeRotation.getW();
        final float sinHalfAngleAbs = relativeRotation.getVectorV().length();
        final float dotProduct = relativeRotation.getVectorV().dot(mA1);
        if (dotProduct >= 0) {
            hingeAngle = 2 * (float) Math.atan2(sinHalfAngleAbs, cosHalfAngle);
        } else {
            hingeAngle = 2 * (float) Math.atan2(sinHalfAngleAbs, -cosHalfAngle);
        }
        hingeAngle = computeNormalizedAngle(hingeAngle);
        return computeCorrespondingAngleNearLimits(hingeAngle, mLowerLimit, mUpperLimit);
    }

    /**
     * This structure is used to gather the information needed to create a hinge joint. This structure will be used to create the actual hinge joint.
     */
    public static class HingeJointInfo extends ConstraintInfo {
        private final Vector3 anchorPointWorldSpace;
        private final Vector3 rotationAxisWorld;
        private final boolean isLimitEnabled;
        private final boolean isMotorEnabled;
        private final float minAngleLimit;
        private final float maxAngleLimit;
        private final float motorSpeed;
        private final float maxMotorTorque;

        /**
         * Constructs a new unlimited and non-motored hinge joint info from the both bodies, the initial anchor point in world space and the init axis, also in world space.
         *
         * @param body1 The first body
         * @param body2 The second body
         * @param initAnchorPointWorldSpace The initial anchor point in world space
         * @param initRotationAxisWorld The initial axis in world space
         */
        public HingeJointInfo(RigidBody body1, RigidBody body2, Vector3 initAnchorPointWorldSpace, Vector3 initRotationAxisWorld) {
            super(body1, body2, ConstraintType.HINGEJOINT);
            anchorPointWorldSpace = initAnchorPointWorldSpace;
            rotationAxisWorld = initRotationAxisWorld;
            isLimitEnabled = false;
            isMotorEnabled = false;
            minAngleLimit = -1;
            maxAngleLimit = 1;
            motorSpeed = 0;
            maxMotorTorque = 0;
        }

        /**
         * Constructs a new limited but non-motored hinge joint info from the both bodies, the initial anchor point in world space, the initial axis, also in world space, and the upper and lower
         * limits of the joint.
         *
         * @param body1 The first body
         * @param body2 The second body
         * @param initAnchorPointWorldSpace The initial anchor point in world space
         * @param initRotationAxisWorld The initial axis in world space
         * @param initMinAngleLimit The initial lower limit
         * @param initMaxAngleLimit The initial upper limit
         */
        public HingeJointInfo(RigidBody body1, RigidBody body2, Vector3 initAnchorPointWorldSpace, Vector3 initRotationAxisWorld, float initMinAngleLimit, float initMaxAngleLimit) {
            super(body1, body2, ConstraintType.HINGEJOINT);
            anchorPointWorldSpace = initAnchorPointWorldSpace;
            rotationAxisWorld = initRotationAxisWorld;
            isLimitEnabled = true;
            isMotorEnabled = false;
            minAngleLimit = initMinAngleLimit;
            maxAngleLimit = initMaxAngleLimit;
            motorSpeed = 0;
            maxMotorTorque = 0;
        }

        /**
         * Constructs a new limited and motored hinge joint info from the both bodies, the initial anchor point in world space, the initial axis, also in world space, the upper and lower limits of the
         * joint, the motor speed and the maximum motor torque.
         *
         * @param body1 The first body
         * @param body2 The second body
         * @param initAnchorPointWorldSpace The initial anchor point in world space
         * @param initRotationAxisWorld The initial axis in world space
         * @param initMinAngleLimit The initial lower limit
         * @param initMaxAngleLimit The initial upper limit
         * @param initMotorSpeed The initial motor speed
         * @param initMaxMotorTorque The initial maximum motor torque
         */
        public HingeJointInfo(RigidBody body1, RigidBody body2, Vector3 initAnchorPointWorldSpace, Vector3 initRotationAxisWorld, float initMinAngleLimit, float initMaxAngleLimit,
                              float initMotorSpeed, float initMaxMotorTorque) {
            super(body1, body2, ConstraintType.HINGEJOINT);
            anchorPointWorldSpace = initAnchorPointWorldSpace;
            rotationAxisWorld = initRotationAxisWorld;
            isLimitEnabled = true;
            isMotorEnabled = true;
            minAngleLimit = initMinAngleLimit;
            maxAngleLimit = initMaxAngleLimit;
            motorSpeed = initMotorSpeed;
            maxMotorTorque = initMaxMotorTorque;
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
         * Returns the rotation axis in world space.
         *
         * @return The axis in world space
         */
        public Vector3 getRotationAxisWorld() {
            return rotationAxisWorld;
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
         * Returns the lower angle limit.
         *
         * @return The lower limit
         */
        public float getMinAngleLimit() {
            return minAngleLimit;
        }

        /**
         * Returns the upper angle limit.
         *
         * @return The upper limit
         */
        public float getMaxAngleLimit() {
            return maxAngleLimit;
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
         * Returns the max motor torque.
         *
         * @return The motor torque
         */
        public float getMaxMotorTorque() {
            return maxMotorTorque;
        }
    }
}
