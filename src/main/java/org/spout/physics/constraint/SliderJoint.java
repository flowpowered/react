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
    private final Vector3 mLocalAnchorPointBody1 = new Vector3();
    private final Vector3 mLocalAnchorPointBody2 = new Vector3();
    private final Vector3 mSliderAxisBody1;
    private final Quaternion mInitOrientationDifference;
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
    private final Vector2 mImpulseTranslation;
    private final Vector3 mImpulseRotation;
    private float mImpulseLowerLimit;
    private float mImpulseUpperLimit;
    private boolean mIsLimitsActive;
    private final Vector3 mSliderAxisWorld = new Vector3();
    private float mLowerLimit;
    private float mUpperLimit;
    private boolean mIsLowerLimitViolated;
    private boolean mIsUpperLimitViolated;

    /**
     * Constructs a slider joint from provided slider joint info.
     *
     * @param jointInfo The joint info
     */
    public SliderJoint(SliderJointInfo jointInfo) {
        super(jointInfo);
        if (jointInfo.getUpperLimit() < 0) {
            throw new IllegalArgumentException("Upper limit must be greater or equal to 0");
        }
        if (jointInfo.getLowerLimit() > 0) {
            throw new IllegalArgumentException("Lower limit must be smaller or equal to 0");
        }
        mImpulseLowerLimit = 0;
        mImpulseUpperLimit = 0;
        mIsLimitsActive = jointInfo.isLimitsActive();
        mLowerLimit = jointInfo.getLowerLimit();
        mUpperLimit = jointInfo.getUpperLimit();
        mImpulseTranslation = new Vector2(0, 0);
        mImpulseRotation = new Vector3(0, 0, 0);
        final Transform transform1 = mBody1.getTransform();
        final Transform transform2 = mBody2.getTransform();
        mLocalAnchorPointBody1.set(Transform.multiply(transform1.getInverse(), jointInfo.getAnchorPointWorldSpace()));
        mLocalAnchorPointBody2.set(Transform.multiply(transform2.getInverse(), jointInfo.getAnchorPointWorldSpace()));
        mInitOrientationDifference = Quaternion.multiply(transform2.getOrientation(), transform1.getOrientation().getInverse());
        mInitOrientationDifference.normalize();
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
        final Matrix3x3 I1 = mBody1.getInertiaTensorInverseWorld();
        final Matrix3x3 I2 = mBody2.getInertiaTensorInverseWorld();
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
        mIsLowerLimitViolated = lowerLimitError <= 0;
        mIsUpperLimitViolated = upperLimitError <= 0;
        mR2CrossN1.set(mR2.cross(mN1));
        mR2CrossN2.set(mR2.cross(mN2));
        mR2CrossSliderAxis.set(mR2.cross(mSliderAxisWorld));
        final Vector3 r1PlusU = Vector3.add(mR1, u);
        mR1PlusUCrossN1.set((r1PlusU).cross(mN1));
        mR1PlusUCrossN2.set((r1PlusU).cross(mN2));
        mR1PlusUCrossSliderAxis.set((r1PlusU).cross(mSliderAxisWorld));
        float sumInverseMass = 0;
        final Vector3 I1R1PlusUCrossN1 = new Vector3(0, 0, 0);
        final Vector3 I1R1PlusUCrossN2 = new Vector3(0, 0, 0);
        final Vector3 I2R2CrossN1 = new Vector3(0, 0, 0);
        final Vector3 I2R2CrossN2 = new Vector3(0, 0, 0);
        if (mBody1.getIsMotionEnabled()) {
            sumInverseMass += mBody1.getMassInverse();
            I1R1PlusUCrossN1.set(Matrix3x3.multiply(I1, mR1PlusUCrossN1));
            I1R1PlusUCrossN2.set(Matrix3x3.multiply(I1, mR1PlusUCrossN2));
        }
        if (mBody2.getIsMotionEnabled()) {
            sumInverseMass += mBody2.getMassInverse();
            I2R2CrossN1.set(Matrix3x3.multiply(I2, mR2CrossN1));
            I2R2CrossN2.set(Matrix3x3.multiply(I2, mR2CrossN2));
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
        final float beta = 0.2f;     // TODO : Use a constant here
        final float biasFactor = (beta / constraintSolverData.getTimeStep());
        if (mPositionCorrectionTechnique == JointsPositionCorrectionTechnique.BAUMGARTE_JOINTS) {
            mBTranslation.setX(u.dot(mN1));
            mBTranslation.setY(u.dot(mN2));
            mBTranslation.multiply(biasFactor);
        }
        mInverseMassMatrixRotationConstraint.setToZero();
        if (mBody1.getIsMotionEnabled()) {
            mInverseMassMatrixRotationConstraint.add(I1);
        }
        if (mBody2.getIsMotionEnabled()) {
            mInverseMassMatrixRotationConstraint.add(I2);
        }
        if (mBody1.getIsMotionEnabled() || mBody2.getIsMotionEnabled()) {
            mInverseMassMatrixRotationConstraint.set(mInverseMassMatrixRotationConstraint.getInverse());
        }
        mBRotation.setToZero();
        if (mPositionCorrectionTechnique == JointsPositionCorrectionTechnique.BAUMGARTE_JOINTS) {
            final Quaternion currentOrientationDifference = Quaternion.multiply(orientationBody2, orientationBody1.getInverse());
            currentOrientationDifference.normalize();
            final Quaternion qError = Quaternion.multiply(currentOrientationDifference, mInitOrientationDifference.getInverse());
            mBRotation.set(Vector3.multiply(biasFactor * 2, qError.getVectorV()));
        }
        mInverseMassMatrixLimit = sumInverseMass + mR1PlusUCrossSliderAxis.dot(Matrix3x3.multiply(I1, mR1PlusUCrossSliderAxis))
                + mR2CrossSliderAxis.dot(Matrix3x3.multiply(I2, mR2CrossSliderAxis));
        mBLowerLimit = 0;
        if (mPositionCorrectionTechnique == JointsPositionCorrectionTechnique.BAUMGARTE_JOINTS) {
            mBLowerLimit = biasFactor * lowerLimitError;
        }
        mBUpperLimit = 0;
        if (mPositionCorrectionTechnique == JointsPositionCorrectionTechnique.BAUMGARTE_JOINTS) {
            mBUpperLimit = biasFactor * upperLimitError;
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
        final Vector3 linearImpulseBody1 = Vector3.subtract(Vector3.multiply(Vector3.negate(mN1), mImpulseTranslation.getX()), Vector3.multiply(mN2, mImpulseTranslation.getY()));
        final Vector3 angularImpulseBody1 = Vector3.add(Vector3.multiply(Vector3.negate(mR1PlusUCrossN1), mImpulseTranslation.getX()), Vector3.multiply(mR1PlusUCrossN2, mImpulseTranslation.getY()));
        final Vector3 linearImpulseBody2 = Vector3.negate(linearImpulseBody1);
        final Vector3 angularImpulseBody2 = Vector3.subtract(Vector3.multiply(mR2CrossN1, mImpulseTranslation.getX()), Vector3.multiply(mR2CrossN2, mImpulseTranslation.getY()));
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
        final float el1 = -mN1.dot(v1) - w1.dot(mR1PlusUCrossN1) + mN1.dot(v2) + w2.dot(mR2CrossN1);
        final float el2 = -mN2.dot(v1) - w1.dot(mR1PlusUCrossN2) + mN2.dot(v2) + w2.dot(mR2CrossN2);
        final Vector2 JvTranslation = new Vector2(el1, el2);
        final Vector2 deltaLambda = Matrix2x2.multiply(mInverseMassMatrixTranslationConstraint, Vector2.subtract(Vector2.negate(JvTranslation), mBTranslation));
        mImpulseTranslation.add(deltaLambda);
        final Vector3 linearImpulseBody1 = Vector3.subtract(Vector3.multiply(Vector3.negate(mN1), deltaLambda.getX()), Vector3.multiply(mN2, deltaLambda.getY()));
        final Vector3 angularImpulseBody1 = Vector3.subtract(Vector3.multiply(Vector3.negate(mR1PlusUCrossN1), deltaLambda.getX()), Vector3.multiply(mR1PlusUCrossN2, deltaLambda.getY()));
        final Vector3 linearImpulseBody2 = Vector3.negate(linearImpulseBody1);
        final Vector3 angularImpulseBody2 = Vector3.add(Vector3.multiply(mR2CrossN1, deltaLambda.getX()), Vector3.multiply(mR2CrossN2, deltaLambda.getY()));
        if (mBody1.getIsMotionEnabled()) {
            v1.add(Vector3.multiply(inverseMassBody1, linearImpulseBody1));
            w1.add(Matrix3x3.multiply(I1, angularImpulseBody1));
        }
        if (mBody2.getIsMotionEnabled()) {
            v2.add(Vector3.multiply(inverseMassBody2, linearImpulseBody2));
            w2.add(Matrix3x3.multiply(I2, angularImpulseBody2));
        }
        final Vector3 JvRotation = Vector3.subtract(w2, w1);
        final Vector3 deltaLambda2 = Matrix3x3.multiply(mInverseMassMatrixRotationConstraint, Vector3.subtract(Vector3.negate(JvRotation), mBRotation));
        mImpulseRotation.add(deltaLambda2);
        angularImpulseBody1.set(Vector3.negate(deltaLambda2));
        angularImpulseBody2.set(deltaLambda2);
        if (mBody1.getIsMotionEnabled()) {
            w1.add(Matrix3x3.multiply(I1, angularImpulseBody1));
        }
        if (mBody2.getIsMotionEnabled()) {
            w2.add(Matrix3x3.multiply(I2, angularImpulseBody2));
        }
        if (mIsLimitsActive) {
            if (mIsLowerLimitViolated) {
                final float JvLowerLimit = mSliderAxisWorld.dot(v2) + mR2CrossSliderAxis.dot(w2) - mSliderAxisWorld.dot(v1) - mR1PlusUCrossSliderAxis.dot(w1);
                float deltaLambdaLower = mInverseMassMatrixLimit * (-JvLowerLimit - mBLowerLimit);
                final float lambdaTemp = mImpulseLowerLimit;
                mImpulseLowerLimit = Math.max(mImpulseLowerLimit + deltaLambdaLower, 0);
                deltaLambdaLower = mImpulseLowerLimit - lambdaTemp;
                linearImpulseBody1.set(Vector3.multiply(-deltaLambdaLower, mSliderAxisWorld));
                angularImpulseBody1.set(Vector3.multiply(-deltaLambdaLower, mR1PlusUCrossSliderAxis));
                linearImpulseBody2.set(Vector3.negate(linearImpulseBody1));
                angularImpulseBody2.set(Vector3.multiply(deltaLambdaLower, mR2CrossSliderAxis));
                if (mBody1.getIsMotionEnabled()) {
                    v1.add(Vector3.multiply(inverseMassBody1, linearImpulseBody1));
                    w1.add(Matrix3x3.multiply(I1, angularImpulseBody1));
                }
                if (mBody2.getIsMotionEnabled()) {
                    v2.add(Vector3.multiply(inverseMassBody2, linearImpulseBody2));
                    w2.add(Matrix3x3.multiply(I2, angularImpulseBody2));
                }
            }
            if (mIsUpperLimitViolated) {
                final float JvUpperLimit = mSliderAxisWorld.dot(v1) + mR1PlusUCrossSliderAxis.dot(w1) - mSliderAxisWorld.dot(v2) - mR2CrossSliderAxis.dot(w2);
                float deltaLambdaUpper = mInverseMassMatrixLimit * (-JvUpperLimit - mBUpperLimit);
                final float lambdaTemp = mImpulseUpperLimit;
                mImpulseUpperLimit = Math.max(mImpulseUpperLimit + deltaLambdaUpper, 0);
                deltaLambdaUpper = mImpulseUpperLimit - lambdaTemp;
                linearImpulseBody1.set(Vector3.multiply(deltaLambdaUpper, mSliderAxisWorld));
                angularImpulseBody1.set(Vector3.multiply(deltaLambdaUpper, mR1PlusUCrossSliderAxis));
                linearImpulseBody2.set(Vector3.negate(linearImpulseBody1));
                angularImpulseBody2.set(Vector3.multiply(-deltaLambdaUpper, mR2CrossSliderAxis));
                if (mBody1.getIsMotionEnabled()) {
                    v1.add(Vector3.multiply(inverseMassBody1, linearImpulseBody1));
                    w1.add(Matrix3x3.multiply(I1, angularImpulseBody1));
                }
                if (mBody2.getIsMotionEnabled()) {
                    v2.add(Vector3.multiply(inverseMassBody2, linearImpulseBody2));
                    w2.add(Matrix3x3.multiply(I2, angularImpulseBody2));
                }
            }
        }
    }

    @Override
    public void solvePositionConstraint(ConstraintSolverData constraintSolverData) {
    }

    /**
     * This structure is used to gather the information needed to create a slider joint. This structure will be used to create the actual slider joint.
     */
    public static class SliderJointInfo extends ConstraintInfo {
        private final Vector3 anchorPointWorldSpace = new Vector3();
        private final Vector3 sliderAxisWorldSpace = new Vector3();
        private boolean isLimitsActive;
        private float lowerLimit;
        private float upperLimit;

        /**
         * Constructs a new unlimited slider joint info from the both bodies, the initial anchor point in world space and the init axis, also in world space.
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
            isLimitsActive = false;
            lowerLimit = -1;
            upperLimit = 1;
        }

        /**
         * Constructs a new limited slider joint info from the both bodies, the initial anchor point in world space and the init axis, also in world space.
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
            isLimitsActive = true;
            lowerLimit = initLowerLimit;
            upperLimit = initUpperLimit;
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

        /**
         * Returns the axis in world space.
         *
         * @return The axis in world space
         */
        public Vector3 getSliderAxisWorldSpace() {
            return sliderAxisWorldSpace;
        }

        /**
         * Sets the axis, in world space.
         *
         * @param sliderAxisWorldSpace The axis in world space
         */
        public void setSliderAxisWorldSpace(Vector3 sliderAxisWorldSpace) {
            this.sliderAxisWorldSpace.set(sliderAxisWorldSpace);
        }

        /**
         * Returns true if the limits are active.
         *
         * @return Whether or not the limits are active
         */
        public boolean isLimitsActive() {
            return isLimitsActive;
        }

        /**
         * Returns the lower limit.
         *
         * @return The lower limit
         */
        public float getLowerLimit() {
            return lowerLimit;
        }

        /**
         * Returns the upper limit.
         *
         * @return The upper limit
         */
        public float getUpperLimit() {
            return upperLimit;
        }
    }
}
