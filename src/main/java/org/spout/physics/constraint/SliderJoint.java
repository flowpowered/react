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
    private final Vector3 mU1World = new Vector3();
    private final Vector3 mU2World = new Vector3();
    private final Vector3 mN1 = new Vector3();
    private final Vector3 mN2 = new Vector3();
    private final Vector3 mU1WorldCrossN1 = new Vector3();
    private final Vector3 mU1WorldCrossN2 = new Vector3();
    private final Vector3 mU2WorldCrossN1 = new Vector3();
    private final Vector3 mU2WorldCrossN2 = new Vector3();
    private final Matrix2x2 mInverseMassMatrixTranslationConstraint = new Matrix2x2();
    private final Matrix3x3 mInverseMassMatrixRotationConstraint = new Matrix3x3();
    private final Vector2 mImpulseTranslation;
    private final Vector3 mImpulseRotation;

    /**
     * Constructs a slider joint from provided slider joint info.
     *
     * @param jointInfo The joint info
     */
    public SliderJoint(SliderJointInfo jointInfo) {
        super(jointInfo);
        mImpulseTranslation = new Vector2(0, 0);
        mImpulseRotation = new Vector3(0, 0, 0);
        mLocalAnchorPointBody1.set(Transform.multiply(mBody1.getTransform().getInverse(), jointInfo.getAnchorPointWorldSpace()));
        mLocalAnchorPointBody2.set(Transform.multiply(mBody2.getTransform().getInverse(), jointInfo.getAnchorPointWorldSpace()));
    }

    @Override
    public void initBeforeSolve(ConstraintSolverData constraintSolverData) {
        mIndexBody1 = constraintSolverData.getMapBodyToConstrainedVelocityIndex().get(mBody1);
        mIndexBody2 = constraintSolverData.getMapBodyToConstrainedVelocityIndex().get(mBody2);
        final Quaternion orientationBody1 = mBody1.getTransform().getOrientation();
        final Quaternion orientationBody2 = mBody2.getTransform().getOrientation();
        final Matrix3x3 I1 = mBody1.getInertiaTensorInverseWorld();
        final Matrix3x3 I2 = mBody2.getInertiaTensorInverseWorld();
        mU1World.set(Quaternion.multiply(orientationBody1, mLocalAnchorPointBody1));
        mU2World.set(Quaternion.multiply(orientationBody2, mLocalAnchorPointBody2));
        mN1.set(mU1World.getOneUnitOrthogonalVector());
        mN2.set(mU1World.cross(mN1));
        mU1WorldCrossN1.set(mN2);
        mU1WorldCrossN2.set(mU1World.cross(mN2));
        mU2WorldCrossN1.set(mU2World.cross(mN1));
        mU2WorldCrossN2.set(mU2World.cross(mN2));
        final float n1Dotn1 = mN1.lengthSquare();
        final float n2Dotn2 = mN2.lengthSquare();
        final float n1Dotn2 = mN1.dot(mN2);
        final float sumInverseMass = mBody1.getMassInverse() + mBody2.getMassInverse();
        final Vector3 I1U2CrossN1 = Matrix3x3.multiply(I1, mU2WorldCrossN1);
        final Vector3 I1U2CrossN2 = Matrix3x3.multiply(I1, mU2WorldCrossN2);
        final Vector3 I2U1CrossN1 = Matrix3x3.multiply(I2, mU1WorldCrossN1);
        final Vector3 I2U1CrossN2 = Matrix3x3.multiply(I2, mU1WorldCrossN2);
        final float el11 = sumInverseMass * (n1Dotn1) + mU2WorldCrossN1.dot(I1U2CrossN1) + mU1WorldCrossN1.dot(I2U1CrossN1);
        final float el12 = sumInverseMass * (n1Dotn2) + mU2WorldCrossN1.dot(I1U2CrossN2) + mU1WorldCrossN1.dot(I2U1CrossN2);
        final float el21 = sumInverseMass * (n1Dotn2) + mU2WorldCrossN2.dot(I1U2CrossN1) + mU1WorldCrossN2.dot(I2U1CrossN1);
        final float el22 = sumInverseMass * (n2Dotn2) + mU2WorldCrossN2.dot(I1U2CrossN2) + mU1WorldCrossN2.dot(I2U1CrossN2);
        final Matrix2x2 matrixKTranslation = new Matrix2x2(el11, el12, el21, el22);
        mInverseMassMatrixTranslationConstraint.set(matrixKTranslation.getInverse());
        mInverseMassMatrixRotationConstraint.set(Matrix3x3.add(I1, I2));
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
        final Vector3 angularImpulseBody1 = Vector3.add(Vector3.multiply(mU2WorldCrossN1, mImpulseTranslation.getX()), Vector3.multiply(mU2WorldCrossN2, mImpulseTranslation.getY()));
        final Vector3 linearImpulseBody2 = Vector3.add(Vector3.multiply(mN1, mImpulseTranslation.getX()), Vector3.multiply(mN2, mImpulseTranslation.getY()));
        final Vector3 angularImpulseBody2 = Vector3.subtract(Vector3.multiply(Vector3.negate(mU1WorldCrossN1), mImpulseTranslation.getX()), Vector3.multiply(mU1WorldCrossN2, mImpulseTranslation.getY()));
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
        final Vector3 x1 = mBody1.getTransform().getPosition();
        final Vector3 x2 = mBody2.getTransform().getPosition();
        final Vector3 v1 = constraintSolverData.getLinearVelocities().get(mIndexBody1);
        final Vector3 v2 = constraintSolverData.getLinearVelocities().get(mIndexBody2);
        final Vector3 w1 = constraintSolverData.getAngularVelocities().get(mIndexBody1);
        final Vector3 w2 = constraintSolverData.getAngularVelocities().get(mIndexBody2);
        final float inverseMassBody1 = mBody1.getMassInverse();
        final float inverseMassBody2 = mBody2.getMassInverse();
        final Matrix3x3 I1 = mBody1.getInertiaTensorInverseWorld();
        final Matrix3x3 I2 = mBody2.getInertiaTensorInverseWorld();
        final float el1 = -mN1.dot(v1) + mU2WorldCrossN1.dot(w1) + mN1.dot(v2) - mU1WorldCrossN1.dot(w2);
        final float el2 = -mN2.dot(v1) + mU2WorldCrossN2.dot(w1) + mN2.dot(v2) - mU1WorldCrossN2.dot(w2);
        final Vector2 JvTranslation = new Vector2(el1, el2);
        final Vector3 JvRotation = Vector3.subtract(w2, w1);
        final Vector2 bTranslation = new Vector2(0, 0);
        final float beta = 0.2f;     // TODO : Use a constant here
        final float biasFactor = (beta / constraintSolverData.getTimeStep());
        if (mPositionCorrectionTechnique == JointsPositionCorrectionTechnique.BAUMGARTE_JOINTS) {
            final Vector3 deltaV = Vector3.subtract(Vector3.subtract(Vector3.add(x2, mU2World), x1), mU1World);
            bTranslation.setX(biasFactor * deltaV.dot(mN1));
            bTranslation.setY(biasFactor * deltaV.dot(mN2));
        }
        final Vector3 bRotation = new Vector3(0, 0, 0);
        if (mPositionCorrectionTechnique == JointsPositionCorrectionTechnique.BAUMGARTE_JOINTS) {
            final Quaternion q1 = mBody1.getTransform().getOrientation();
            final Quaternion q2 = mBody2.getTransform().getOrientation();
            final Quaternion qDiff = Quaternion.multiply(q1, q2.getInverse());
            bRotation.set(Vector3.multiply(2, qDiff.getVectorV()));
        }
        final Vector2 deltaLambda = Matrix2x2.multiply(mInverseMassMatrixTranslationConstraint, Vector2.subtract(Vector2.negate(JvTranslation), bTranslation));
        mImpulseTranslation.add(deltaLambda);
        final Vector3 deltaLambda2 = Matrix3x3.multiply(mInverseMassMatrixRotationConstraint, Vector3.subtract(Vector3.negate(JvRotation), bRotation));
        mImpulseRotation.add(deltaLambda2);
        final Vector3 linearImpulseBody1 = Vector3.subtract(Vector3.multiply(Vector3.negate(mN1), deltaLambda.getX()), Vector3.multiply(mN2, deltaLambda.getY()));
        final Vector3 angularImpulseBody1 = Vector3.add(Vector3.multiply(mU2WorldCrossN1, deltaLambda.getX()), Vector3.multiply(mU2WorldCrossN2, deltaLambda.getY()));
        final Vector3 linearImpulseBody2 = Vector3.add(Vector3.multiply(mN1, deltaLambda.getX()), Vector3.multiply(mN2, deltaLambda.getY()));
        final Vector3 angularImpulseBody2 = Vector3.subtract(Vector3.multiply(Vector3.negate(mU1WorldCrossN1), deltaLambda.getX()), Vector3.multiply(mU1WorldCrossN2, deltaLambda.getY()));
        angularImpulseBody1.add(Vector3.negate(deltaLambda2));
        angularImpulseBody2.add(deltaLambda2);
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
    public void solvePositionConstraint(ConstraintSolverData constraintSolverData) {
    }

    /**
     * This structure is used to gather the information needed to create a slider joint. This structure will be used to create the actual slider joint.
     */
    public class SliderJointInfo extends ConstraintInfo {
        private final Vector3 anchorPointWorldSpace = new Vector3();
        private final Vector3 axisWorldSpace = new Vector3();

        /**
         * Constructs a new slider joint info from the both bodies, the initial anchor point in world space and the init axis, also in world space.
         *
         * @param body1 The first body
         * @param body2 The second body
         * @param initAnchorPointWorldSpace The initial anchor point in world space
         * @param initAxisWorldSpace The initial axis in world space
         */
        public SliderJointInfo(RigidBody body1, RigidBody body2, Vector3 initAnchorPointWorldSpace, Vector3 initAxisWorldSpace) {
            super(body1, body2, ConstraintType.SLIDERJOINT);
            anchorPointWorldSpace.set(initAnchorPointWorldSpace);
            axisWorldSpace.set(initAxisWorldSpace);
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
        public Vector3 getAxisWorldSpace() {
            return axisWorldSpace;
        }

        /**
         * Sets the axis, in world space.
         *
         * @param axisWorldSpace The axis in world space
         */
        public void setAxisWorldSpace(Vector3 axisWorldSpace) {
            this.axisWorldSpace.set(axisWorldSpace);
        }
    }
}
