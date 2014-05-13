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

import org.spout.physics.body.RigidBody;
import org.spout.physics.constraint.ConstraintSolver.ConstraintSolverData;
import org.spout.physics.math.Matrix3x3;
import org.spout.physics.math.Quaternion;
import org.spout.physics.math.Transform;
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

    /**
     * Constructs a slider joint from provided slider joint info.
     *
     * @param jointInfo The joint info
     */
    public SliderJoint(SliderJointInfo jointInfo) {
        super(jointInfo);
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
        final float sumInverseMass = mBody1.getMassInverse() + mBody2.getMassInverse();
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
        final Matrix3x3 inverseInertiaTensorBody1 = mBody1.getInertiaTensorInverseWorld();
        final Matrix3x3 inverseInertiaTensorBody2 = mBody2.getInertiaTensorInverseWorld();
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
