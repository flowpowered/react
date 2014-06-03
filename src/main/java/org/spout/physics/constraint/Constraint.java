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

/**
 * This is the base class of a constraint in the physics engine. A constraint can be a collision contact or a joint for instance.
 */
public abstract class Constraint {
    protected final RigidBody mBody1;
    protected final RigidBody mBody2;
    protected final boolean mActive;
    protected final ConstraintType mType;
    protected int mIndexBody1;
    protected int mIndexBody2;
    protected final JointsPositionCorrectionTechnique mPositionCorrectionTechnique;
    protected final boolean mIsCollisionEnabled;
    protected boolean mIsAlreadyInIsland;

    /**
     * Constructs a new constraint from the provided constraint info.
     *
     * @param constraintInfo The constraint info for this constraint
     */
    public Constraint(ConstraintInfo constraintInfo) {
        mBody1 = constraintInfo.getFirstBody();
        mBody2 = constraintInfo.getSecondBody();
        if (mBody1 == null) {
            throw new IllegalArgumentException("First body cannot be null");
        }
        if (mBody2 == null) {
            throw new IllegalArgumentException("Second body cannot be null");
        }
        mActive = true;
        mType = constraintInfo.getType();
        mPositionCorrectionTechnique = constraintInfo.getPositionCorrectionTechnique();
        mIsCollisionEnabled = constraintInfo.isCollisionEnabled();
        mIsAlreadyInIsland = false;
    }

    /**
     * Initializes before solving the constraint.
     *
     * @param constraintSolverData The related data
     */
    public abstract void initBeforeSolve(ConstraintSolverData constraintSolverData);

    /**
     * Warm-starts the constraint (apply the previous impulse at the beginning of the step).
     *
     * @param constraintSolverData The related data
     */
    public abstract void warmstart(ConstraintSolverData constraintSolverData);

    /**
     * Solves the velocity constraint.
     *
     * @param constraintSolverData The related data
     */
    public abstract void solveVelocityConstraint(ConstraintSolverData constraintSolverData);

    /**
     * Solves the position constraint.
     *
     * @param constraintSolverData The related data
     */
    public abstract void solvePositionConstraint(ConstraintSolverData constraintSolverData);

    /**
     * Gets the first body.
     *
     * @return The first body
     */
    public RigidBody getFirstBody() {
        return mBody1;
    }

    /**
     * Gets the second body.
     *
     * @return The second body
     */
    public RigidBody getSecondBody() {
        return mBody2;
    }

    /**
     * Returns true if the constraint is active, false if not.
     *
     * @return Whether or not the constraint is active
     */
    public boolean isActive() {
        return mActive;
    }

    /**
     * Gets the type of constraint.
     *
     * @return The constraint type
     */
    public ConstraintType getType() {
        return mType;
    }

    /**
     * Returns true if the collision between the two bodies of the constraint is enabled.
     *
     * @return Whether or not the collision is enabled
     */
    public boolean isCollisionEnabled() {
        return mIsCollisionEnabled;
    }

    /**
     * Returns true if the joint has already been added into an island.
     *
     * @return Whether or not the joint is already in an island
     */
    public boolean isAlreadyInIsland() {
        return mIsAlreadyInIsland;
    }

    /**
     * Sets whether or not this joint has already been added into an island.
     *
     * @param isAlreadyInIsland Whether or not the joint is already in an island
     */
    public void setIsAlreadyInIsland(boolean isAlreadyInIsland) {
        mIsAlreadyInIsland = isAlreadyInIsland;
    }

    /**
     * An enumeration of the possible constraint types (contact).
     */
    public static enum ConstraintType {
        CONTACT,
        BALLSOCKETJOINT,
        SLIDERJOINT,
        HINGEJOINT,
        FIXEDJOINT
    }

    /**
     * This structure is used to gather the information needed to create a constraint.
     */
    public static class ConstraintInfo {
        private RigidBody body1;
        private RigidBody body2;
        private final ConstraintType type;
        private boolean isCollisionEnabled;
        private JointsPositionCorrectionTechnique positionCorrectionTechnique;

        /**
         * Constructs a new constraint info from the constraint type.
         *
         * @param type The type of this constraint
         */
        public ConstraintInfo(ConstraintType type) {
            body1 = null;
            body2 = null;
            this.type = type;
            positionCorrectionTechnique = JointsPositionCorrectionTechnique.NON_LINEAR_GAUSS_SEIDEL;
            isCollisionEnabled = true;
        }

        /**
         * Constructs a new constraint info from the constraint type and the two bodies involved.
         *
         * @param body1 The first involved body
         * @param body2 The second involved body
         * @param type The type of this constraint
         */
        public ConstraintInfo(RigidBody body1, RigidBody body2, ConstraintType type) {
            this.body1 = body1;
            this.body2 = body2;
            this.type = type;
            positionCorrectionTechnique = JointsPositionCorrectionTechnique.NON_LINEAR_GAUSS_SEIDEL;
            isCollisionEnabled = true;
        }

        /**
         * Returns the constraint type.
         *
         * @return The type
         */
        public ConstraintType getType() {
            return type;
        }

        /**
         * Returns the first body involved in the constraint.
         *
         * @return The first body
         */
        public RigidBody getFirstBody() {
            return body1;
        }

        /**
         * Returns the second body involved in the constraint.
         *
         * @return The second body
         */
        public RigidBody getSecondBody() {
            return body2;
        }

        /**
         * Sets the first body involved in the contact.
         *
         * @param body1 The The first involved body
         */
        public void setFirstBody(RigidBody body1) {
            this.body1 = body1;
        }

        /**
         * Sets the second body involved in the contact.
         *
         * @param body2 The second involved body
         */
        public void setSecondBody(RigidBody body2) {
            this.body2 = body2;
        }

        /**
         * Returns true if collisions are enabled.
         *
         * @return Whether or not collisions are enabled
         */
        public boolean isCollisionEnabled() {
            return isCollisionEnabled;
        }

        /**
         * Sets collisions as enabled or not.
         *
         * @param isCollisionEnabled The new state of collision enabled
         */
        public void setCollisionEnabled(boolean isCollisionEnabled) {
            this.isCollisionEnabled = isCollisionEnabled;
        }

        /**
         * Returns the joints position correction technique.
         *
         * @return The correction technique
         */
        public JointsPositionCorrectionTechnique getPositionCorrectionTechnique() {
            return positionCorrectionTechnique;
        }

        /**
         * Sets the position correction technique.
         *
         * @param positionCorrectionTechnique The position correction technique
         */
        public void setPositionCorrectionTechnique(JointsPositionCorrectionTechnique positionCorrectionTechnique) {
            this.positionCorrectionTechnique = positionCorrectionTechnique;
        }
    }

    /**
     * This structure represents a single element of a linked list of joints.
     */
    public static class JointListElement {
        private Constraint joint;
        private JointListElement next;

        /**
         * Constructs a new joint list element from the initial joint and next list element.
         *
         * @param initJoint The joint
         * @param initNext The next element
         */
        public JointListElement(Constraint initJoint, JointListElement initNext) {
            joint = initJoint;
            next = initNext;
        }

        /**
         * Returns the joint in this list element.
         *
         * @return The joint
         */
        public Constraint getJoint() {
            return joint;
        }

        /**
         * Sets the joint in this list element.
         *
         * @param joint The joint
         */
        public void setJoint(Constraint joint) {
            this.joint = joint;
        }

        /**
         * Returns the next element in the list.
         *
         * @return The next element
         */
        public JointListElement getNext() {
            return next;
        }

        /**
         * Sets the next element in the list.
         *
         * @param next The next element
         */
        public void setNext(JointListElement next) {
            this.next = next;
        }
    }
}
