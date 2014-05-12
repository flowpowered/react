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

/**
 * This is the base class of a constraint in the physics engine. A constraint can be a collision contact or a joint for instance.
 */
public class Constraint {
    protected final RigidBody mBody1;
    protected final RigidBody mBody2;
    protected final boolean mActive;
    protected final ConstraintType mType;

    /**
     * Constructs a new constraint from the provided constraint info.
     *
     * @param constraintInfo The constraint info for this constraint
     */
    public Constraint(ConstraintInfo constraintInfo) {
        if (constraintInfo.body1 == null) {
            throw new IllegalArgumentException("First body cannot be null");
        }
        if (constraintInfo.body2 == null) {
            throw new IllegalArgumentException("Second body cannot be null");
        }
        mBody1 = constraintInfo.body1;
        mBody2 = constraintInfo.body2;
        mActive = true;
        mType = constraintInfo.type;
    }

    /**
     * Gets the first body.
     *
     * @return The first body
     */
    public RigidBody getBody1() {
        return mBody1;
    }

    /**
     * Gets the second body.
     *
     * @return The second body
     */
    public RigidBody getBody2() {
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
     * An enumeration of the possible constraint types (contact).
     */
    public static enum ConstraintType {
        CONTACT,
        BALLSOCKETJOINT
    }

    /**
     * This structure is used to gather the information needed to create a constraint.
     */
    public static class ConstraintInfo {
        private RigidBody body1;
        private RigidBody body2;
        private ConstraintType type;

        /**
         * Constructs a new constraint info from the constraint type.
         *
         * @param type The type of this constraint
         */
        public ConstraintInfo(ConstraintType type) {
            body1 = null;
            body2 = null;
            this.type = type;
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
    }
}
