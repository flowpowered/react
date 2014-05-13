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

import org.spout.physics.constraint.ConstraintSolver.ConstraintSolverData;
import org.spout.physics.math.Transform;
import org.spout.physics.math.Vector3;

/**
 * Represents a collision contact point between two bodies in the physics engine. The ContactPoint class inherits from the Constraint class.
 */
public class ContactPoint extends Constraint {
    private final Vector3 mNormal;
    private float mPenetrationDepth;
    private final Vector3 mLocalPointOnBody1;
    private final Vector3 mLocalPointOnBody2;
    private final Vector3 mWorldPointOnBody1;
    private final Vector3 mWorldPointOnBody2;
    private boolean mIsRestingContact;
    private final Vector3 mFrictionVector1;
    private final Vector3 mFrictionVector2;
    private float mPenetrationImpulse;
    private float mFrictionImpulse1;
    private float mFrictionImpulse2;

    /**
     * Constructs a new contact point from the contact info.
     *
     * @param contactInfo The contact info
     */
    public ContactPoint(ContactPointInfo contactInfo) {
        super(contactInfo);
        if (contactInfo.getPenetrationDepth() <= 0) {
            throw new IllegalArgumentException("Penetration depth must be greater than zero");
        }
        mNormal = new Vector3(contactInfo.getNormal());
        mPenetrationDepth = contactInfo.getPenetrationDepth();
        mLocalPointOnBody1 = new Vector3(contactInfo.getFirstLocalPoint());
        mLocalPointOnBody2 = new Vector3(contactInfo.getSecondLocalPoint());
        mWorldPointOnBody1 = Transform.multiply(contactInfo.getFirstBody().getTransform(), contactInfo.getFirstLocalPoint());
        mWorldPointOnBody2 = Transform.multiply(contactInfo.getSecondBody().getTransform(), contactInfo.getSecondLocalPoint());
        mIsRestingContact = false;
        mFrictionVector1 = new Vector3(0, 0, 0);
        mFrictionVector2 = new Vector3(0, 0, 0);
    }

    @Override
    public void initBeforeSolve(ConstraintSolverData constraintSolverData) {
    }

    @Override
    public void solveVelocityConstraint(ConstraintSolverData constraintSolverData) {
    }

    @Override
    public void solvePositionConstraint(ConstraintSolverData constraintSolverData) {

    }

    /**
     * Gets the normal vector of the contact.
     *
     * @return The contact's normal vector
     */
    public Vector3 getNormal() {
        return mNormal;
    }

    /**
     * Gets the penetration depth of the contact.
     *
     * @return The penetration depth
     */
    public float getPenetrationDepth() {
        return mPenetrationDepth;
    }

    /**
     * Sets the penetration depth of the contact.
     *
     * @param penetrationDepth The penetration depth to set
     */
    public void setPenetrationDepth(float penetrationDepth) {
        this.mPenetrationDepth = penetrationDepth;
    }

    /**
     * Gets the contact point on the first body.
     *
     * @return The contact point
     */
    public Vector3 getLocalPointOnFirstBody() {
        return mLocalPointOnBody1;
    }

    /**
     * Gets the contact point on the second body.
     *
     * @return The contact point
     */
    public Vector3 getLocalPointOnSecondBody() {
        return mLocalPointOnBody2;
    }

    /**
     * Gets the contact point in world space on the first body.
     *
     * @return The contact point
     */
    public Vector3 getWorldPointOnFirstBody() {
        return mWorldPointOnBody1;
    }

    /**
     * Gets the contact point in world space on the second body.
     *
     * @return The contact point
     */
    public Vector3 getWorldPointOnSecondBody() {
        return mWorldPointOnBody2;
    }

    /**
     * Sets the contact point in world space on the first body.
     *
     * @param worldPoint The contact point in world space
     */
    public void setWorldPointOnFirstBody(Vector3 worldPoint) {
        mWorldPointOnBody1.set(worldPoint);
    }

    /**
     * Sets the contact point in world space on the second body.
     *
     * @param worldPoint The contact point in world space
     */
    public void setWorldPointOnSecondBody(Vector3 worldPoint) {
        mWorldPointOnBody2.set(worldPoint);
    }

    /**
     * Returns true if the contact is a resting contact, false if not.
     *
     * @return Whether or not the contact is a resting contact
     */
    public boolean isRestingContact() {
        return mIsRestingContact;
    }

    /**
     * Sets if the contact is a resting contact.
     *
     * @param isRestingContact True for a resting contact, false if otherwise.
     */
    public void setRestingContact(boolean isRestingContact) {
        mIsRestingContact = isRestingContact;
    }

    /**
     * Gets the first friction vector.
     *
     * @return The friction vector
     */
    public Vector3 getFirstFrictionVector() {
        return mFrictionVector1;
    }

    /**
     * Sets the first friction vector.
     *
     * @param firstFrictionVector The friction vector to set
     */
    public void setFirstFrictionVector(Vector3 firstFrictionVector) {
        mFrictionVector1.set(firstFrictionVector);
    }

    /**
     * Gets the second friction vector.
     *
     * @return The friction vector
     */
    public Vector3 getSecondFrictionVector() {
        return mFrictionVector2;
    }

    /**
     * Sets the second friction vector.
     *
     * @param secondFrictionVector The friction vector to set
     */
    public void setSecondFrictionVector(Vector3 secondFrictionVector) {
        mFrictionVector2.set(secondFrictionVector);
    }

    /**
     * Returns the cached penetration impulse.
     *
     * @return the penetration impulse
     */
    public float getPenetrationImpulse() {
        return mPenetrationImpulse;
    }

    /**
     * Returns the cached first friction impulse.
     *
     * @return the first friction impulse
     */
    public float getFirstFrictionImpulse() {
        return mFrictionImpulse1;
    }

    /**
     * Returns the cached second friction impulse.
     *
     * @return the second friction impulse
     */
    public float getSecondFrictionImpulse() {
        return mFrictionImpulse2;
    }

    /**
     * Sets the cached penetration impulse.
     *
     * @param impulse the penetration impulse
     */
    public void setPenetrationImpulse(float impulse) {
        mPenetrationImpulse = impulse;
    }

    /**
     * Sets the first cached friction impulse.
     *
     * @param impulse the first friction impulse
     */
    public void setFirstFrictionImpulse(float impulse) {
        mFrictionImpulse1 = impulse;
    }

    /**
     * Sets the second cached friction impulse.
     *
     * @param impulse the second friction impulse
     */
    public void setSecondFrictionImpulse(float impulse) {
        mFrictionImpulse2 = impulse;
    }

    /**
     * This structure contains information about a collision contact computed during the narrow-phase collision detection. This information is used to compute the contact set for a contact between two
     * bodies.
     */
    public static class ContactPointInfo extends ConstraintInfo {
        private final Vector3 normal = new Vector3();
        private float penetrationDepth;
        private final Vector3 localPoint1 = new Vector3();
        private final Vector3 localPoint2 = new Vector3();

        /**
         * Constructs a new empty contact point info.
         */
        public ContactPointInfo() {
            super(ConstraintType.CONTACT);
        }

        /**
         * Constructs a new contact point info from the contact normal, penetration depth and local contact points on both bodies.
         *
         * @param normal The contact normal
         * @param penetrationDepth The penetration depth
         * @param localPoint1 The contact point on the first body
         * @param localPoint2 The contact point on the second body
         */
        public ContactPointInfo(Vector3 normal, float penetrationDepth, Vector3 localPoint1, Vector3 localPoint2) {
            super(ConstraintType.CONTACT);
            this.normal.set(normal);
            this.penetrationDepth = penetrationDepth;
            this.localPoint1.set(localPoint1);
            this.localPoint2.set(localPoint2);
        }

        /**
         * Gets the normal vector of the contact.
         *
         * @return The contact's normal vector
         */
        public Vector3 getNormal() {
            return normal;
        }

        /**
         * Gets the penetration depth of the contact.
         *
         * @return The penetration depth
         */
        public float getPenetrationDepth() {
            return penetrationDepth;
        }

        /**
         * Gets the contact point on the first body.
         *
         * @return The contact point
         */
        public Vector3 getFirstLocalPoint() {
            return localPoint1;
        }

        /**
         * Gets the contact point on the second body.
         *
         * @return The contact point
         */
        public Vector3 getSecondLocalPoint() {
            return localPoint2;
        }

        /**
         * Sets the normal vector of the contact.
         *
         * @param normal The contact normal
         */
        public void setNormal(Vector3 normal) {
            this.normal.set(normal);
        }

        /**
         * Sets the penetration depth of the contact.
         *
         * @param penetrationDepth The penetration depth
         */
        public void setPenetrationDepth(float penetrationDepth) {
            this.penetrationDepth = penetrationDepth;
        }

        /**
         * Sets the contact point on the first body.
         *
         * @param localPoint1 The contact point on the first body
         */
        public void setFirstLocalPoint(Vector3 localPoint1) {
            this.localPoint1.set(localPoint1);
        }

        /**
         * Sets the contact point on the second body.
         *
         * @param localPoint2 The contact point on the second body
         */
        public void setSecondLocalPoint(Vector3 localPoint2) {
            this.localPoint2.set(localPoint2);
        }

        /**
         * Sets all the values of this contact point info.
         *
         * @param normal The contact normal
         * @param penetrationDepth The penetration depth
         * @param localPoint1 The contact point on the first body
         * @param localPoint2 The contact point on the second body
         */
        public void set(Vector3 normal, float penetrationDepth, Vector3 localPoint1, Vector3 localPoint2) {
            setNormal(normal);
            setPenetrationDepth(penetrationDepth);
            setFirstLocalPoint(localPoint1);
            setSecondLocalPoint(localPoint2);
        }
    }
}
