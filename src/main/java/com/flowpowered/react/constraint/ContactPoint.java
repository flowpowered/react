/*
 * This file is part of React, licensed under the MIT License (MIT).
 *
 * Copyright (c) 2013 Flow Powered <https://flowpowered.com/>
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://danielchappuis.ch>
 * React is re-licensed with permission from ReactPhysics3D author.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
package com.flowpowered.react.constraint;

import com.flowpowered.react.body.RigidBody;
import com.flowpowered.react.math.Transform;
import com.flowpowered.react.math.Vector3;

/**
 * Represents a collision contact point between two bodies in the physics engine.
 */
public class ContactPoint {
    private final RigidBody mBody1;
    private final RigidBody mBody2;
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
        if (contactInfo.getPenetrationDepth() <= 0) {
            throw new IllegalArgumentException("Penetration depth must be greater than zero");
        }
        mBody1 = contactInfo.getFirstBody();
        mBody2 = contactInfo.getSecondBody();
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

    /**
     * Returns the first body in the contact.
     *
     * @return The first body
     */
    public RigidBody getFirstBody() {
        return mBody1;
    }

    /**
     * Returns the second body in the contact.
     *
     * @return The second body
     */
    public RigidBody getSecondBody() {
        return mBody2;
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
    public static class ContactPointInfo {
        private RigidBody body1;
        private RigidBody body2;
        private final Vector3 normal = new Vector3();
        private float penetrationDepth;
        private final Vector3 localPoint1 = new Vector3();
        private final Vector3 localPoint2 = new Vector3();

        /**
         * Constructs a new empty contact point info.
         */
        public ContactPointInfo() {
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
            this.normal.set(normal);
            this.penetrationDepth = penetrationDepth;
            this.localPoint1.set(localPoint1);
            this.localPoint2.set(localPoint2);
        }

        /**
         * Returns the first body involved in the contact.
         *
         * @return The first body
         */
        public RigidBody getFirstBody() {
            return body1;
        }

        /**
         * Returns the second body involved in the contact.
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
