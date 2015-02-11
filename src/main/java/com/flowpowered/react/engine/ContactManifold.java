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
package com.flowpowered.react.engine;

import com.flowpowered.react.ReactDefaults;
import com.flowpowered.react.body.CollisionBody;
import com.flowpowered.react.constraint.ContactPoint;
import com.flowpowered.react.math.Transform;
import com.flowpowered.react.math.Vector3;

/**
 * Represents the set of contact points between two bodies. The contact manifold is implemented in a way to cache the contact points among the frames for better stability, following the "Contact
 * Generation" presentation by Erwin Coumans at the GDC 2010 conference (bullet.googlecode.com/files/GDC10_Coumans_Erwin_Contact.pdf). Some code from this class is based on the implementation of the
 * btPersistentManifold class from the Bullet physics engine (www.http://bulletphysics.org). The contacts between two bodies are added one after the other in the cache. When the cache is full, one
 * point needs to be removed. The idea is to keep the point with the deepest penetration depth and  producing the larger area (for a more stable contact manifold). The new added point is always kept.
 */
public class ContactManifold {
    public static final int MAX_CONTACT_POINTS_IN_MANIFOLD = 4;
    private final CollisionBody mBody1;
    private final CollisionBody mBody2;
    private final ContactPoint[] mContactPoints = new ContactPoint[MAX_CONTACT_POINTS_IN_MANIFOLD];
    private int mNbContactPoints = 0;
    private final Vector3 mFrictionVector1 = new Vector3();
    private final Vector3 mFrictionVector2 = new Vector3();
    private float mFrictionImpulse1 = 0;
    private float mFrictionImpulse2 = 0;
    private float mFrictionTwistImpulse = 0;
    private boolean mIsAlreadyInIsland;

    /**
     * Constructs a new contact manifold from the first and second body.
     *
     * @param body1 The first body
     * @param body2 The second body
     */
    public ContactManifold(CollisionBody body1, CollisionBody body2) {
        mBody1 = body1;
        mBody2 = body2;
        mIsAlreadyInIsland = false;
    }

    /**
     * Returns the first body in the manifold.
     *
     * @return The first body
     */
    public CollisionBody getFirstBody() {
        return mBody1;
    }

    /**
     * Returns the second body in the manifold.
     *
     * @return The second body
     */
    public CollisionBody getSecondBody() {
        return mBody2;
    }

    /**
     * Gets the number of contact points in the manifold.
     *
     * @return The number of contact points
     */
    public int getNbContactPoints() {
        return mNbContactPoints;
    }

    /**
     * Gets the first friction vector3 at the center of the contact manifold.
     *
     * @return The first friction vector
     */
    public Vector3 getFirstFrictionVector() {
        return mFrictionVector1;
    }

    /**
     * Sets the first friction vector3 at the center of the contact manifold.
     *
     * @param frictionVector1 The friction vector to set
     */
    public void setFirstFrictionVector(Vector3 frictionVector1) {
        mFrictionVector1.set(frictionVector1);
    }

    /**
     * Gets the second friction vector3 at the center of the contact manifold.
     *
     * @return The second friction vector
     */
    public Vector3 getSecondFrictionVector() {
        return mFrictionVector2;
    }

    /**
     * Sets the second friction vector3 at the center of the contact manifold.
     *
     * @param frictionVector2 The friction vector to set
     */
    public void setSecondFrictionVector(Vector3 frictionVector2) {
        mFrictionVector2.set(frictionVector2);
    }

    /**
     * Gets the accumulated impulse for the first friction.
     *
     * @return The accumulated impulse
     */
    public float getFirstFrictionImpulse() {
        return mFrictionImpulse1;
    }

    /**
     * Sets the accumulated impulse for the first friction.
     *
     * @param frictionImpulse1 The impulse to set
     */
    public void setFirstFrictionImpulse(float frictionImpulse1) {
        mFrictionImpulse1 = frictionImpulse1;
    }

    /**
     * Gets the accumulated impulse for the second friction.
     *
     * @return The accumulated impulse
     */
    public float getSecondFrictionImpulse() {
        return mFrictionImpulse2;
    }

    /**
     * Sets the accumulated impulse for the second friction.
     *
     * @param frictionImpulse2 The impulse to set
     */
    public void setSecondFrictionImpulse(float frictionImpulse2) {
        mFrictionImpulse2 = frictionImpulse2;
    }

    /**
     * Gets the accumulated impulse for the friction twist.
     *
     * @return The accumulated twist impulse
     */
    public float getFrictionTwistImpulse() {
        return mFrictionTwistImpulse;
    }

    /**
     * Sets the accumulated impulse for the friction twist.
     *
     * @param frictionTwistImpulse The twist impulse to set
     */
    public void setFrictionTwistImpulse(float frictionTwistImpulse) {
        mFrictionTwistImpulse = frictionTwistImpulse;
    }

    /**
     * Gets the contact point of the manifold at the desired index, which is greater or equal to zero and smaller than {@link #getNbContactPoints()}.
     *
     * @param index The index of the contact point
     * @return The contact point
     * @throws IllegalArgumentException If the index is smaller than zero or greater than the number of constraints, as defined by {@link #getNbContactPoints()}
     */
    public ContactPoint getContactPoint(int index) {
        if (index < 0 || index >= mNbContactPoints) {
            throw new IllegalArgumentException("index must be greater than zero and smaller than nbContatPoints");
        }
        return mContactPoints[index];
    }

    /**
     * Returns true if the contact manifold has already been added into an island.
     *
     * @return Whether or not the contact manifold is already in an island
     */
    public boolean isAlreadyInIsland() {
        return mIsAlreadyInIsland;
    }

    /**
     * Sets whether or not this contact manifold has already been added into an island.
     *
     * @param isAlreadyInIsland Whether or not the contact manifold is already in an island
     */
    public void setIsAlreadyInIsland(boolean isAlreadyInIsland) {
        mIsAlreadyInIsland = isAlreadyInIsland;
    }

    /**
     * Adds a contact point in the manifold.
     *
     * @param contact The contact point to add
     */
    public void addContactPoint(ContactPoint contact) {
        for (int i = 0; i < mNbContactPoints; i++) {
            final float distance = Vector3.subtract(mContactPoints[i].getWorldPointOnFirstBody(),
                    contact.getWorldPointOnFirstBody()).lengthSquare();
            if (distance <= ReactDefaults.PERSISTENT_CONTACT_DIST_THRESHOLD * ReactDefaults.PERSISTENT_CONTACT_DIST_THRESHOLD) {
                return;
            }
        }
        if (mNbContactPoints == MAX_CONTACT_POINTS_IN_MANIFOLD) {
            final int indexMaxPenetration = getIndexOfDeepestPenetration(contact);
            final int indexToRemove = getIndexToRemove(indexMaxPenetration, contact.getLocalPointOnFirstBody());
            removeContactPoint(indexToRemove);
        }
        mContactPoints[mNbContactPoints] = contact;
        mNbContactPoints++;
    }

    /**
     * Clears the contact manifold. Removes all contact points.
     */
    public void clear() {
        for (int i = 0; i < mNbContactPoints; i++) {
            mContactPoints[i] = null;
        }
        mNbContactPoints = 0;
    }

    // Removes a contact point from the manifold.
    private void removeContactPoint(int index) {
        if (index >= mNbContactPoints) {
            throw new IllegalArgumentException("index must be smaller than nbContactPoints");
        }
        if (mNbContactPoints <= 0) {
            throw new IllegalStateException("nbContactPoints must be greater than zero");
        }
        mContactPoints[index] = null;
        if (index < mNbContactPoints - 1) {
            mContactPoints[index] = mContactPoints[mNbContactPoints - 1];
        }
        mNbContactPoints--;
    }

    /**
     * Updates the contact manifold. First the world space coordinates of the current contacts in the manifold are recomputed from the corresponding transforms for the bodies because they have moved.
     * Then we remove the contacts with a negative penetration depth (meaning that the bodies are not penetrating anymore) and with a distance between the contact points in the plane orthogonal to the
     * contact normal that is too large.
     *
     * @param transform1 The transform of the first body
     * @param transform2 The transform of the second body
     */
    public void update(Transform transform1, Transform transform2) {
        if (mNbContactPoints == 0) {
            return;
        }
        for (int i = 0; i < mNbContactPoints; i++) {
            mContactPoints[i].setWorldPointOnFirstBody(Transform.multiply(transform1, mContactPoints[i].getLocalPointOnFirstBody()));
            mContactPoints[i].setWorldPointOnSecondBody(Transform.multiply(transform2, mContactPoints[i].getLocalPointOnSecondBody()));
            mContactPoints[i].setPenetrationDepth(Vector3.subtract(mContactPoints[i].getWorldPointOnFirstBody(), mContactPoints[i]
                    .getWorldPointOnSecondBody()).dot(mContactPoints[i].getNormal()));
        }
        final float squarePersistentContactThreshold = ReactDefaults.PERSISTENT_CONTACT_DIST_THRESHOLD *
                ReactDefaults.PERSISTENT_CONTACT_DIST_THRESHOLD;
        for (int i = mNbContactPoints - 1; i >= 0; i--) {
            if (i >= mNbContactPoints) {
                throw new IllegalStateException("i must be smaller than nbContactPoints");
            }
            final float distanceNormal = -mContactPoints[i].getPenetrationDepth();
            if (distanceNormal > squarePersistentContactThreshold) {
                removeContactPoint(i);
            } else {
                final Vector3 projOfPoint1 = Vector3.add(
                        mContactPoints[i].getWorldPointOnFirstBody(),
                        Vector3.multiply(mContactPoints[i].getNormal(), distanceNormal));
                final Vector3 projDifference = Vector3.subtract(mContactPoints[i].getWorldPointOnSecondBody(), projOfPoint1);
                if (projDifference.lengthSquare() > squarePersistentContactThreshold) {
                    removeContactPoint(i);
                }
            }
        }
    }

    // Returns the index of the contact point with the largest penetration depth.
    // This corresponding contact will be kept in the cache.
    // The method returns -1 if the new contact is the deepest.
    private int getIndexOfDeepestPenetration(ContactPoint newContact) {
        if (mNbContactPoints != MAX_CONTACT_POINTS_IN_MANIFOLD) {
            throw new IllegalStateException("nbContactPoints must be equal to MAX_CONTACT_POINTS_IN_MANIFOLD");
        }
        int indexMaxPenetrationDepth = -1;
        float maxPenetrationDepth = newContact.getPenetrationDepth();
        for (int i = 0; i < mNbContactPoints; i++) {
            if (mContactPoints[i].getPenetrationDepth() > maxPenetrationDepth) {
                maxPenetrationDepth = mContactPoints[i].getPenetrationDepth();
                indexMaxPenetrationDepth = i;
            }
        }
        return indexMaxPenetrationDepth;
    }

    // Returns the index that will be removed.
    // The index of the contact point with the largest penetration depth is given as a parameter.
    // This contact won't be removed. Given this contact, we compute the different area; we want
    // to keep the contacts with the largest area. The new point is also kept.
    // In order to compute the area of a quadrilateral, we use the formula :
    // Area = 0.5 * ||AC x BD|| where AC and BD form the diagonals of the quadrilateral.
    // Note that when we compute this area, we do not calculate it exactly but only estimate it,
    // because we do not compute the actual diagonals of the quadrilateral.
    // Therefore, this is only a guess, which is faster to compute. This idea comes from the Bullet Physics library
    // by Erwin Coumans (http://wwww.bulletphysics.org).
    private int getIndexToRemove(int indexMaxPenetration, Vector3 newPoint) {
        if (mNbContactPoints != MAX_CONTACT_POINTS_IN_MANIFOLD) {
            throw new IllegalStateException("nbContactPoints must be equal to MAX_CONTACT_POINTS_IN_MANIFOLD");
        }
        final float area123N;
        final float area023N;
        final float area013N;
        final float area012N;
        if (indexMaxPenetration != 0) {
            final Vector3 vector1 = Vector3.subtract(newPoint, mContactPoints[1].getLocalPointOnFirstBody());
            final Vector3 vector2 = Vector3.subtract(mContactPoints[3].getLocalPointOnFirstBody(), mContactPoints[2].getLocalPointOnFirstBody());
            final Vector3 crossProduct = vector1.cross(vector2);
            area123N = crossProduct.lengthSquare();
        } else {
            area123N = 0;
        }
        if (indexMaxPenetration != 1) {
            final Vector3 vector1 = Vector3.subtract(newPoint, mContactPoints[0].getLocalPointOnFirstBody());
            final Vector3 vector2 = Vector3.subtract(mContactPoints[3].getLocalPointOnFirstBody(), mContactPoints[2].getLocalPointOnFirstBody());
            final Vector3 crossProduct = vector1.cross(vector2);
            area023N = crossProduct.lengthSquare();
        } else {
            area023N = 1;
        }
        if (indexMaxPenetration != 2) {
            final Vector3 vector1 = Vector3.subtract(newPoint, mContactPoints[0].getLocalPointOnFirstBody());
            final Vector3 vector2 = Vector3.subtract(mContactPoints[3].getLocalPointOnFirstBody(), mContactPoints[1].getLocalPointOnFirstBody());
            final Vector3 crossProduct = vector1.cross(vector2);
            area013N = crossProduct.lengthSquare();
        } else {
            area013N = 2;
        }
        if (indexMaxPenetration != 3) {
            final Vector3 vector1 = Vector3.subtract(newPoint, mContactPoints[0].getLocalPointOnFirstBody());
            final Vector3 vector2 = Vector3.subtract(mContactPoints[2].getLocalPointOnFirstBody(), mContactPoints[1].getLocalPointOnFirstBody());
            final Vector3 crossProduct = vector1.cross(vector2);
            area012N = crossProduct.lengthSquare();
        } else {
            area012N = 3;
        }
        return getMaxArea(area123N, area023N, area013N, area012N);
    }

    // Returns the index of maximum area.
    private int getMaxArea(float area123N, float area023N, float area013N, float area012N) {
        if (area123N < area023N) {
            if (area023N < area013N) {
                if (area013N < area012N) {
                    return 3;
                } else {
                    return 2;
                }
            } else {
                if (area023N < area012N) {
                    return 3;
                } else {
                    return 1;
                }
            }
        } else {
            if (area123N < area013N) {
                if (area013N < area012N) {
                    return 3;
                } else {
                    return 2;
                }
            } else {
                if (area123N < area012N) {
                    return 3;
                } else {
                    return 0;
                }
            }
        }
    }

    /**
     * This structure represents a single element of a linked list of contact manifolds.
     */
    public static class ContactManifoldListElement {
        private ContactManifold contactManifold;
        private ContactManifoldListElement next;

        /**
         * Constructs a new contact manifold list element from the initial contact manifold and next list element.
         *
         * @param initContactManifold The contact manifold
         * @param initNext The next element
         */
        public ContactManifoldListElement(ContactManifold initContactManifold, ContactManifoldListElement initNext) {
            contactManifold = initContactManifold;
            next = initNext;
        }

        /**
         * Returns the contact manifold in this list element.
         *
         * @return The contact manifold
         */
        public ContactManifold getContactManifold() {
            return contactManifold;
        }

        /**
         * Sets the contact manifold in this list element.
         *
         * @param contactManifold The contact manifold
         */
        public void setContactManifold(ContactManifold contactManifold) {
            this.contactManifold = contactManifold;
        }

        /**
         * Returns the next element in the list.
         *
         * @return The next element
         */
        public ContactManifoldListElement getNext() {
            return next;
        }

        /**
         * Sets the next element in the list.
         *
         * @param next The next element
         */
        public void setNext(ContactManifoldListElement next) {
            this.next = next;
        }
    }
}
