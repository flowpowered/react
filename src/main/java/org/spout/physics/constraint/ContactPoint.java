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

import org.spout.math.vector.Vector3;
import org.spout.physics.body.RigidBody;
import org.spout.physics.collision.ContactInfo;
import org.spout.physics.math.Transform;

/**
 * Represents a collision contact point between two bodies in the physics engine. The ContactPoint
 * class inherits from the Constraint class.
 */
public class ContactPoint extends Constraint {
	private Vector3 mNormal;
	private float mPenetrationDepth;
	private Vector3 mLocalPointOnBody1;
	private Vector3 mLocalPointOnBody2;
	private Vector3 mWorldPointOnBody1;
	private Vector3 mWorldPointOnBody2;
	private boolean mIsRestingContact = false;
	private Vector3 mFrictionVector1;
	private Vector3 mFrictionVector2;

	/**
	 * Constructs a new contact point from the first and second body, and the contact info.
	 *
	 * @param body1 The first body
	 * @param body2 The second body
	 * @param contactInfo The contact info
	 */
	public ContactPoint(RigidBody body1, RigidBody body2, ContactInfo contactInfo) {
		super(body1, body2, 3, true, ConstraintType.CONTACT);
		if (contactInfo.getPenetrationDepth() <= 0) {
			throw new IllegalArgumentException("penetraionDepth must be greater than zero");
		}
		mNormal = contactInfo.getNormal();
		mPenetrationDepth = contactInfo.getPenetrationDepth();
		mLocalPointOnBody1 = contactInfo.getFirstLocalPoint();
		mLocalPointOnBody2 = contactInfo.getSecondLocalPoint();
		mWorldPointOnBody1 = Transform.multiply(body1.getTransform(), contactInfo.getFirstLocalPoint());
		mWorldPointOnBody2 = Transform.multiply(body2.getTransform(), contactInfo.getSecondLocalPoint());
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
		mWorldPointOnBody1 = worldPoint;
	}

	/**
	 * Sets the contact point in world space on the second body.
	 *
	 * @param worldPoint The contact point in world space
	 */
	public void setWorldPointOnSecondBody(Vector3 worldPoint) {
		mWorldPointOnBody2 = worldPoint;
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
		mFrictionVector1 = firstFrictionVector;
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
		mFrictionVector2 = secondFrictionVector;
	}
}
