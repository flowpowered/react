/*
 * This file is part of JReactPhysics3D.
 *
 * Copyright (c) 2013 Spout LLC <http://www.spout.org/>
 * JReactPhysics3D is licensed under the Spout License Version 1.
 *
 * JReactPhysics3D is free software: you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * In addition, 180 days after any changes are published, you can use the
 * software, incorporating those changes, under the terms of the MIT license,
 * as described in the Spout License Version 1.
 *
 * JReactPhysics3D is distributed in the hope that it will be useful, but WITHOUT ANY
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
package org.spout.jreactphysics3d.constraint;

import java.util.Arrays;
import java.util.Vector;

import org.spout.jreactphysics3d.body.RigidBody;
import org.spout.jreactphysics3d.collision.ContactInfo;
import org.spout.jreactphysics3d.mathematics.Transform;
import org.spout.jreactphysics3d.mathematics.Vector3;

/**
 * Represents a collision contact point between two bodies in the physics engine. The ContactPoint
 * class inherits from the Constraint class.
 */
public class ContactPoint extends Constraint {
	protected final Vector3 mNormal = new Vector3();
	protected float mPenetrationDepth;
	protected final Vector3 mLocalPointOnBody1 = new Vector3();
	protected final Vector3 mLocalPointOnBody2 = new Vector3();
	protected final Vector3 mWorldPointOnBody1 = new Vector3();
	protected final Vector3 mWorldPointOnBody2 = new Vector3();
	protected boolean mIsRestingContact;
	protected final Vector<Vector3> mFrictionVectors = new Vector<Vector3>(2);

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
		mNormal.set(contactInfo.getNormal());
		mPenetrationDepth = contactInfo.getPenetrationDepth();
		mLocalPointOnBody1.set(contactInfo.getLocalPoint1());
		mLocalPointOnBody2.set(contactInfo.getLocalPoint2());
		mWorldPointOnBody1.set(Transform.multiply(body1.getTransform(), contactInfo.getLocalPoint1()));
		mWorldPointOnBody2.set(Transform.multiply(body2.getTransform(), contactInfo.getLocalPoint2()));
		mIsRestingContact = false;
		mFrictionVectors.addAll(Arrays.asList(new Vector3(0, 0, 0), new Vector3(0, 0, 0)));
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
	public Vector3 getLocalPointOnBody1() {
		return mLocalPointOnBody1;
	}

	/**
	 * Gets the contact point on the second body.
	 *
	 * @return The contact point
	 */
	public Vector3 getLocalPointOnBody2() {
		return mLocalPointOnBody2;
	}

	/**
	 * Gets the contact point in world space on the first body.
	 *
	 * @return The contact point
	 */
	public Vector3 getWorldPointOnBody1() {
		return mWorldPointOnBody1;
	}

	/**
	 * Gets the contact point in world space on the second body.
	 *
	 * @return The contact point
	 */
	public Vector3 getWorldPointOnBody2() {
		return mWorldPointOnBody2;
	}

	/**
	 * Sets the contact point in world space on the first body.
	 *
	 * @param worldPoint The contact point in world space
	 */
	public void setWorldPointOnBody1(Vector3 worldPoint) {
		mWorldPointOnBody1.set(worldPoint);
	}

	/**
	 * Sets the contact point in world space on the second body.
	 *
	 * @param worldPoint The contact point in world space
	 */
	public void setWorldPointOnBody2(Vector3 worldPoint) {
		mWorldPointOnBody2.set(worldPoint);
	}

	/**
	 * Returns true if the contact is a resting contact, false if not.
	 *
	 * @return Whether or not the contact is a resting contact
	 */
	public boolean getIsRestingContact() {
		return mIsRestingContact;
	}

	/**
	 * Sets if the contact is a resting contact.
	 *
	 * @param isRestingContact True for a resting contact, false if otherwise.
	 */
	public void setIsRestingContact(boolean isRestingContact) {
		mIsRestingContact = isRestingContact;
	}

	/**
	 * Gets the first friction vector.
	 *
	 * @return The friction vector
	 */
	public Vector3 getFrictionVector1() {
		return mFrictionVectors.get(0);
	}

	/**
	 * Sets the first friction vector.
	 *
	 * @param frictionVector1 The friction vector to set
	 */
	public void setFrictionVector1(Vector3 frictionVector1) {
		mFrictionVectors.set(0, frictionVector1);
	}

	/**
	 * Gets the second friction vector.
	 *
	 * @return The friction vector
	 */
	public Vector3 getFrictionVector2() {
		return mFrictionVectors.get(1);
	}

	/**
	 * Sets the second friction vector.
	 *
	 * @param frictionVector2 The friction vector to set
	 */
	public void setFrictionVector2(Vector3 frictionVector2) {
		mFrictionVectors.set(1, frictionVector2);
	}
}
