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
package org.spout.physics.engine;

import org.spout.physics.body.CollisionBody;
import org.spout.physics.constraint.ContactPoint;
import org.spout.physics.math.Vector3;

/**
 * Represents a pair of two bodies that are overlapping during the broad-phase collision detection.
 * It is created when the two bodies start to overlap and is destroyed when they do not overlap
 * anymore. This class contains a contact manifold that stores all the contact points between the
 * two bodies.
 */
public class OverlappingPair {
	private final CollisionBody mBody1;
	private final CollisionBody mBody2;
	private final ContactManifold mContactManifold = new ContactManifold();
	private final Vector3 mCachedSeparatingAxis = new Vector3(1, 1, 1);

	/**
	 * Constructs a new overlapping pair from the first and second body.
	 *
	 * @param body1 The first body
	 * @param body2 The second body
	 */
	public OverlappingPair(CollisionBody body1, CollisionBody body2) {
		mBody1 = body1;
		mBody2 = body2;
	}

	/**
	 * Gets the first body.
	 *
	 * @return The first body
	 */
	public CollisionBody getFirstBody() {
		return mBody1;
	}

	/**
	 * Gets the second body.
	 *
	 * @return The second body
	 */
	public CollisionBody getSecondBody() {
		return mBody2;
	}

	/**
	 * Adds a contact point to the contact manifold.
	 *
	 * @param contact The contact point to add
	 */
	public void addContact(ContactPoint contact) {
		mContactManifold.addContactPoint(contact);
	}

	/**
	 * Updates the contact manifold.
	 */
	public void update() {
		mContactManifold.update(mBody1.getTransform(), mBody2.getTransform());
	}

	/**
	 * Gets the cached separating axis.
	 *
	 * @return The cached separating axis
	 */
	public Vector3 getCachedSeparatingAxis() {
		return mCachedSeparatingAxis;
	}

	/**
	 * Sets the cached separating axis.
	 *
	 * @param axis The separating axis to set
	 */
	public void setCachedSeparatingAxis(Vector3 axis) {
		mCachedSeparatingAxis.set(axis);
	}

	/**
	 * Gets the number of contact points in the contact manifold.
	 *
	 * @return The number of contact points
	 */
	public int getNbContactPoints() {
		return mContactManifold.getNbContactPoints();
	}

	/**
	 * Gets the contact manifold.
	 *
	 * @return The contact manifold
	 */
	public ContactManifold getContactManifold() {
		return mContactManifold;
	}
}
