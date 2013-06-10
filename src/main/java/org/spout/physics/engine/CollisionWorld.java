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

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import gnu.trove.list.TIntList;
import gnu.trove.list.array.TIntArrayList;

import org.spout.physics.Utilities.IntPair;
import org.spout.physics.body.CollisionBody;
import org.spout.physics.collision.BroadPhasePair;
import org.spout.physics.collision.CollisionDetection;
import org.spout.physics.collision.CollisionListener;
import org.spout.physics.collision.ContactInfo;
import org.spout.physics.collision.RayCaster;
import org.spout.physics.collision.shape.CollisionShape;
import org.spout.physics.math.Transform;
import org.spout.physics.math.Vector3;

/**
 * Represents a world where it is possible to move bodies by hand and to test collisions between
 * them. In this kind of world the body movement is not computed using the laws of physics.
 */
public abstract class CollisionWorld {
	protected final CollisionDetection mCollisionDetection;
	protected final Set<CollisionBody> mBodies = new HashSet<CollisionBody>();
	protected final Map<IntPair, OverlappingPair> mOverlappingPairs = new HashMap<IntPair, OverlappingPair>();
	protected int mCurrentBodyID = 0;
	protected final TIntList mFreeBodiesIDs = new TIntArrayList();

	/**
	 * Constructs a new empty collision world.
	 */
	protected CollisionWorld() {
		mCollisionDetection = new CollisionDetection(this);
	}

	/**
	 * Notifies the world about a new broad-phase overlapping pair.
	 *
	 * @param addedPair The pair that was added
	 */
	public abstract void notifyAddedOverlappingPair(BroadPhasePair addedPair);

	/**
	 * Notifies the world about a removed broad-phase overlapping pair.
	 *
	 * @param removedPair The pair that was removed
	 */
	public abstract void notifyRemovedOverlappingPair(BroadPhasePair removedPair);

	/**
	 * Notifies the world about a new narrow-phase contact.
	 *
	 * @param pair The pair of bodies in contact
	 * @param contactInfo The information for the contact
	 */
	public abstract void notifyNewContact(BroadPhasePair pair, ContactInfo contactInfo);

	/**
	 * Updates the overlapping pair.
	 *
	 * @param pair The pair to update
	 */
	public abstract void updateOverlappingPair(BroadPhasePair pair);

	/**
	 * Gets the collision listener for the collision detection, or null if none has been set.
	 *
	 * @return The collision listener, or null is absent
	 */
	public CollisionListener getCollisionListener() {
		return mCollisionDetection.getCollisionListener();
	}

	/**
	 * Sets the collision listener for the collision detection.
	 *
	 * @param listener The listener to use
	 */
	public void setCollisionListener(CollisionListener listener) {
		mCollisionDetection.setCollisionListener(listener);
	}

	/**
	 * Gets the set of the bodies of the physics world.
	 *
	 * @return The {@link java.util.Set} of {@link org.spout.physics.body.CollisionBody}
	 */
	public Set<CollisionBody> getBodies() {
		return mBodies;
	}

	/**
	 * Creates a collision body and adds it to the world.
	 *
	 * @param transform The transform (position and orientation) of the body
	 * @param collisionShape The shape of the body
	 * @return The new collision body
	 */
	public CollisionBody createCollisionBody(Transform transform, CollisionShape collisionShape) {
		final int bodyID = computeNextAvailableBodyID();
		if (bodyID >= Integer.MAX_VALUE) {
			throw new IllegalStateException("body id must be smaller than the integer max value");
		}
		final CollisionBody collisionBody = new CollisionBody(transform, collisionShape, bodyID);
		mBodies.add(collisionBody);
		mCollisionDetection.addBody(collisionBody);
		return collisionBody;
	}

	/**
	 * Destroys a collision body.
	 *
	 * @param collisionBody The collision body to destroy
	 */
	public void destroyCollisionBody(CollisionBody collisionBody) {
		mCollisionDetection.removeBody(collisionBody);
		mFreeBodiesIDs.add(collisionBody.getID());
		mBodies.remove(collisionBody);
	}

	/**
	 * Finds the closest of the bodies intersecting with the ray to the ray start. The ray is defined
	 * by a starting point and a direction.
	 *
	 * @param rayStart The ray starting point
	 * @param rayDir The ray direction
	 * @return The closest body to the ray start
	 */
	public CollisionBody findClosestIntersectingBody(Vector3 rayStart, Vector3 rayDir) {
		return RayCaster.findClosestIntersectingBody(rayStart, rayDir, mBodies);
	}

	/**
	 * Finds the furthest of the bodies intersecting with the ray from the ray start. The ray is
	 * defined by a starting point and a direction.
	 *
	 * @param rayStart The ray starting point
	 * @param rayDir The ray direction
	 * @return The furthest body from the ray start
	 */
	public CollisionBody findFurthestIntersectingBody(Vector3 rayStart, Vector3 rayDir) {
		return RayCaster.findFurthestIntersectingBody(rayStart, rayDir, mBodies);
	}

	/**
	 * Finds all of the bodies intersecting with the ray. The ray is defined by a starting point and a
	 * direction.
	 *
	 * @param rayStart The ray starting point
	 * @param rayDir The ray direction
	 * @return All of the intersection bodies, in no particular order
	 */
	public Set<CollisionBody> findIntersectingBodies(Vector3 rayStart, Vector3 rayDir) {
		return RayCaster.findIntersectingBodies(rayStart, rayDir, mBodies);
	}

	/**
	 * Returns the next available body ID.
	 *
	 * @return An available boy ID
	 */
	protected int computeNextAvailableBodyID() {
		final int bodyID;
		if (!mFreeBodiesIDs.isEmpty()) {
			bodyID = mFreeBodiesIDs.get(mFreeBodiesIDs.size() - 1);
			mFreeBodiesIDs.remove(mFreeBodiesIDs.size() - 1);
		} else {
			bodyID = mCurrentBodyID;
			mCurrentBodyID++;
		}
		return bodyID;
	}
}
