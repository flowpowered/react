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
package org.spout.jreactphysics3d.engine;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.Map;
import java.util.Set;
import java.util.Vector;

import org.spout.jreactphysics3d.Utilities.IntPair;
import org.spout.jreactphysics3d.body.CollisionBody;
import org.spout.jreactphysics3d.collision.BroadPhasePair;
import org.spout.jreactphysics3d.collision.CollisionDetection;
import org.spout.jreactphysics3d.collision.ContactInfo;
import org.spout.jreactphysics3d.collision.shape.CollisionShape;
import org.spout.jreactphysics3d.mathematics.Transform;

/**
 * Represents a world where it is possible to move bodies by hand and to test collisions between
 * them. In this kind of world the body movement is not computed using the laws of physics.
 */
public abstract class CollisionWorld {
	protected final CollisionDetection mCollisionDetection;
	protected final Set<CollisionBody> mBodies = new HashSet<CollisionBody>();
	protected final Map<IntPair, OverlappingPair> mOverlappingPairs = new HashMap<IntPair, OverlappingPair>();
	private int mCurrentBodyID;
	protected final Vector<Integer> mFreeBodiesIDs = new Vector<Integer>();

	/**
	 * Constructs a new empty collision world.
	 */
	public CollisionWorld() {
		mCurrentBodyID = 0;
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
	 * Gets an iterator to the beginning of the bodies of the physics world.
	 *
	 * @return The {@link java.util.Iterator} of {@link org.spout.jreactphysics3d.body.CollisionBody}
	 */
	public Iterator<CollisionBody> getBodiesBeginIterator() {
		return mBodies.iterator();
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
	 * Returns the next available body ID.
	 *
	 * @return An available boy ID
	 */
	protected int computeNextAvailableBodyID() {
		final int bodyID;
		if (!mFreeBodiesIDs.isEmpty()) {
			bodyID = mFreeBodiesIDs.lastElement();
			mFreeBodiesIDs.removeElementAt(mFreeBodiesIDs.size() - 1);
		} else {
			bodyID = mCurrentBodyID;
			mCurrentBodyID++;
		}
		return bodyID;
	}
}
