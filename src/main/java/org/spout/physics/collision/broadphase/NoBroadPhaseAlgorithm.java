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
package org.spout.physics.collision.broadphase;

import java.util.HashSet;
import java.util.Set;

import org.spout.physics.body.CollisionBody;
import org.spout.physics.collision.CollisionDetection;
import org.spout.physics.collision.shape.AABB;

/**
 * This class implements a broad-phase algorithm that does nothing. It should be use if we don't
 * want to perform a broad-phase for the collision detection.
 */
public class NoBroadPhaseAlgorithm extends BroadPhaseAlgorithm {
	private final Set<CollisionBody> mBodies = new HashSet<CollisionBody>();

	/**
	 * Constructs a new no broad-phase algorithm from the collision detection it's associated to.
	 *
	 * @param collisionDetection The collision detection
	 */
	public NoBroadPhaseAlgorithm(CollisionDetection collisionDetection) {
		super(collisionDetection);
	}

	@Override
	public void addObject(CollisionBody body, AABB aabb) {
		for (CollisionBody collisionBody : mBodies) {
			if (body.isMotionEnabled() || collisionBody.isMotionEnabled()) {
				mPairManager.addPair(collisionBody, body);
			}
		}
		mBodies.add(body);
	}

	@Override
	public void removeObject(CollisionBody body) {
		for (CollisionBody collisionBody : mBodies) {
			if (collisionBody.getID() != body.getID()) {
				mPairManager.removePair(collisionBody.getID(), body.getID());
			}
		}
		mBodies.remove(body);
	}

	@Override
	public void updateObject(CollisionBody body, AABB aabb) {
	}
}
