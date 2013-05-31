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
package org.spout.jreactphysics3d.collision;

import org.spout.jreactphysics3d.Utilities.IntPair;
import org.spout.jreactphysics3d.body.CollisionBody;
import org.spout.jreactphysics3d.mathematics.Vector3;

/**
 * Represents a pair of bodies during the broad-phase collision detection.
 */
public class BroadPhasePair {
	public CollisionBody body1;
	public CollisionBody body2;
	public final Vector3 previousSeparatingAxis;

	/**
	 * Constructs a new broad phase pair from the first and the second body.
	 *
	 * @param body1 The first body
	 * @param body2 The second body
	 */
	public BroadPhasePair(CollisionBody body1, CollisionBody body2) {
		this.body1 = body1;
		this.body2 = body2;
		previousSeparatingAxis = new Vector3(1, 1, 1);
	}

	/**
	 * Gets the pair of bodies indexes (IDs) as a pair of integers.
	 *
	 * @return The pair of body indexes (IDs)
	 */
	public IntPair getBodiesIndexPair() {
		IntPair indexPair = body1.getID() < body2.getID() ?
				new IntPair(body1.getID(), body2.getID()) :
				new IntPair(body2.getID(), body1.getID());
		if (indexPair.getFirst() == indexPair.getSecond()) {
			throw new IllegalStateException("First int of the pair cannot be equal to the second int of the pair");
		}
		return indexPair;
	}

	/**
	 * Returns true this broad phase pair is smaller than the other broad phase pair, false if not.
	 *
	 * @param broadPhasePair2 The broad phase pair to compare with
	 * @return True if this broad phase pair is smaller, false if not
	 */
	public boolean isSmallerThan(BroadPhasePair broadPhasePair2) {
		return body1.isSmallerThan(broadPhasePair2.body1) ? true : (body2.isSmallerThan(broadPhasePair2.body2));
	}

	/**
	 * Returns true this broad phase pair is greater than the other broad phase pair, false if not.
	 *
	 * @param broadPhasePair2 The broad phase pair to compare with
	 * @return True if this broad phase pair is greater, false if not
	 */
	public boolean isGreaterThan(BroadPhasePair broadPhasePair2) {
		return body1.isGreaterThan(broadPhasePair2.body1) ? true : (body2.isGreaterThan(broadPhasePair2.body2));
	}

	/**
	 * Returns true this broad phase pair is equal than the other broad phase pair, false if not.
	 *
	 * @param broadPhasePair2 The broad phase pair to compare with
	 * @return True if this broad phase pair equal, false if not
	 */
	public boolean isEqualTo(BroadPhasePair broadPhasePair2) {
		return body1.isEqualTo(broadPhasePair2.body1) && body2.isEqualTo(broadPhasePair2.body2);
	}

	/**
	 * Returns true this broad phase pair is not equal than the other broad phase pair, false if not.
	 *
	 * @param broadPhasePair2 The broad phase pair to compare with
	 * @return True if this broad phase pair is not equal, false if not
	 */
	public boolean isNotEqualTo(BroadPhasePair broadPhasePair2) {
		return body1.isNotEqualTo(broadPhasePair2.body1) || body2.isNotEqualTo(broadPhasePair2.body2);
	}
}
