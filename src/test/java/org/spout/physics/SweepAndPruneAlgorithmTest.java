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
package org.spout.physics;

import java.util.HashSet;
import java.util.Set;

import org.junit.Assert;
import org.junit.Test;

import org.spout.physics.body.CollisionBody;
import org.spout.physics.collision.broadphase.SweepAndPruneAlgorithm;
import org.spout.physics.collision.shape.AABB;

public class SweepAndPruneAlgorithmTest {
	private static final int BODY_COUNT = 100;
	private static int ID = 0;

	@Test
	public void test() {
		final SweepAndPruneAlgorithm sweepAndPrune = new SweepAndPruneAlgorithm(Dummies.newCollisionDetection());
		final Set<CollisionBody> bodies = new HashSet<CollisionBody>();
		for (int i = 0; i < BODY_COUNT / 10; i++) {
			final CollisionBody body = Dummies.newCollisionBody(ID++);
			bodies.add(body);
			sweepAndPrune.addObject(body, Dummies.newAABB());
		}
		Assert.assertEquals(BODY_COUNT / 10, sweepAndPrune.getNbObjects());
		Assert.assertTrue(countNotNull(sweepAndPrune.getOverlappingPairs()) >= 0);
		for (int i = 0; i < BODY_COUNT / 2; i++) {
			final AABB aabb = Dummies.newAABB();
			final CollisionBody body0 = Dummies.newCollisionBody(ID++);
			bodies.add(body0);
			sweepAndPrune.addObject(body0, aabb);
			final CollisionBody body1 = Dummies.newCollisionBody(ID++);
			bodies.add(body1);
			sweepAndPrune.addObject(body1, Dummies.newIntersectingAABB(aabb));
		}
		Assert.assertEquals(BODY_COUNT + BODY_COUNT / 10, sweepAndPrune.getNbObjects());
		Assert.assertTrue(countNotNull(sweepAndPrune.getOverlappingPairs()) >= BODY_COUNT / 2);
		for (CollisionBody body : bodies) {
			sweepAndPrune.removeObject(body);
		}
		Assert.assertEquals(0, sweepAndPrune.getNbObjects());
	}

	private static int countNotNull(Object[] array) {
		if (array == null) {
			return 0;
		}
		int count = 0;
		for (int i = 0; i < array.length; i++) {
			if (array[i] != null) {
				count++;
			}
		}
		return count;
	}
}
