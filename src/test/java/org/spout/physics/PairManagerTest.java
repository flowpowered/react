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

import org.junit.Assert;
import org.junit.Test;

import org.spout.physics.collision.broadphase.PairManager;
import org.spout.physics.collision.broadphase.PairManager.BodyPair;

public class PairManagerTest {
	@Test
	public void test() {
		final PairManager manager = new PairManager(Dummies.newCollisionDetection());
		Assert.assertEquals(manager.getNbOverlappingPairs(), 0);
		for (int i = 0; i < 20; i += 2) {
			final BodyPair pair = manager.addPair(Dummies.newCollisionBody(i), Dummies.newCollisionBody(i + 1));
			Assert.assertNotNull(pair);
			Assert.assertEquals(pair.getFirstBody().getID(), i);
			Assert.assertEquals(pair.getSecondBody().getID(), i + 1);
		}
		Assert.assertEquals(manager.getNbOverlappingPairs(), 10);
		for (int i = 0; i < 20; i += 2) {
			final BodyPair pair = manager.findPair(i, i + 1);
			Assert.assertNotNull(pair);
			Assert.assertEquals(pair.getFirstBody().getID(), i);
			Assert.assertEquals(pair.getSecondBody().getID(), i + 1);
		}
		Assert.assertEquals(manager.getNbOverlappingPairs(), 10);
		for (int i = 0; i < 20; i += 2) {
			Assert.assertTrue(manager.removePair(i, i + 1));
		}
		Assert.assertEquals(manager.getNbOverlappingPairs(), 0);
	}
}