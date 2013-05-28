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
package org.spout.jreactphysics3d;

import org.junit.Assert;
import org.junit.Test;

import org.spout.jreactphysics3d.body.CollisionBody;
import org.spout.jreactphysics3d.collision.broadphase.PairManager;
import org.spout.jreactphysics3d.collision.broadphase.PairManager.BodyPair;
import org.spout.jreactphysics3d.collision.shape.BoxShape;
import org.spout.jreactphysics3d.mathematics.Transform;
import org.spout.jreactphysics3d.mathematics.Vector3;

public class PairManagerTest {
	@Test
	public void test() {
		// TODO: when CollisionDetection gets implements, this test will fail because null is passed.
		final PairManager manager = new PairManager(null);
		Assert.assertEquals(manager.getNbOverlappingPairs(), 0);
		for (int i = 0; i < 20; i += 2) {
			final BodyPair pair = manager.addPair(newBody(i), newBody(i + 1));
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

	private static CollisionBody newBody(int id) {
		return new CollisionBody(Transform.identity(), new BoxShape(new Vector3()), id);
	}
}