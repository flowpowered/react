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

import java.util.HashSet;
import java.util.Random;
import java.util.Set;

import org.junit.Assert;
import org.junit.Test;

import org.spout.jreactphysics3d.body.CollisionBody;
import org.spout.jreactphysics3d.collision.broadphase.SweepAndPruneAlgorithm;
import org.spout.jreactphysics3d.collision.shape.AABB;
import org.spout.jreactphysics3d.collision.shape.BoxShape;
import org.spout.jreactphysics3d.mathematics.Transform;
import org.spout.jreactphysics3d.mathematics.Vector3;

public class SweepAndPruneAlgorithmTest {
	private static final int BODY_COUNT = 100;
	private static final Random RANDOM = new Random();
	private static int ID = 0;

	@Test
	public void test() {
		// TODO: when CollisionDetection gets implements, this test will fail because null is passed.
		final SweepAndPruneAlgorithm sweepAndPrune = new SweepAndPruneAlgorithm(null);
		final Set<CollisionBody> bodies = new HashSet<CollisionBody>();
		for (int i = 0; i < BODY_COUNT / 10; i++) {
			final CollisionBody body = newCollisionBody();
			bodies.add(body);
			sweepAndPrune.addObject(body, newAABB());
		}
		Assert.assertEquals(BODY_COUNT / 10, sweepAndPrune.getNbObjects());
		Assert.assertTrue(countNotNull(sweepAndPrune.beginOverlappingPairsPointer()) >= 0);
		for (int i = 0; i < BODY_COUNT / 2; i++) {
			final AABB aabb = newAABB();
			final CollisionBody body0 = newCollisionBody();
			bodies.add(body0);
			sweepAndPrune.addObject(body0, aabb);
			final CollisionBody body1 = newCollisionBody();
			bodies.add(body1);
			sweepAndPrune.addObject(body1, newIntersectingAABB(aabb));
		}
		Assert.assertEquals(BODY_COUNT + BODY_COUNT / 10, sweepAndPrune.getNbObjects());
		Assert.assertTrue(countNotNull(sweepAndPrune.beginOverlappingPairsPointer()) >= BODY_COUNT / 2);
		for (CollisionBody body : bodies) {
			sweepAndPrune.removeObject(body);
		}
		Assert.assertEquals(0, sweepAndPrune.getNbObjects());
	}

	private static CollisionBody newCollisionBody() {
		return new CollisionBody(Transform.identity(), new BoxShape(new Vector3()), ID++);
	}

	private static AABB newAABB() {
		final Vector3 min = new Vector3(RANDOM.nextInt(21) - 10, RANDOM.nextInt(21) - 10, RANDOM.nextInt(21) - 10);
		return new AABB(min, Vector3.add(min, new Vector3(RANDOM.nextInt(3) + 2, RANDOM.nextInt(3) + 2, RANDOM.nextInt(3) + 2)));
	}

	private static AABB newIntersectingAABB(AABB with) {
		final Vector3 wMin = with.getMin();
		final Vector3 wSize = Vector3.subtract(with.getMax(), wMin);
		final int iSizeX = RANDOM.nextInt((int) wSize.getX() + 1);
		final int iSizeY = RANDOM.nextInt((int) wSize.getY() + 1);
		final int iSizeZ = RANDOM.nextInt((int) wSize.getZ() + 1);
		final int eSizeX = RANDOM.nextInt(3) + 2;
		final int eSizeY = RANDOM.nextInt(3) + 2;
		final int eSizeZ = RANDOM.nextInt(3) + 2;
		final Vector3 min = Vector3.subtract(wMin, new Vector3(eSizeX, eSizeY, eSizeZ));
		final Vector3 max = Vector3.add(wMin, new Vector3(iSizeX, iSizeY, iSizeZ));
		return new AABB(min, max);
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
