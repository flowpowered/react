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
package org.spout.physics.collision.dynamicphase;

import java.util.HashSet;
import java.util.Set;

import org.spout.physics.ReactDefaults;
import org.spout.physics.body.ImmobileRigidBody;
import org.spout.physics.body.MobileRigidBody;
import org.spout.physics.collision.shape.AABB;
import org.spout.physics.engine.DynamicDynamicsWorld;
import org.spout.physics.math.Vector3;

/**
 * A phase of the physics tick where bodies is dynamically added via the {@link org.spout.physics.engine.DynamicDynamicsWorld}'s {@link org.spout.physics.engine.dynamic.DynamicWorldInfo}.
 */
public class DynamicPhase {
	private final DynamicDynamicsWorld dynamicWorld;

	public DynamicPhase(final DynamicDynamicsWorld dynamicWorld) {
		this.dynamicWorld = dynamicWorld;
	}

	/**
	 * Sweeps for {@link ImmobileRigidBody}s around the body provided.
	 * <p>
	 *     The algorithm will ask for all bodies within the bounds of the
	 *     bodies' AABB bounds + a scaling factor.
	 * </p>
	 * @param body mobile body to scan around
	 * @return A set of all bodies in range
	 */
	public Set<ImmobileRigidBody> getBodiesInRange(final MobileRigidBody body) {
		final AABB aabb = body.getAABB();
		//Grab object coords
		final Vector3 max = Vector3.subtract(aabb.getMax(), aabb.getCenter());
		final Vector3 min = Vector3.subtract(aabb.getMin(), aabb.getCenter());
		//Scale coords
		max.multiply(ReactDefaults.AABB_SCALAR);
		min.multiply(ReactDefaults.AABB_SCALAR);
		//Grab world coords
		max.add(aabb.getCenter());
		min.add(aabb.getCenter());

		final int startx = (int) Math.floor(min.getX());
		final int starty = (int) Math.floor(min.getY());
		final int startz = (int) Math.floor(min.getZ());

		final int endx = (int) Math.ceil(max.getX());
		final int endy = (int) Math.ceil(max.getY());
		final int endz = (int) Math.ceil(max.getZ());

		final Set<ImmobileRigidBody> foundBodies = new HashSet<ImmobileRigidBody>();

		for (int xx = startx; xx <= endx; xx++) {
			for (int yy = starty; yy <= endy; yy++) {
				for (int zz = startz; zz <= endz; zz++) {
					final ImmobileRigidBody immobile = dynamicWorld.getDynamicInfo().getBody(xx, yy, zz);
					if (immobile == null) {
						continue;
					}
					foundBodies.add(immobile);
				}
			}
		}
		return foundBodies;
	}
}
