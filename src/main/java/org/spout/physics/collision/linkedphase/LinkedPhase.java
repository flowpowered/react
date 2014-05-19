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
package org.spout.physics.collision.linkedphase;

import java.util.HashSet;
import java.util.Set;

import org.spout.physics.ReactDefaults;
import org.spout.physics.body.RigidBody;
import org.spout.physics.collision.shape.AABB;
import org.spout.physics.engine.linked.LinkedDynamicsWorld;
import org.spout.physics.math.Vector3;

/**
 * A phase of the physics tick where bodies are added via the {@link org.spout.physics.engine.linked.LinkedDynamicsWorld}'s {@link org.spout.physics.engine.linked.LinkedWorldInfo}.
 */
public class LinkedPhase {
    private final LinkedDynamicsWorld linkedWorld;

    /**
     * Constructs a new linked phase from the linked dynamics world.
     *
     * @param linkedWorld The linked dynamics world this phase is associate to
     */
    public LinkedPhase(LinkedDynamicsWorld linkedWorld) {
        this.linkedWorld = linkedWorld;
    }

    /**
     * Sweeps for {@link RigidBody}s around the body provided. <p> The algorithm will ask for all bodies within the bounds of the bodies' AABB scaled by a {@link
     * ReactDefaults#LINKED_PHASE_AABB_SCALING}. </p>
     *
     * @param body The mobile body to scan around
     * @return A set of all bodies in range
     */
    public Set<RigidBody> getBodiesInRange(RigidBody body) {
        final AABB aabb = body.getAABB();
        // To object coords
        final Vector3 max = Vector3.subtract(aabb.getMax(), aabb.getCenter());
        final Vector3 min = Vector3.subtract(aabb.getMin(), aabb.getCenter());
        // Scale the coords
        max.multiply(ReactDefaults.LINKED_PHASE_AABB_SCALING);
        min.multiply(ReactDefaults.LINKED_PHASE_AABB_SCALING);
        // Back to world coords
        max.add(aabb.getCenter());
        min.add(aabb.getCenter());
        final int startX = (int) Math.floor(min.getX());
        final int startY = (int) Math.floor(min.getY());
        final int startZ = (int) Math.floor(min.getZ());
        final int endX = (int) Math.ceil(max.getX());
        final int endY = (int) Math.ceil(max.getY());
        final int endZ = (int) Math.ceil(max.getZ());
        final Set<RigidBody> foundBodies = new HashSet<>();
        for (int xx = startX; xx <= endX; xx++) {
            for (int yy = startY; yy <= endY; yy++) {
                for (int zz = startZ; zz <= endZ; zz++) {
                    final RigidBody immobile = linkedWorld.getLinkedInfo().getBody(xx, yy, zz);
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
