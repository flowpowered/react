/*
 * This file is part of React, licensed under the MIT License (MIT).
 *
 * Copyright (c) 2013 Flow Powered <https://flowpowered.com/>
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://danielchappuis.ch>
 * React is re-licensed with permission from ReactPhysics3D author.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
package com.flowpowered.react.collision.linkedphase;

import java.util.HashSet;
import java.util.Set;

import com.flowpowered.react.ReactDefaults;
import com.flowpowered.react.body.RigidBody;
import com.flowpowered.react.collision.shape.AABB;
import com.flowpowered.react.engine.linked.LinkedDynamicsWorld;
import com.flowpowered.react.math.Vector3;

/**
 * A phase of the physics tick where bodies are added via the {@link com.flowpowered.react.engine.linked.LinkedDynamicsWorld}'s {@link com.flowpowered.react.engine.linked.LinkedWorldInfo}.
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
