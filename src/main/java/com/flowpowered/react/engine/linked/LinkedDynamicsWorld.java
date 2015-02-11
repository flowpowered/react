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
package com.flowpowered.react.engine.linked;

import java.util.Collection;
import java.util.HashSet;
import java.util.Set;

import com.flowpowered.react.body.RigidBody;
import com.flowpowered.react.engine.DynamicsWorld;
import com.flowpowered.react.math.Vector3;

/**
 * Represents a dynamics world linked to another world. Information is shared between them through an instance of {@link LinkedWorldInfo}.
 */
public class LinkedDynamicsWorld extends DynamicsWorld {
    private final LinkedWorldInfo info;
    private final Set<RigidBody> linkedBodies = new HashSet<>();

    /**
     * Constructs a new linked dynamics world from the gravity, the time step and the linked world information.
     *
     * @param gravity The gravity
     * @param timeStep The simulation time step
     * @param info The linked world information
     */
    public LinkedDynamicsWorld(Vector3 gravity, float timeStep, LinkedWorldInfo info) {
        super(gravity, timeStep);
        this.info = info;
    }

    /**
     * Constructs a new linked dynamics world from the gravity, the default time step and the linked world information.
     *
     * @param gravity The gravity
     * @param info The linked world information
     */
    public LinkedDynamicsWorld(Vector3 gravity, LinkedWorldInfo info) {
        super(gravity);
        this.info = info;
    }

    @Override
    public void tick() {
        super.tick();
        clearLinkedBodies();
    }

    /**
     * Returns the {@link com.flowpowered.react.engine.linked.LinkedWorldInfo} of this world.
     *
     * @return The linked info
     */
    public LinkedWorldInfo getLinkedInfo() {
        return info;
    }

    /**
     * Adds {@link RigidBody}s to this world. These will be cleared at the end of the physics tick.
     *
     * @param bodies The bodies to add
     */
    public void addLinkedBodies(Collection<RigidBody> bodies) {
        linkedBodies.addAll(bodies);
        for (RigidBody body : bodies) {
            addRigidBodyIgnoreTick(body);
        }
    }

    // Clears all bodies tracked in the world.
    private void clearLinkedBodies() {
        for (RigidBody linkedBody : linkedBodies) {
            // We're in a safe part of the tick so we can clear them immediately
            destroyRigidBodyImmediately(linkedBody);
        }
        linkedBodies.clear();
    }
}
