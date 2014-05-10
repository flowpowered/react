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
package org.spout.physics.engine.linked;

import java.util.Collection;
import java.util.HashSet;
import java.util.Set;

import org.spout.physics.body.ImmobileRigidBody;
import org.spout.physics.engine.DynamicsWorld;
import org.spout.physics.math.Vector3;

/**
 * Represents a dynamics world linked to another world. Information is shared between them through an instance of {@link LinkedWorldInfo}.
 */
public class LinkedDynamicsWorld extends DynamicsWorld {
    private final LinkedWorldInfo info;
    private final Set<ImmobileRigidBody> linkedBodies = new HashSet<>();

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
    public void update() {
        super.update();
        clearLinkedBodies();
    }

    /**
     * Returns the {@link org.spout.physics.engine.linked.LinkedWorldInfo} of this world.
     *
     * @return The linked info
     */
    public LinkedWorldInfo getLinkedInfo() {
        return info;
    }

    /**
     * Adds {@link ImmobileRigidBody}s to this world. These will be cleared at the end of the physics tick.
     *
     * @param bodies The bodies to add
     */
    public void addLinkedBodies(final Collection<ImmobileRigidBody> bodies) {
        linkedBodies.addAll(bodies);
        for (ImmobileRigidBody body : bodies) {
            addRigidBodyIgnoreTick(body);
        }
    }

    // Clears all bodies tracked in the world.
    private void clearLinkedBodies() {
        for (ImmobileRigidBody linkedBody : linkedBodies) {
            destroyRigidBody(linkedBody);
        }
        linkedBodies.clear();
    }
}
