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
package org.spout.physics.engine;

import org.spout.physics.body.RigidBody;
import org.spout.physics.constraint.Joint;

/**
 * An island represent an isolated group of awake bodies that are connected with each other by some constraints (contacts or joints).
 */
public class Island {
    private RigidBody[] mBodies;
    private ContactManifold[] mContactManifolds;
    private Joint[] mJoints;
    private int mNbBodies;
    private int mNbContactManifolds;
    private int mNbJoints;

    /**
     * Constructs a new island from the maximum number of bodies, the maximum number of contact manifolds and the maximum number of joints.
     *
     * @param nbMaxBodies The maximum number of bodies
     * @param nbMaxContactManifolds The maximum number of contact manifolds
     * @param nbMaxJoints The maximum number of joints
     */
    public Island(int nbMaxBodies, int nbMaxContactManifolds, int nbMaxJoints) {
        mNbBodies = 0;
        mNbContactManifolds = 0;
        mNbJoints = 0;
        mBodies = new RigidBody[nbMaxBodies];
        mContactManifolds = new ContactManifold[nbMaxContactManifolds];
        mJoints = new Joint[nbMaxJoints];
    }

    /**
     * Adds a body into the island
     *
     * @param body The body
     */
    public void addBody(RigidBody body) {
        if (body.isSleeping()) {
            throw new IllegalArgumentException("Body to add is sleeping");
        }
        mBodies[mNbBodies] = body;
        mNbBodies++;
    }

    /**
     * Adds a contact manifold into the island
     *
     * @param contactManifold The contact manifold
     */
    public void addContactManifold(ContactManifold contactManifold) {
        mContactManifolds[mNbContactManifolds] = contactManifold;
        mNbContactManifolds++;
    }

    /**
     * Adds a joint into the island.
     *
     * @param joint The joint
     */
    public void addJoint(Joint joint) {
        mJoints[mNbJoints] = joint;
        mNbJoints++;
    }

    /**
     * Returns the number of bodies in the island.
     *
     * @return The number of bodies
     */
    public int getNbBodies() {
        return mNbBodies;
    }

    /**
     * Returns the number of contact manifolds in the island.
     *
     * @return The number of contact manifolds
     */
    public int getNbContactManifolds() {
        return mNbContactManifolds;
    }

    /**
     * Returns the number of joints in the island.
     *
     * @return The number of joints
     */
    public int getNbJoints() {
        return mNbJoints;
    }

    /**
     * Returns the array of bodies.
     *
     * @return The array of bodies
     */
    public RigidBody[] getBodies() {
        return mBodies;
    }

    /**
     * Returns the array of contact manifolds.
     *
     * @return The array of contact manifold
     */
    public ContactManifold[] getContactManifolds() {
        return mContactManifolds;
    }

    /**
     * Returns the array of joints.
     *
     * @return The array of joints
     */
    public Joint[] getJoints() {
        return mJoints;
    }
}
