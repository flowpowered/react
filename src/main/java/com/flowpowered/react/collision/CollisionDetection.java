/*
 * This file is part of Flow React, licensed under the MIT License (MIT).
 *
 * Copyright (c) 2013 Flow Powered <https://flowpowered.com/>
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
package com.flowpowered.react.collision;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;

import com.flowpowered.react.Utilities.IntPair;
import com.flowpowered.react.body.CollisionBody;
import com.flowpowered.react.body.RigidBody;
import com.flowpowered.react.collision.broadphase.BroadPhaseAlgorithm;
import com.flowpowered.react.collision.broadphase.PairManager.BodyPair;
import com.flowpowered.react.collision.broadphase.SweepAndPruneAlgorithm;
import com.flowpowered.react.collision.linkedphase.LinkedPhase;
import com.flowpowered.react.collision.narrowphase.GJK.GJKAlgorithm;
import com.flowpowered.react.collision.narrowphase.NarrowPhaseAlgorithm;
import com.flowpowered.react.collision.narrowphase.SphereVsSphereAlgorithm;
import com.flowpowered.react.collision.shape.CollisionShape;
import com.flowpowered.react.collision.shape.CollisionShape.CollisionShapeType;
import com.flowpowered.react.constraint.ContactPoint.ContactPointInfo;
import com.flowpowered.react.engine.CollisionWorld;
import com.flowpowered.react.engine.linked.LinkedDynamicsWorld;

/**
 * This class computes the collision detection algorithms. We first perform a broad-phase algorithm to know which pairs of bodies can collide and then we run a narrow-phase algorithm to compute the
 * collision contacts between the bodies.
 */
public class CollisionDetection {
    private final CollisionWorld mWorld;
    private final Map<IntPair, BroadPhasePair> mOverlappingPairs = new HashMap<>();
    private final BroadPhaseAlgorithm mBroadPhaseAlgorithm;
    private final GJKAlgorithm mNarrowPhaseGJKAlgorithm = new GJKAlgorithm();
    private final SphereVsSphereAlgorithm mNarrowPhaseSphereVsSphereAlgorithm = new SphereVsSphereAlgorithm();
    private final Set<IntPair> mNoCollisionPairs = new HashSet<>();
    private final LinkedPhase mLinkedPhase;

    /**
     * Constructs a new collision detection from the collision world.
     *
     * @param world The world
     */
    public CollisionDetection(CollisionWorld world) {
        mWorld = world;
        mBroadPhaseAlgorithm = new SweepAndPruneAlgorithm(this);
        if (world instanceof LinkedDynamicsWorld) {
            mLinkedPhase = new LinkedPhase((LinkedDynamicsWorld) mWorld);
        } else {
            mLinkedPhase = null;
        }
    }

    /**
     * Adds a body to the collision detection.
     *
     * @param body The body to add
     */
    public void addBody(CollisionBody body) {
        mBroadPhaseAlgorithm.addObject(body, body.getAABB());
    }

    /**
     * Removes a body from the collision detection.
     *
     * @param body The body to remove
     */
    public void removeBody(CollisionBody body) {
        mBroadPhaseAlgorithm.removeObject(body);
    }

    /**
     * Add a pair of bodies that cannot collide with each other.
     *
     * @param body1 The first body
     * @param body2 The second body
     */
    public void addNoCollisionPair(CollisionBody body1, CollisionBody body2) {
        mNoCollisionPairs.add(BroadPhasePair.computeBodiesIndexPair(body1, body2));
    }

    /**
     * Removes a pair of bodies that cannot collide with each other.
     *
     * @param body1 The first body
     * @param body2 The second body
     */
    public void removeNoCollisionPair(CollisionBody body1, CollisionBody body2) {
        mNoCollisionPairs.remove(BroadPhasePair.computeBodiesIndexPair(body1, body2));
    }

    /**
     * Computes the collision detection.
     */
    public void computeCollisionDetection() {
        if (mLinkedPhase != null) {
            computeLinkedPhase();
        }
        computeBroadPhase();
        computeNarrowPhase();
    }

    // Computes the linked-phase collision detection.
    private void computeLinkedPhase() {
        final List<CollisionBody> bodies = new ArrayList<>(mWorld.getBodies());
        final List<RigidBody> foundBodies = new ArrayList<>();
        for (CollisionBody body : bodies) {
            if (!(body instanceof RigidBody)) {
                continue;
            }
            for (RigidBody rigidBody : mLinkedPhase.getBodiesInRange((RigidBody) body)) {
                foundBodies.add(rigidBody);
            }
        }
        ((LinkedDynamicsWorld) mWorld).addLinkedBodies(foundBodies);
    }

    // Computes the broad-phase collision detection.
    private void computeBroadPhase() {
        for (CollisionBody body : mWorld.getBodies()) {
            if (body.getHasMoved()) {
                mBroadPhaseAlgorithm.updateObject(body, body.getAABB());
            }
        }
    }

    // Computes the narrow-phase collision detection.
    private void computeNarrowPhase() {
        for (Entry<IntPair, BroadPhasePair> entry : mOverlappingPairs.entrySet()) {
            final ContactPointInfo contactInfo = new ContactPointInfo();
            final BroadPhasePair pair = entry.getValue();
            if (pair == null) {
                throw new IllegalStateException("pair cannot be null");
            }
            final CollisionBody body1 = pair.getFirstBody();
            final CollisionBody body2 = pair.getSecondBody();
            mWorld.updateOverlappingPair(pair);
            if (mNoCollisionPairs.contains(pair.getBodiesIndexPair())) {
                continue;
            }
            if (body1.isSleeping() && body2.isSleeping()) {
                continue;
            }
            final NarrowPhaseAlgorithm narrowPhaseAlgorithm = selectNarrowPhaseAlgorithm(body1.getCollisionShape(), body2.getCollisionShape());
            narrowPhaseAlgorithm.setCurrentOverlappingPair(pair);
            if (narrowPhaseAlgorithm.testCollision(body1.getCollisionShape(), body1.getTransform(), body2.getCollisionShape(), body2.getTransform(), contactInfo)) {
                contactInfo.setFirstBody((RigidBody) body1);
                contactInfo.setSecondBody((RigidBody) body2);
                mWorld.notifyNewContact(pair, contactInfo);
            }
        }
    }

    /**
     * Allows the broad phase to notify the collision detection about an overlapping pair. This method is called by a broad-phase collision detection algorithm.
     *
     * @param addedPair The pair that was added
     */
    public void broadPhaseNotifyAddedOverlappingPair(BodyPair addedPair) {
        final IntPair indexPair = addedPair.getBodiesIndexPair();
        final BroadPhasePair broadPhasePair = new BroadPhasePair(addedPair.getFirstBody(), addedPair.getSecondBody());
        final BroadPhasePair old = mOverlappingPairs.put(indexPair, broadPhasePair);
        if (old != null) {
            throw new IllegalStateException("the pair already existed in the overlapping pairs map");
        }
        mWorld.notifyAddedOverlappingPair(broadPhasePair);
    }

    /**
     * Allows the broad phase to notify the collision detection about a removed overlapping pair.
     *
     * @param removedPair The pair that was removed
     */
    public void broadPhaseNotifyRemovedOverlappingPair(BodyPair removedPair) {
        final IntPair indexPair = removedPair.getBodiesIndexPair();
        final BroadPhasePair broadPhasePair = mOverlappingPairs.get(indexPair);
        if (broadPhasePair == null) {
            throw new IllegalStateException("the removed pair must be in the map");
        }
        mWorld.notifyRemovedOverlappingPair(broadPhasePair);
        mOverlappingPairs.remove(indexPair);
    }

    // Selects the narrow-phase collision algorithm to use given two collision shapes.
    private NarrowPhaseAlgorithm selectNarrowPhaseAlgorithm(CollisionShape collisionShape1, CollisionShape collisionShape2) {
        if (collisionShape1.getType() == CollisionShapeType.SPHERE && collisionShape2.getType() == CollisionShapeType.SPHERE) {
            return mNarrowPhaseSphereVsSphereAlgorithm;
        } else {
            return mNarrowPhaseGJKAlgorithm;
        }
    }
}
