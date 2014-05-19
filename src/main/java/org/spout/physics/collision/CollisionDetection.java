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
package org.spout.physics.collision;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;

import org.spout.physics.Utilities.IntPair;
import org.spout.physics.body.CollisionBody;
import org.spout.physics.body.RigidBody;
import org.spout.physics.collision.broadphase.BroadPhaseAlgorithm;
import org.spout.physics.collision.broadphase.PairManager.BodyPair;
import org.spout.physics.collision.broadphase.SweepAndPruneAlgorithm;
import org.spout.physics.collision.linkedphase.LinkedPhase;
import org.spout.physics.collision.narrowphase.GJK.GJKAlgorithm;
import org.spout.physics.collision.narrowphase.NarrowPhaseAlgorithm;
import org.spout.physics.collision.narrowphase.SphereVsSphereAlgorithm;
import org.spout.physics.collision.shape.CollisionShape;
import org.spout.physics.collision.shape.CollisionShape.CollisionShapeType;
import org.spout.physics.constraint.ContactPoint.ContactPointInfo;
import org.spout.physics.engine.CollisionWorld;
import org.spout.physics.engine.linked.LinkedDynamicsWorld;

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
    private final List<CollisionListener> mCallBacks = new ArrayList<>();
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
     * Gets the collision listeners for collision detection.
     *
     * @return The collision listeners
     */
    public List<CollisionListener> getListeners() {
        return mCallBacks;
    }

    /**
     * Adds a collision listener for the collision detection.
     *
     * @param listener The listener to add
     */
    public void addListener(CollisionListener listener) {
        mCallBacks.add(listener);
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
            final NarrowPhaseAlgorithm narrowPhaseAlgorithm = selectNarrowPhaseAlgorithm(body1.getCollisionShape(), body2.getCollisionShape());
            narrowPhaseAlgorithm.setCurrentOverlappingPair(pair);
            if (narrowPhaseAlgorithm.testCollision(body1.getCollisionShape(), body1.getTransform(), body2.getCollisionShape(), body2.getTransform(), contactInfo)) {
                contactInfo.setFirstBody((RigidBody) body1);
                contactInfo.setSecondBody((RigidBody) body2);
                if (mCallBacks.isEmpty()) {
                    mWorld.notifyNewContact(pair, contactInfo);
                } else {
                    boolean cancel = false;
                    for (CollisionListener listener : mCallBacks) {
                        if (listener.onCollide(body1, body2, contactInfo)) {
                            cancel = true;
                            break;
                        }
                    }
                    if (!cancel) {
                        mWorld.notifyNewContact(pair, contactInfo);
                    }
                }
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
