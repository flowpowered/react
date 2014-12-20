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
package com.flowpowered.react.engine;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import gnu.trove.stack.TIntStack;
import gnu.trove.stack.array.TIntArrayStack;

import com.flowpowered.react.Utilities.IntPair;
import com.flowpowered.react.body.CollisionBody;
import com.flowpowered.react.collision.BroadPhasePair;
import com.flowpowered.react.collision.CollisionDetection;
import com.flowpowered.react.collision.RayCaster;
import com.flowpowered.react.collision.RayCaster.IntersectedBody;
import com.flowpowered.react.collision.shape.CollisionShape;
import com.flowpowered.react.constraint.ContactPoint.ContactPointInfo;
import com.flowpowered.react.math.Vector3;

/**
 * Represents a world where it is possible to move bodies by hand and to test collisions between them. In this kind of world the body movement is not computed using the laws of physics.
 */
public abstract class CollisionWorld {
    protected final CollisionDetection mCollisionDetection;
    protected final Set<CollisionBody> mBodies = new HashSet<>();
    protected final List<CollisionShape> mCollisionShapes = new ArrayList<>();
    protected final Map<IntPair, OverlappingPair> mOverlappingPairs = new HashMap<>();
    protected int mCurrentBodyID = 0;
    protected final TIntStack mFreeBodiesIDs = new TIntArrayStack();

    /**
     * Constructs a new empty collision world.
     */
    protected CollisionWorld() {
        mCollisionDetection = new CollisionDetection(this);
    }

    /**
     * Notifies the world about a new broad-phase overlapping pair.
     *
     * @param addedPair The pair that was added
     */
    public abstract void notifyAddedOverlappingPair(BroadPhasePair addedPair);

    /**
     * Notifies the world about a removed broad-phase overlapping pair.
     *
     * @param removedPair The pair that was removed
     */
    public abstract void notifyRemovedOverlappingPair(BroadPhasePair removedPair);

    /**
     * Notifies the world about a new narrow-phase contact.
     *
     * @param pair The pair of bodies in contact
     * @param contactInfo The information for the contact
     */
    public abstract void notifyNewContact(BroadPhasePair pair, ContactPointInfo contactInfo);

    /**
     * Updates the overlapping pair.
     *
     * @param pair The pair to update
     */
    public abstract void updateOverlappingPair(BroadPhasePair pair);

    /**
     * Gets the set of the bodies of the physics world.
     *
     * @return The {@link java.util.Set} of {@link com.flowpowered.react.body.CollisionBody}
     */
    public Set<CollisionBody> getBodies() {
        return mBodies;
    }

    /**
     * Finds the closest of the bodies in the world intersecting with the ray to the ray start. The ray is defined by a starting point and a direction. This method returns an {@link IntersectedBody}
     * object containing the body and the intersection point.
     *
     * @param rayStart The ray starting point
     * @param rayDir The ray direction
     * @return The closest body to the ray start and its intersection point
     */
    public IntersectedBody findClosestIntersectingBody(Vector3 rayStart, Vector3 rayDir) {
        return RayCaster.findClosestIntersectingBody(rayStart, rayDir, mBodies);
    }

    /**
     * Finds the furthest of the bodies in the world intersecting with the ray from the ray start. The ray is defined by a starting point and a direction. This method returns an {@link
     * IntersectedBody} object containing the body and the intersection point.
     *
     * @param rayStart The ray starting point
     * @param rayDir The ray direction
     * @return The furthest body from the ray start and its intersection point
     */
    public IntersectedBody findFurthestIntersectingBody(Vector3 rayStart, Vector3 rayDir) {
        return RayCaster.findFurthestIntersectingBody(rayStart, rayDir, mBodies);
    }

    /**
     * Finds all of the bodies in the world intersecting with the ray. The ray is defined by a starting point and a direction. The bodies are returned mapped with the closest intersection point.
     *
     * @param rayStart The ray starting point
     * @param rayDir The ray direction
     * @return All of the intersection bodies, in no particular order, mapped to the distance vector
     */
    public Map<CollisionBody, Vector3> findIntersectingBodies(Vector3 rayStart, Vector3 rayDir) {
        return RayCaster.findIntersectingBodies(rayStart, rayDir, mBodies);
    }

    /**
     * Returns the next available body ID for this world.
     *
     * @return The next available id
     * @throws IllegalStateException If the id for the body is greater than Integer.MAX_VALUE
     */
    public int getNextFreeID() {
        final int bodyID;
        if (mFreeBodiesIDs.size() != 0) {
            bodyID = mFreeBodiesIDs.pop();
        } else {
            bodyID = mCurrentBodyID;
            mCurrentBodyID++;
        }
        if (bodyID >= Integer.MAX_VALUE) {
            throw new IllegalStateException("body id cannot be larger or equal to the largest integer");
        }
        return bodyID;
    }

    /**
     * Creates a new collision shape. First, this methods checks that the new collision shape does not exist yet in the world. If it already exists, we do not allocate memory for a new one but instead
     * we reuse the existing one. The goal is to only allocate memory for a single collision shape if this one is used for several bodies in the world.
     *
     * @param collisionShape The collision shape to create
     * @return The desired collision shape
     */
    protected CollisionShape createCollisionShape(CollisionShape collisionShape) {
        for (CollisionShape shape : mCollisionShapes) {
            if (collisionShape.equals(shape)) {
                shape.incrementNbSimilarCreatedShapes();
                return shape;
            }
        }
        final CollisionShape newCollisionShape = collisionShape.clone();
        mCollisionShapes.add(newCollisionShape);
        newCollisionShape.incrementNbSimilarCreatedShapes();
        return newCollisionShape;
    }

    /**
     * Removes a collision shape. First, we check if another body is still using the same collision shape. If so, we keep the allocated collision shape. If it is not the case, we can deallocate the
     * memory associated with the collision shape.
     *
     * @param collisionShape The collision shape to remove
     */
    protected void removeCollisionShape(CollisionShape collisionShape) {
        if (collisionShape.getNbSimilarCreatedShapes() == 0) {
            throw new IllegalStateException("Expected at least one similar collision shape remaining");
        }
        collisionShape.decrementNbSimilarCreatedShapes();
        if (collisionShape.getNbSimilarCreatedShapes() == 0) {
            mCollisionShapes.remove(collisionShape);
        }
    }
}
