/*
 * This file is part of Flow React, licensed under the MIT License (MIT).
 *
 * Copyright (c) 2013 Flow Powered <https://flowpowered.com/>
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://danielchappuis.ch>
 * Flow React is re-licensed with permission from ReactPhysics3D author.
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
package com.flowpowered.react.collision.narrowphase;

import com.flowpowered.react.collision.BroadPhasePair;
import com.flowpowered.react.collision.shape.CollisionShape;
import com.flowpowered.react.constraint.ContactPoint.ContactPointInfo;
import com.flowpowered.react.math.Transform;

/**
 * This class is an abstract class that represents an algorithm used to perform the narrow-phase of a collision detection. The goal of the narrow phase algorithm is to compute contact information of a
 * collision between two bodies.
 */
public abstract class NarrowPhaseAlgorithm {
    protected BroadPhasePair mCurrentOverlappingPair = null;

    /**
     * Sets the current overlapping pair.
     *
     * @param overlappingPair The overlapping pair
     */
    public void setCurrentOverlappingPair(BroadPhasePair overlappingPair) {
        mCurrentOverlappingPair = overlappingPair;
    }

    /**
     * Returns true and computes the contact info if the two bounding volume collide. If they do not, this method returns false and the contact info will remain unchanged. The new contact info is
     * stored in the {@code ContactInfo} parameter.
     *
     * @param collisionShape1 The first collisionShape of the collision
     * @param transform1 The first shape's transform
     * @param collisionShape2 The second collisionShape of the collision
     * @param transform2 The second shape's transform
     * @param contactInfo Where to store the contact info of this collision
     * @return True if the volumes collided, false if not
     */
    public abstract boolean testCollision(
            CollisionShape collisionShape1, Transform transform1,
            CollisionShape collisionShape2, Transform transform2,
            ContactPointInfo contactInfo);
}
