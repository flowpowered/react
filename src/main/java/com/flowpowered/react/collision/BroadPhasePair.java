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
package com.flowpowered.react.collision;

import com.flowpowered.react.Utilities.IntPair;
import com.flowpowered.react.body.CollisionBody;
import com.flowpowered.react.math.Vector3;

/**
 * Represents a pair of bodies during the broad-phase collision detection.
 */
public class BroadPhasePair {
    private CollisionBody body1;
    private CollisionBody body2;
    private final Vector3 previousSeparatingAxis = new Vector3(1, 1, 1);

    /**
     * Constructs a new broad phase pair from the first and the second body.
     *
     * @param body1 The first body
     * @param body2 The second body
     */
    public BroadPhasePair(CollisionBody body1, CollisionBody body2) {
        this.body1 = body1;
        this.body2 = body2;
    }

    /**
     * Gets the pair of bodies indexes (IDs) as a pair of integers.
     *
     * @return The pair of body indexes (IDs)
     */
    public IntPair getBodiesIndexPair() {
        return computeBodiesIndexPair(body1, body2);
    }

    /**
     * Gets the previous separating axis.
     *
     * @return The previous separating axis
     */
    public Vector3 getPreviousSeparatingAxis() {
        return previousSeparatingAxis;
    }

    /**
     * Sets the previous separating axis.
     *
     * @param previousSeparatingAxis The axis to set
     */
    public void setPreviousSeparatingAxis(Vector3 previousSeparatingAxis) {
        this.previousSeparatingAxis.set(previousSeparatingAxis);
    }

    /**
     * Gets the first body of the pair.
     *
     * @return The first body
     */
    public CollisionBody getFirstBody() {
        return body1;
    }

    /**
     * Sets the first body of the pair.
     *
     * @param body1 The first body
     */
    public void setFirstBody(CollisionBody body1) {
        this.body1 = body1;
    }

    /**
     * Gets the second body of the pair.
     *
     * @return The second body
     */
    public CollisionBody getSecondBody() {
        return body2;
    }

    /**
     * Sets the second body of the pair.
     *
     * @param body2 The second body
     */
    public void setSecondBody(CollisionBody body2) {
        this.body2 = body2;
    }

    /**
     * Returns true this broad phase pair is smaller than the other broad phase pair, false if not.
     *
     * @param broadPhasePair2 The broad phase pair to compare with
     * @return True if this broad phase pair is smaller, false if not
     */
    public boolean isSmallerThan(BroadPhasePair broadPhasePair2) {
        return body1.isSmallerThan(broadPhasePair2.body1) || (body2.isSmallerThan(broadPhasePair2.body2));
    }

    /**
     * Returns true this broad phase pair is greater than the other broad phase pair, false if not.
     *
     * @param broadPhasePair2 The broad phase pair to compare with
     * @return True if this broad phase pair is greater, false if not
     */
    public boolean isGreaterThan(BroadPhasePair broadPhasePair2) {
        return body1.isGreaterThan(broadPhasePair2.body1) || (body2.isGreaterThan(broadPhasePair2.body2));
    }

    /**
     * Returns true this broad phase pair is equal than the other broad phase pair, false if not.
     *
     * @param broadPhasePair2 The broad phase pair to compare with
     * @return True if this broad phase pair equal, false if not
     */
    public boolean isEqualTo(BroadPhasePair broadPhasePair2) {
        return body1.isEqualTo(broadPhasePair2.body1) && body2.isEqualTo(broadPhasePair2.body2);
    }

    /**
     * Returns true this broad phase pair is not equal than the other broad phase pair, false if not.
     *
     * @param broadPhasePair2 The broad phase pair to compare with
     * @return True if this broad phase pair is not equal, false if not
     */
    public boolean isNotEqualTo(BroadPhasePair broadPhasePair2) {
        return body1.isNotEqualTo(broadPhasePair2.body1) || body2.isNotEqualTo(broadPhasePair2.body2);
    }

    /**
     * Converts the pair of bodies to a pair of indexes (IDs) as a pair of integers.
     *
     * @param body1 The first body
     * @param body2 The second body
     * @return The pair of body indexes (IDs)
     */
    public static IntPair computeBodiesIndexPair(CollisionBody body1, CollisionBody body2) {
        final IntPair indexPair = body1.getID() < body2.getID() ?
                new IntPair(body1.getID(), body2.getID()) :
                new IntPair(body2.getID(), body1.getID());
        if (indexPair.getFirst() == indexPair.getSecond()) {
            throw new IllegalStateException("First int of the pair cannot be equal to the second int of the pair");
        }
        return indexPair;
    }
}
