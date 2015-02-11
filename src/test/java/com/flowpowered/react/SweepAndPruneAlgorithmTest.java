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
package com.flowpowered.react;

import java.util.HashSet;
import java.util.Set;

import org.junit.Assert;
import org.junit.Test;

import com.flowpowered.react.body.CollisionBody;
import com.flowpowered.react.collision.broadphase.SweepAndPruneAlgorithm;
import com.flowpowered.react.collision.shape.AABB;

public class SweepAndPruneAlgorithmTest {
    private static final int BODY_COUNT = 100;
    private int ID = 0;

    @Test
    public void test() {
        final SweepAndPruneAlgorithm sweepAndPrune = new SweepAndPruneAlgorithm(Dummies.newCollisionDetection());
        final Set<CollisionBody> bodies = new HashSet<>();
        for (int repeat = 0; repeat < 2; repeat++) {
            for (int i = 0; i < BODY_COUNT / 10; i++) {
                final CollisionBody body = Dummies.newCollisionBody(ID++);
                bodies.add(body);
                sweepAndPrune.addObject(body, Dummies.newAABB());
            }
            Assert.assertEquals(BODY_COUNT / 10, sweepAndPrune.getNbObjects());
            Assert.assertTrue(countNotNull(sweepAndPrune.getOverlappingPairs()) >= 0);
            for (int i = 0; i < BODY_COUNT / 2; i++) {
                final AABB aabb = Dummies.newAABB();
                final CollisionBody body0 = Dummies.newCollisionBody(ID++);
                bodies.add(body0);
                sweepAndPrune.addObject(body0, aabb);
                final CollisionBody body1 = Dummies.newCollisionBody(ID++);
                bodies.add(body1);
                sweepAndPrune.addObject(body1, Dummies.newIntersectingAABB(aabb));
            }
            Assert.assertEquals(BODY_COUNT + BODY_COUNT / 10, sweepAndPrune.getNbObjects());
            Assert.assertTrue(countNotNull(sweepAndPrune.getOverlappingPairs()) >= BODY_COUNT / 2);
            for (CollisionBody body : bodies) {
                sweepAndPrune.removeObject(body);
            }
            Assert.assertEquals(0, sweepAndPrune.getNbObjects());
            bodies.clear();
            ID = 0;
        }
    }

    private static int countNotNull(Object[] array) {
        if (array == null) {
            return 0;
        }
        int count = 0;
        for (Object object : array) {
            if (object != null) {
                count++;
            }
        }
        return count;
    }
}
