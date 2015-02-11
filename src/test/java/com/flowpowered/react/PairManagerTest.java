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

import org.junit.Assert;
import org.junit.Test;

import com.flowpowered.react.collision.broadphase.PairManager;
import com.flowpowered.react.collision.broadphase.PairManager.BodyPair;

public class PairManagerTest {
    @Test
    public void test() {
        final PairManager manager = new PairManager(Dummies.newCollisionDetection());
        Assert.assertEquals(manager.getNbOverlappingPairs(), 0);
        for (int i = 0; i < 20; i += 2) {
            final BodyPair pair = manager.addPair(Dummies.newCollisionBody(i), Dummies.newCollisionBody(i + 1));
            Assert.assertNotNull(pair);
            Assert.assertEquals(pair.getFirstBody().getID(), i);
            Assert.assertEquals(pair.getSecondBody().getID(), i + 1);
        }
        Assert.assertEquals(manager.getNbOverlappingPairs(), 10);
        for (int i = 0; i < 20; i += 2) {
            final BodyPair pair = manager.findPair(i, i + 1);
            Assert.assertNotNull(pair);
            Assert.assertEquals(pair.getFirstBody().getID(), i);
            Assert.assertEquals(pair.getSecondBody().getID(), i + 1);
        }
        Assert.assertEquals(manager.getNbOverlappingPairs(), 10);
        for (int i = 0; i < 20; i += 2) {
            Assert.assertTrue(manager.removePair(i, i + 1));
        }
        Assert.assertEquals(manager.getNbOverlappingPairs(), 0);
    }
}