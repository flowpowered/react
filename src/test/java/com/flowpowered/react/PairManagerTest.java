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