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
package com.flowpowered.react;

import org.junit.Assert;
import org.junit.Test;

import com.flowpowered.react.body.RigidBody;
import com.flowpowered.react.collision.shape.BoxShape;
import com.flowpowered.react.constraint.ContactPoint.ContactPointInfo;
import com.flowpowered.react.engine.DynamicsWorld;
import com.flowpowered.react.engine.EventListener;
import com.flowpowered.react.math.Matrix3x3;
import com.flowpowered.react.math.Quaternion;
import com.flowpowered.react.math.Transform;
import com.flowpowered.react.math.Vector3;

public class DynamicsWorldTest {
    private static final float RUN_TIME = 2;
    private int beginContactCount = 0;
    private int newContactCount = 0;

    @Test
    public void test() throws InterruptedException {
        final float timeStep = ReactDefaults.DEFAULT_TIMESTEP;
        final DynamicsWorld world = new DynamicsWorld(new Vector3(0, -9.81f, 0), timeStep);
        world.setEventListener(new TestListener());
        world.start();
        Thread.sleep(200);
        // We want to do one update with no bodies
        world.update();
        world.stop();
        final BoxShape floorShape = new BoxShape(new Vector3(10, 0.5f, 10));
        final Transform floorTransform = new Transform(new Vector3(0, 0, 0), Quaternion.identity());
        final Matrix3x3 floorInertia = new Matrix3x3();
        final float floorMass = 100;
        floorShape.computeLocalInertiaTensor(floorInertia, floorMass);
        final RigidBody floor = world.createRigidBody(floorTransform, floorMass, floorInertia, floorShape);
        floor.enableMotion(false);
        final BoxShape boxShape = new BoxShape(new Vector3(1, 1, 1));
        final Transform boxTransform = new Transform(new Vector3(0, 5, 0), Quaternion.identity());
        final Matrix3x3 boxInertia = new Matrix3x3();
        final float boxMass = 5;
        boxShape.computeLocalInertiaTensor(boxInertia, boxMass);
        final RigidBody box = world.createRigidBody(boxTransform, boxMass, boxInertia, boxShape);
        final int stepCount = Math.round((1 / timeStep) * RUN_TIME);
        final int sleepTime = Math.round(timeStep * 1000);
        world.start();
        for (int i = 0; i < stepCount; i++) {
            final long start = System.nanoTime();
            world.update();
            final long delta = Math.round((System.nanoTime() - start) / 1000000d);
            Thread.sleep(Math.max(sleepTime - delta, 0));
        }
        world.destroyRigidBody(floor);
        world.destroyRigidBody(box);
        world.stop();
        Assert.assertTrue("There was no contact in the simulation", beginContactCount > 0);
        Assert.assertTrue("There were more contacts begun than new contacts", newContactCount > beginContactCount);
    }

    private class TestListener implements EventListener {
        @Override
        public void beginContact(ContactPointInfo contactInfo) {
            beginContactCount++;
        }

        @Override
        public void newContact(ContactPointInfo contactInfo) {
            newContactCount++;
        }
    }
}
