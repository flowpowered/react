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

import java.util.Random;

import org.junit.Assert;
import org.junit.Test;

import com.flowpowered.react.collision.narrowphase.SphereVsSphereAlgorithm;
import com.flowpowered.react.collision.shape.SphereShape;
import com.flowpowered.react.constraint.ContactPoint.ContactPointInfo;
import com.flowpowered.react.math.Transform;
import com.flowpowered.react.math.Vector3;

public class SphereVsSphereAlgorithmTest {
    private static final Random RANDOM = new Random();

    @Test
    public void test() {
        final SphereVsSphereAlgorithm sphereVsSphere = new SphereVsSphereAlgorithm();
        for (int i = 0; i < 100; i++) {
            final SphereShape s1 = Dummies.newSphereShape();
            final Transform t1 = Dummies.newTransform();
            final SphereShape s2 = Dummies.newSphereShape();
            final Transform t2 = Dummies.newTransform();
            if (i < 50) {
                nonCollideSpheres(s1, t1, s2, t2);
                Assert.assertFalse(sphereVsSphere.testCollision(s1, t1, s2, t2, new ContactPointInfo()));
            } else {
                collideSpheres(s1, t1, s2, t2);
                Assert.assertTrue(sphereVsSphere.testCollision(s1, t1, s2, t2, new ContactPointInfo()));
            }
        }
    }

    private static void collideSpheres(SphereShape s1, Transform t1, SphereShape s2, Transform t2) {
        final Vector3 pos1 = t1.getPosition();
        final float rad1 = s1.getRadius();
        final float phi1 = RANDOM.nextFloat() * 2 * (float) Math.PI;
        final float theta1 = RANDOM.nextFloat() * 2 * (float) Math.PI;
        final float r1 = RANDOM.nextFloat() * rad1;
        final Vector3 in = new Vector3(
                r1 * (float) Math.sin(theta1) * (float) Math.cos(phi1),
                r1 * (float) Math.sin(theta1) * (float) Math.sin(phi1),
                r1 * (float) Math.cos(theta1))
                .add(pos1);
        final float phi2 = RANDOM.nextFloat() * 2 * (float) Math.PI;
        final float theta2 = RANDOM.nextFloat() * 2 * (float) Math.PI;
        final float r2 = s2.getRadius();
        final Vector3 pos2 = new Vector3(
                r2 * (float) Math.sin(theta2) * (float) Math.cos(phi2),
                r2 * (float) Math.sin(theta2) * (float) Math.sin(phi2),
                r2 * (float) Math.cos(theta2))
                .add(in);
        t2.setPosition(pos2);
    }

    private static void nonCollideSpheres(SphereShape s1, Transform t1, SphereShape s2, Transform t2) {
        final Vector3 pos1 = t1.getPosition();
        final float rad1 = s1.getRadius();
        final float rad2 = s2.getRadius();
        final float phi = RANDOM.nextFloat() * 2 * (float) Math.PI;
        final float theta = RANDOM.nextFloat() * 2 * (float) Math.PI;
        final float r = rad1 + rad2 + RANDOM.nextInt(5) + 0.01f;
        final Vector3 pos2 = new Vector3(
                r * (float) Math.sin(theta) * (float) Math.cos(phi),
                r * (float) Math.sin(theta) * (float) Math.sin(phi),
                r * (float) Math.cos(theta))
                .add(pos1);
        t2.setPosition(pos2);
    }
}
