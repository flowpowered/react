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

import java.util.Random;

import org.junit.Assert;
import org.junit.Test;

import com.flowpowered.react.body.CollisionBody;
import com.flowpowered.react.body.RigidBody;
import com.flowpowered.react.collision.BroadPhasePair;
import com.flowpowered.react.collision.narrowphase.GJK.GJKAlgorithm;
import com.flowpowered.react.collision.shape.BoxShape;
import com.flowpowered.react.collision.shape.CapsuleShape;
import com.flowpowered.react.collision.shape.CollisionShape;
import com.flowpowered.react.collision.shape.ConeShape;
import com.flowpowered.react.collision.shape.CylinderShape;
import com.flowpowered.react.collision.shape.SphereShape;
import com.flowpowered.react.constraint.ContactPoint.ContactPointInfo;
import com.flowpowered.react.math.Matrix3x3;
import com.flowpowered.react.math.Transform;
import com.flowpowered.react.math.Vector3;

public class GJKAndEPAAlgorithmTest {
    private static final Random RANDOM = new Random();
    private static int ID = 0;

    @Test
    public void test() {
        final GJKAlgorithm gjkAlgorithm = new GJKAlgorithm();
        int collideFails = 0;
        int nonCollideFails = 0;
        for (int i = 0; i < 100; i++) {
            final CollisionShape s1 = Dummies.newCollisionShape();
            final Transform t1 = Dummies.newTransform();
            final CollisionShape s2 = Dummies.newCollisionShape();
            final Transform t2 = Dummies.newTransform();
            if (i < 50) {
                collideShapes(s1, t1, s2, t2);
                final BroadPhasePair pair = pair(s1, t1, s2, t2);
                gjkAlgorithm.setCurrentOverlappingPair(pair);
                if (!gjkAlgorithm.testCollision(s1, t1, s2, t2, new ContactPointInfo())) {
                    collideFails++;
                }
            } else {
                nonCollideShapes(s1, t1, s2, t2);
                final BroadPhasePair pair = pair(s1, t1, s2, t2);
                gjkAlgorithm.setCurrentOverlappingPair(pair);
                if (gjkAlgorithm.testCollision(s1, t1, s2, t2, new ContactPointInfo())) {
                    nonCollideFails++;
                }
            }
        }
        Assert.assertTrue(collideFails <= 3);
        Assert.assertTrue(nonCollideFails <= 3);
    }

    private static void collideShapes(CollisionShape s1, Transform t1, CollisionShape s2, Transform t2) {
        final Vector3 in = new Vector3();
        switch (s1.getType()) {
            case CONVEX_MESH:
                s1 = new BoxShape(new Vector3(1, 1, 1));
            case BOX: {
                final BoxShape box = (BoxShape) s1;
                final Vector3 extend = box.getExtent();
                in.setAllValues(
                        RANDOM.nextFloat() * extend.getX() * 2 - extend.getX(),
                        RANDOM.nextFloat() * extend.getY() * 2 - extend.getY(),
                        RANDOM.nextFloat() * extend.getZ() * 2 - extend.getZ());
                break;
            }
            case CONE: {
                final ConeShape cone = (ConeShape) s1;
                final float height = cone.getHeight();
                final float y = RANDOM.nextFloat() * height - height / 2;
                final float r = RANDOM.nextFloat() * (height / 2 - y) * (cone.getRadius() / height);
                final double theta = RANDOM.nextDouble() * 2 * Math.PI;
                final float x = r * (float) Math.cos(theta);
                final float z = r * (float) Math.sin(theta);
                in.setAllValues(x, y, z);
                break;
            }
            case CYLINDER: {
                final CylinderShape cylinder = (CylinderShape) s1;
                final double theta = RANDOM.nextDouble() * 2 * Math.PI;
                final float r = RANDOM.nextFloat() * cylinder.getRadius();
                in.setAllValues(
                        r * (float) Math.cos(theta),
                        RANDOM.nextFloat() * cylinder.getHeight() - cylinder.getHeight() / 2,
                        r * (float) Math.sin(theta));
                break;
            }
            case SPHERE: {
                final SphereShape sphere = (SphereShape) s1;
                final double phi = RANDOM.nextDouble() * 2 * Math.PI;
                final double theta = RANDOM.nextDouble() * 2 * Math.PI;
                final float r = RANDOM.nextFloat() * sphere.getRadius();
                in.setAllValues(
                        r * (float) Math.sin(theta) * (float) Math.cos(phi),
                        r * (float) Math.sin(theta) * (float) Math.sin(phi),
                        r * (float) Math.cos(theta));
                break;
            }
            case CAPSULE: {
                final CapsuleShape capsule = (CapsuleShape) s1;
                final float radius = capsule.getRadius();
                final float halfHeight = capsule.getHeight() / 2;
                final float totalHeight = capsule.getHeight() + radius * 2;
                final float y = RANDOM.nextFloat() * totalHeight - totalHeight / 2;
                final double theta = RANDOM.nextDouble() * 2 * Math.PI;
                final float r;
                if (y >= -halfHeight && y <= halfHeight) {
                    r = RANDOM.nextFloat() * radius;
                } else {
                    final float dy = Math.abs(y) - halfHeight;
                    r = RANDOM.nextFloat() * (float) Math.sqrt(radius * radius - dy * dy);
                }
                in.setAllValues(
                        r * (float) Math.cos(theta),
                        y,
                        r * (float) Math.sin(theta));
                break;
            }
            default:
                throw new IllegalStateException("Unknown collision shape for s1");
        }
        final Vector3 surface = new Vector3();
        switch (s2.getType()) {
            case CONVEX_MESH:
                s2 = new BoxShape(new Vector3(1, 1, 1));
            case BOX: {
                final BoxShape box = (BoxShape) s2;
                final Vector3 extend = box.getExtent();
                switch (RANDOM.nextInt(3)) {
                    case 0: {
                        surface.setAllValues(
                                RANDOM.nextBoolean() ? extend.getX() : -extend.getX(),
                                RANDOM.nextFloat() * extend.getY() * 2 - extend.getY(),
                                RANDOM.nextFloat() * extend.getZ() * 2 - extend.getZ());
                        break;
                    }
                    case 1: {
                        surface.setAllValues(
                                RANDOM.nextFloat() * extend.getX() * 2 - extend.getX(),
                                RANDOM.nextBoolean() ? extend.getY() : -extend.getY(),
                                RANDOM.nextFloat() * extend.getZ() * 2 - extend.getZ());
                        break;
                    }
                    case 2: {
                        surface.setAllValues(
                                RANDOM.nextFloat() * extend.getX() * 2 - extend.getX(),
                                RANDOM.nextFloat() * extend.getY() * 2 - extend.getY(),
                                RANDOM.nextBoolean() ? extend.getZ() : -extend.getZ());
                        break;
                    }
                }
                break;
            }
            case CONE: {
                final ConeShape cone = (ConeShape) s2;
                final float height = cone.getHeight();
                final float y = RANDOM.nextFloat() * height - height / 2;
                final float r = y == -height / 2 ? RANDOM.nextFloat() : (height / 2 - y) * (cone.getRadius() / height);
                final double theta = RANDOM.nextDouble() * 2 * Math.PI;
                final float x = r * (float) Math.cos(theta);
                final float z = r * (float) Math.sin(theta);
                surface.setAllValues(x, y, z);
                break;
            }
            case CYLINDER: {
                final CylinderShape cylinder = (CylinderShape) s2;
                final float y = RANDOM.nextFloat() * cylinder.getHeight() - cylinder.getHeight() / 2;
                final float r = Math.abs(y) == cylinder.getHeight() / 2 ? RANDOM.nextFloat() : cylinder.getRadius();
                final double theta = RANDOM.nextDouble() * 2 * Math.PI;
                final float x = r * (float) Math.cos(theta);
                final float z = r * (float) Math.sin(theta);
                surface.setAllValues(x, y, z);
                break;
            }
            case SPHERE: {
                final SphereShape sphere = (SphereShape) s2;
                final double phi = RANDOM.nextDouble() * 2 * Math.PI;
                final double theta = RANDOM.nextDouble() * 2 * Math.PI;
                final float r = sphere.getRadius();
                surface.setAllValues(
                        r * (float) Math.sin(theta) * (float) Math.cos(phi),
                        r * (float) Math.sin(theta) * (float) Math.sin(phi),
                        r * (float) Math.cos(theta));
                break;
            }
            case CAPSULE: {
                final CapsuleShape capsule = (CapsuleShape) s2;
                final float radius = capsule.getRadius();
                final float halfHeight = capsule.getHeight() / 2;
                final float totalHeight = capsule.getHeight() + radius * 2;
                final float y = RANDOM.nextFloat() * totalHeight - totalHeight / 2;
                final double theta = RANDOM.nextDouble() * 2 * Math.PI;
                final float r;
                if (y >= -halfHeight && y <= halfHeight) {
                    r = radius;
                } else {
                    final float dy = Math.abs(y) - halfHeight;
                    r = (float) Math.sqrt(radius * radius - dy * dy);
                }
                surface.setAllValues(
                        r * (float) Math.cos(theta),
                        y,
                        r * (float) Math.sin(theta));
                break;
            }
            default:
                throw new IllegalStateException("Unknown collision shape for s2");
        }
        in.set(Transform.multiply(t1, in));
        surface.set(Matrix3x3.multiply(t2.getOrientation().getMatrix(), surface));
        t2.setPosition(Vector3.subtract(in, surface.multiply(0.95f)));
    }

    private static void nonCollideShapes(CollisionShape s1, Transform t1, CollisionShape s2, Transform t2) {
        final float radius1;
        switch (s1.getType()) {
            case CONVEX_MESH:
                s1 = new BoxShape(new Vector3(1, 1, 1));
            case BOX: {
                final BoxShape box = (BoxShape) s1;
                radius1 = box.getExtent().length();
                break;
            }
            case CONE: {
                final ConeShape cone = (ConeShape) s1;
                final float h = cone.getHeight() / 2;
                final float r = cone.getRadius();
                radius1 = (float) Math.sqrt(h * h + r * r);
                break;
            }
            case CYLINDER: {
                final CylinderShape cylinder = (CylinderShape) s1;
                final float h = cylinder.getHeight() / 2;
                final float r = cylinder.getRadius();
                radius1 = (float) Math.sqrt(h * h + r * r);
                break;
            }
            case SPHERE: {
                final SphereShape sphere = (SphereShape) s1;
                radius1 = sphere.getRadius();
                break;
            }
            case CAPSULE: {
                final CapsuleShape capsule = (CapsuleShape) s1;
                radius1 = capsule.getHeight() / 2 + capsule.getRadius();
                break;
            }
            default:
                throw new IllegalStateException("Unknown collision shape for s1");
        }
        final float radius2;
        switch (s2.getType()) {
            case CONVEX_MESH:
                s2 = new BoxShape(new Vector3(1, 1, 1));
            case BOX: {
                final BoxShape box = (BoxShape) s2;
                radius2 = box.getExtent().length();
                break;
            }
            case CONE: {
                final ConeShape cone = (ConeShape) s2;
                final float h = cone.getHeight() / 2;
                final float r = cone.getRadius();
                radius2 = (float) Math.sqrt(h * h + r * r);
                break;
            }
            case CYLINDER: {
                final CylinderShape cylinder = (CylinderShape) s2;
                final float h = cylinder.getHeight() / 2;
                final float r = cylinder.getRadius();
                radius2 = (float) Math.sqrt(h * h + r * r);
                break;
            }
            case SPHERE: {
                final SphereShape sphere = (SphereShape) s2;
                radius2 = sphere.getRadius();
                break;
            }
            case CAPSULE: {
                final CapsuleShape capsule = (CapsuleShape) s2;
                radius2 = capsule.getHeight() / 2 + capsule.getRadius();
                break;
            }
            default:
                throw new IllegalStateException("Unknown collision shape for s2");
        }
        final double phi = RANDOM.nextDouble() * 2 * Math.PI;
        final double theta = RANDOM.nextDouble() * 2 * Math.PI;
        final float r = radius1 + radius2 + 0.1f;
        final Vector3 dist = new Vector3(
                r * (float) Math.sin(theta) * (float) Math.cos(phi),
                r * (float) Math.sin(theta) * (float) Math.sin(phi),
                r * (float) Math.cos(theta));
        t2.setPosition(Vector3.add(t1.getPosition(), dist));
    }

    private static BroadPhasePair pair(CollisionShape s1, Transform t1, CollisionShape s2, Transform t2) {
        final CollisionBody b1 = new RigidBody(t1, 0, Matrix3x3.identity(), s1, ID++);
        final CollisionBody b2 = new RigidBody(t2, 0, Matrix3x3.identity(), s2, ID++);
        b1.updateAABB();
        b2.updateAABB();
        return new BroadPhasePair(b1, b2);
    }
}
