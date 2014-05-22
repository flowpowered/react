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
package org.spout.physics;

import java.util.Random;

import org.spout.physics.body.CollisionBody;
import org.spout.physics.body.RigidBody;
import org.spout.physics.collision.BroadPhasePair;
import org.spout.physics.collision.CollisionDetection;
import org.spout.physics.collision.shape.AABB;
import org.spout.physics.collision.shape.BoxShape;
import org.spout.physics.collision.shape.CollisionShape;
import org.spout.physics.collision.shape.ConeShape;
import org.spout.physics.collision.shape.CylinderShape;
import org.spout.physics.collision.shape.SphereShape;
import org.spout.physics.constraint.ContactPoint.ContactPointInfo;
import org.spout.physics.engine.CollisionWorld;
import org.spout.physics.math.Matrix3x3;
import org.spout.physics.math.Quaternion;
import org.spout.physics.math.Transform;
import org.spout.physics.math.Vector3;

/**
 * Provides fake implementations and objects for tests.
 */
public class Dummies {
    private static final Random RANDOM = new Random();

    public static Vector3 newPosition() {
        return new Vector3(RANDOM.nextInt(21) - 10, RANDOM.nextInt(21) - 10, RANDOM.nextInt(21) - 10);
    }

    public static Quaternion newOrientation() {
        final float phi = RANDOM.nextFloat() * 2 * (float) Math.PI;
        final float theta = RANDOM.nextFloat() * 2 * (float) Math.PI;
        final float x = (float) Math.sin(theta) * (float) Math.cos(phi);
        final float y = (float) Math.sin(theta) * (float) Math.sin(phi);
        final float z = (float) Math.cos(theta);
        final float halfAngle = (float) Math.toRadians(RANDOM.nextFloat() * 2 * Math.PI) / 2;
        final float q = (float) Math.sin(halfAngle);
        return new Quaternion(x * q, y * q, z * q, (float) Math.cos(halfAngle));
    }

    public static Transform newTransform() {
        return new Transform(newPosition(), newOrientation());
    }

    public static CollisionBody newCollisionBody(int id) {
        return new RigidBody(Transform.identity(), 0, Matrix3x3.identity(), new BoxShape(new Vector3(1, 1, 1)), id);
    }

    public static AABB newAABB() {
        final Vector3 min = newPosition();
        return new AABB(min, Vector3.add(min, new Vector3(RANDOM.nextInt(5) + 4, RANDOM.nextInt(5) + 4, RANDOM.nextInt(5) + 4)));
    }

    public static AABB newIntersectingAABB(AABB with) {
        final Vector3 wMin = with.getMin();
        final Vector3 wSize = Vector3.subtract(with.getMax(), wMin);
        final int iSizeX = RANDOM.nextInt((int) wSize.getX() + 1);
        final int iSizeY = RANDOM.nextInt((int) wSize.getY() + 1);
        final int iSizeZ = RANDOM.nextInt((int) wSize.getZ() + 1);
        final int eSizeX = RANDOM.nextInt(5) + 4;
        final int eSizeY = RANDOM.nextInt(5) + 4;
        final int eSizeZ = RANDOM.nextInt(5) + 4;
        final Vector3 min = Vector3.subtract(wMin, new Vector3(eSizeX, eSizeY, eSizeZ));
        final Vector3 max = Vector3.add(wMin, new Vector3(iSizeX, iSizeY, iSizeZ));
        return new AABB(min, max);
    }

    public static BoxShape newBoxShape() {
        return new BoxShape(new Vector3(RANDOM.nextInt(5) + 4, RANDOM.nextInt(5) + 4, RANDOM.nextInt(5) + 4));
    }

    public static ConeShape newConeShape() {
        return new ConeShape(RANDOM.nextInt(5) + 4, RANDOM.nextInt(5) + 4);
    }

    public static CylinderShape newCylinderShape() {
        return new CylinderShape(RANDOM.nextInt(5) + 4, RANDOM.nextInt(5) + 4);
    }

    public static SphereShape newSphereShape() {
        return new SphereShape(RANDOM.nextInt(5) + 4);
    }

    public static CollisionShape newCollisionShape() {
        switch (RANDOM.nextInt(4)) {
            case 0:
                return newBoxShape();
            case 1:
                return newConeShape();
            case 2:
                return newCylinderShape();
            case 3:
                return newSphereShape();
            default:
                throw new IllegalStateException("random int larger than shape types count");
        }
    }

    public static CollisionWorld newCollisionWorld() {
        return new DummyCollisionWorld();
    }

    public static CollisionDetection newCollisionDetection() {
        return new CollisionDetection(newCollisionWorld());
    }

    private static class DummyCollisionWorld extends CollisionWorld {
        private DummyCollisionWorld() {
        }

        @Override
        public void notifyAddedOverlappingPair(BroadPhasePair addedPair) {
        }

        @Override
        public void notifyRemovedOverlappingPair(BroadPhasePair removedPair) {
        }

        @Override
        public void notifyNewContact(BroadPhasePair pair, ContactPointInfo contactInfo) {
        }

        @Override
        public void updateOverlappingPair(BroadPhasePair pair) {
        }
    }
}
