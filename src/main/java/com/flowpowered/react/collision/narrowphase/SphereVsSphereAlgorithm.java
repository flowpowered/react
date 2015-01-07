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

import com.flowpowered.react.collision.shape.CollisionShape;
import com.flowpowered.react.collision.shape.SphereShape;
import com.flowpowered.react.constraint.ContactPoint.ContactPointInfo;
import com.flowpowered.react.math.Transform;
import com.flowpowered.react.math.Vector3;

/**
 * This class is used to compute the narrow-phase collision detection between two sphere shaped collision volumes.
 */
public class SphereVsSphereAlgorithm extends NarrowPhaseAlgorithm {
    @Override
    public boolean testCollision(CollisionShape collisionShape1, Transform transform1,
                                 CollisionShape collisionShape2, Transform transform2,
                                 ContactPointInfo contactInfo) {
        final SphereShape sphereShape1 = (SphereShape) collisionShape1;
        final SphereShape sphereShape2 = (SphereShape) collisionShape2;
        final Vector3 vectorBetweenCenters = Vector3.subtract(transform2.getPosition(), transform1.getPosition());
        final float squaredDistanceBetweenCenters = vectorBetweenCenters.lengthSquare();
        final float sumRadius = sphereShape1.getRadius() + sphereShape2.getRadius();
        if (squaredDistanceBetweenCenters <= sumRadius * sumRadius) {
            final Vector3 centerSphere2InBody1LocalSpace = Transform.multiply(transform1.getInverse(), transform2.getPosition());
            final Vector3 centerSphere1InBody2LocalSpace = Transform.multiply(transform2.getInverse(), transform1.getPosition());
            final Vector3 intersectionOnBody1 = Vector3.multiply(sphereShape1.getRadius(), centerSphere2InBody1LocalSpace.getUnit());
            final Vector3 intersectionOnBody2 = Vector3.multiply(sphereShape2.getRadius(), centerSphere1InBody2LocalSpace.getUnit());
            final float penetrationDepth = sumRadius - (float) Math.sqrt(squaredDistanceBetweenCenters);
            contactInfo.set(vectorBetweenCenters.getUnit(), penetrationDepth, intersectionOnBody1, intersectionOnBody2);
            return true;
        }
        return false;
    }
}
