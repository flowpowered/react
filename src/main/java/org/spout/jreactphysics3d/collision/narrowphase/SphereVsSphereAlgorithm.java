/*
 * This file is part of JReactPhysics3D.
 *
 * Copyright (c) 2013 Spout LLC <http://www.spout.org/>
 * JReactPhysics3D is licensed under the Spout License Version 1.
 *
 * JReactPhysics3D is free software: you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * In addition, 180 days after any changes are published, you can use the
 * software, incorporating those changes, under the terms of the MIT license,
 * as described in the Spout License Version 1.
 *
 * JReactPhysics3D is distributed in the hope that it will be useful, but WITHOUT ANY
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
package org.spout.jreactphysics3d.collision.narrowphase;

import org.spout.jreactphysics3d.collision.ContactInfo;
import org.spout.jreactphysics3d.collision.shape.CollisionShape;
import org.spout.jreactphysics3d.collision.shape.SphereShape;
import org.spout.jreactphysics3d.mathematics.Transform;
import org.spout.jreactphysics3d.mathematics.Vector3;

/**
 * This class is used to compute the narrow-phase collision detection between two sphere shaped
 * collision volumes.
 */
public class SphereVsSphereAlgorithm extends NarrowPhaseAlgorithm {
	@Override
	public boolean testCollision(CollisionShape collisionShape1, Transform transform1,
								 CollisionShape collisionShape2, Transform transform2,
								 ContactInfo contactInfo) {
		final SphereShape sphereShape1 = (SphereShape) collisionShape1;
		final SphereShape sphereShape2 = (SphereShape) collisionShape2;
		final Vector3 vectorBetweenCenters = Vector3.subtract(transform2.getPosition(), transform1.getPosition());
		final float squaredDistanceBetweenCenters = vectorBetweenCenters.lengthSquare();
		final float sumRadius = sphereShape1.getRadius() + sphereShape2.getRadius();
		if (squaredDistanceBetweenCenters <= sumRadius * sumRadius) {
			final Vector3 centerSphere2InBody1LocalSpace = Transform.multiply(transform1.inverse(), transform2.getPosition());
			final Vector3 centerSphere1InBody2LocalSpace = Transform.multiply(transform2.inverse(), transform1.getPosition());
			final Vector3 intersectionOnBody1 = Vector3.multiply(sphereShape1.getRadius(), centerSphere2InBody1LocalSpace.getUnit());
			final Vector3 intersectionOnBody2 = Vector3.multiply(sphereShape2.getRadius(), centerSphere1InBody2LocalSpace.getUnit());
			final float penetrationDepth = sumRadius - (float) Math.sqrt(squaredDistanceBetweenCenters);
			contactInfo.set(
					vectorBetweenCenters.getUnit(), penetrationDepth,
					intersectionOnBody1, intersectionOnBody2);
			return true;
		}
		return false;
	}
}
