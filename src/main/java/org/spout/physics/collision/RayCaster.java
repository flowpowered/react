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
package org.spout.physics.collision;

import java.util.Collection;
import java.util.HashSet;
import java.util.Set;

import org.spout.physics.body.CollisionBody;
import org.spout.physics.collision.shape.BoxShape;
import org.spout.physics.collision.shape.CollisionShape;
import org.spout.physics.collision.shape.ConeShape;
import org.spout.physics.collision.shape.CylinderShape;
import org.spout.physics.collision.shape.SphereShape;
import org.spout.physics.math.Matrix3x3;
import org.spout.physics.math.Transform;
import org.spout.physics.math.Vector3;

/**
 * Performs ray casting on collision shapes, finding the ones that intersect the ray.
 */
public class RayCaster {
	/**
	 * Finds the closest of the bodies intersecting with the ray to the ray start. The ray is defined
	 * by a starting point and a direction.
	 *
	 * @param rayStart The ray starting point
	 * @param rayDir The ray direction
	 * @param bodies The bodies to check for intersection
	 * @return The closest body to the ray start
	 */
	public static CollisionBody findClosestIntersectingBody(Vector3 rayStart, Vector3 rayDir, Collection<CollisionBody> bodies) {
		final Set<CollisionBody> intersecting = findIntersectingBodies(rayStart, rayDir, bodies);
		CollisionBody closest = null;
		float closestDistance = Float.MAX_VALUE;
		for (CollisionBody body : intersecting) {
			final float distance = Vector3.subtract(body.getTransform().getPosition(), rayStart).lengthSquare();
			if (distance < closestDistance) {
				closest = body;
				closestDistance = distance;
			}
		}
		return closest;
	}

	/**
	 * Finds the furthest of the bodies intersecting with the ray from the ray start. The ray is
	 * defined by a starting point and a direction.
	 *
	 * @param rayStart The ray starting point
	 * @param rayDir The ray direction
	 * @param bodies The bodies to check for intersection
	 * @return The furthest body from the ray start
	 */
	public static CollisionBody findFurthestIntersectingBody(Vector3 rayStart, Vector3 rayDir, Collection<CollisionBody> bodies) {
		final Set<CollisionBody> intersecting = findIntersectingBodies(rayStart, rayDir, bodies);
		CollisionBody furthest = null;
		float furthestDistance = Float.MIN_VALUE;
		for (CollisionBody body : intersecting) {
			final float distance = Vector3.subtract(body.getTransform().getPosition(), rayStart).lengthSquare();
			if (distance > furthestDistance) {
				furthest = body;
				furthestDistance = distance;
			}
		}
		return furthest;
	}

	/**
	 * Finds all of the bodies intersecting with the ray. The ray is defined by a starting point and a
	 * direction.
	 *
	 * @param rayStart The ray starting point
	 * @param rayDir The ray direction
	 * @param bodies The bodies to check for intersection
	 * @return All of the intersection bodies, in no particular order
	 */
	public static Set<CollisionBody> findIntersectingBodies(Vector3 rayStart, Vector3 rayDir, Collection<CollisionBody> bodies) {
		final Set<CollisionBody> intersecting = new HashSet<CollisionBody>();
		for (CollisionBody body : bodies) {
			if (intersects(rayStart, rayDir, body.getCollisionShape(), body.getTransform())) {
				intersecting.add(body);
			}
		}
		return intersecting;
	}

	// Tests for intersection between a ray defined by a starting point and a direction and a collision shape
	private static boolean intersects(Vector3 rayStart, Vector3 rayDir, CollisionShape shape, Transform transform) {
		final Transform worldToObject = transform.inverse();
		final Vector3 objRayStart = Transform.multiply(worldToObject, rayStart);
		final Vector3 objRayDir = Matrix3x3.multiply(worldToObject.getOrientation().getMatrix(), rayDir);
		switch (shape.getType()) {
			case BOX:
				return intersects(objRayStart, objRayDir, (BoxShape) shape);
			case SPHERE:
				return intersects(objRayStart, objRayDir, (SphereShape) shape);
			case CONE:
				return intersects(objRayStart, objRayDir, (ConeShape) shape);
			case CYLINDER:
				return intersects(objRayStart, objRayDir, (CylinderShape) shape);
			default:
				throw new IllegalArgumentException("unknown collision shape");
		}
	}

	// Tests for intersection between a ray defined by a starting point and a direction and a box
	private static boolean intersects(Vector3 rayStart, Vector3 rayDir, BoxShape box) {
		final Vector3 extent = box.getExtent();
		final Vector3 min = Vector3.negate(extent);
		final Vector3 max = extent;
		float txMin;
		float txMax;
		if (rayDir.getX() >= 0) {
			txMin = (min.getX() - rayStart.getX()) / rayDir.getX();
			txMax = (max.getX() - rayStart.getX()) / rayDir.getX();
		} else {
			txMin = (max.getX() - rayStart.getX()) / rayDir.getX();
			txMax = (min.getX() - rayStart.getX()) / rayDir.getX();
		}
		final float tyMin;
		final float tyMax;
		if (rayDir.getY() >= 0) {
			tyMin = (min.getY() - rayStart.getY()) / rayDir.getY();
			tyMax = (max.getY() - rayStart.getY()) / rayDir.getY();
		} else {
			tyMin = (max.getY() - rayStart.getY()) / rayDir.getY();
			tyMax = (min.getY() - rayStart.getY()) / rayDir.getY();
		}
		if (txMin > tyMax || tyMin > txMax) {
			return false;
		}
		if (tyMin > txMin) {
			txMin = tyMin;
		}
		if (tyMax < txMax) {
			txMax = tyMax;
		}
		final float tzMin;
		final float tzMax;
		if (rayDir.getZ() >= 0) {
			tzMin = (min.getZ() - rayStart.getZ()) / rayDir.getZ();
			tzMax = (max.getZ() - rayStart.getZ()) / rayDir.getZ();
		} else {
			tzMin = (max.getZ() - rayStart.getZ()) / rayDir.getZ();
			tzMax = (min.getZ() - rayStart.getZ()) / rayDir.getZ();
		}
		if (txMin > tzMax || tzMin > txMax) {
			return false;
		}
		if (tzMax < txMax) {
			txMax = tzMax;
		}
		return txMax >= 0;
	}

	// Tests for intersection between a ray defined by a starting point and a direction and a sphere
	private static boolean intersects(Vector3 rayStart, Vector3 rayDir, SphereShape sphere) {
		final float a = rayDir.dot(rayDir);
		final float b = Vector3.multiply(rayDir, 2).dot(rayStart);
		final float r = sphere.getRadius();
		final float c = rayStart.dot(rayStart) - r * r;
		final float discriminant = b * b - 4 * a * c;
		if (discriminant < 0) {
			return false;
		}
		final float discriminantRoot = (float) Math.sqrt(discriminant);
		final float t0 = (-b + discriminantRoot) / (2 * a);
		final float t1 = (-b - discriminantRoot) / (2 * a);
		return Math.max(t0, t1) >= 0;
	}

	// Tests for intersection between a ray defined by a starting point and a direction and a cone
	private static boolean intersects(Vector3 rayStart, Vector3 rayDir, ConeShape cone) {
		final float vx = rayDir.getX();
		final float vy = rayDir.getY();
		final float vz = rayDir.getZ();
		final float px = rayStart.getX();
		final float py = rayStart.getY();
		final float pz = rayStart.getZ();
		final float r = cone.getRadius() / 2;
		final float h = cone.getHeight() / 2;
		final float r2 = r * r;
		final float h2 = h * h;
		final float a = vx * vx + vz * vz - (vy * vy * r2) / h2;
		final float b = 2 * px * vx + 2 * pz * vz - (2 * r2 * py * vy) / h2 + (2 * r2 * vy) / h;
		final float c = px * px + pz * pz - r * r - (r2 * py * py) / h2 + (2 * r2 * py) / h;
		final float discriminant = b * b - 4 * a * c;
		if (discriminant >= 0) {
			final float discriminantRoot = (float) Math.sqrt(discriminant);
			final float t0 = (-b + discriminantRoot) / (2 * a);
			final float t1 = (-b - discriminantRoot) / (2 * a);
			if (Math.max(t0, t1) >= 0) {
				final float ry0 = py + vy * t0;
				final float ry1 = py + vy * t1;
				if (ry0 >= -h && ry0 <= h && ry1 >= -h && ry1 <= h) {
					return true;
				}
			}
		}
		if (vy == 0) {
			return false;
		}
		final float t = (-h - py) / vy;
		if (t < 0) {
			return false;
		}
		final float rx1 = px + vx * t;
		final float rz1 = pz + vz * t;
		return rx1 * rx1 + rz1 * rz1 <= 4 * r * r;
	}

	// Tests for intersection between a ray defined by a starting point and a direction and a cylinder
	private static boolean intersects(Vector3 rayStart, Vector3 rayDir, CylinderShape cylinder) {
		final float vx = rayDir.getX();
		final float vy = rayDir.getY();
		final float vz = rayDir.getZ();
		final float px = rayStart.getX();
		final float py = rayStart.getY();
		final float pz = rayStart.getZ();
		final float r = cylinder.getRadius();
		final float h = cylinder.getHeight() / 2;
		final float r2 = r * r;
		final float a = vx * vx + vz * vz;
		final float b = 2 * px * vx + 2 * pz * vz;
		final float c = px * px + pz * pz - r2;
		final float discriminant = b * b - 4 * a * c;
		if (discriminant >= 0) {
			final float discriminantRoot = (float) Math.sqrt(discriminant);
			final float t0 = (-b + discriminantRoot) / (2 * a);
			final float t1 = (-b - discriminantRoot) / (2 * a);
			if (Math.max(t0, t1) >= 0) {
				final float ry0 = py + vy * t0;
				final float ry1 = py + vy * t1;
				if (ry0 >= -h && ry0 <= h && ry1 >= -h && ry1 <= h) {
					return true;
				}
			}
		}
		if (vy == 0) {
			return false;
		}
		final float t0 = (h - py) / vy;
		if (t0 >= 0) {
			final float rx0 = px + vx * t0;
			final float rz0 = pz + vz * t0;
			if (rx0 * rx0 + rz0 * rz0 <= r2) {
				return true;
			}
		}
		final float t1 = (-h - py) / vy;
		if (t1 < 0) {
			return false;
		}
		final float rx1 = px + vx * t1;
		final float rz1 = pz + vz * t1;
		return rx1 * rx1 + rz1 * rz1 <= r2;
	}
}
