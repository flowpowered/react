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
import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;

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
	 * by a starting point and a direction. This method returns an {@link IntersectedBody} object
	 * containing the body and the intersection point.
	 *
	 * @param rayStart The ray starting point
	 * @param rayDir The ray direction
	 * @param bodies The bodies to check for intersection
	 * @return The closest body to the ray start and its intersection point
	 */
	public static IntersectedBody findClosestIntersectingBody(Vector3 rayStart, Vector3 rayDir,
															  Collection<CollisionBody> bodies) {
		final Map<CollisionBody, Vector3> intersecting = findIntersectingBodies(rayStart, rayDir, bodies);
		CollisionBody closest = null;
		float closestDistance = Float.MAX_VALUE;
		Vector3 closestIntersectionPoint = null;
		for (Entry<CollisionBody, Vector3> entry : intersecting.entrySet()) {
			final Vector3 intersectionPoint = entry.getValue();
			final float distance = Vector3.subtract(intersectionPoint, rayStart).lengthSquare();
			if (distance < closestDistance) {
				closest = entry.getKey();
				closestDistance = distance;
				closestIntersectionPoint = intersectionPoint;
			}
		}
		return closest != null ? new IntersectedBody(closest, closestIntersectionPoint) : null;
	}

	/**
	 * Finds the furthest of the bodies intersecting with the ray from the ray start. The ray is
	 * defined by a starting point and a direction. This method returns an {@link IntersectedBody}
	 * object containing the body and the intersection point.
	 *
	 * @param rayStart The ray starting point
	 * @param rayDir The ray direction
	 * @param bodies The bodies to check for intersection
	 * @return The furthest body from the ray start and its intersection point
	 */
	public static IntersectedBody findFurthestIntersectingBody(Vector3 rayStart, Vector3 rayDir,
															   Collection<CollisionBody> bodies) {
		final Map<CollisionBody, Vector3> intersecting = findIntersectingBodies(rayStart, rayDir, bodies);
		CollisionBody furthest = null;
		float furthestDistance = Float.MIN_VALUE;
		Vector3 furthestIntersectionPoint = null;
		for (Entry<CollisionBody, Vector3> entry : intersecting.entrySet()) {
			final Vector3 intersectionPoint = entry.getValue();
			final float distance = Vector3.subtract(intersectionPoint, rayStart).lengthSquare();
			if (distance > furthestDistance) {
				furthest = entry.getKey();
				furthestDistance = distance;
				furthestIntersectionPoint = intersectionPoint;
			}
		}
		return furthest != null ? new IntersectedBody(furthest, furthestIntersectionPoint) : null;
	}

	/**
	 * Finds all of the bodies intersecting with the ray. The ray is defined by a starting point and a
	 * direction. The bodies are returned mapped with the closest intersection point.
	 *
	 * @param rayStart The ray starting point
	 * @param rayDir The ray direction
	 * @param bodies The bodies to check for intersection
	 * @return All of the intersection bodies, in no particular order, mapped to the intersection
	 *         point
	 */
	public static Map<CollisionBody, Vector3> findIntersectingBodies(Vector3 rayStart, Vector3 rayDir,
																	 Collection<CollisionBody> bodies) {
		final Map<CollisionBody, Vector3> intersecting = new HashMap<CollisionBody, Vector3>();
		final Vector3 intersectionPoint = new Vector3();
		for (CollisionBody body : bodies) {
			if (intersects(rayStart, rayDir, body.getCollisionShape(), body.getTransform(), intersectionPoint)) {
				intersecting.put(body, new Vector3(intersectionPoint));
			}
		}
		return intersecting;
	}

	// Tests for intersection between a ray defined by a starting point and a direction and a collision shape
	private static boolean intersects(Vector3 rayStart, Vector3 rayDir,
									  CollisionShape shape, Transform transform,
									  Vector3 intersectionPoint) {
		final Transform worldToObject = transform.inverse();
		final Vector3 objRayStart = Transform.multiply(worldToObject, rayStart);
		final Vector3 objRayDir = Matrix3x3.multiply(worldToObject.getOrientation().getMatrix(), rayDir);
		final boolean intersects;
		switch (shape.getType()) {
			case BOX:
				intersects = intersects(objRayStart, objRayDir, (BoxShape) shape, intersectionPoint);
				break;
			case SPHERE:
				intersects = intersects(objRayStart, objRayDir, (SphereShape) shape, intersectionPoint);
				break;
			case CONE:
				intersects = intersects(objRayStart, objRayDir, (ConeShape) shape, intersectionPoint);
				break;
			case CYLINDER:
				intersects = intersects(objRayStart, objRayDir, (CylinderShape) shape, intersectionPoint);
				break;
			default:
				throw new IllegalArgumentException("unknown collision shape");
		}
		if (intersects) {
			intersectionPoint.set(Transform.multiply(transform, intersectionPoint));
			return true;
		}
		return false;
	}

	// Tests for intersection between a ray defined by a starting point and a direction and a box
	private static boolean intersects(Vector3 rayStart, Vector3 rayDir,
									  BoxShape box, Vector3 intersectionPoint) {
		final Vector3 extent = box.getExtent();
		final Vector3 min = Vector3.negate(extent);
		final Vector3 max = extent;
		float t0;
		float t1;
		if (rayDir.getX() >= 0) {
			t0 = (min.getX() - rayStart.getX()) / rayDir.getX();
			t1 = (max.getX() - rayStart.getX()) / rayDir.getX();
		} else {
			t0 = (max.getX() - rayStart.getX()) / rayDir.getX();
			t1 = (min.getX() - rayStart.getX()) / rayDir.getX();
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
		if (t0 > tyMax || tyMin > t1) {
			return false;
		}
		if (tyMin > t0) {
			t0 = tyMin;
		}
		if (tyMax < t1) {
			t1 = tyMax;
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
		if (t0 > tzMax || tzMin > t1) {
			return false;
		}
		if (tzMin > t0) {
			t0 = tzMin;
		}
		if (tzMax < t1) {
			t1 = tzMax;
		}
		if (t1 >= 0) {
			final float t;
			if (t0 >= 0) {
				t = t0;
			} else {
				t = t1;
			}
			intersectionPoint.set(Vector3.add(rayStart, Vector3.multiply(rayDir, t)));
			return true;
		}
		return false;
	}

	// Tests for intersection between a ray defined by a starting point and a direction and a sphere
	private static boolean intersects(Vector3 rayStart, Vector3 rayDir,
									  SphereShape sphere, Vector3 intersectionPoint) {
		final float a = rayDir.dot(rayDir);
		final float b = Vector3.multiply(rayDir, 2).dot(rayStart);
		final float r = sphere.getRadius();
		final float c = rayStart.dot(rayStart) - r * r;
		final float discriminant = b * b - 4 * a * c;
		if (discriminant < 0) {
			return false;
		}
		final float discriminantRoot = (float) Math.sqrt(discriminant);
		float t0 = (-b + discriminantRoot) / (2 * a);
		float t1 = (-b - discriminantRoot) / (2 * a);
		if (t0 > t1) {
			final float temp = t1;
			t1 = t0;
			t0 = temp;
		}
		if (t1 >= 0) {
			final float t;
			if (t0 >= 0) {
				t = t0;
			} else {
				t = t1;
			}
			intersectionPoint.set(Vector3.add(rayStart, Vector3.multiply(rayDir, t)));
			return true;
		}
		return false;
	}

	// Tests for intersection between a ray defined by a starting point and a direction and a cone
	private static boolean intersects(Vector3 rayStart, Vector3 rayDir,
									  ConeShape cone, Vector3 intersectionPoint) {
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
			float t0 = (-b + discriminantRoot) / (2 * a);
			float t1 = (-b - discriminantRoot) / (2 * a);
			if (t0 > t1) {
				final float temp = t1;
				t1 = t0;
				t0 = temp;
			}
			if (t1 >= 0) {
				if (t0 >= 0) {
					final float ry0 = py + vy * t0;
					if (ry0 >= -h && ry0 <= h) {
						intersectionPoint.set(Vector3.add(rayStart, Vector3.multiply(rayDir, t0)));
						return true;
					}
				}
				final float ry1 = py + vy * t1;
				if (ry1 >= -h && ry1 <= h) {
					intersectionPoint.set(Vector3.add(rayStart, Vector3.multiply(rayDir, t1)));
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
		if (rx1 * rx1 + rz1 * rz1 <= 4 * r * r) {
			intersectionPoint.set(Vector3.add(rayStart, Vector3.multiply(rayDir, t)));
			return true;
		}
		return false;
	}

	// Tests for intersection between a ray defined by a starting point and a direction and a cylinder
	private static boolean intersects(Vector3 rayStart, Vector3 rayDir,
									  CylinderShape cylinder, Vector3 intersectionPoint) {
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
			float t0 = (-b + discriminantRoot) / (2 * a);
			float t1 = (-b - discriminantRoot) / (2 * a);
			if (t0 > t1) {
				final float temp = t1;
				t1 = t0;
				t0 = temp;
			}
			if (t1 >= 0) {
				if (t0 >= 0) {
					final float ry0 = py + vy * t0;
					if (ry0 >= -h && ry0 <= h) {
						intersectionPoint.set(Vector3.add(rayStart, Vector3.multiply(rayDir, t0)));
						return true;
					}
				}
				final float ry1 = py + vy * t1;
				if (ry1 >= -h && ry1 <= h) {
					intersectionPoint.set(Vector3.add(rayStart, Vector3.multiply(rayDir, t1)));
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
				intersectionPoint.set(Vector3.add(rayStart, Vector3.multiply(rayDir, t0)));
				return true;
			}
		}
		final float t1 = (-h - py) / vy;
		if (t1 < 0) {
			return false;
		}
		final float rx1 = px + vx * t1;
		final float rz1 = pz + vz * t1;
		if (rx1 * rx1 + rz1 * rz1 <= r2) {
			intersectionPoint.set(Vector3.add(rayStart, Vector3.multiply(rayDir, t1)));
			return true;
		}
		return false;
	}

	/**
	 * Represents a body that was intersected by a ray. This class stores the body and the intersection
	 * point.
	 */
	public static class IntersectedBody {
		private final CollisionBody body;
		private final Vector3 intersectionPoint;

		private IntersectedBody(CollisionBody body, Vector3 intersectionPoint) {
			this.body = body;
			this.intersectionPoint = intersectionPoint;
		}

		/**
		 * Gets the intersected body.
		 *
		 * @return The body
		 */
		public CollisionBody getBody() {
			return body;
		}

		/**
		 * Gets the intersection point in world space.
		 *
		 * @return The intersection point
		 */
		public Vector3 getIntersectionPoint() {
			return intersectionPoint;
		}
	}
}
