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

import org.junit.Assert;
import org.junit.Test;

import org.spout.physics.body.CollisionBody;
import org.spout.physics.collision.BroadPhasePair;
import org.spout.physics.collision.ContactInfo;
import org.spout.physics.collision.narrowphase.GJK.GJKAlgorithm;
import org.spout.physics.collision.shape.BoxShape;
import org.spout.physics.collision.shape.CollisionShape;
import org.spout.physics.collision.shape.ConeShape;
import org.spout.physics.collision.shape.CylinderShape;
import org.spout.physics.collision.shape.SphereShape;
import org.spout.physics.mathematics.Matrix3x3;
import org.spout.physics.mathematics.Transform;
import org.spout.physics.mathematics.Vector3;

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
				if (!gjkAlgorithm.testCollision(s1, t1, s2, t2, new ContactInfo())) {
					collideFails++;
				}
			} else {
				nonCollideShapes(s1, t1, s2, t2);
				final BroadPhasePair pair = pair(s1, t1, s2, t2);
				gjkAlgorithm.setCurrentOverlappingPair(pair);
				if (gjkAlgorithm.testCollision(s1, t1, s2, t2, new ContactInfo())) {
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
			case BOX: {
				final BoxShape box = (BoxShape) s1;
				final Vector3 extend = box.getExtent();
				in.setAllValues(
						RANDOM.nextInt((int) extend.getX() * 2 + 1) - extend.getX(),
						RANDOM.nextInt((int) extend.getY() * 2 + 1) - extend.getY(),
						RANDOM.nextInt((int) extend.getZ() * 2 + 1) - extend.getZ());
				break;
			}
			case CONE: {
				final ConeShape cone = (ConeShape) s1;
				final float height = cone.getHeight();
				final float y = RANDOM.nextInt((int) height + 1) - height / 2;
				final float r = RANDOM.nextFloat() * (height - y - height / 2) * (cone.getRadius() / height);
				final float x = r * (float) Math.cos(RANDOM.nextDouble() * 2 * Math.PI);
				final float z = r * (float) Math.sin(RANDOM.nextDouble() * 2 * Math.PI);
				in.setAllValues(x, y, z);
				break;
			}
			case CYLINDER: {
				final CylinderShape cylinder = (CylinderShape) s1;
				final float r = RANDOM.nextFloat() * cylinder.getRadius();
				in.setAllValues(
						r * (float) Math.cos(RANDOM.nextDouble() * 2 * Math.PI),
						RANDOM.nextInt((int) cylinder.getHeight() + 1) - cylinder.getHeight() / 2,
						r * (float) Math.sin(RANDOM.nextDouble() * 2 * Math.PI));
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
			default:
				throw new IllegalStateException("Unknown collision shape for s1");
		}
		final Vector3 surface = new Vector3();
		switch (s2.getType()) {
			case BOX: {
				final BoxShape box = (BoxShape) s2;
				final Vector3 extend = box.getExtent();
				switch (RANDOM.nextInt(3)) {
					case 0: {
						surface.setAllValues(
								RANDOM.nextBoolean() ? extend.getX() : -extend.getX(),
								RANDOM.nextInt((int) extend.getY() * 2 + 1) - extend.getY(),
								RANDOM.nextInt((int) extend.getZ() * 2 + 1) - extend.getZ());
					}
					case 1: {
						surface.setAllValues(
								RANDOM.nextInt((int) extend.getX() * 2 + 1) - extend.getX(),
								RANDOM.nextBoolean() ? extend.getY() : -extend.getY(),
								RANDOM.nextInt((int) extend.getZ() * 2 + 1) - extend.getZ());
					}
					case 2: {
						surface.setAllValues(
								RANDOM.nextInt((int) extend.getX() * 2 + 1) - extend.getX(),
								RANDOM.nextInt((int) extend.getY() * 2 + 1) - extend.getY(),
								RANDOM.nextBoolean() ? extend.getZ() : -extend.getZ());
					}
				}
				break;
			}
			case CONE: {
				final ConeShape cone = (ConeShape) s2;
				final float height = cone.getHeight();
				final float y = RANDOM.nextInt((int) height + 1) - height / 2;
				final float r;
				if (y == -height / 2) {
					r = RANDOM.nextFloat();
				} else {
					r = (height - y - height / 2) * (cone.getRadius() / height);
				}
				final float x = r * (float) Math.cos(RANDOM.nextDouble() * 2 * Math.PI);
				final float z = r * (float) Math.sin(RANDOM.nextDouble() * 2 * Math.PI);
				surface.setAllValues(x, y, z);
				break;
			}
			case CYLINDER: {
				final CylinderShape cylinder = (CylinderShape) s2;
				final float y = RANDOM.nextInt((int) cylinder.getHeight() + 1) - cylinder.getHeight() / 2;
				final float r = Math.abs(y) == cylinder.getHeight() / 2 ? RANDOM.nextFloat() : cylinder.getRadius();
				final float x = r * (float) Math.cos(RANDOM.nextDouble() * 2 * Math.PI);
				final float z = r * (float) Math.sin(RANDOM.nextDouble() * 2 * Math.PI);
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
			case BOX: {
				BoxShape box = (BoxShape) s1;
				radius1 = box.getExtent().length();
				break;
			}
			case CONE: {
				ConeShape cone = (ConeShape) s1;
				final float h = cone.getHeight() / 2;
				final float r = cone.getRadius();
				radius1 = (float) Math.sqrt(h * h + r * r);
				break;
			}
			case CYLINDER: {
				CylinderShape cylinder = (CylinderShape) s1;
				final float h = cylinder.getHeight() / 2;
				final float r = cylinder.getRadius();
				radius1 = (float) Math.sqrt(h * h + r * r);
				break;
			}
			case SPHERE: {
				SphereShape sphere = (SphereShape) s1;
				radius1 = sphere.getRadius();
				break;
			}
			default:
				throw new IllegalStateException("Unknown collision shape for s1");
		}
		final float radius2;
		switch (s2.getType()) {
			case BOX: {
				BoxShape box = (BoxShape) s2;
				radius2 = box.getExtent().length();
				break;
			}
			case CONE: {
				ConeShape cone = (ConeShape) s2;
				final float h = cone.getHeight() / 2;
				final float r = cone.getRadius();
				radius2 = (float) Math.sqrt(h * h + r * r);
				break;
			}
			case CYLINDER: {
				CylinderShape cylinder = (CylinderShape) s2;
				final float h = cylinder.getHeight() / 2;
				final float r = cylinder.getRadius();
				radius2 = (float) Math.sqrt(h * h + r * r);
				break;
			}
			case SPHERE: {
				SphereShape sphere = (SphereShape) s2;
				radius2 = sphere.getRadius();
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
		final CollisionBody b1 = new CollisionBody(t1, s1, ID++);
		final CollisionBody b2 = new CollisionBody(t2, s2, ID++);
		b1.updateAABB();
		b2.updateAABB();
		return new BroadPhasePair(b1, b2);
	}
}
