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
package org.spout.jreactphysics3d.collision;

import org.spout.jreactphysics3d.mathematics.Vector3;

/**
 * This class contains information about a collision contact computed during the narrow-phase
 * collision detection. This information is used to compute the contact set for a contact between
 * two bodies. The information fields are immutable.
 */
public class ContactInfo {
	private final Vector3 normal;
	private float penetrationDepth;
	private final Vector3 localPoint1;
	private final Vector3 localPoint2;

	/**
	 * Default constructor. Sets all the fields.
	 *
	 * @param normal The normal vector for the collision contact in world space
	 * @param penetrationDepth Penetration depth of the contact
	 * @param localPoint1 Contact point of body 1 in local space of body 1
	 * @param localPoint2 Contact point of body 2 in local space of body 2
	 */
	public ContactInfo(Vector3 normal, float penetrationDepth, Vector3 localPoint1, Vector3 localPoint2) {
		this.normal = normal;
		this.penetrationDepth = penetrationDepth;
		this.localPoint1 = localPoint1;
		this.localPoint2 = localPoint2;
	}

	/**
	 * Gets the normal vector for the collision contact in world space.
	 *
	 * @return The normal vector
	 */
	public Vector3 getNormal() {
		return normal;
	}

	/**
	 * Sets the normal vector for the collision contact in world space.
	 *
	 * @param normal The normal vector to set
	 */
	public void setNormal(Vector3 normal) {
		this.normal.set(normal);
	}

	/**
	 * Gets the penetration depth of the contact.
	 *
	 * @return The penetration depth
	 */
	public float getPenetrationDepth() {
		return penetrationDepth;
	}

	/**
	 * Sets the penetration depth of the contact.
	 *
	 * @param penetrationDepth The penetration depth to set
	 */
	public void setPenetrationDepth(float penetrationDepth) {
		this.penetrationDepth = penetrationDepth;
	}

	/**
	 * Gets the contact point of the first body in local space of the first body.
	 *
	 * @return The local point of contact for the first body
	 */
	public Vector3 getLocalPoint1() {
		return localPoint1;
	}

	/**
	 * Sets the contact point of the first body in local space of the first body.
	 *
	 * @param localPoint1 The contact point to set for the first body
	 */
	public void setLocalPoint1(Vector3 localPoint1) {
		this.localPoint1.set(localPoint1);
	}

	/**
	 * Gets the contact point of second body body in local space of second body body.
	 *
	 * @return The local point of contact for second body body
	 */
	public Vector3 getLocalPoint2() {
		return localPoint2;
	}

	/**
	 * Sets the contact point of the second body in local space of the second body.
	 *
	 * @param localPoint2 The contact point to set for second body body
	 */
	public void setLocalPoint2(Vector3 localPoint2) {
		this.localPoint2.set(localPoint2);
	}

	/**
	 * Sets all the values of this contact info.
	 *
	 * @param normal The normal vector to set
	 * @param penetrationDepth The penetration depth to set
	 * @param localPoint1 The contact point to set for the first body
	 * @param localPoint2 The contact point to set for second body body
	 */
	public void set(Vector3 normal, float penetrationDepth, Vector3 localPoint1, Vector3 localPoint2) {
		setNormal(normal);
		setPenetrationDepth(penetrationDepth);
		setLocalPoint1(localPoint1);
		setLocalPoint2(localPoint2);
	}
}
