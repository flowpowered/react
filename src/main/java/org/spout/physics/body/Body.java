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
package org.spout.physics.body;

/**
 * This is the base class for a body in the physics engine.
 */
public abstract class Body {
	protected final int mID;

    /* A pointer to the owner of the body (if any) */
    private Object userPointer;

	/**
	 * Construct a new body from its ID.
	 *
	 * @param id The body's ID
	 */
	protected Body(int id) {
		mID = id;
	}

	/**
	 * Gets the body's unique ID.
	 *
	 * @return The body's ID
	 */
	public int getID() {
		return mID;
	}

    public Object getUserPointer() {
        return userPointer;
    }

    public void setUserPointer(final Object userPointer) {
        this.userPointer = userPointer;
    }

	/**
	 * Returns true this body's ID is smaller than the other body's ID, false if not.
	 *
	 * @param other The body to compare with
	 * @return True if this body is smaller, false if not
	 */
	public boolean isSmallerThan(Body other) {
		return mID < other.getID();
	}

	/**
	 * Returns true this body's ID is greater than the other body's ID, false if not.
	 *
	 * @param other The body to compare with
	 * @return True if this body is greater, false if not
	 */
	public boolean isGreaterThan(Body other) {
		return mID > other.getID();
	}

	/**
	 * Returns true this body's ID is equal than the other body's ID, false if not.
	 *
	 * @param other The body to compare with
	 * @return True if this body is equal, false if not
	 */
	public boolean isEqualTo(Body other) {
		return mID == other.getID();
	}

	/**
	 * Returns true this body's ID is not equal than the other body's ID, false if not.
	 *
	 * @param other The body to compare with
	 * @return True if this body is not equal, false if not
	 */
	public boolean isNotEqualTo(Body other) {
		return mID != other.getID();
	}
}
