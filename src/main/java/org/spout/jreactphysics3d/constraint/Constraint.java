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
package org.spout.jreactphysics3d.constraint;

import java.util.Vector;

import org.spout.jreactphysics3d.body.RigidBody;

/**
 * This is the base class of a constraint in the physics engine. A constraint can be a collision
 * contact or a joint for instance. Each constraint can be made of several auxiliary "mathematical
 * constraints" needed to represent the main constraint.
 */
public class Constraint {
	protected final RigidBody mBody1;
	protected final RigidBody mBody2;
	protected final boolean mActive;
	protected final int mNbConstraints;
	protected final ConstraintType mType;
	protected final Vector<Float> mCachedLambdas = new Vector<Float>();

	/**
	 * Constructs a new constraint from the two bodies, the number of auxiliary constraints, the
	 * activity status and the type of constraint.
	 *
	 * @param body1 The first body
	 * @param body2 The second body
	 * @param nbConstraints The number of auxiliary constraints
	 * @param active True if this constraint is active, false if not
	 * @param type The type of this constraint
	 */
	public Constraint(RigidBody body1, RigidBody body2, int nbConstraints, boolean active, ConstraintType type) {
		mBody1 = body1;
		mBody2 = body2;
		mActive = active;
		mNbConstraints = nbConstraints;
		mType = type;
		for (int i = 0; i < nbConstraints; i++) {
			mCachedLambdas.add(0f);
		}
	}

	/**
	 * Gets the first body.
	 *
	 * @return The first body
	 */
	public RigidBody getBody1() {
		return mBody1;
	}

	/**
	 * Gets the second body.
	 *
	 * @return The second body
	 */
	public RigidBody getBody2() {
		return mBody2;
	}

	/**
	 * Returns true if the constraint is active, false if not.
	 *
	 * @return Whether or not the constraint is active
	 */
	public boolean isActive() {
		return mActive;
	}

	/**
	 * Gets the type of constraint.
	 *
	 * @return The constraint type
	 */
	public ConstraintType getType() {
		return mType;
	}

	/**
	 * Gets the number auxiliary constraints.
	 *
	 * @return The amount of auxiliary constraints
	 */
	public int getNbConstraints() {
		return mNbConstraints;
	}

	/**
	 * Gets the previous lambda value at the desired index.
	 *
	 * @param index The index of the lambda value
	 * @return The lambda value
	 * @throws IllegalArgumentException If the index is greater than the number of constraints, as
	 * defined by {@link #getNbConstraints()};
	 */
	public float getCachedLambda(int index) {
		if (index >= mNbConstraints) {
			throw new IllegalArgumentException("index cannot be greater than nbConstraints");
		}
		return mCachedLambdas.get(index);
	}

	/**
	 * Sets the lambda value at the desired index.
	 *
	 * @param index The index to set the lambda value at
	 * @param lambda The lambda value to set
	 * @throws IllegalArgumentException If the index is greater than the number of constraints, as
	 * defined by {@link #getNbConstraints()};
	 */
	public void setCachedLambda(int index, float lambda) {
		if (index >= mNbConstraints) {
			throw new IllegalArgumentException("index cannot be greater than nbConstraints");
		}
		mCachedLambdas.set(index, lambda);
	}

	/**
	 * An enumeration of the possible constraint types (contact).
	 */
	public static enum ConstraintType {
		CONTACT
	}
}
