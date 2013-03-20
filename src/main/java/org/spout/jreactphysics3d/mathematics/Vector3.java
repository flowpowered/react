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
package org.spout.jreactphysics3d.mathematics;

/**
 * This class represents a 3D vector in space.
 *
 */
public class Vector3 {
	/**
	 * X_AXIS, represents the x axis in the vector.
	 * Value of 0
	 */
	public static final int X_AXIS = 0;
	/**
	 * Y_AXIS, represents the y axis in the vector.
	 * Value of 1
	 */
	public static final int Y_AXIS = 1;
	/**
	 * Z_AXIS, represents the z axis in the vector.
	 * Value of 2
	 */
	public static final int Z_AXIS = 2;

	private float x;
	private float y;
	private float z;

	/**
	 * Default constructor.
	 * All values are 0.0F
	 */
	public Vector3() {
		x = 0.0F;
		y = 0.0F;
		z = 0.0F;
	}

	/**
	 * Constructor with arguments.
	 * 
	 * @param x value
	 * @param y value
	 * @param z value
	 */
	public Vector3(float x, float y, float z) {
		this.x = x;
		this.y = y;
		this.z = z;
	}

	/**
	 * Copy constructor
	 * 
	 * @param vector to copy
	 */
	public Vector3(Vector3 vector) {
		this.x = vector.getX();
		this.y = vector.getY();
		this.z = vector.getZ();
	}

	/**
	 * Sets the x value of the vector
	 * 
	 * @param x value to set
	 */
	public void setX(float x) {
		this.x = x;
	}

	/**
	 * Sets the y value of the vector
	 * 
	 * @param y value to set
	 */
	public void setY(float y) {
		this.y = y;
	}

	/**
	 * Sets the z value of the vector
	 * 
	 * @param z value to set
	 */
	public void setZ(float z) {
		this.z = z;
	}

	/**
	 * Sets all values of the vector
	 * 
	 * @param x value to set
	 * @param y value to set
	 * @param z value to set
	 */
	public void setAllValues(float x, float y, float z) {
		this.x = x;
		this.y = y;
		this.z = z;
	}

	/**
	 * Gets the x value of the vector
	 * 
	 * @return {@link float} x value
	 */
	public float getX() {
		return x;
	}

	/**
	 * Gets the y value of the vector
	 * 
	 * @return {@link float} y value
	 */
	public float getY() {
		return y;
	}

	/**
	 * Gets the z value of the vector
	 * 
	 * @return {@link float} z value
	 */
	public float getZ() {
		return z;
	}

	/**
	 * Return the length of the vector
	 * 
	 * @return {@link float} length of the vector
	 */
	public float length() {
		return (float) Math.sqrt(x * x + y * y + z * z);
	}

	/**
	 * Return the square of the length of the vector
	 * 
	 * @return {@link float} square length of the vector
	 */
	public float lengthSquare() {
		return x * x + y * y + z * z;
	}

	/**
	 * Scalar product of two vectors
	 * 
	 * @param vector to compute scalar product with
	 * @return {@link float} scalar product
	 */
	public float dot(Vector3 vector) {
		return x * vector.getX() + y * vector.getY() + z * vector.getZ();
	}

	/**
	 * Cross product of two vectors
	 * 
	 * @param vector to computer cross product with
	 * @return new {@link Vector3} cross product
	 */
	public Vector3 cross(Vector3 vector) {
		return new Vector3(this.y * vector.z - this.z * vector.y, this.z * vector.x - this.x * vector.z, this.x * vector.y - this.y * vector.x);
	}

	/**
	 * Return the corresponding absolute value vector
	 * 
	 * @return new {@link Vector3} absolute value vector
	 */
	public Vector3 getAbsoluteVector() {
		return new Vector3(Math.abs(x), Math.abs(y), Math.abs(z));
	}

	/**
	 * Return true if two vectors are parallel
	 * 
	 * @param vector to compare with
	 * @return true if the two vectors are parallel
	 */
	public boolean isParallelWith(Vector3 vector) {
		return Mathematics.approxEquals(dot(vector), length() * vector.length());
	}

	/**
	 * Return the axis with the minimal value
	 * 
	 * @return {@link int} axis with minimal value
	 */
	public int getMinAxis() {
		return (x < y ? (x < z ? X_AXIS : Z_AXIS) : (y < z ? Y_AXIS : Z_AXIS));
	}

	/**
	 * Return the axis with the maximum value
	 * 
	 * @return {@link int} axis with maximum value
	 */
	public int getMaxAxis() {
		return (x < y ? (y < z ? Z_AXIS : Y_AXIS) : (x < z ? Z_AXIS : X_AXIS));
	}

	/**
	 * True if the vector is unit, otherwise false
	 * 
	 * @return true if the vector is unit, otherwise false
	 */
	public boolean isUnit() {
		return Mathematics.approxEquals(lengthSquare(), 1.0F);
	}

	/**
	 * True if the vector is the zero vector
	 * 
	 * @return true if the vector is the zero vector
	 */
	public boolean isZero() {
		return Mathematics.approxEquals(lengthSquare(), 0.0F);
	}

	/**
	 * Return the corresponding unit vector
	 * 
	 * @return new unit {@link Vector3} corresponding to this vector
	 */
	public Vector3 getUnit() {
		if (this.isZero()) {
			throw new UnsupportedOperationException("Unable to get the unit vector for the zero vector");
		}
		final float lengthVector = length();
		final float lengthInv = 1.0F / lengthVector;
		return new Vector3(x * lengthInv, y * lengthInv, z * lengthInv);
	}

	/**
	 * Return an orthogonal vector of this vector
	 * 
	 * @return an orthogonal {@link Vector3} of the current vector
	 */
	public Vector3 getOneOrthogonalVector() {
		if (this.isZero()) {
			throw new UnsupportedOperationException("Unable to get an orthogonal vector for the zero vector");
		}

		Vector3 vector = new Vector3();
		if (!Mathematics.approxEquals(x, 0.0F)) { // if x != 0
			vector.setY(x);
			vector.setZ((-2 * x * y * z + 2 * x * z) / (2 * (z * z + x * x)));
			vector.setX((-x * y - z * vector.z) / x);
			return vector;
		}
		if (!Mathematics.approxEquals(y, 0.0F)) { // if y != 0
			vector.setZ(y);
			vector.setX((-2 * x * y * z + 2 * x * y) / (2 * (y * y + x * x)));
			vector.setY((-z * y - x * vector.x) / y);
			return vector;
		}
		if (!Mathematics.approxEquals(z, 0.0F)) { // if z != 0
			vector.setX(z);
			vector.setY((-2 * x * y * z + 2 * y * z) / (2 * (z * z + y * y)));
			vector.setZ((-x * z - y * vector.y) / z);
			return vector;
		}
		return vector;
	}

	/**
	 * Adds a vector3 to this vector, then returns the result.
	 * Does not create a new vector.
	 * 
	 * @param vector to add to this one
	 * @return this vector, after adding is finished
	 */
	public Vector3 add(Vector3 vector) {
		x += vector.x;
		y += vector.y;
		z += vector.z;
		return this;
	}

	/**
	 * Subtracts a vector3 from this vector, then returns the result.
	 * Does not create a new vector.
	 * 
	 * @param vector to subtract from this one
	 * @return this vector, after subtraction is finished
	 */
	public Vector3 subtract(Vector3 vector) {
		x -= vector.x;
		y -= vector.y;
		z -= vector.z;
		return this;
	}

	/**
	 * Multiplies this vector by a specified amount.
	 * 
	 * @param amount to multiply by
	 * @return this vector, after multiplication is finished
	 */
	public Vector3 multiply(float amount) {
		x *= amount;
		y *= amount;
		z *= amount;
		return this;
	}

	/**
	 * Gets the corresponding float value from this vector based on the requested axis.<br><br>
	 * 
	 * Valid axis are:<br>
	 * {@link Vector3#X_AXIS}<br>
	 * {@link Vector3#Y_AXIS}<br>
	 * {@link Vector3#Z_AXIS}<br>
	 * 
	 * @param axis to get; {@link Vector3#X_AXIS} OR {@link Vector3#Y_AXIS} OR {@link Vector3#Z_AXIS}
	 * @return {@link float} value of the axis
	 */
	public float getAxis(int axis) {
		if (axis == X_AXIS) {
			return x;
		}

		if (axis == Y_AXIS) {
			return y;
		}

		if (axis == Z_AXIS) {
			return z;
		}

		throw new UnsupportedOperationException("Must specify 0, 1, or 2 as an axis. (Vector3.X_AXIS, Vector3.Y_AXIS, Vector3.Z_AXIS");
	}
	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + Float.floatToIntBits(x);
		result = prime * result + Float.floatToIntBits(y);
		result = prime * result + Float.floatToIntBits(z);
		return result;
	}

	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		Vector3 other = (Vector3) obj;
		if (Float.floatToIntBits(x) != Float.floatToIntBits(other.x))
			return false;
		if (Float.floatToIntBits(y) != Float.floatToIntBits(other.y))
			return false;
		if (Float.floatToIntBits(z) != Float.floatToIntBits(other.z))
			return false;
		return true;
	}
}
