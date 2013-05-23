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
 * Represents a 3D vector in space.
 *
 */
public class Vector3 {
	/**
	 * X_AXIS, represents the x axis in the vector. Value of 0
	 */
	public static final int X_AXIS = 0;
	/**
	 * Y_AXIS, represents the y axis in the vector. Value of 1
	 */
	public static final int Y_AXIS = 1;
	/**
	 * Z_AXIS, represents the z axis in the vector. Value of 2
	 */
	public static final int Z_AXIS = 2;
	private float x;
	private float y;
	private float z;

	/**
	 * Default constructor. All values are 0.0F
	 */
	public Vector3() {
		this(0, 0, 0);
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
		this(vector.getX(), vector.getY(), vector.getZ());
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
	 * Sets the values of this vector3 to those of the provided vector3.
	 *
	 * @param vector The vector3 to copy the values from
	 */
	public Vector3 set(Vector3 vector) {
		x = vector.getX();
		y = vector.getY();
		z = vector.getZ();
		return this;
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
	 * Sets the x, y and z values to zero.
	 */
	public void setToZero() {
		x = 0;
		y = 0;
		z = 0;
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
		return Mathematics.approxEquals(lengthSquare(), 1);
	}

	/**
	 * True if the vector is the zero vector
	 *
	 * @return true if the vector is the zero vector
	 */
	public boolean isZero() {
		return Mathematics.approxEquals(lengthSquare(), 0);
	}

	/**
	 * Return the corresponding unit vector. Creates a new vector.
	 *
	 * @return new unit {@link Vector3} corresponding to this vector
	 */
	public Vector3 getUnit() {
		final float lengthVector = length();
		if (Mathematics.approxEquals(lengthVector, 0)) {
			throw new IllegalArgumentException("Cannot normalize the zero vector");
		}
		final float lengthInv = 1 / lengthVector;
		return new Vector3(x * lengthInv, y * lengthInv, z * lengthInv);
	}

	/**
	 * Return an orthogonal vector of this vector
	 *
	 * @return an orthogonal {@link Vector3} of the current vector
	 */
	public Vector3 getOneUnitOrthogonalVector() {
		if (Mathematics.approxEquals(length(), 0)) {
			throw new IllegalArgumentException("Cannot normalize the zero vector");
		}
		final Vector3 vectorAbs = new Vector3(Math.abs(x), Math.abs(y), Math.abs(z));
		final int minElement = vectorAbs.getMinAxis();
		if (minElement == 0) {
			return new Vector3(0, -z, y).divide((float) Math.sqrt(y * y + z * z));
		} else if (minElement == 1) {
			return new Vector3(-z, 0, x).divide((float) Math.sqrt(x * x + z * z));
		} else {
			return new Vector3(-y, x, 0).divide((float) Math.sqrt(x * x + y * y));
		}
	}

	/**
	 * Normalizes the vector. Doesn't create a new vector.
	 *
	 * @return This vector after normalization
	 */
	public Vector3 normalize() {
		final float length = length();
		if (Mathematics.approxEquals(length, 0)) {
			throw new IllegalArgumentException("Cannot normalize the zero vector");
		}
		x /= length;
		y /= length;
		z /= length;
		return this;
	}

	/**
	 * Return the corresponding absolute value vector. Creates a new vector.
	 *
	 * @return new {@link Vector3} absolute value vector
	 */
	public Vector3 getAbsoluteVector() {
		return new Vector3(
				Math.abs(x),
				Math.abs(y),
				Math.abs(z));
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
	 * Crosses a vector3 with this vector. Creates a new vector.
	 *
	 * @param vector to compute the cross product with
	 * @return a new vector, result of the cross product
	 */
	public Vector3 cross(Vector3 vector) {
		return new Vector3(
				y * vector.getZ() - z * vector.getY(),
				z * vector.getX() - x * vector.getZ(),
				x * vector.getY() - y * vector.getX());
	}

	/**
	 * Adds a vector3 to this vector, then returns the result. Does not create a new vector.
	 *
	 * @param vector to add to this one
	 * @return this vector, after addition is finished
	 */
	public Vector3 add(Vector3 vector) {
		x += vector.getX();
		y += vector.getY();
		z += vector.getZ();
		return this;
	}

	/**
	 * Negates the components of this vector, then returns the result. Does not create a new vector.
	 *
	 * @return this vector, after negation is finished
	 */
	public Vector3 negate() {
		x = -x;
		y = -y;
		z = -z;
		return this;
	}

	/**
	 * Subtracts a vector3 from this vector, then returns the result. Does not create a new vector.
	 *
	 * @param vector to subtract from this one
	 * @return the difference of this vector and the other vector
	 */
	public Vector3 subtract(Vector3 vector) {
		x -= vector.getX();
		y -= vector.getY();
		z -= vector.getZ();
		return this;
	}

	/**
	 * Multiplies this vector by a specified value. Does not create a new vector.
	 *
	 * @param value to multiply by
	 * @return this vector, after multiplication is finished
	 */
	public Vector3 multiply(float value) {
		x *= value;
		y *= value;
		z *= value;
		return this;
	}

	/**
	 * Divides this vector by a specified value. Does not create a new vector.
	 *
	 * @param value to multiply by
	 * @return this vector, after division is finished
	 */
	public Vector3 divide(float value) {
		if (Mathematics.approxEquals(value, 0)) {
			throw new IllegalArgumentException("Cannot divide by zero");
		}
		x /= value;
		y /= value;
		z /= value;
		return this;
	}

	/**
	 * Gets the corresponding float value from this vector based on the requested axis.<br><br>
	 *
	 * Valid axis are:<br> {@link Vector3#X_AXIS}<br> {@link Vector3#Y_AXIS}<br>
	 * {@link Vector3#Z_AXIS}<br>
	 *
	 * @param axis to get; {@link Vector3#X_AXIS} OR {@link Vector3#Y_AXIS} OR
	 * {@link Vector3#Z_AXIS}
	 * @return {@link float} value of the axis
	 */
	public float get(int axis) {
		switch (axis) {
			case X_AXIS:
				return x;
			case Y_AXIS:
				return y;
			case Z_AXIS:
				return z;
		}
		throw new UnsupportedOperationException("Must specify 0, 1, or 2 as an axis. (Vector3.X_AXIS, Vector3.Y_AXIS, Vector3.Z_AXIS");
	}

	/**
	 * Sets the corresponding float value from this vector based on the requested axis.<br><br>
	 *
	 * Valid axis are:<br> {@link Vector3#X_AXIS}<br> {@link Vector3#Y_AXIS}<br>
	 * {@link Vector3#Z_AXIS}<br>
	 *
	 * @param axis to set; {@link Vector3#X_AXIS} OR {@link Vector3#Y_AXIS} OR
	 * {@link Vector3#Z_AXIS}
	 * @param value {@link float} value for the axis
	 */
	public float set(int axis, float value) {
		switch (axis) {
			case X_AXIS:
				x = value;
				break;
			case Y_AXIS:
				y = value;
				break;
			case Z_AXIS:
				z = value;
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
		if (!(obj instanceof Vector3)) {
			return false;
		}
		Vector3 other = (Vector3) obj;
		if (Float.floatToIntBits(x) != Float.floatToIntBits(other.x)) {
			return false;
		}
		if (Float.floatToIntBits(y) != Float.floatToIntBits(other.y)) {
			return false;
		}
		if (Float.floatToIntBits(z) != Float.floatToIntBits(other.z)) {
			return false;
		}
		return true;
	}

	/**
	 * Adds a vector3 to another vector3. Creates a new vector.
	 *
	 * @param vector1 the first vector
	 * @param vector2 the second vector
	 * @return the sum of the two vectors
	 */
	public static Vector3 add(Vector3 vector1, Vector3 vector2) {
		return new Vector3(
				vector1.getX() + vector2.getX(),
				vector1.getY() + vector2.getY(),
				vector1.getZ() + vector2.getZ());
	}

	/**
	 * Negates the components of this vector. Creates a new vector.
	 *
	 * @param vector the vector to negate
	 * @return the negative vector for this vector
	 */
	public static Vector3 negate(Vector3 vector) {
		return new Vector3(
				-vector.getX(),
				-vector.getY(),
				-vector.getZ());
	}

	/**
	 * Subtracts a vector3 from another vector3. Creates a new vector.
	 *
	 * @param vector1 the first vector
	 * @param vector2 the second vector
	 * @return the difference of the two vectors
	 */
	public static Vector3 subtract(Vector3 vector1, Vector3 vector2) {
		return new Vector3(
				vector1.getX() - vector2.getX(),
				vector1.getY() - vector2.getY(),
				vector1.getZ() - vector2.getZ());
	}

	/**
	 * Multiplies the vector by a specified value. Creates a new vector.
	 *
	 * @param vector the vector
	 * @param value the value
	 * @return the product of the vector and the value
	 */
	public static Vector3 multiply(Vector3 vector, float value) {
		return new Vector3(
				vector.getX() * value,
				vector.getY() * value,
				vector.getZ() * value);
	}

	/**
	 * Divides this vector by a specified value. Creates a new vector.
	 *
	 * @param vector the vector
	 * @param value the value
	 * @return the quotient (vector3) of the vector and the value
	 */
	public static Vector3 divide(Vector3 vector, float value) {
		if (Mathematics.approxEquals(value, 0)) {
			throw new IllegalArgumentException("Cannot divide by zero");
		}
		return new Vector3(
				vector.getX() / value,
				vector.getY() / value,
				vector.getZ() / value);
	}
}
