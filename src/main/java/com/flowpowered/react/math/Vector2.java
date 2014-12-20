/*
 * This file is part of Flow React, licensed under the MIT License (MIT).
 *
 * Copyright (c) 2013 Flow Powered <https://flowpowered.com/>
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
package com.flowpowered.react.math;

import com.flowpowered.react.ReactDefaults;

/**
 *
 */
public class Vector2 {
    /**
     * X_AXIS, represents the x axis in the vector. Value of 0
     */
    public static final int X_AXIS = 0;
    /**
     * Y_AXIS, represents the y axis in the vector. Value of 1
     */
    public static final int Y_AXIS = 1;
    private float x;
    private float y;

    /**
     * Default constructor. All values are 0.0F
     */
    public Vector2() {
        this(0, 0);
    }

    /**
     * Copy constructor
     *
     * @param vector to copy
     */
    public Vector2(Vector2 vector) {
        this(vector.getX(), vector.getY());
    }

    /**
     * Constructor with arguments.
     *
     * @param x value
     * @param y value
     */
    public Vector2(float x, float y) {
        setAllValues(x, y);
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
     * Sets all values of the vector
     *
     * @param x value to set
     * @param y value to set
     */
    public final void setAllValues(float x, float y) {
        this.x = x;
        this.y = y;
    }

    /**
     * Sets the values of this vector2 to those of the provided vector2.
     *
     * @param vector The vector2 to copy the values from
     */
    public Vector2 set(Vector2 vector) {
        setAllValues(vector.getX(), vector.getY());
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
     * Sets the x, y and z values to zero.
     */
    public void setToZero() {
        setAllValues(0, 0);
    }

    /**
     * Return the length of the vector
     *
     * @return {@link float} length of the vector
     */
    public float length() {
        return (float) Math.sqrt(x * x + y * y);
    }

    /**
     * Return the square of the length of the vector
     *
     * @return {@link float} square length of the vector
     */
    public float lengthSquare() {
        return x * x + y * y;
    }

    /**
     * Return the axis with the minimal value
     *
     * @return {@link int} axis with minimal value
     */
    public int getMinAxis() {
        return x < y ? X_AXIS : Y_AXIS;
    }

    /**
     * Return the axis with the maximum value
     *
     * @return {@link int} axis with maximum value
     */
    public int getMaxAxis() {
        return x < y ? Y_AXIS : X_AXIS;
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
     * @return new unit {@link Vector2} corresponding to this vector
     */
    public Vector2 getUnit() {
        final float lengthVector = length();
        if (lengthVector <= ReactDefaults.MACHINE_EPSILON) {
            throw new IllegalArgumentException("Cannot normalize the zero vector");
        }
        final float lengthInv = 1 / lengthVector;
        return new Vector2(x * lengthInv, y * lengthInv);
    }

    /**
     * Return an orthogonal vector of this vector
     *
     * @return an orthogonal {@link Vector2} of the current vector
     */
    public Vector2 getOneUnitOrthogonalVector() {
        final float l = length();
        if (l <= ReactDefaults.MACHINE_EPSILON) {
            throw new IllegalArgumentException("Cannot normalize the zero vector");
        }
        return new Vector2(-y / l, x / l);
    }

    /**
     * Normalizes the vector. Doesn't create a new vector.
     *
     * @return This vector after normalization
     */
    public Vector2 normalize() {
        final float l = length();
        if (l <= ReactDefaults.MACHINE_EPSILON) {
            throw new IllegalArgumentException("Cannot normalize the zero vector");
        }
        x /= l;
        y /= l;
        return this;
    }

    /**
     * Return the corresponding absolute value vector. Creates a new vector.
     *
     * @return new {@link Vector2} absolute value vector
     */
    public Vector2 getAbsoluteVector() {
        return new Vector2(
                Math.abs(x),
                Math.abs(y));
    }

    /**
     * Scalar product of two vectors
     *
     * @param vector to compute scalar product with
     * @return {@link float} scalar product
     */
    public float dot(Vector2 vector) {
        return x * vector.getX() + y * vector.getY();
    }

    /**
     * Adds a vector2 to this vector, then returns the result. Does not create a new vector.
     *
     * @param vector to add to this one
     * @return this vector, after addition is finished
     */
    public Vector2 add(Vector2 vector) {
        x += vector.getX();
        y += vector.getY();
        return this;
    }

    /**
     * Negates the components of this vector, then returns the result. Does not create a new vector.
     *
     * @return this vector, after negation is finished
     */
    public Vector2 negate() {
        setAllValues(-x, -y);
        return this;
    }

    /**
     * Subtracts a vector2 from this vector, then returns the result. Does not create a new vector.
     *
     * @param vector to subtract from this one
     * @return the difference of this vector and the other vector
     */
    public Vector2 subtract(Vector2 vector) {
        x -= vector.getX();
        y -= vector.getY();
        return this;
    }

    /**
     * Multiplies this vector by a specified value. Does not create a new vector.
     *
     * @param value to multiply by
     * @return this vector, after multiplication is finished
     */
    public Vector2 multiply(float value) {
        x *= value;
        y *= value;
        return this;
    }

    /**
     * Divides this vector by a specified value. Does not create a new vector.
     *
     * @param value to multiply by
     * @return this vector, after division is finished
     */
    public Vector2 divide(float value) {
        if (value <= ReactDefaults.MACHINE_EPSILON) {
            throw new IllegalArgumentException("Cannot divide by zero");
        }
        x /= value;
        y /= value;
        return this;
    }

    /**
     * Gets the corresponding float value from this vector based on the requested axis.<br><br> <p> Valid axis are:<br> {@link Vector2#X_AXIS}<br> {@link Vector2#Y_AXIS}
     *
     * @param axis to get; {@link Vector2#X_AXIS} OR {@link Vector2#Y_AXIS}
     * @return The value of the axis
     */
    public float get(int axis) {
        switch (axis) {
            case X_AXIS:
                return x;
            case Y_AXIS:
                return y;
        }
        throw new UnsupportedOperationException("Must specify 0 or 1 as an axis. (Vector2.X_AXIS, Vector2.Y_AXIS)");
    }

    /**
     * Sets the corresponding float value from this vector based on the requested axis.<br><br> <p> Valid axis are:<br> {@link Vector2#X_AXIS}<br> {@link Vector2#Y_AXIS}
     *
     * @param axis to set; {@link Vector2#X_AXIS} OR {@link Vector2#Y_AXIS}
     * @param value The value for the axis
     */
    public void set(int axis, float value) {
        switch (axis) {
            case X_AXIS:
                x = value;
                return;
            case Y_AXIS:
                y = value;
                return;
        }
        throw new UnsupportedOperationException("Must specify 0, or 1 as an axis. (Vector2.X_AXIS, Vector2.Y_AXIS)");
    }

    @Override
    public int hashCode() {
        final int prime = 31;
        int result = 1;
        result = prime * result + Float.floatToIntBits(x);
        result = prime * result + Float.floatToIntBits(y);
        return result;
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (!(obj instanceof Vector2)) {
            return false;
        }
        Vector2 other = (Vector2) obj;
        if (Float.floatToIntBits(x) != Float.floatToIntBits(other.x)) {
            return false;
        }
        if (Float.floatToIntBits(y) != Float.floatToIntBits(other.y)) {
            return false;
        }
        return true;
    }

    @Override
    public String toString() {
        return "(" + x + ", " + y + ")";
    }

    /**
     * Adds a vector2 to another vector2. Creates a new vector.
     *
     * @param vector1 the first vector
     * @param vector2 the second vector
     * @return the sum of the two vectors
     */
    public static Vector2 add(Vector2 vector1, Vector2 vector2) {
        return new Vector2(
                vector1.getX() + vector2.getX(),
                vector1.getY() + vector2.getY());
    }

    /**
     * Negates the components of this vector. Creates a new vector.
     *
     * @param vector the vector to negate
     * @return the negative vector for this vector
     */
    public static Vector2 negate(Vector2 vector) {
        return new Vector2(
                -vector.getX(),
                -vector.getY());
    }

    /**
     * Subtracts a vector2 from another vector2. Creates a new vector.
     *
     * @param vector1 the first vector
     * @param vector2 the second vector
     * @return the difference of the two vectors
     */
    public static Vector2 subtract(Vector2 vector1, Vector2 vector2) {
        return new Vector2(
                vector1.getX() - vector2.getX(),
                vector1.getY() - vector2.getY());
    }

    /**
     * Multiplies the value by a specified vector. Creates a new vector.
     *
     * @param value the value
     * @param vector the vector
     * @return the product of the value and the vector
     */
    public static Vector2 multiply(float value, Vector2 vector) {
        return multiply(vector, value);
    }

    /**
     * Multiplies the vector by a specified value. Creates a new vector.
     *
     * @param vector the vector
     * @param value the value
     * @return the product of the vector and the value
     */
    public static Vector2 multiply(Vector2 vector, float value) {
        return new Vector2(
                vector.getX() * value,
                vector.getY() * value);
    }

    /**
     * Divides this vector by a specified value. Creates a new vector.
     *
     * @param vector the vector
     * @param value the value
     * @return the quotient (vector2) of the vector and the value
     */
    public static Vector2 divide(Vector2 vector, float value) {
        if (value <= ReactDefaults.MACHINE_EPSILON) {
            throw new IllegalArgumentException("Cannot divide by zero");
        }
        return new Vector2(
                vector.getX() / value,
                vector.getY() / value);
    }
}
