/*
 * This file is part of React, licensed under the MIT License (MIT).
 *
 * Copyright (c) 2013 Flow Powered <https://flowpowered.com/>
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://danielchappuis.ch>
 * React is re-licensed with permission from ReactPhysics3D author.
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
 * Represents a 4D vector in space.
 */
public class Vector4 {
    /**
     * X_AXIS, represents the x axis in the vector. Value of 0.
     */
    public static final int X_AXIS = 0;
    /**
     * Y_AXIS, represents the y axis in the vector. Value of 1.
     */
    public static final int Y_AXIS = 1;
    /**
     * Z_AXIS, represents the z axis in the vector. Value of 2.
     */
    public static final int Z_AXIS = 2;
    /**
     * W_AXIS, represents the w axis in the vector. Value of 3.
     */
    public static final int W_AXIS = 3;
    private float x;
    private float y;
    private float z;
    private float w;

    /**
     * Default constructor. All values are 0.
     */
    public Vector4() {
        this(0, 0, 0, 0);
    }

    /**
     * Copy constructor
     *
     * @param vector to copy
     */
    public Vector4(Vector4 vector) {
        this(vector.getX(), vector.getY(), vector.getZ(), vector.getW());
    }

    /**
     * Constructor with arguments.
     *
     * @param x value
     * @param y value
     * @param z value
     * @param w value
     */
    public Vector4(float x, float y, float z, float w) {
        setAllValues(x, y, z, w);
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
     * Sets the w value of the vector
     *
     * @param w value to set
     */
    public void setW(float w) {
        this.w = w;
    }

    /**
     * Sets all values of the vector
     *
     * @param x value to set
     * @param y value to set
     * @param z value to set
     * @param w value to set
     */
    public final void setAllValues(float x, float y, float z, float w) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.w = w;
    }

    /**
     * Sets the values of this vector4 to those of the provided vector4.
     *
     * @param vector The vector4 to copy the values from
     */
    public Vector4 set(Vector4 vector) {
        setAllValues(vector.getX(), vector.getY(), vector.getZ(), vector.getW());
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
     * Gets the w value of the vector
     *
     * @return {@link float} w value
     */
    public float getW() {
        return w;
    }

    /**
     * Sets the x, y, z and w values to zero.
     */
    public void setToZero() {
        setAllValues(0, 0, 0, 0);
    }

    /**
     * Return the length of the vector
     *
     * @return {@link float} length of the vector
     */
    public float length() {
        return (float) Math.sqrt(x * x + y * y + z * z + w * w);
    }

    /**
     * Return the square of the length of the vector
     *
     * @return {@link float} square length of the vector
     */
    public float lengthSquare() {
        return x * x + y * y + z * z + w * w;
    }

    /**
     * Return the axis with the minimal value
     *
     * @return {@link int} axis with minimal value
     */
    public int getMinAxis() {
        float value = x;
        int axis = 0;
        if (y < value) {
            value = y;
            axis = 1;
        }
        if (z < value) {
            value = z;
            axis = 2;
        }
        if (w < value) {
            axis = 3;
        }
        return axis;
    }

    /**
     * Return the axis with the maximum value
     *
     * @return {@link int} axis with maximum value
     */
    public int getMaxAxis() {
        float value = x;
        int axis = 0;
        if (y > value) {
            value = y;
            axis = 1;
        }
        if (z > value) {
            value = z;
            axis = 2;
        }
        if (w > value) {
            axis = 3;
        }
        return axis;
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
     * @return new unit {@link com.flowpowered.react.math.Vector4} corresponding to this vector
     */
    public Vector4 getUnit() {
        final float lengthVector = length();
        if (lengthVector <= ReactDefaults.MACHINE_EPSILON) {
            throw new IllegalArgumentException("Cannot normalize the zero vector");
        }
        final float lengthInv = 1 / lengthVector;
        return new Vector4(x * lengthInv, y * lengthInv, z * lengthInv, w * lengthInv);
    }

    /**
     * Normalizes the vector. Doesn't create a new vector.
     *
     * @return This vector after normalization
     */
    public Vector4 normalize() {
        final float l = length();
        if (l <= ReactDefaults.MACHINE_EPSILON) {
            throw new IllegalArgumentException("Cannot normalize the zero vector");
        }
        x /= l;
        y /= l;
        z /= l;
        w /= l;
        return this;
    }

    /**
     * Return the corresponding absolute value vector. Creates a new vector.
     *
     * @return new {@link com.flowpowered.react.math.Vector4} absolute value vector
     */
    public Vector4 getAbsoluteVector() {
        return new Vector4(
                Math.abs(x),
                Math.abs(y),
                Math.abs(z),
                Math.abs(w));
    }

    /**
     * Scalar product of two vectors
     *
     * @param vector to compute scalar product with
     * @return {@link float} scalar product
     */
    public float dot(Vector4 vector) {
        return x * vector.getX() + y * vector.getY() + z * vector.getZ() + w * vector.getW();
    }

    /**
     * Adds a vector3 to this vector, then returns the result. Does not create a new vector.
     *
     * @param vector to add to this one
     * @return this vector, after addition is finished
     */
    public Vector4 add(Vector4 vector) {
        x += vector.getX();
        y += vector.getY();
        z += vector.getZ();
        w += vector.getW();
        return this;
    }

    /**
     * Negates the components of this vector, then returns the result. Does not create a new vector.
     *
     * @return this vector, after negation is finished
     */
    public Vector4 negate() {
        setAllValues(-x, -y, -z, -w);
        return this;
    }

    /**
     * Subtracts a vector3 from this vector, then returns the result. Does not create a new vector.
     *
     * @param vector to subtract from this one
     * @return the difference of this vector and the other vector
     */
    public Vector4 subtract(Vector4 vector) {
        x -= vector.getX();
        y -= vector.getY();
        z -= vector.getZ();
        w -= vector.getW();
        return this;
    }

    /**
     * Multiplies this vector by a specified value. Does not create a new vector.
     *
     * @param value to multiply by
     * @return this vector, after multiplication is finished
     */
    public Vector4 multiply(float value) {
        x *= value;
        y *= value;
        z *= value;
        w *= value;
        return this;
    }

    /**
     * Divides this vector by a specified value. Does not create a new vector.
     *
     * @param value to multiply by
     * @return this vector, after division is finished
     */
    public Vector4 divide(float value) {
        if (value <= ReactDefaults.MACHINE_EPSILON) {
            throw new IllegalArgumentException("Cannot divide by zero");
        }
        x /= value;
        y /= value;
        z /= value;
        w /= value;
        return this;
    }

    /**
     * Gets the corresponding float value from this vector based on the requested axis.<br><br> <p> Valid axis are:<br> {@link #X_AXIS}<br> {@link #Y_AXIS}<br> {@link #Z_AXIS}<br> {@link #W_AXIS}
     *
     * @param axis to set; {@link #X_AXIS} OR {@link #Y_AXIS} OR {@link #Z_AXIS} OR {@link #W_AXIS}
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
            case W_AXIS:
                return w;
        }
        throw new UnsupportedOperationException("Must specify 0, 1, 2 or 3 as an axis. (Vector3.X_AXIS, Vector3.Y_AXIS, Vector3.Z_AXIS or Vector3.W_AXIS)");
    }

    /**
     * Sets the corresponding float value from this vector based on the requested axis.<br><br> <p> Valid axis are:<br> {@link #X_AXIS}<br> {@link #Y_AXIS}<br> {@link #Z_AXIS}<br> {@link #W_AXIS}
     *
     * @param axis to set; {@link #X_AXIS} OR {@link #Y_AXIS} OR {@link #Z_AXIS} OR {@link #W_AXIS}
     * @param value {@link float} value for the axis
     */
    public void set(int axis, float value) {
        switch (axis) {
            case X_AXIS:
                x = value;
                return;
            case Y_AXIS:
                y = value;
                return;
            case Z_AXIS:
                z = value;
                return;
            case W_AXIS:
                w = value;
                return;
        }
        throw new UnsupportedOperationException("Must specify 0, 1, 2 or 3 as an axis. (Vector3.X_AXIS, Vector3.Y_AXIS, Vector3.Z_AXIS or Vector3.W_AXIS)");
    }

    @Override
    public int hashCode() {
        final int prime = 31;
        int result = 1;
        result = prime * result + Float.floatToIntBits(x);
        result = prime * result + Float.floatToIntBits(y);
        result = prime * result + Float.floatToIntBits(z);
        result = prime * result + Float.floatToIntBits(w);
        return result;
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (!(obj instanceof Vector4)) {
            return false;
        }
        Vector4 other = (Vector4) obj;
        if (Float.floatToIntBits(x) != Float.floatToIntBits(other.x)) {
            return false;
        }
        if (Float.floatToIntBits(y) != Float.floatToIntBits(other.y)) {
            return false;
        }
        if (Float.floatToIntBits(z) != Float.floatToIntBits(other.z)) {
            return false;
        }
        if (Float.floatToIntBits(w) != Float.floatToIntBits(other.w)) {
            return false;
        }
        return true;
    }

    @Override
    public String toString() {
        return "(" + x + ", " + y + ", " + z + ", " + w + ")";
    }

    /**
     * Adds a vector4 to another vector4. Creates a new vector.
     *
     * @param vector1 the first vector
     * @param vector2 the second vector
     * @return the sum of the two vectors
     */
    public static Vector4 add(Vector4 vector1, Vector4 vector2) {
        return new Vector4(
                vector1.getX() + vector2.getX(),
                vector1.getY() + vector2.getY(),
                vector1.getZ() + vector2.getZ(),
                vector1.getW() + vector2.getW());
    }

    /**
     * Negates the components of this vector. Creates a new vector.
     *
     * @param vector the vector to negate
     * @return the negative vector for this vector
     */
    public static Vector4 negate(Vector4 vector) {
        return new Vector4(
                -vector.getX(),
                -vector.getY(),
                -vector.getZ(),
                -vector.getW());
    }

    /**
     * Subtracts a vector4 from another vector4. Creates a new vector.
     *
     * @param vector1 the first vector
     * @param vector2 the second vector
     * @return the difference of the two vectors
     */
    public static Vector4 subtract(Vector4 vector1, Vector4 vector2) {
        return new Vector4(
                vector1.getX() - vector2.getX(),
                vector1.getY() - vector2.getY(),
                vector1.getZ() - vector2.getZ(),
                vector1.getW() - vector2.getW());
    }

    /**
     * Multiplies the value by a specified vector. Creates a new vector.
     *
     * @param value the value
     * @param vector the vector
     * @return the product of the value and the vector
     */
    public static Vector4 multiply(float value, Vector4 vector) {
        return multiply(vector, value);
    }

    /**
     * Multiplies the vector by a specified value. Creates a new vector.
     *
     * @param vector the vector
     * @param value the value
     * @return the product of the vector and the value
     */
    public static Vector4 multiply(Vector4 vector, float value) {
        return new Vector4(
                vector.getX() * value,
                vector.getY() * value,
                vector.getZ() * value,
                vector.getW() * value);
    }

    /**
     * Divides this vector by a specified value. Creates a new vector.
     *
     * @param vector the vector
     * @param value the value
     * @return the quotient (vector4) of the vector and the value
     */
    public static Vector4 divide(Vector4 vector, float value) {
        if (value <= ReactDefaults.MACHINE_EPSILON) {
            throw new IllegalArgumentException("Cannot divide by zero");
        }
        return new Vector4(
                vector.getX() / value,
                vector.getY() / value,
                vector.getZ() / value,
                vector.getW() / value);
    }
}
