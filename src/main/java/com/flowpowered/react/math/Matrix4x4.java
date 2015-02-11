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
package com.flowpowered.react.math;

import java.util.Arrays;

/**
 * Represents a 4x4 matrix.
 */
public class Matrix4x4 {
    /**
     * The first row of the matrix. Value of 0.
     */
    public static final int FIRST_ROW = 0;
    /**
     * The second row of the matrix. Value of 1.
     */
    public static final int SECOND_ROW = 1;
    /**
     * The third row of the matrix. Value of 2.
     */
    public static final int THIRD_ROW = 2;
    /**
     * The fourth row of the matrix. Value of 3.
     */
    public static final int FOURTH_ROW = 3;
    /**
     * The first column of the matrix. Value of 0.
     */
    public static final int FIRST_COLUMN = 0;
    /**
     * The second column of the matrix. Value of 1.
     */
    public static final int SECOND_COLUMN = 1;
    /**
     * The third column of the matrix. Value of 2.
     */
    public static final int THIRD_COLUMN = 2;
    /**
     * The fourth column of the matrix. Value of 3.
     */
    public static final int FOURTH_COLUMN = 3;
    private final Vector4[] mRows = {
            new Vector4(),
            new Vector4(),
            new Vector4(),
            new Vector4()
    };

    /**
     * Default constructor. All values are 0.
     */
    public Matrix4x4() {
        this(0);
    }

    /**
     * Constructs a new 4x4 matrix with all values set to the provided one.
     *
     * @param value The value to use
     */
    public Matrix4x4(float value) {
        this(
                value, value, value, value,
                value, value, value, value,
                value, value, value, value,
                value, value, value, value);
    }

    /**
     * Copy constructor.
     *
     * @param matrix The matrix to copy
     */
    public Matrix4x4(Matrix4x4 matrix) {
        this(
                matrix.get(0, 0), matrix.get(0, 1), matrix.get(0, 2), matrix.get(0, 3),
                matrix.get(1, 0), matrix.get(1, 1), matrix.get(1, 2), matrix.get(1, 3),
                matrix.get(2, 0), matrix.get(2, 1), matrix.get(2, 2), matrix.get(2, 3),
                matrix.get(3, 0), matrix.get(3, 1), matrix.get(3, 2), matrix.get(3, 3));
    }

    /**
     * Construct a new matrix from all the values.
     *
     * @param a1 The value for 0,0
     * @param a2 The value for 0,1
     * @param a3 The value for 0,2
     * @param a4 The value for 0,3
     * @param b1 The value for 1,0
     * @param b2 The value for 1,1
     * @param b3 The value for 1,2
     * @param b4 The value for 1,3
     * @param c1 The value for 2,0
     * @param c2 The value for 2,1
     * @param c3 The value for 2,2
     * @param c4 The value for 2,3
     * @param d1 The value for 3,0
     * @param d2 The value for 3,1
     * @param d3 The value for 3,2
     * @param d4 The value for 3,3
     */
    public Matrix4x4(float a1, float a2, float a3, float a4,
                     float b1, float b2, float b3, float b4,
                     float c1, float c2, float c3, float c4,
                     float d1, float d2, float d3, float d4) {
        setAllValues(a1, a2, a3, a4, b1, b2, b3, b4, c1, c2, c3, c4, d1, d2, d3, d4);
    }

    /**
     * Sets all the values to the provided ones.
     *
     * @param a1 The value for 0,0
     * @param a2 The value for 0,1
     * @param a3 The value for 0,2
     * @param a4 The value for 0,3
     * @param b1 The value for 1,0
     * @param b2 The value for 1,1
     * @param b3 The value for 1,2
     * @param b4 The value for 1,3
     * @param c1 The value for 2,0
     * @param c2 The value for 2,1
     * @param c3 The value for 2,2
     * @param c4 The value for 2,3
     * @param d1 The value for 3,0
     * @param d2 The value for 3,1
     * @param d3 The value for 3,2
     * @param d4 The value for 3,3
     */
    public final void setAllValues(float a1, float a2, float a3, float a4,
                                   float b1, float b2, float b3, float b4,
                                   float c1, float c2, float c3, float c4,
                                   float d1, float d2, float d3, float d4) {
        mRows[0].setAllValues(a1, a2, a3, a4);
        mRows[1].setAllValues(b1, b2, b3, b4);
        mRows[2].setAllValues(c1, c2, c3, c4);
        mRows[3].setAllValues(d1, d2, d3, d4);
    }

    /**
     * Sets the values of this matrix to those of the provided matrix.
     *
     * @param matrix The matrix to copy the values from
     * @return This matrix for chained calls
     */
    public Matrix4x4 set(Matrix4x4 matrix) {
        setAllValues(
                matrix.get(0, 0), matrix.get(0, 1), matrix.get(0, 2), matrix.get(0, 3),
                matrix.get(1, 0), matrix.get(1, 1), matrix.get(1, 2), matrix.get(1, 3),
                matrix.get(2, 0), matrix.get(2, 1), matrix.get(2, 2), matrix.get(2, 3),
                matrix.get(3, 0), matrix.get(3, 1), matrix.get(3, 2), matrix.get(3, 3));
        return this;
    }

    /**
     * Gets the desired row as a vector4.
     *
     * @param row The row number, either {@link #FIRST_ROW}, {@link #SECOND_ROW}, {@link #THIRD_ROW}, {@link #FOURTH_ROW}
     * @return The vector4 for the row
     */
    public Vector4 get(int row) {
        return mRows[row];
    }

    /**
     * Gets the desired value at the row and column.
     *
     * @param row The row number, either {@link #FIRST_ROW}, {@link #SECOND_ROW}, {@link #THIRD_ROW}, {@link #FOURTH_ROW}
     * @param col The column number, either {@link #FIRST_COLUMN}, {@link #SECOND_COLUMN}, {@link #THIRD_COLUMN}, {@link #FOURTH_COLUMN}
     * @return The value at the row and column
     */
    public float get(int row, int col) {
        return mRows[row].get(col);
    }

    /**
     * Gets the desired column as a vector3.
     *
     * @param col The column number, either {@link #FIRST_COLUMN}, {@link #SECOND_COLUMN}, {@link #THIRD_COLUMN}, {@link #FOURTH_COLUMN}
     * @return The column as a vector3
     */
    public Vector4 getColumn(int col) {
        return new Vector4(mRows[0].get(col), mRows[1].get(col), mRows[2].get(col), mRows[3].get(col));
    }

    /**
     * Gets the desired row as a vector4.
     *
     * @param row The row number, either {@link #FIRST_ROW}, {@link #SECOND_ROW}, {@link #THIRD_ROW}, {@link #FOURTH_ROW}
     * @return The vector4 for the row
     */
    public Vector4 getRow(int row) {
        return mRows[row];
    }

    /**
     * Sets the value at the row and column to the desired value.
     *
     * @param row The row number, either {@link #FIRST_ROW}, {@link #SECOND_ROW}, {@link #THIRD_ROW}, {@link #FOURTH_ROW}
     * @param col The column number, either {@link #FIRST_COLUMN}, {@link #SECOND_COLUMN}, {@link #THIRD_COLUMN}, {@link #FOURTH_COLUMN}
     * @param value The value to set at the row and column
     */
    public void set(int row, int col, float value) {
        mRows[row].set(col, value);
    }

    /**
     * Sets all the values at zero.
     */
    public void setToZero() {
        mRows[0].setToZero();
        mRows[1].setToZero();
        mRows[2].setToZero();
        mRows[3].setToZero();
    }

    /**
     * Sets all the matrix values to identity.
     */
    public void setToIdentity() {
        mRows[0].setAllValues(1, 0, 0, 0);
        mRows[1].setAllValues(0, 1, 0, 0);
        mRows[2].setAllValues(0, 0, 1, 0);
        mRows[3].setAllValues(0, 0, 0, 1);
    }

    /**
     * Transposes the matrix to a new matrix and returns it.
     *
     * @return The new matrix which is the transposed version of this one
     */
    public Matrix4x4 getTranspose() {
        return new Matrix4x4(
                mRows[0].get(0), mRows[1].get(0), mRows[2].get(0), mRows[3].get(0),
                mRows[0].get(1), mRows[1].get(1), mRows[2].get(1), mRows[3].get(1),
                mRows[0].get(2), mRows[1].get(2), mRows[2].get(2), mRows[3].get(2),
                mRows[0].get(3), mRows[1].get(3), mRows[2].get(3), mRows[3].get(3));
    }

    /**
     * Calculates the determinant of this matrix and returns it.
     *
     * @return The determinant of this matrix
     */
    public float getDeterminant() {
        float det = mRows[0].get(0) * (
                (mRows[1].get(1) * mRows[2].get(2) * mRows[3].get(3)
                        + mRows[2].get(1) * mRows[3].get(2) * mRows[1].get(3)
                        + mRows[3].get(1) * mRows[1].get(2) * mRows[2].get(3))
                        - mRows[3].get(1) * mRows[2].get(2) * mRows[1].get(3)
                        - mRows[1].get(1) * mRows[3].get(2) * mRows[2].get(3)
                        - mRows[2].get(1) * mRows[1].get(2) * mRows[3].get(3));
        det -= mRows[1].get(0) * (
                (mRows[0].get(1) * mRows[2].get(2) * mRows[3].get(3)
                        + mRows[2].get(1) * mRows[3].get(2) * mRows[0].get(3)
                        + mRows[3].get(1) * mRows[0].get(2) * mRows[2].get(3))
                        - mRows[3].get(1) * mRows[2].get(2) * mRows[0].get(3)
                        - mRows[0].get(1) * mRows[3].get(2) * mRows[2].get(3)
                        - mRows[2].get(1) * mRows[0].get(2) * mRows[3].get(3));
        det += mRows[2].get(0) * (
                (mRows[0].get(1) * mRows[1].get(2) * mRows[3].get(3)
                        + mRows[1].get(1) * mRows[3].get(2) * mRows[0].get(3)
                        + mRows[3].get(1) * mRows[0].get(2) * mRows[1].get(3))
                        - mRows[3].get(1) * mRows[1].get(2) * mRows[0].get(3)
                        - mRows[0].get(1) * mRows[3].get(2) * mRows[1].get(3)
                        - mRows[1].get(1) * mRows[0].get(2) * mRows[3].get(3));
        det -= mRows[3].get(0) * (
                (mRows[0].get(1) * mRows[1].get(2) * mRows[2].get(3)
                        + mRows[1].get(1) * mRows[2].get(2) * mRows[0].get(3)
                        + mRows[2].get(1) * mRows[0].get(2) * mRows[1].get(3))
                        - mRows[2].get(1) * mRows[1].get(2) * mRows[0].get(3)
                        - mRows[0].get(1) * mRows[2].get(2) * mRows[1].get(3)
                        - mRows[1].get(1) * mRows[0].get(2) * mRows[2].get(3));
        return det;
    }

    /**
     * Calculates the matrix's inverse and returns it.
     *
     * @return The inverse of this matrix
     */
    public Matrix4x4 getInverse() {
        final float determinant = getDeterminant();
        if (determinant != 0) {
            final Matrix4x4 tempMatrix = new Matrix4x4();
            final float t00 = determinant3x3(mRows[1].get(1), mRows[2].get(1), mRows[3].get(1), mRows[1].get(2),
                    mRows[2].get(2), mRows[3].get(2), mRows[1].get(3), mRows[2].get(3), mRows[3].get(3));
            final float t01 = -determinant3x3(mRows[0].get(1), mRows[2].get(1), mRows[3].get(1), mRows[0].get(2),
                    mRows[2].get(2), mRows[3].get(2), mRows[0].get(3), mRows[2].get(3), mRows[3].get(3));
            final float t02 = determinant3x3(mRows[0].get(1), mRows[1].get(1), mRows[3].get(1), mRows[0].get(2),
                    mRows[1].get(2), mRows[3].get(2), mRows[0].get(3), mRows[1].get(3), mRows[3].get(3));
            final float t03 = -determinant3x3(mRows[0].get(1), mRows[1].get(1), mRows[2].get(1), mRows[0].get(2),
                    mRows[1].get(2), mRows[2].get(2), mRows[0].get(3), mRows[1].get(3), mRows[2].get(3));
            final float t10 = -determinant3x3(mRows[1].get(0), mRows[2].get(0), mRows[3].get(0), mRows[1].get(2),
                    mRows[2].get(2), mRows[3].get(2), mRows[1].get(3), mRows[2].get(3), mRows[3].get(3));
            final float t11 = determinant3x3(mRows[0].get(0), mRows[2].get(0), mRows[3].get(0), mRows[0].get(2),
                    mRows[2].get(2), mRows[3].get(2), mRows[0].get(3), mRows[2].get(3), mRows[3].get(3));
            final float t12 = -determinant3x3(mRows[0].get(0), mRows[1].get(0), mRows[3].get(0), mRows[0].get(2),
                    mRows[1].get(2), mRows[3].get(2), mRows[0].get(3), mRows[1].get(3), mRows[3].get(3));
            final float t13 = determinant3x3(mRows[0].get(0), mRows[1].get(0), mRows[2].get(0), mRows[0].get(2),
                    mRows[1].get(2), mRows[2].get(2), mRows[0].get(3), mRows[1].get(3), mRows[2].get(3));
            final float t20 = determinant3x3(mRows[1].get(0), mRows[2].get(0), mRows[3].get(0), mRows[1].get(1),
                    mRows[2].get(1), mRows[3].get(1), mRows[1].get(3), mRows[2].get(3), mRows[3].get(3));
            final float t21 = -determinant3x3(mRows[0].get(0), mRows[2].get(0), mRows[3].get(0), mRows[0].get(1),
                    mRows[2].get(1), mRows[3].get(1), mRows[0].get(3), mRows[2].get(3), mRows[3].get(3));
            final float t22 = determinant3x3(mRows[0].get(0), mRows[1].get(0), mRows[3].get(0), mRows[0].get(1),
                    mRows[1].get(1), mRows[3].get(1), mRows[0].get(3), mRows[1].get(3), mRows[3].get(3));
            final float t23 = -determinant3x3(mRows[0].get(0), mRows[1].get(0), mRows[2].get(0), mRows[0].get(1),
                    mRows[1].get(1), mRows[2].get(1), mRows[0].get(3), mRows[1].get(3), mRows[2].get(3));
            final float t30 = -determinant3x3(mRows[1].get(0), mRows[2].get(0), mRows[3].get(0), mRows[1].get(1),
                    mRows[2].get(1), mRows[3].get(1), mRows[1].get(2), mRows[2].get(2), mRows[3].get(2));
            final float t31 = determinant3x3(mRows[0].get(0), mRows[2].get(0), mRows[3].get(0), mRows[0].get(1),
                    mRows[2].get(1), mRows[3].get(1), mRows[0].get(2), mRows[2].get(2), mRows[3].get(2));
            final float t32 = -determinant3x3(mRows[0].get(0), mRows[1].get(0), mRows[3].get(0), mRows[0].get(1),
                    mRows[1].get(1), mRows[3].get(1), mRows[0].get(2), mRows[1].get(2), mRows[3].get(2));
            final float t33 = determinant3x3(mRows[0].get(0), mRows[1].get(0), mRows[2].get(0), mRows[0].get(1),
                    mRows[1].get(1), mRows[2].get(1), mRows[0].get(2), mRows[1].get(2), mRows[2].get(2));
            final float determinant_inv = 1 / determinant;
            tempMatrix.set(0, 0, t00 * determinant_inv);
            tempMatrix.set(1, 1, t11 * determinant_inv);
            tempMatrix.set(2, 2, t22 * determinant_inv);
            tempMatrix.set(3, 3, t33 * determinant_inv);
            tempMatrix.set(1, 0, t10 * determinant_inv);
            tempMatrix.set(0, 1, t01 * determinant_inv);
            tempMatrix.set(0, 2, t02 * determinant_inv);
            tempMatrix.set(2, 0, t20 * determinant_inv);
            tempMatrix.set(2, 1, t21 * determinant_inv);
            tempMatrix.set(1, 2, t12 * determinant_inv);
            tempMatrix.set(3, 0, t30 * determinant_inv);
            tempMatrix.set(0, 3, t03 * determinant_inv);
            tempMatrix.set(3, 1, t31 * determinant_inv);
            tempMatrix.set(1, 3, t13 * determinant_inv);
            tempMatrix.set(2, 3, t23 * determinant_inv);
            tempMatrix.set(3, 2, t32 * determinant_inv);
            return tempMatrix;
        } else {
            return null;
        }
    }

    /**
     * Calculates the trace of this matrix and returns it.
     *
     * @return The trace of this matrix
     */
    public float getTrace() {
        return mRows[0].get(0) + mRows[1].get(1) + mRows[2].get(2) + mRows[3].get(3);
    }

    /**
     * Creates a matrix where each value is the absolute of the value for this matrix and returns it.
     *
     * @return A new matrix which is the absolute version of this one
     */
    public Matrix4x4 getAbsoluteMatrix() {
        return new Matrix4x4(
                Math.abs(mRows[0].get(0)), Math.abs(mRows[0].get(1)), Math.abs(mRows[0].get(2)), Math.abs(mRows[0].get(3)),
                Math.abs(mRows[1].get(0)), Math.abs(mRows[1].get(1)), Math.abs(mRows[1].get(2)), Math.abs(mRows[1].get(3)),
                Math.abs(mRows[2].get(0)), Math.abs(mRows[2].get(1)), Math.abs(mRows[2].get(2)), Math.abs(mRows[2].get(3)),
                Math.abs(mRows[3].get(0)), Math.abs(mRows[3].get(1)), Math.abs(mRows[3].get(2)), Math.abs(mRows[3].get(3)));
    }

    /**
     * Adds the matrix to this one. Doesn't create a new matrix.
     *
     * @param matrix The matrix to add
     * @return This matrix for chained calls
     */
    public Matrix4x4 add(Matrix4x4 matrix) {
        mRows[0].set(0, mRows[0].get(0) + matrix.get(0, 0));
        mRows[0].set(1, mRows[0].get(1) + matrix.get(0, 1));
        mRows[0].set(2, mRows[0].get(2) + matrix.get(0, 2));
        mRows[0].set(3, mRows[0].get(3) + matrix.get(0, 3));
        mRows[1].set(0, mRows[1].get(0) + matrix.get(1, 0));
        mRows[1].set(1, mRows[1].get(1) + matrix.get(1, 1));
        mRows[1].set(2, mRows[1].get(2) + matrix.get(1, 2));
        mRows[1].set(3, mRows[1].get(3) + matrix.get(1, 3));
        mRows[2].set(0, mRows[2].get(0) + matrix.get(2, 0));
        mRows[2].set(1, mRows[2].get(1) + matrix.get(2, 1));
        mRows[2].set(2, mRows[2].get(2) + matrix.get(2, 2));
        mRows[2].set(3, mRows[2].get(3) + matrix.get(2, 3));
        mRows[3].set(0, mRows[3].get(0) + matrix.get(3, 0));
        mRows[3].set(1, mRows[3].get(1) + matrix.get(3, 1));
        mRows[3].set(2, mRows[3].get(2) + matrix.get(3, 2));
        mRows[3].set(3, mRows[3].get(3) + matrix.get(3, 3));
        return this;
    }

    /**
     * Subtracts the matrix to this one. Doesn't create a new matrix.
     *
     * @param matrix The matrix to subtract
     * @return This matrix for chained calls
     */
    public Matrix4x4 subtract(Matrix4x4 matrix) {
        mRows[0].set(0, mRows[0].get(0) - matrix.get(0, 0));
        mRows[0].set(1, mRows[0].get(1) - matrix.get(0, 1));
        mRows[0].set(2, mRows[0].get(2) - matrix.get(0, 2));
        mRows[0].set(3, mRows[0].get(3) - matrix.get(0, 3));
        mRows[1].set(0, mRows[1].get(0) - matrix.get(1, 0));
        mRows[1].set(1, mRows[1].get(1) - matrix.get(1, 1));
        mRows[1].set(2, mRows[1].get(2) - matrix.get(1, 2));
        mRows[1].set(3, mRows[1].get(3) - matrix.get(1, 3));
        mRows[2].set(0, mRows[2].get(0) - matrix.get(2, 0));
        mRows[2].set(1, mRows[2].get(1) - matrix.get(2, 1));
        mRows[2].set(2, mRows[2].get(2) - matrix.get(2, 2));
        mRows[2].set(3, mRows[2].get(3) - matrix.get(2, 3));
        mRows[3].set(0, mRows[3].get(0) - matrix.get(3, 0));
        mRows[3].set(1, mRows[3].get(1) - matrix.get(3, 1));
        mRows[3].set(2, mRows[3].get(2) - matrix.get(3, 2));
        mRows[3].set(3, mRows[3].get(3) - matrix.get(3, 3));
        return this;
    }

    /**
     * Multiplies the matrix to this one. Doesn't create a new matrix.
     *
     * @param value The value to multiply
     * @return This matrix for chained calls
     */
    public Matrix4x4 multiply(float value) {
        mRows[0].set(0, mRows[0].get(0) * value);
        mRows[0].set(1, mRows[0].get(1) * value);
        mRows[0].set(2, mRows[0].get(2) * value);
        mRows[0].set(3, mRows[0].get(3) * value);
        mRows[1].set(0, mRows[1].get(0) * value);
        mRows[1].set(1, mRows[1].get(1) * value);
        mRows[1].set(2, mRows[1].get(2) * value);
        mRows[1].set(3, mRows[1].get(3) * value);
        mRows[2].set(0, mRows[2].get(0) * value);
        mRows[2].set(1, mRows[2].get(1) * value);
        mRows[2].set(2, mRows[2].get(2) * value);
        mRows[2].set(3, mRows[2].get(3) * value);
        mRows[3].set(0, mRows[3].get(0) * value);
        mRows[3].set(1, mRows[3].get(1) * value);
        mRows[3].set(2, mRows[3].get(2) * value);
        mRows[3].set(3, mRows[3].get(3) * value);
        return this;
    }

    @Override
    public int hashCode() {
        int hash = 7;
        hash = 83 * hash + Arrays.deepHashCode(mRows);
        return hash;
    }

    @Override
    public boolean equals(Object obj) {
        if (!(obj instanceof Matrix4x4)) {
            return false;
        }
        final Matrix4x4 other = (Matrix4x4) obj;
        return Arrays.deepEquals(mRows, other.mRows);
    }

    /**
     * Returns a new identity matrix.
     *
     * @return A new identity matrix
     */
    public static Matrix4x4 identity() {
        return new Matrix4x4(
                1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1);
    }

    /**
     * Adds the two matrices and returns the result as a new matrix.
     *
     * @param matrix1 The first matrix
     * @param matrix2 The second matrix
     * @return The resulting matrix
     */
    public static Matrix4x4 add(Matrix4x4 matrix1, Matrix4x4 matrix2) {
        return new Matrix4x4(
                matrix1.get(0, 0) + matrix2.get(0, 0), matrix1.get(0, 1) + matrix2.get(0, 1),
                matrix1.get(0, 2) + matrix2.get(0, 2), matrix1.get(0, 3) + matrix2.get(0, 3),
                matrix1.get(1, 0) + matrix2.get(1, 0), matrix1.get(1, 1) + matrix2.get(1, 1),
                matrix1.get(1, 2) + matrix2.get(1, 2), matrix1.get(1, 3) + matrix2.get(1, 3),
                matrix1.get(2, 0) + matrix2.get(2, 0), matrix1.get(2, 1) + matrix2.get(2, 1),
                matrix1.get(2, 2) + matrix2.get(2, 2), matrix1.get(2, 3) + matrix2.get(2, 3),
                matrix1.get(3, 0) + matrix2.get(3, 0), matrix1.get(3, 1) + matrix2.get(3, 1),
                matrix1.get(3, 2) + matrix2.get(3, 2), matrix1.get(3, 3) + matrix2.get(3, 3));
    }

    /**
     * Subtracts the two matrices and returns the result as a new matrix.
     *
     * @param matrix1 The first matrix
     * @param matrix2 The second matrix
     * @return The resulting matrix
     */
    public static Matrix4x4 subtract(Matrix4x4 matrix1, Matrix4x4 matrix2) {
        return new Matrix4x4(
                matrix1.get(0, 0) - matrix2.get(0, 0), matrix1.get(0, 1) - matrix2.get(0, 1),
                matrix1.get(0, 2) - matrix2.get(0, 2), matrix1.get(0, 3) - matrix2.get(0, 3),
                matrix1.get(1, 0) - matrix2.get(1, 0), matrix1.get(1, 1) - matrix2.get(1, 1),
                matrix1.get(1, 2) - matrix2.get(1, 2), matrix1.get(1, 3) - matrix2.get(1, 3),
                matrix1.get(2, 0) - matrix2.get(2, 0), matrix1.get(2, 1) - matrix2.get(2, 1),
                matrix1.get(2, 2) - matrix2.get(2, 2), matrix1.get(2, 3) - matrix2.get(2, 3),
                matrix1.get(3, 0) - matrix2.get(3, 0), matrix1.get(3, 1) - matrix2.get(3, 1),
                matrix1.get(3, 2) - matrix2.get(3, 2), matrix1.get(3, 3) - matrix2.get(3, 3));
    }

    /**
     * Negates the matrix and returns the result as a new matrix.
     *
     * @param matrix The matrix to negate
     * @return The negated version of the matrix
     */
    public static Matrix4x4 negate(Matrix4x4 matrix) {
        return new Matrix4x4(
                -matrix.get(0, 0), -matrix.get(0, 1), -matrix.get(0, 2), -matrix.get(0, 3),
                -matrix.get(1, 0), -matrix.get(1, 1), -matrix.get(1, 2), -matrix.get(1, 3),
                -matrix.get(2, 0), -matrix.get(2, 1), -matrix.get(2, 2), -matrix.get(2, 3),
                -matrix.get(3, 0), -matrix.get(3, 1), -matrix.get(3, 2), -matrix.get(3, 3));
    }

    /**
     * Multiplies a float value by a matrix and returns the result as a new matrix.
     *
     * @param value The value
     * @param matrix The matrix
     * @return The resulting matrix
     */
    public static Matrix4x4 multiply(float value, Matrix4x4 matrix) {
        return multiply(matrix, value);
    }

    /**
     * Multiplies a matrix by a float value and returns the result as a new matrix.
     *
     * @param matrix The matrix
     * @param value The value
     * @return The resulting matrix
     */
    public static Matrix4x4 multiply(Matrix4x4 matrix, float value) {
        return new Matrix4x4(
                matrix.get(0, 0) * value, matrix.get(0, 1) * value, matrix.get(0, 2) * value, matrix.get(0, 3) * value,
                matrix.get(1, 0) * value, matrix.get(1, 1) * value, matrix.get(1, 2) * value, matrix.get(1, 3) * value,
                matrix.get(2, 0) * value, matrix.get(2, 1) * value, matrix.get(2, 2) * value, matrix.get(2, 3) * value,
                matrix.get(3, 0) * value, matrix.get(3, 1) * value, matrix.get(3, 2) * value, matrix.get(3, 3) * value);
    }

    /**
     * Multiplies the two matrices and returns the result as a new matrix.
     *
     * @param matrix1 The first matrix
     * @param matrix2 The second matrix
     * @return The resulting matrix
     */
    public static Matrix4x4 multiply(Matrix4x4 matrix1, Matrix4x4 matrix2) {
        final float m00 = matrix1.get(0, 0) * matrix2.get(0, 0) + matrix1.get(0, 1) * matrix2.get(1, 0)
                + matrix1.get(0, 2) * matrix2.get(2, 0) + matrix1.get(0, 3) * matrix2.get(3, 0);
        final float m01 = matrix1.get(1, 0) * matrix2.get(0, 0) + matrix1.get(1, 1) * matrix2.get(1, 0)
                + matrix1.get(1, 2) * matrix2.get(2, 0) + matrix1.get(1, 3) * matrix2.get(3, 0);
        final float m02 = matrix1.get(2, 0) * matrix2.get(0, 0) + matrix1.get(2, 1) * matrix2.get(1, 0)
                + matrix1.get(2, 2) * matrix2.get(2, 0) + matrix1.get(2, 3) * matrix2.get(3, 0);
        final float m03 = matrix1.get(3, 0) * matrix2.get(0, 0) + matrix1.get(3, 1) * matrix2.get(1, 0)
                + matrix1.get(3, 2) * matrix2.get(2, 0) + matrix1.get(3, 3) * matrix2.get(3, 0);
        final float m10 = matrix1.get(0, 0) * matrix2.get(0, 1) + matrix1.get(0, 1) * matrix2.get(1, 1)
                + matrix1.get(0, 2) * matrix2.get(2, 1) + matrix1.get(0, 3) * matrix2.get(3, 1);
        final float m11 = matrix1.get(0, 1) * matrix2.get(1, 0) + matrix1.get(1, 1) * matrix2.get(1, 1)
                + matrix1.get(1, 2) * matrix2.get(2, 1) + matrix1.get(1, 3) * matrix2.get(3, 1);
        final float m12 = matrix1.get(2, 0) * matrix2.get(0, 1) + matrix1.get(2, 1) * matrix2.get(1, 1)
                + matrix1.get(2, 2) * matrix2.get(2, 1) + matrix1.get(2, 3) * matrix2.get(3, 1);
        final float m13 = matrix1.get(3, 0) * matrix2.get(0, 1) + matrix1.get(3, 1) * matrix2.get(1, 1)
                + matrix1.get(3, 2) * matrix2.get(2, 1) + matrix1.get(3, 3) * matrix2.get(3, 1);
        final float m20 = matrix1.get(0, 0) * matrix2.get(0, 2) + matrix1.get(0, 1) * matrix2.get(1, 2)
                + matrix1.get(0, 2) * matrix2.get(2, 2) + matrix1.get(0, 3) * matrix2.get(3, 2);
        final float m21 = matrix1.get(1, 0) * matrix2.get(0, 2) + matrix1.get(1, 1) * matrix2.get(1, 2)
                + matrix1.get(1, 2) * matrix2.get(2, 2) + matrix1.get(1, 3) * matrix2.get(3, 2);
        final float m22 = matrix1.get(2, 0) * matrix2.get(0, 2) + matrix1.get(2, 1) * matrix2.get(1, 2)
                + matrix1.get(2, 2) * matrix2.get(2, 2) + matrix1.get(2, 3) * matrix2.get(3, 2);
        final float m23 = matrix1.get(3, 0) * matrix2.get(0, 2) + matrix1.get(3, 1) * matrix2.get(1, 2)
                + matrix1.get(3, 2) * matrix2.get(2, 2) + matrix1.get(3, 3) * matrix2.get(3, 2);
        final float m30 = matrix1.get(0, 0) * matrix2.get(0, 3) + matrix1.get(0, 1) * matrix2.get(1, 3)
                + matrix1.get(0, 2) * matrix2.get(2, 3) + matrix1.get(0, 3) * matrix2.get(3, 3);
        final float m31 = matrix1.get(1, 0) * matrix2.get(0, 3) + matrix1.get(1, 1) * matrix2.get(1, 3)
                + matrix1.get(1, 2) * matrix2.get(2, 3) + matrix1.get(1, 3) * matrix2.get(3, 3);
        final float m32 = matrix1.get(2, 0) * matrix2.get(0, 3) + matrix1.get(2, 1) * matrix2.get(1, 3)
                + matrix1.get(2, 2) * matrix2.get(2, 3) + matrix1.get(2, 3) * matrix2.get(3, 3);
        final float m33 = matrix1.get(3, 0) * matrix2.get(0, 3) + matrix1.get(3, 1) * matrix2.get(1, 3)
                + matrix1.get(3, 2) * matrix2.get(2, 3) + matrix1.get(3, 3) * matrix2.get(3, 3);
        return new Matrix4x4(
                m00, m10, m20, m30,
                m01, m11, m21, m31,
                m02, m12, m22, m32,
                m03, m13, m23, m33);
    }

    /**
     * Multiplies a matrix by a vector and returns the result as a new vector.
     *
     * @param matrix The matrix
     * @param vector The vector
     * @return The resulting matrix
     */
    public static Vector4 multiply(Matrix4x4 matrix, Vector4 vector) {
        return new Vector4(
                matrix.get(0, 0) * vector.getX() + matrix.get(0, 1) * vector.getY() + matrix.get(0, 2) * vector.getZ() + matrix.get(0, 3) * vector.getW(),
                matrix.get(1, 0) * vector.getX() + matrix.get(1, 1) * vector.getY() + matrix.get(1, 2) * vector.getZ() + matrix.get(1, 3) * vector.getW(),
                matrix.get(2, 0) * vector.getX() + matrix.get(2, 1) * vector.getY() + matrix.get(2, 2) * vector.getZ() + matrix.get(2, 3) * vector.getW(),
                matrix.get(3, 0) * vector.getX() + matrix.get(3, 1) * vector.getY() + matrix.get(3, 2) * vector.getZ() + matrix.get(3, 3) * vector.getW());
    }

    // Returns the determinant of a 3x3 matrix
    private static float determinant3x3(float t00, float t01, float t02,
                                        float t10, float t11, float t12,
                                        float t20, float t21, float t22) {
        return t00 * (t11 * t22 - t12 * t21) + t01 * (t12 * t20 - t10 * t22) + t02 * (t10 * t21 - t11 * t20);
    }
}
