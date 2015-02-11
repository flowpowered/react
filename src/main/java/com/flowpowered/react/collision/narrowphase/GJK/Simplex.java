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
package com.flowpowered.react.collision.narrowphase.GJK;

import com.flowpowered.react.math.Vector3;

/**
 * Represents a simplex which is a set of 3D points. This class is used in the GJK algorithm. This implementation is based on the implementation discussed in the book "Collision Detection in 3D
 * Environments". This class implements the Johnson's algorithm for computing the point of a simplex that is closest to the origin and  the smallest simplex needed to represent that closest point.
 */
public class Simplex {
    private final Vector3[] mPoints = {
            new Vector3(), new Vector3(), new Vector3(), new Vector3()
    };
    private final float[] mPointsLengthSquare = new float[4];
    private float mMaxLengthSquare;
    private final Vector3[] mSuppPointsA = {
            new Vector3(), new Vector3(), new Vector3(), new Vector3()
    };
    private final Vector3[] mSuppPointsB = {
            new Vector3(), new Vector3(), new Vector3(), new Vector3()
    };
    private final Vector3[][] mDiffLength = {
            {new Vector3(), new Vector3(), new Vector3(), new Vector3()},
            {new Vector3(), new Vector3(), new Vector3(), new Vector3()},
            {new Vector3(), new Vector3(), new Vector3(), new Vector3()},
            {new Vector3(), new Vector3(), new Vector3(), new Vector3()}
    };
    private final float[][] mDet = new float[16][4];
    private final float[][] mNormSquare = new float[4][4];
    /// 4 bits that identify the current points of the simplex
    /// For instance, 0101 means that points[1] and points[3] are in the simplex
    private int mBitsCurrentSimplex = 0x0;
    /// Number between 1 and 4 that identify the last found support point
    private int mLastFound;
    /// Position of the last found support point (lastFoundBit = 0x1 << lastFound)
    private int mLastFoundBit;
    /// allBits = bitsCurrentSimplex | lastFoundBit;
    private int mAllBits = 0x0;

    /**
     * Returns true if the simplex contains 4 points, false if not.
     *
     * @return Whether or not the simplex contains 4 points
     */
    public boolean isFull() {
        return mBitsCurrentSimplex == 0xf;
    }

    /**
     * Returns true if the simple is empty, false if not
     *
     * @return Whether or not the simplex is empty
     */
    public boolean isEmpty() {
        return mBitsCurrentSimplex == 0x0;
    }

    /**
     * Gets the maximum squared length of a point.
     *
     * @return The maximum squared length of a point
     */
    public float getMaxLengthSquareOfAPoint() {
        return mMaxLengthSquare;
    }

    /**
     * Adds a new support point of (A-B) in the simplex.
     *
     * @param point The support point of object (A-B) => point = suppPointA - suppPointB
     * @param suppPointA The support point of object A in a direction -v
     * @param suppPointB The support point of object B in a direction v
     */
    public void addPoint(Vector3 point, Vector3 suppPointA, Vector3 suppPointB) {
        if (isFull()) {
            throw new IllegalStateException("simplex is full");
        }
        mLastFound = 0;
        mLastFoundBit = 0x1;
        while (overlap(mBitsCurrentSimplex, mLastFoundBit)) {
            mLastFound++;
            mLastFoundBit <<= 1;
        }
        if (mLastFound < 0 && mLastFound >= 4) {
            throw new IllegalStateException("lastFount must be greater or equal to zero and smaller than four");
        }
        mPoints[mLastFound].set(point);
        mPointsLengthSquare[mLastFound] = point.dot(point);
        mAllBits = mBitsCurrentSimplex | mLastFoundBit;
        updateCache();
        computeDeterminants();
        mSuppPointsA[mLastFound].set(suppPointA);
        mSuppPointsB[mLastFound].set(suppPointB);
    }

    /**
     * Returns true if the point is in the simplex, false if not.
     *
     * @param point The point to verify for presence
     * @return Whether or not the point was found in the simplex
     */
    public boolean isPointInSimplex(Vector3 point) {
        for (int i = 0, bit = 0x1; i < 4; i++, bit <<= 1) {
            if (overlap(mAllBits, bit) && point.equals(mPoints[i])) {
                return true;
            }
        }
        return false;
    }

    // Updates the cached values used during the GJK algorithm.
    private void updateCache() {
        for (int i = 0, bit = 0x1; i < 4; i++, bit <<= 1) {
            if (overlap(mBitsCurrentSimplex, bit)) {
                mDiffLength[i][mLastFound] = Vector3.subtract(mPoints[i], mPoints[mLastFound]);
                mDiffLength[mLastFound][i] = Vector3.negate(mDiffLength[i][mLastFound]);
                mNormSquare[i][mLastFound] = mNormSquare[mLastFound][i] = mDiffLength[i][mLastFound].dot(mDiffLength[i][mLastFound]);
            }
        }
    }

    /**
     * Gets the points of the simplex. These will be stored in the arrays. The returned integer is the number of vertices.
     *
     * @param suppPointsA The support points of object A in a direction -v
     * @param suppPointsB The support points of object B in a direction v
     * @param points The support points of object (A-B) => point = suppPointA - suppPointB
     * @return The number of vertices
     */
    public int getSimplex(Vector3[] suppPointsA, Vector3[] suppPointsB, Vector3[] points) {
        int nbVertices = 0;
        for (int i = 0, bit = 0x1; i < 4; i++, bit <<= 1) {
            if (overlap(mBitsCurrentSimplex, bit)) {
                suppPointsA[nbVertices] = new Vector3(mSuppPointsA[nbVertices]);
                suppPointsB[nbVertices] = new Vector3(mSuppPointsB[nbVertices]);
                points[nbVertices] = new Vector3(mPoints[nbVertices]);
                nbVertices++;
            }
        }
        return nbVertices;
    }

    // Computes the cached determinant values.
    private void computeDeterminants() {
        mDet[mLastFoundBit][mLastFound] = 1;
        if (!isEmpty()) {
            for (int i = 0, bitI = 0x1; i < 4; i++, bitI <<= 1) {
                if (overlap(mBitsCurrentSimplex, bitI)) {
                    final int bit2 = bitI | mLastFoundBit;
                    mDet[bit2][i] = mDiffLength[mLastFound][i].dot(mPoints[mLastFound]);
                    mDet[bit2][mLastFound] = mDiffLength[i][mLastFound].dot(mPoints[i]);
                    for (int j = 0, bitJ = 0x1; j < i; j++, bitJ <<= 1) {
                        if (overlap(mBitsCurrentSimplex, bitJ)) {
                            int k;
                            int bit3 = bitJ | bit2;
                            k = mNormSquare[i][j] < mNormSquare[mLastFound][j] ? i : mLastFound;
                            mDet[bit3][j] = mDet[bit2][i] * mDiffLength[k][j].dot(mPoints[i])
                                    + mDet[bit2][mLastFound] * mDiffLength[k][j].dot(mPoints[mLastFound]);
                            k = mNormSquare[j][i] < mNormSquare[mLastFound][i] ? j : mLastFound;
                            mDet[bit3][i] = mDet[bitJ | mLastFoundBit][j] * mDiffLength[k][i].dot(mPoints[j])
                                    + mDet[bitJ | mLastFoundBit][mLastFound] * mDiffLength[k][i].dot(mPoints[mLastFound]);
                            k = mNormSquare[i][mLastFound] < mNormSquare[j][mLastFound] ? i : j;
                            mDet[bit3][mLastFound] = mDet[bitJ | bitI][j] * mDiffLength[k][mLastFound].dot(mPoints[j])
                                    + mDet[bitJ | bitI][i] * mDiffLength[k][mLastFound].dot(mPoints[i]);
                        }
                    }
                }
            }
            if (mAllBits == 0xf) {
                int k;
                k = mNormSquare[1][0] < mNormSquare[2][0] ?
                        (mNormSquare[1][0] < mNormSquare[3][0] ? 1 : 3) :
                        (mNormSquare[2][0] < mNormSquare[3][0] ? 2 : 3);
                mDet[0xf][0] = mDet[0xe][1] * mDiffLength[k][0].dot(mPoints[1])
                        + mDet[0xe][2] * mDiffLength[k][0].dot(mPoints[2])
                        + mDet[0xe][3] * mDiffLength[k][0].dot(mPoints[3]);
                k = mNormSquare[0][1] < mNormSquare[2][1] ?
                        (mNormSquare[0][1] < mNormSquare[3][1] ? 0 : 3) :
                        (mNormSquare[2][1] < mNormSquare[3][1] ? 2 : 3);
                mDet[0xf][1] = mDet[0xd][0] * mDiffLength[k][1].dot(mPoints[0])
                        + mDet[0xd][2] * mDiffLength[k][1].dot(mPoints[2])
                        + mDet[0xd][3] * mDiffLength[k][1].dot(mPoints[3]);
                k = mNormSquare[0][2] < mNormSquare[1][2] ?
                        (mNormSquare[0][2] < mNormSquare[3][2] ? 0 : 3) :
                        (mNormSquare[1][2] < mNormSquare[3][2] ? 1 : 3);
                mDet[0xf][2] = mDet[0xb][0] * mDiffLength[k][2].dot(mPoints[0])
                        + mDet[0xb][1] * mDiffLength[k][2].dot(mPoints[1])
                        + mDet[0xb][3] * mDiffLength[k][2].dot(mPoints[3]);
                k = mNormSquare[0][3] < mNormSquare[1][3] ?
                        (mNormSquare[0][3] < mNormSquare[2][3] ? 0 : 2) :
                        (mNormSquare[1][3] < mNormSquare[2][3] ? 1 : 2);
                mDet[0xf][3] = mDet[0x7][0] * mDiffLength[k][3].dot(mPoints[0])
                        + mDet[0x7][1] * mDiffLength[k][3].dot(mPoints[1])
                        + mDet[0x7][2] * mDiffLength[k][3].dot(mPoints[2]);
            }
        }
    }

    // Returns true if the subset is a proper subset.
    // A proper subset X is a subset where for all point "y_i" in X, we have detX_i value bigger than zero.
    private boolean isProperSubset(int subset) {
        for (int i = 0, bit = 0x1; i < 4; i++, bit <<= 1) {
            if (overlap(subset, bit) && mDet[subset][i] <= 0) {
                return false;
            }
        }
        return true;
    }

    /**
     * Returns true if the set is affinely dependent. A set if affinely dependent if a point of the set is an affine combination of other points in the set.
     *
     * @return Whether or not the set is affinely dependent
     */
    public boolean isAffinelyDependent() {
        float sum = 0;
        for (int i = 0, bit = 0x1; i < 4; i++, bit <<= 1) {
            if (overlap(mAllBits, bit)) {
                sum += mDet[mAllBits][i];
            }
        }
        return sum <= 0;
    }

    // Returns true if the subset is a valid one for the closest point computation.
    // A subset X is valid if:
    //    1. delta(X)_i > 0 for each "i" in I_x and
    //    2. delta(X U {y_j})_j <= 0 for each "j" not in I_x_
    private boolean isValidSubset(int subset) {
        for (int i = 0, bit = 0x1; i < 4; i++, bit <<= 1) {
            if (overlap(mAllBits, bit)) {
                if (overlap(subset, bit)) {
                    if (mDet[subset][i] <= 0) {
                        return false;
                    }
                } else if (mDet[subset | bit][i] > 0) {
                    return false;
                }
            }
        }
        return true;
    }

    /**
     * Computes the closest points "pA" and "pB" of objects A and B.
     *
     * @param pA sum(lambda_i * a_i), where "a_i" is the support point of object A, with lambda_i = deltaX_i / deltaX
     * @param pB sum(lambda_i * b_i), where "b_i" is the support point of object B, with lambda_i = deltaX_i / deltaX
     */
    public void computeClosestPointsOfAAndB(Vector3 pA, Vector3 pB) {
        float deltaX = 0;
        pA.setAllValues(0, 0, 0);
        pB.setAllValues(0, 0, 0);
        for (int i = 0, bit = 0x1; i < 4; i++, bit <<= 1) {
            if (overlap(mBitsCurrentSimplex, bit)) {
                deltaX += mDet[mBitsCurrentSimplex][i];
                pA.add(Vector3.multiply(mDet[mBitsCurrentSimplex][i], mSuppPointsA[i]));
                pB.add(Vector3.multiply(mDet[mBitsCurrentSimplex][i], mSuppPointsB[i]));
            }
        }
        if (deltaX <= 0) {
            throw new IllegalStateException("deltaX must be greater than zero");
        }
        final float factor = 1 / deltaX;
        pA.multiply(factor);
        pB.multiply(factor);
    }

    /**
     * Compute the closest point "v" to the origin of the current simplex. This method executes the Johnson's algorithm for computing the point "v" of a simplex that is the closest to the origin. The
     * method returns true if a closest point has been found, false if not. The closest point is store in the passed vector, if found.
     *
     * @param v The vector in which to store the closest point
     * @return Whether or not the closest point has been found
     */
    public boolean computeClosestPoint(Vector3 v) {
        for (int subset = mBitsCurrentSimplex; subset != 0x0; subset--) {
            if (isSubset(subset, mBitsCurrentSimplex) && isValidSubset(subset | mLastFoundBit)) {
                mBitsCurrentSimplex = subset | mLastFoundBit;
                v.set(computeClosestPointForSubset(mBitsCurrentSimplex));
                return true;
            }
        }
        if (isValidSubset(mLastFoundBit)) {
            mBitsCurrentSimplex = mLastFoundBit;
            mMaxLengthSquare = mPointsLengthSquare[mLastFound];
            v.set(mPoints[mLastFound]);
            return true;
        }
        return false;
    }

    /**
     * Backups the closest point to the passed vector.
     *
     * @param v The vector in which to store the closest point
     */
    public void backupClosestPointInSimplex(Vector3 v) {
        float minDistSquare = Float.MAX_VALUE;
        for (int bit = mAllBits; bit != 0x0; bit--) {
            if (isSubset(bit, mAllBits) && isProperSubset(bit)) {
                final Vector3 u = computeClosestPointForSubset(bit);
                final float distSquare = u.dot(u);
                if (distSquare < minDistSquare) {
                    minDistSquare = distSquare;
                    mBitsCurrentSimplex = bit;
                    v.set(u);
                }
            }
        }
    }

    // Returns the closest point "v" in the convex hull of the points in the subset represented
    // by the bits "subset".
    private Vector3 computeClosestPointForSubset(int subset) {
        final Vector3 v = new Vector3(0, 0, 0);
        mMaxLengthSquare = 0;
        float deltaX = 0;
        for (int i = 0, bit = 0x1; i < 4; i++, bit <<= 1) {
            if (overlap(subset, bit)) {
                deltaX += mDet[subset][i];
                if (mMaxLengthSquare < mPointsLengthSquare[i]) {
                    mMaxLengthSquare = mPointsLengthSquare[i];
                }
                v.add(Vector3.multiply(mDet[subset][i], mPoints[i]));
            }
        }
        if (deltaX <= 0) {
            throw new IllegalStateException("deltaX must be greater than zero");
        }
        return Vector3.multiply(1 / deltaX, v);
    }

    // Returns true if some bits of "a" overlap with bits of "b".
    private static boolean overlap(int a, int b) {
        return (a & b) != 0x0;
    }

    // Returns true if the bits of "b" is a subset of the bits of "a".
    private static boolean isSubset(int a, int b) {
        return (a & b) == a;
    }
}
