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
package org.spout.physics.collision.broadphase;

import gnu.trove.iterator.TObjectIntIterator;
import gnu.trove.list.TIntList;
import gnu.trove.list.array.TIntArrayList;
import gnu.trove.map.TObjectIntMap;
import gnu.trove.map.hash.TObjectIntHashMap;

import org.spout.physics.body.CollisionBody;
import org.spout.physics.collision.CollisionDetection;
import org.spout.physics.collision.shape.AABB;
import org.spout.physics.math.Vector3;

/**
 * This class implements the Sweep-And-Prune (SAP) broad-phase collision detection algorithm. This class implements an array-based version of the algorithm from Pierre Terdiman that is described here:
 * www.codercorner.com/SAP.pdf.
 */
public class SweepAndPruneAlgorithm extends BroadPhaseAlgorithm {
    private static final int INVALID_INDEX = Integer.MAX_VALUE;
    private static final int NB_SENTINELS = 2;
    private BoxAABB[] mBoxes = null;
    private final EndPoint[][] mEndPoints = {null, null, null};
    private int mNbBoxes = 0;
    private int mNbMaxBoxes = 0;
    private final TIntList mFreeBoxIndices = new TIntArrayList();
    private final TObjectIntMap<CollisionBody> mMapBodyToBoxIndex = new TObjectIntHashMap<>();

    /**
     * Constructs a new sweep and prune algorithm from the collision detection it's associated to.
     *
     * @param collisionDetection The collision detection
     */
    public SweepAndPruneAlgorithm(CollisionDetection collisionDetection) {
        super(collisionDetection);
    }

    /**
     * Gets the number of objects managed by this algorithm.
     *
     * @return The number of objects
     */
    public int getNbObjects() {
        return mNbBoxes;
    }

    @Override
    public void addObject(CollisionBody body, AABB aabb) {
        if (body == null) {
            throw new IllegalArgumentException("Attempting to add a null collision body");
        }
        if (aabb == null) {
            throw new IllegalArgumentException("Attempting to add a null AABB");
        }
        final Vector3 extend = Vector3.subtract(aabb.getMax(), aabb.getMin());
        if (extend.getX() < 0 || extend.getY() < 0 || extend.getZ() < 0) {
            throw new IllegalStateException("AABB for body: " + body + " is invalid! AABB is " + aabb);
        }
        final int boxIndex;
        if (mFreeBoxIndices.size() != 0) {
            boxIndex = mFreeBoxIndices.removeAt(mFreeBoxIndices.size() - 1);
        } else {
            if (mNbBoxes == mNbMaxBoxes) {
                resizeArrays();
            }
            boxIndex = mNbBoxes;
        }
        final int indexLimitEndPoint = 2 * mNbBoxes + NB_SENTINELS - 1;
        for (int axis = 0; axis < 3; axis++) {
            final EndPoint maxLimitEndPoint = mEndPoints[axis][indexLimitEndPoint];
            if (mEndPoints[axis][0].getBoxID() != INVALID_INDEX || !mEndPoints[axis][0].isMin()) {
                throw new IllegalStateException("The box ID for the first end point of the axis must" +
                        " be equal to INVALID_INDEX and the end point must be a minimum");
            }
            if (maxLimitEndPoint.getBoxID() != INVALID_INDEX || maxLimitEndPoint.isMin()) {
                throw new IllegalStateException("The box ID for the limit end point of the axis must" +
                        " be equal to INVALID_INDEX and the end point must be a maximum");
            }
            if (mEndPoints[axis][indexLimitEndPoint + 2] == null) {
                mEndPoints[axis][indexLimitEndPoint + 2] = new EndPoint();
            }
            final EndPoint newMaxLimitEndPoint = mEndPoints[axis][indexLimitEndPoint + 2];
            newMaxLimitEndPoint.setValues(maxLimitEndPoint.getBoxID(), maxLimitEndPoint.isMin(),
                    maxLimitEndPoint.getValue());
        }
        if (mBoxes[boxIndex] == null) {
            mBoxes[boxIndex] = new BoxAABB();
        }
        final BoxAABB box = mBoxes[boxIndex];
        box.setBody(body);
        final long maxEndPointValue = encodeFloatIntoInteger(Float.MAX_VALUE) - 1;
        final long minEndPointValue = encodeFloatIntoInteger(Float.MAX_VALUE) - 2;
        for (int axis = 0; axis < 3; axis++) {
            box.getMin()[axis] = indexLimitEndPoint;
            box.getMax()[axis] = indexLimitEndPoint + 1;
            final EndPoint minimumEndPoint = mEndPoints[axis][box.getMin()[axis]];
            minimumEndPoint.setValues(boxIndex, true, minEndPointValue);
            if (mEndPoints[axis][box.getMax()[axis]] == null) {
                mEndPoints[axis][box.getMax()[axis]] = new EndPoint();
            }
            final EndPoint maximumEndPoint = mEndPoints[axis][box.getMax()[axis]];
            maximumEndPoint.setValues(boxIndex, false, maxEndPointValue);
        }
        mMapBodyToBoxIndex.put(body, boxIndex);
        mNbBoxes++;
        updateObject(body, aabb);
    }

    @Override
    public void removeObject(CollisionBody body) {
        final long maxEndPointValue = encodeFloatIntoInteger(Float.MAX_VALUE) - 1;
        final long minEndPointValue = encodeFloatIntoInteger(Float.MAX_VALUE) - 2;
        final AABBInt aabbInt = new AABBInt(minEndPointValue, maxEndPointValue);
        updateObjectIntegerAABB(body, aabbInt);
        final int boxIndex = mMapBodyToBoxIndex.get(body);
        final int indexLimitEndPoint = 2 * mNbBoxes + NB_SENTINELS - 1;
        for (int axis = 0; axis < 3; axis++) {
            final EndPoint maxLimitEndPoint = mEndPoints[axis][indexLimitEndPoint];
            if (mEndPoints[axis][0].getBoxID() != INVALID_INDEX || !mEndPoints[axis][0].isMin()) {
                throw new IllegalStateException("The box ID for the first end point of the axis must" +
                        " be equal to INVALID_INDEX and the end point must be a minimum");
            }
            if (maxLimitEndPoint.getBoxID() != INVALID_INDEX || maxLimitEndPoint.isMin()) {
                throw new IllegalStateException("The box ID for the limit end point of the axis must" +
                        " be equal to INVALID_INDEX and the end point must be a maximum");
            }
            final EndPoint newMaxLimitEndPoint = mEndPoints[axis][indexLimitEndPoint - 2];
            newMaxLimitEndPoint.setValues(maxLimitEndPoint.getBoxID(), maxLimitEndPoint.isMin(), maxLimitEndPoint.getValue());
        }
        mFreeBoxIndices.add(boxIndex);
        mMapBodyToBoxIndex.remove(body);
        mNbBoxes--;
        final int nextPowerOf2 = PairManager.computeNextPowerOfTwo((mNbBoxes - 1) / 100);
        if (nextPowerOf2 * 100 < mNbMaxBoxes) {
            shrinkArrays();
        }
    }

    @Override
    public void updateObject(CollisionBody body, AABB aabb) {
        final AABBInt aabbInt = new AABBInt(aabb);
        updateObjectIntegerAABB(body, aabbInt);
    }

    public void updateObjectIntegerAABB(CollisionBody body, AABBInt aabbInt) {
        final int boxIndex = mMapBodyToBoxIndex.get(body);
        final BoxAABB box = mBoxes[boxIndex];
        for (int axis = 0; axis < 3; axis++) {
            final int otherAxis1 = (1 << axis) & 3;
            final int otherAxis2 = (1 << otherAxis1) & 3;
            final EndPoint[] startEndPointsCurrentAxis = mEndPoints[axis];
            // -------- Update the minimum end-point ------------//
            EndPoint currentMinEndPoint = startEndPointsCurrentAxis[box.getMin()[axis]];
            int currentMinEndPointIndex = box.getMin()[axis];
            if (!currentMinEndPoint.isMin()) {
                throw new IllegalStateException("currentMinEndPoint must be a minimum");
            }
            long limit = aabbInt.getMin()[axis];
            if (limit < currentMinEndPoint.getValue()) {
                final EndPoint savedEndPoint = currentMinEndPoint;
                int indexEndPoint = currentMinEndPointIndex;
                final int savedEndPointIndex = indexEndPoint;
                currentMinEndPoint.setValue(limit);
                while ((currentMinEndPoint = startEndPointsCurrentAxis[--currentMinEndPointIndex]).getValue() > limit) {
                    final BoxAABB id1 = mBoxes[currentMinEndPoint.getBoxID()];
                    final boolean isMin = currentMinEndPoint.isMin();
                    if (!isMin) {
                        if (!box.equals(id1) && (box.getBody().getIsMotionEnabled() || id1.getBody().getIsMotionEnabled())) {
                            if (testIntersect2D(box, id1, otherAxis1, otherAxis2) &&
                                    testIntersect1DSortedAABBs(id1, aabbInt, startEndPointsCurrentAxis, axis)) {
                                mPairManager.addPair(body, id1.getBody());
                            }
                        }
                        id1.getMax()[axis] = indexEndPoint--;
                    } else {
                        id1.getMin()[axis] = indexEndPoint--;
                    }
                    startEndPointsCurrentAxis[currentMinEndPointIndex + 1] = currentMinEndPoint;
                }
                if (savedEndPointIndex != indexEndPoint) {
                    if (savedEndPoint.isMin()) {
                        mBoxes[savedEndPoint.getBoxID()].getMin()[axis] = indexEndPoint;
                    } else {
                        mBoxes[savedEndPoint.getBoxID()].getMax()[axis] = indexEndPoint;
                    }
                    startEndPointsCurrentAxis[indexEndPoint] = savedEndPoint;
                }
            } else if (limit > currentMinEndPoint.getValue()) {
                final EndPoint savedEndPoint = currentMinEndPoint;
                int indexEndPoint = currentMinEndPointIndex;
                final int savedEndPointIndex = indexEndPoint;
                currentMinEndPoint.setValue(limit);
                while ((currentMinEndPoint = startEndPointsCurrentAxis[++currentMinEndPointIndex]).getValue() < limit) {
                    final BoxAABB id1 = mBoxes[currentMinEndPoint.getBoxID()];
                    final boolean isMin = currentMinEndPoint.isMin();
                    if (!isMin) {
                        if (!box.equals(id1) && (box.getBody().getIsMotionEnabled() || id1.getBody().getIsMotionEnabled())) {
                            if (testIntersect2D(box, id1, otherAxis1, otherAxis2)) {
                                mPairManager.removePair(body.getID(), id1.getBody().getID());
                            }
                        }
                        id1.getMax()[axis] = indexEndPoint++;
                    } else {
                        id1.getMin()[axis] = indexEndPoint++;
                    }
                    startEndPointsCurrentAxis[currentMinEndPointIndex - 1] = currentMinEndPoint;
                }
                if (savedEndPointIndex != indexEndPoint) {
                    if (savedEndPoint.isMin()) {
                        mBoxes[savedEndPoint.getBoxID()].getMin()[axis] = indexEndPoint;
                    } else {
                        mBoxes[savedEndPoint.getBoxID()].getMax()[axis] = indexEndPoint;
                    }
                    startEndPointsCurrentAxis[indexEndPoint] = savedEndPoint;
                }
            }
            // ------- Update the maximum end-point ------------ //
            EndPoint currentMaxEndPoint = startEndPointsCurrentAxis[box.getMax()[axis]];
            int currentMaxEndPointIndex = box.getMax()[axis];
            if (currentMaxEndPoint.isMin()) {
                throw new IllegalStateException("currentMinEndPoint must not be a minimum");
            }
            limit = aabbInt.getMax()[axis];
            if (limit > currentMaxEndPoint.getValue()) {
                final EndPoint savedEndPoint = currentMaxEndPoint;
                int indexEndPoint = currentMaxEndPointIndex;
                final int savedEndPointIndex = indexEndPoint;
                currentMaxEndPoint.setValue(limit);
                while ((currentMaxEndPoint = startEndPointsCurrentAxis[++currentMaxEndPointIndex]).getValue() < limit) {
                    final BoxAABB id1 = mBoxes[currentMaxEndPoint.getBoxID()];
                    final boolean isMin = currentMaxEndPoint.isMin();
                    if (isMin) {
                        if (!box.equals(id1) && (box.getBody().getIsMotionEnabled() || id1.getBody().getIsMotionEnabled())) {
                            if (testIntersect2D(box, id1, otherAxis1, otherAxis2) &&
                                    testIntersect1DSortedAABBs(id1, aabbInt, startEndPointsCurrentAxis, axis)) {
                                mPairManager.addPair(body, id1.getBody());
                            }
                        }
                        id1.getMin()[axis] = indexEndPoint++;
                    } else {
                        id1.getMax()[axis] = indexEndPoint++;
                    }
                    startEndPointsCurrentAxis[currentMaxEndPointIndex - 1] = currentMaxEndPoint;
                }
                if (savedEndPointIndex != indexEndPoint) {
                    if (savedEndPoint.isMin()) {
                        mBoxes[savedEndPoint.getBoxID()].getMin()[axis] = indexEndPoint;
                    } else {
                        mBoxes[savedEndPoint.getBoxID()].getMax()[axis] = indexEndPoint;
                    }
                    startEndPointsCurrentAxis[indexEndPoint] = savedEndPoint;
                }
            } else if (limit < currentMaxEndPoint.getValue()) {
                final EndPoint savedEndPoint = currentMaxEndPoint;
                int indexEndPoint = currentMaxEndPointIndex;
                final int savedEndPointIndex = indexEndPoint;
                currentMaxEndPoint.setValue(limit);
                while ((currentMaxEndPoint = startEndPointsCurrentAxis[--currentMaxEndPointIndex]).getValue() > limit) {
                    final BoxAABB id1 = mBoxes[currentMaxEndPoint.getBoxID()];
                    final boolean isMin = currentMaxEndPoint.isMin();
                    if (isMin) {
                        if (!box.equals(id1) && (box.getBody().getIsMotionEnabled() || id1.getBody().getIsMotionEnabled())) {
                            if (testIntersect2D(box, id1, otherAxis1, otherAxis2)) {
                                mPairManager.removePair(body.getID(), id1.getBody().getID());
                            }
                        }
                        id1.getMin()[axis] = indexEndPoint--;
                    } else {
                        id1.getMax()[axis] = indexEndPoint--;
                    }
                    startEndPointsCurrentAxis[currentMaxEndPointIndex + 1] = currentMaxEndPoint;
                }
                if (savedEndPointIndex != indexEndPoint) {
                    if (savedEndPoint.isMin()) {
                        mBoxes[savedEndPoint.getBoxID()].getMin()[axis] = indexEndPoint;
                    } else {
                        mBoxes[savedEndPoint.getBoxID()].getMax()[axis] = indexEndPoint;
                    }
                    startEndPointsCurrentAxis[indexEndPoint] = savedEndPoint;
                }
            }
        }
    }

    // Re-sizes the boxes and end-points arrays when they are full.
    private void resizeArrays() {
        final int newNbMaxBoxes = mNbMaxBoxes != 0 ? 2 * mNbMaxBoxes : 100;
        final int nbEndPoints = mNbBoxes * 2 + NB_SENTINELS;
        final int newNbEndPoints = newNbMaxBoxes * 2 + NB_SENTINELS;
        final BoxAABB[] newBoxesArray = new BoxAABB[newNbMaxBoxes];
        final EndPoint[] newEndPointsXArray = new EndPoint[newNbEndPoints];
        final EndPoint[] newEndPointsYArray = new EndPoint[newNbEndPoints];
        final EndPoint[] newEndPointsZArray = new EndPoint[newNbEndPoints];
        if (mNbBoxes > 0) {
            System.arraycopy(mBoxes, 0, newBoxesArray, 0, mNbBoxes);
            System.arraycopy(mEndPoints[0], 0, newEndPointsXArray, 0, nbEndPoints);
            System.arraycopy(mEndPoints[1], 0, newEndPointsYArray, 0, nbEndPoints);
            System.arraycopy(mEndPoints[2], 0, newEndPointsZArray, 0, nbEndPoints);
        } else {
            final long min = encodeFloatIntoInteger(-Float.MAX_VALUE);
            final long max = encodeFloatIntoInteger(Float.MAX_VALUE);
            newEndPointsXArray[0] = new EndPoint();
            newEndPointsXArray[0].setValues(INVALID_INDEX, true, min);
            newEndPointsXArray[1] = new EndPoint();
            newEndPointsXArray[1].setValues(INVALID_INDEX, false, max);
            newEndPointsYArray[0] = new EndPoint();
            newEndPointsYArray[0].setValues(INVALID_INDEX, true, min);
            newEndPointsYArray[1] = new EndPoint();
            newEndPointsYArray[1].setValues(INVALID_INDEX, false, max);
            newEndPointsZArray[0] = new EndPoint();
            newEndPointsZArray[0].setValues(INVALID_INDEX, true, min);
            newEndPointsZArray[1] = new EndPoint();
            newEndPointsZArray[1].setValues(INVALID_INDEX, false, max);
        }
        mBoxes = newBoxesArray;
        mEndPoints[0] = newEndPointsXArray;
        mEndPoints[1] = newEndPointsYArray;
        mEndPoints[2] = newEndPointsZArray;
        mNbMaxBoxes = newNbMaxBoxes;
    }

    // Shrinks the boxes and end-points arrays when too much memory is allocated.
    private void shrinkArrays() {
        final int nextPowerOf2 = PairManager.computeNextPowerOfTwo((mNbBoxes - 1) / 100);
        final int newNbMaxBoxes = mNbBoxes > 100 ? nextPowerOf2 * 100 : 100;
        final int nbEndPoints = mNbBoxes * 2 + NB_SENTINELS;
        final int newNbEndPoints = newNbMaxBoxes * 2 + NB_SENTINELS;
        if (newNbMaxBoxes >= mNbMaxBoxes) {
            throw new IllegalStateException("The new maximum number of boxes can't be greater or equal to the old one");
        }
        mFreeBoxIndices.sort();
        final TObjectIntMap<CollisionBody> newMapBodyToBoxIndex = new TObjectIntHashMap<>();
        final TObjectIntIterator<CollisionBody> it = mMapBodyToBoxIndex.iterator();
        while (it.hasNext()) {
            it.advance();
            final CollisionBody body = it.key();
            final int boxIndex = it.value();
            if (boxIndex >= mNbBoxes) {
                if (mFreeBoxIndices.isEmpty()) {
                    throw new IllegalStateException("The list of free box indices can't be empty");
                }
                final int newBoxIndex = mFreeBoxIndices.removeAt(0);
                if (newBoxIndex >= mNbBoxes) {
                    throw new IllegalStateException("The new box index can't be greater or equal to number of boxes");
                }
                final BoxAABB oldBox = mBoxes[boxIndex];
                final BoxAABB newBox = mBoxes[newBoxIndex];
                if (oldBox.getBody().getID() != body.getID()) {
                    throw new IllegalStateException("The old box body ID can't be equal to body ID");
                }
                newBox.setBody(oldBox.getBody());
                for (int axis = 0; axis < 3; axis++) {
                    newBox.setMin(axis, oldBox.getMin()[axis]);
                    newBox.setMax(axis, oldBox.getMax()[axis]);
                    final EndPoint minimumEndPoint = mEndPoints[axis][newBox.getMin()[axis]];
                    final EndPoint maximumEndPoint = mEndPoints[axis][newBox.getMax()[axis]];
                    if (minimumEndPoint.getBoxID() != boxIndex) {
                        throw new IllegalStateException("The minimum end point box ID can't be equal to box index");
                    }
                    if (maximumEndPoint.getBoxID() != boxIndex) {
                        throw new IllegalStateException("The maximum end point box ID can't be equal to box index");
                    }
                    minimumEndPoint.setBoxID(newBoxIndex);
                    maximumEndPoint.setBoxID(newBoxIndex);
                }
                newMapBodyToBoxIndex.put(body, newBoxIndex);
            } else {
                newMapBodyToBoxIndex.put(body, boxIndex);
            }
        }
        if (newMapBodyToBoxIndex.size() != mMapBodyToBoxIndex.size()) {
            throw new IllegalStateException("The size of the new map from body to box index must be the same as the old one");
        }
        mMapBodyToBoxIndex.clear();
        mMapBodyToBoxIndex.putAll(newMapBodyToBoxIndex);
        final BoxAABB[] newBoxesArray = new BoxAABB[newNbMaxBoxes];
        final EndPoint[] newEndPointsXArray = new EndPoint[newNbEndPoints];
        final EndPoint[] newEndPointsYArray = new EndPoint[newNbEndPoints];
        final EndPoint[] newEndPointsZArray = new EndPoint[newNbEndPoints];
        System.arraycopy(mBoxes, 0, newBoxesArray, 0, mNbBoxes);
        System.arraycopy(mEndPoints[0], 0, newEndPointsXArray, 0, nbEndPoints);
        System.arraycopy(mEndPoints[1], 0, newEndPointsYArray, 0, nbEndPoints);
        System.arraycopy(mEndPoints[2], 0, newEndPointsZArray, 0, nbEndPoints);
        mBoxes = newBoxesArray;
        mEndPoints[0] = newEndPointsXArray;
        mEndPoints[1] = newEndPointsYArray;
        mEndPoints[2] = newEndPointsZArray;
        mNbMaxBoxes = newNbMaxBoxes;
    }

    // Encodes a floating value into a integer value in order to work with integer
    // comparisons in the Sweep-And-Prune algorithm, for performance.
    // The main issue when encoding a floating number into an integer is keeping
    // the sorting order. This is a problem for negative float numbers.
    // This article describes how to solve this issue: http://www.stereopsis.com/radix.html
    private static long encodeFloatIntoInteger(float number) {
        long intNumber = (long) Float.floatToIntBits(number) & 0xFFFFFFFFl;
        if ((intNumber & 0x80000000l) == 0x80000000l) {
            intNumber = ~intNumber & 0xFFFFFFFFl;
        } else {
            intNumber |= 0x80000000l;
        }
        return intNumber;
    }

    // Checks for the intersection between two boxes that are sorted on the given axis in
    // one dimension. Only one test is necessary here. We know that the minimum of box1 cannot be
    // larger that the maximum of box2 on the axis.
    private static boolean testIntersect1DSortedAABBs(BoxAABB box1, AABBInt box2, EndPoint[] endPointsArray, int axis) {
        return !(endPointsArray[box1.getMax()[axis]].getValue() < box2.getMin()[axis]);
    }

    // Checks for intersection between two boxes in two dimensions. This method is used when
    // we know the that two boxes already overlap on one axis and when we want to test if they
    // also overlap on the two others axes.
    private static boolean testIntersect2D(BoxAABB box1, BoxAABB box2, int axis1, int axis2) {
        return !(box2.getMax()[axis1] < box1.getMin()[axis1] || box1.getMax()[axis1] < box2.getMin()[axis1] ||
                box2.getMax()[axis2] < box1.getMin()[axis2] || box1.getMax()[axis2] < box2.getMin()[axis2]);
    }

    // Represents an end-point of an AABB on one of the three x,y or z axis.
    private static class EndPoint {
        private int boxID;
        private boolean isMin;
        private long value;

        private int getBoxID() {
            return boxID;
        }

        private void setBoxID(int boxID) {
            this.boxID = boxID;
        }

        private boolean isMin() {
            return isMin;
        }

        private long getValue() {
            return value;
        }

        private void setValue(long value) {
            this.value = value;
        }

        private void setValues(int boxID, boolean isMin, long value) {
            this.boxID = boxID;
            this.isMin = isMin;
            this.value = value;
        }

        @Override
        public String toString() {
            return "(" + value + "|" + isMin + "@" + boxID + ")";
        }
    }

    // Represents an AABB in the Sweep-And-Prune algorithm.
    private static class BoxAABB {
        private final int[] min = new int[3];
        private final int[] max = new int[3];
        private CollisionBody body;

        private int[] getMin() {
            return min;
        }

        private void setMin(int i, int v) {
            min[i] = v;
        }

        private int[] getMax() {
            return max;
        }

        private void setMax(int i, int v) {
            max[i] = v;
        }

        private CollisionBody getBody() {
            return body;
        }

        private void setBody(CollisionBody body) {
            this.body = body;
        }

        @Override
        public boolean equals(Object o) {
            if (this == o) {
                return true;
            }
            if (!(o instanceof BoxAABB)) {
                return false;
            }
            final BoxAABB boxAABB = (BoxAABB) o;
            return !(body != null ? !body.equals(boxAABB.getBody()) : boxAABB.getBody() != null);
        }

        @Override
        public int hashCode() {
            return body != null ? body.hashCode() : 0;
        }

        @Override
        public String toString() {
            return "(" + body.getID() + "@" + body.getTransform().getPosition() + ")";
        }
    }

    // Represents an AABB with integer coordinates.
    private static class AABBInt {
        private final long[] min = new long[3];
        private final long[] max = new long[3];

        private long[] getMin() {
            return min;
        }

        private long[] getMax() {
            return max;
        }

        private AABBInt(AABB aabb) {
            min[0] = encodeFloatIntoInteger(aabb.getMin().getX());
            min[1] = encodeFloatIntoInteger(aabb.getMin().getY());
            min[2] = encodeFloatIntoInteger(aabb.getMin().getZ());
            max[0] = encodeFloatIntoInteger(aabb.getMax().getX());
            max[1] = encodeFloatIntoInteger(aabb.getMax().getY());
            max[2] = encodeFloatIntoInteger(aabb.getMax().getZ());
        }

        private AABBInt(long minValue, long maxValue) {
            min[0] = minValue;
            min[1] = minValue;
            min[2] = minValue;
            max[0] = maxValue;
            max[1] = maxValue;
            max[2] = maxValue;
        }
    }
}
