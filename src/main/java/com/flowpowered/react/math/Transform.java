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

/**
 * Represents a position and an orientation in 3D. It can also be seen as representing a translation and a rotation.
 */
public class Transform {
    private final Vector3 mPosition = new Vector3();
    private final Quaternion mOrientation = new Quaternion();

    /**
     * Default constructor. Position will be the zero vector and the rotation, quaternion identity.
     */
    public Transform() {
        this(new Vector3(0, 0, 0), Quaternion.identity());
    }

    /**
     * Constructs a new transform from the position as a vector3 and the orientation as a 3x3 matrix.
     *
     * @param position The position
     * @param orientation The orientation
     */
    public Transform(Vector3 position, Matrix3x3 orientation) {
        this(position, new Quaternion(orientation));
    }

    /**
     * Constructs a new transform from the position as a vector3 and the orientation as a quaternion.
     *
     * @param position The position
     * @param orientation The orientation
     */
    public Transform(Vector3 position, Quaternion orientation) {
        this.mPosition.set(position);
        this.mOrientation.set(orientation);
    }

    /**
     * Copy constructor.
     *
     * @param transform The transform to copy
     */
    public Transform(Transform transform) {
        this(transform.getPosition(), transform.getOrientation());
    }

    /**
     * Gets the position component of this transform as a vector3.
     *
     * @return The position
     */
    public Vector3 getPosition() {
        return mPosition;
    }

    /**
     * Gets the orientation component of this transform as a quaternion.
     *
     * @return The orientation
     */
    public Quaternion getOrientation() {
        return mOrientation;
    }

    /**
     * Sets the position component of this transform to the desired vector3.
     *
     * @param position The position to set
     */
    public void setPosition(Vector3 position) {
        mPosition.set(position);
    }

    /**
     * Sets the orientation component of this transform to the desired quaternion.
     *
     * @param orientation The position to set
     */
    public void setOrientation(Quaternion orientation) {
        mOrientation.set(orientation);
    }

    /**
     * Sets the position and orientation of this transform to those of the provided transform.
     *
     * @param transform The transform to copy the position and orientation from
     */
    public Transform set(Transform transform) {
        mPosition.set(transform.getPosition());
        mOrientation.set(transform.getOrientation());
        return this;
    }

    /**
     * Sets this transform to identity. The vector is set to the zero vector, and the quaternion, to the identity quaternion.
     */
    public void setToIdentity() {
        mPosition.set(new Vector3(0, 0, 0));
        mOrientation.set(Quaternion.identity());
    }

    /**
     * Inverses the rotation and position of this transform and returns it as a new one.
     *
     * @return The transform which is the inverse of this one
     */
    public Transform getInverse() {
        final Quaternion invQuaternion = mOrientation.getInverse();
        final Matrix3x3 invMatrix = invQuaternion.getMatrix();
        return new Transform(Matrix3x3.multiply(invMatrix, Vector3.negate(mPosition)), invQuaternion);
    }

    @Override
    public int hashCode() {
        int hash = 3;
        hash = 11 * hash + mPosition.hashCode();
        hash = 11 * hash + mOrientation.hashCode();
        return hash;
    }

    @Override
    public boolean equals(Object obj) {
        if (!(obj instanceof Transform)) {
            return false;
        }
        final Transform other = (Transform) obj;
        if (mPosition != other.mPosition && !mPosition.equals(other.mPosition)) {
            return false;
        }
        if (mOrientation != other.mOrientation && !mOrientation.equals(other.mOrientation)) {
            return false;
        }
        return true;
    }

    @Override
    public String toString() {
        return "Transform{position= " + mPosition + ", orientation= " + mOrientation + "}";
    }

    /**
     * Returns a new identity transform. That is, a transform with the position as the zero vector and the orientation as the identity quaternion.
     *
     * @return A new identity transform
     */
    public static Transform identity() {
        return new Transform(new Vector3(0, 0, 0), Quaternion.identity());
    }

    /**
     * Multiplies the transform by a vector3 and returns the result as a new vector3.
     *
     * @param transform The transform
     * @param vector The vector
     * @return The result of the multiplication of the transform by a vector3 as a new vector3
     */
    public static Vector3 multiply(Transform transform, Vector3 vector) {
        return Vector3.add(Matrix3x3.multiply(transform.getOrientation().getMatrix(), vector), transform.getPosition());
    }

    /**
     * Multiplies the first transform by the second one and returns the result as a new transform.
     *
     * @param transform1 The first transform
     * @param transform2 The second transform
     * @return The result of the multiplication of the two transforms as a new transform
     */
    public static Transform multiply(Transform transform1, Transform transform2) {
        return new Transform(
                Vector3.add(transform1.getPosition(), Matrix3x3.multiply(transform1.getOrientation().getMatrix(), transform2.getPosition())),
                Quaternion.multiply(transform1.getOrientation(), transform2.getOrientation()));
    }

    /**
     * Interpolates a transform between two other.
     *
     * @param transform1 The first transform
     * @param transform2 The second transform
     * @param percent The percent for the interpolation, between 0 and 1 inclusively
     * @return The interpolated transform
     */
    public static Transform interpolateTransforms(Transform transform1, Transform transform2, float percent) {
        final Vector3 interPosition = Vector3.add(
                Vector3.multiply(transform1.getPosition(), (1 - percent)),
                Vector3.multiply(transform2.getPosition(), percent));
        final Quaternion interOrientation = Quaternion.slerp(transform1.getOrientation(), transform2.getOrientation(), percent);
        return new Transform(interPosition, interOrientation);
    }
}
