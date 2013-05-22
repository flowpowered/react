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
 * Represents a quaternion. The notation q = (x*i, y*j, z*k, w) is used to represent the quaternion.
 */
public class Quaternion {
	private float x;
	private float y;
	private float z;
	private float w;

	public Quaternion() {
		this(0, 0, 0, 0);
	}

	public Quaternion(float x, float y, float z, float w) {
		this.x = x;
		this.y = y;
		this.z = z;
		this.w = w;
	}

	public Quaternion(float w, Vector3 v) {
		this(v.getX(), v.getY(), v.getZ(), w);
	}

	public Quaternion(Quaternion quaternion) {
		this(quaternion.getX(), quaternion.getY(), quaternion.getZ(), quaternion.getW());
	}

	public Quaternion(Matrix3x3 matrix) {
		final float trace = matrix.getTrace();
		if (trace < 0) {
			if (matrix.get(1, 1) > matrix.get(0, 0)) {
				if (matrix.get(2, 2) > matrix.get(1, 1)) {
					final float r = (float) Math.sqrt(matrix.get(2, 2) - matrix.get(0, 0) - matrix.get(1, 1) + 1);
					final float s = 0.5f / r;
					x = (matrix.get(2, 0) + matrix.get(0, 2)) * s;
					y = (matrix.get(1, 2) + matrix.get(2, 1)) * s;
					z = 0.5f * r;
					w = (matrix.get(1, 0) - matrix.get(0, 1)) * s;
				} else {
					final float r = (float) Math.sqrt(matrix.get(1, 1) - matrix.get(2, 2) - matrix.get(0, 0) + 1);
					final float s = 0.5f / r;
					x = (matrix.get(0, 1) + matrix.get(1, 0)) * s;
					y = 0.5f * r;
					z = (matrix.get(1, 2) + matrix.get(2, 1)) * s;
					w = (matrix.get(0, 2) - matrix.get(2, 0)) * s;
				}
			} else if (matrix.get(2, 2) > matrix.get(0, 0)) {
				final float r = (float) Math.sqrt(matrix.get(2, 2) - matrix.get(0, 0) - matrix.get(1, 1) + 1);
				final float s = 0.5f / r;
				x = (matrix.get(2, 0) + matrix.get(0, 2)) * s;
				y = (matrix.get(1, 2) + matrix.get(2, 1)) * s;
				z = 0.5f * r;
				w = (matrix.get(1, 0) - matrix.get(0, 1)) * s;
			} else {
				final float r = (float) Math.sqrt(matrix.get(0, 0) - matrix.get(1, 1) - matrix.get(2, 2) + 1);
				final float s = 0.5f / r;
				x = 0.5f * r;
				y = (matrix.get(0, 1) + matrix.get(1, 0)) * s;
				z = (matrix.get(2, 0) - matrix.get(0, 2)) * s;
				w = (matrix.get(2, 1) - matrix.get(1, 2)) * s;
			}
		} else {
			final float r = (float) Math.sqrt(trace + 1);
			final float s = 0.5f / r;
			x = (matrix.get(2, 1) - matrix.get(1, 2)) * s;
			y = (matrix.get(0, 2) - matrix.get(2, 0)) * s;
			z = (matrix.get(1, 0) - matrix.get(0, 1)) * s;
			w = 0.5f * r;
		}
	}

	public float getX() {
		return x;
	}

	public float getY() {
		return y;
	}

	public float getZ() {
		return z;
	}

	public float getW() {
		return w;
	}

	public void setX(float x) {
		this.x = x;
	}

	public void setY(float y) {
		this.y = y;
	}

	public void setZ(float z) {
		this.z = z;
	}

	public void setW(float w) {
		this.w = w;
	}

	public void set(Quaternion quaternion) {
		x = quaternion.getX();
		y = quaternion.getY();
		z = quaternion.getZ();
		w = quaternion.getW();
	}

	public void setAllValues(float x, float y, float z, float w) {
		this.x = x;
		this.y = y;
		this.z = z;
		this.w = w;
	}

	public void setToZero() {
		x = 0;
		y = 0;
		z = 0;
		w = 0;
	}

	public Vector3 getVectorV() {
		return new Vector3(x, y, z);
	}

	public float length() {
		return (float) Math.sqrt(x * x + y * y + z * z + w * w);
	}

	public void normalize() {
		final float length = length();
		if (Mathematics.approxEquals(length, 0)) {
			throw new IllegalArgumentException("Cannot normalize the zero quaternion");
		}
		x /= length;
		y /= length;
		z /= length;
		w /= length;
	}

	public Quaternion getUnit() {
		final float length = length();
		if (Mathematics.approxEquals(length, 0)) {
			throw new IllegalArgumentException("Cannot normalize the zero quaternion");
		}
		return new Quaternion(
				x / length,
				y / length,
				z / length,
				w / length);
	}

	public Quaternion getConjugate() {
		return new Quaternion(-x, -y, -z, w);
	}

	public Quaternion getInverse() {
		float lengthQuaternion = length();
		lengthQuaternion *= lengthQuaternion;
		if (Mathematics.approxEquals(lengthQuaternion, 0)) {
			throw new IllegalArgumentException("Cannot normalize the zero quaternion");
		}
		return new Quaternion(
				-x / lengthQuaternion,
				-y / lengthQuaternion,
				-z / lengthQuaternion,
				w / lengthQuaternion);
	}

	public float dot(Quaternion quaternion) {
		return x * quaternion.getX() + y * quaternion.getY() + z * quaternion.getZ() + w * quaternion.getW();
	}

	public float getRotationAngleAxis(float angle, Vector3 axis) {
		final Quaternion quaternion;
		if (length() == 1) {
			quaternion = this;
		} else {
			quaternion = getUnit();
		}
		angle = (float) Math.acos(quaternion.getW()) * 2;
		Vector3 rotationAxis = new Vector3(quaternion.getX(), quaternion.getY(), quaternion.getZ()).getUnit();
		axis.setAllValues(rotationAxis.getX(), rotationAxis.getY(), rotationAxis.getZ());
		return angle;
	}

	public Matrix3x3 getMatrix() {
		final float nQ = x * x + y * y + z * z + w * w;
		final float s;
		if (nQ > 0.0) {
			s = 2 / nQ;
		} else {
			s = 0;
		}
		final float xs = x * s;
		final float ys = y * s;
		final float zs = z * s;
		final float wxs = w * xs;
		final float wys = w * ys;
		final float wzs = w * zs;
		final float xxs = x * xs;
		final float xys = x * ys;
		final float xzs = x * zs;
		final float yys = y * ys;
		final float yzs = y * zs;
		final float zzs = z * zs;
		return new Matrix3x3(1 - yys - zzs, xys - wzs, xzs + wys,
				xys + wzs, 1 - xxs - zzs, yzs - wxs,
				xzs - wys, yzs + wxs, 1 - xxs - yys);
	}

	public Quaternion slerp(Quaternion quaternion1, Quaternion quaternion2, float percent) {
		if (percent < 0.0 && percent > 1.0) {
			throw new IllegalArgumentException("\"percent\" must be greater than zero and smaller than one");
		}
		final float invert;
		float cosineTheta = quaternion1.dot(quaternion2);
		if (cosineTheta < 0.0) {
			cosineTheta = -cosineTheta;
			invert = -1;
		} else {
			invert = 1;
		}
		final float epsilon = 0.00001f;
		if (1 - cosineTheta < epsilon) {
			return add(multiply(quaternion1, (1 - percent)), multiply(quaternion2, (percent * invert)));
		}
		final float theta = (float) Math.acos(cosineTheta);
		final float sineTheta = (float) Math.sin(theta);
		final float coeff1 = (float) Math.sin((1 - percent) * theta) / sineTheta;
		final float coeff2 = (float) Math.sin(percent * theta) / sineTheta * invert;
		return add(multiply(quaternion1, coeff1), multiply(quaternion2, coeff2));
	}

	@Override
	public int hashCode() {
		int hash = 7;
		hash = 47 * hash + Float.floatToIntBits(x);
		hash = 47 * hash + Float.floatToIntBits(y);
		hash = 47 * hash + Float.floatToIntBits(z);
		hash = 47 * hash + Float.floatToIntBits(w);
		return hash;
	}

	@Override
	public boolean equals(Object obj) {
		if (!(obj instanceof Quaternion)) {
			return false;
		}
		final Quaternion other = (Quaternion) obj;
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

	public static Quaternion indentity() {
		return new Quaternion(0, 0, 0, 1);
	}

	public static Quaternion add(Quaternion quaternion1, Quaternion quaternion2) {
		return new Quaternion(
				quaternion1.getX() + quaternion2.getX(),
				quaternion1.getY() + quaternion2.getY(),
				quaternion1.getZ() + quaternion2.getZ(),
				quaternion1.getW() + quaternion2.getW());
	}

	public static Quaternion subtract(Quaternion quaternion1, Quaternion quaternion2) {
		return new Quaternion(
				quaternion1.getX() - quaternion2.getX(),
				quaternion1.getY() - quaternion2.getY(),
				quaternion1.getZ() - quaternion2.getZ(),
				quaternion1.getW() - quaternion2.getW());
	}

	public static Quaternion multiply(Quaternion quaternion, float value) {
		return new Quaternion(
				quaternion.getX() * value,
				quaternion.getY() * value,
				quaternion.getZ() * value,
				quaternion.getW() * value);
	}

	public static Quaternion multiply(Quaternion quaternion1, Quaternion quaternion2) {
		return new Quaternion(
				quaternion1.getW() * quaternion2.getW() - quaternion1.getVectorV().dot(quaternion2.getVectorV()),
				quaternion2.getVectorV().multiply(quaternion1.getW()).add(quaternion1.getVectorV().multiply(quaternion2.getW())).
				add(quaternion1.getVectorV().cross(quaternion2.getVectorV())));
	}

	public static Vector3 multiply(Quaternion quaternion, Vector3 vector) {
		final Quaternion p = new Quaternion(vector.getX(), vector.getY(), vector.getZ(), 0);
		return multiply(multiply(p, quaternion), quaternion.getConjugate()).getVectorV();
	}
}
