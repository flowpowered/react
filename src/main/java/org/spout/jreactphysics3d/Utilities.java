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
package org.spout.jreactphysics3d;

/**
 * This class contains static utilities. It was added for the port to implement some C++ code in
 * Java.
 */
public class Utilities {
	/**
	 * Returns the index of an object in an array, or -1 if it can't be found.
	 *
	 * @param array The array to search
	 * @param object The object to look for
	 * @return The index, or -1 if the object wasn't found
	 */
	public static int indexOf(Object[] array, Object object) {
		for (int i = 0; i < array.length; i++) {
			if (object.equals(array[i])) {
				return i;
			}
		}
		return -1;
	}

	/**
	 * Represents a pair of 32 bit integers.
	 */
	public static class IntPair {
		private int first;
		private int second;

		/**
		 * Constructs a new int pair with both ints being 0.
		 */
		public IntPair() {
			this(0, 0);
		}

		/**
		 * Constructs a new int pair with the desired value for each member.
		 *
		 * @param first The value of the first member
		 * @param second The value of the second member
		 */
		public IntPair(int first, int second) {
			this.first = first;
			this.second = second;
		}

		/**
		 * Gets the value of the first member.
		 *
		 * @return The first member's value
		 */
		public int getFirst() {
			return first;
		}

		/**
		 * Sets the first member's value.
		 *
		 * @param first The value for the first member
		 */
		public void setFirst(int first) {
			this.first = first;
		}

		/**
		 * Gets the value of the second member.
		 *
		 * @return The second member's value
		 */
		public int getSecond() {
			return second;
		}

		/**
		 * Sets the second member's value.
		 *
		 * @param second The value for the second member
		 */
		public void setSecond(int second) {
			this.second = second;
		}

		/**
		 * Swaps both members. First becomes second, second becomes first.
		 */
		public void swap() {
			final int temp = first;
			first = second;
			second = temp;
		}
	}

}
