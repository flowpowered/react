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
package org.spout.physics.engine.dynamic;

import org.spout.physics.body.ImmobileRigidBody;

/**
 * A simple class made for implementations with dynamic world planes (ex. Voxel generation) where no assumptions
 * can be made about the world.
 *
 * It is left up to the implementation of this class to provide the body to be used at the x, y, z provided
 */
public abstract class DynamicWorldInfo {
	/**
	 * Fetches the {@link ImmobileRigidBody} at the x, y, z in world space.
	 * <p>
	 *     Implementations of this method are expected to generate a body based on data stored for the
	 *     3D coordinate in world space
	 * </p>
	 * @param x coordinate in world space
	 * @param y coordinate in world space
	 * @param z coordinate in world space
	 * @return The constructed body
	 */
	public abstract ImmobileRigidBody getBody(int x, int y, int z);
}
