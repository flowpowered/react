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
