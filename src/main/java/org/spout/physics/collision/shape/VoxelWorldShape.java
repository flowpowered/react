package org.spout.physics.collision.shape;

import org.spout.physics.body.ImmobileRigidBody;

/**
 * Represents an abstract shape designed for Voxel games.
 *
 * The use of this shape is for when a simulation has a completely dynamic ground (instead of a typical plane.
 * The only method, {@link #getBody(int, int, int)} , is implemented to be able to generate the necessary body to
 * be used instead of the "fake" simulation body and shape.
 */
public abstract class VoxelWorldShape extends CollisionShape {
	public VoxelWorldShape() {
		super(CollisionShapeType.VOXEL);
	}

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
