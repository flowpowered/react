package org.spout.physics.body;

import org.spout.physics.collision.shape.CollisionShape;
import org.spout.physics.collision.shape.VoxelWorldShape;
import org.spout.physics.math.Matrix3x3;
import org.spout.physics.math.Transform;

public class VoxelBody extends ImmobileRigidBody {
	/**
	 * Constructs a new voxel rigid body from its transform, local inertia tensor, collision shape
	 * @param transform The transform (position and orientation)
	 * @param inertiaTensorLocal The local inertial tensor
	 * @param collisionShape The collision shape
	 */
	public VoxelBody(Transform transform, Matrix3x3 inertiaTensorLocal, VoxelWorldShape collisionShape) {
		super(transform, 0, inertiaTensorLocal, collisionShape, -1); //TODO This shouldn't cause an issue as this body isn't added to the real list of bodies
	}

	@Override
	public VoxelWorldShape getCollisionShape() {
		return (VoxelWorldShape) super.getCollisionShape();
	}

	@Override
	public void setCollisionShape(CollisionShape shape) {
		throw new UnsupportedOperationException("Voxel body shape may only be set once in its contructor");
	}
}
