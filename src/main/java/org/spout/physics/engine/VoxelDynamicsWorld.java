package org.spout.physics.engine;

import java.util.HashSet;
import java.util.Set;

import org.spout.physics.body.ImmobileRigidBody;
import org.spout.physics.body.VoxelBody;
import org.spout.physics.collision.shape.VoxelWorldShape;
import org.spout.physics.math.Matrix3x3;
import org.spout.physics.math.Quaternion;
import org.spout.physics.math.Transform;
import org.spout.physics.math.Vector3;

public class VoxelDynamicsWorld extends DynamicsWorld {
	/* The voxel anchor body for this world */
	private VoxelBody body;

	/* The bodies dynamically added each physics tick in the VoxelPhase */
	private final Set<ImmobileRigidBody> voxelBodies = new HashSet<ImmobileRigidBody>();

	public VoxelDynamicsWorld(Vector3 gravity) {
		super(gravity);
	}

	/**
	 * Returns the {@link VoxelBody} of this world
	 * @return the voxel body
	 */
	public VoxelBody getVoxelBody() {
		return body;
	}

	/**
	 * Creates and sets the {@link VoxelBody} which is the 'anchor' of the voxel simulation.
	 * <p>
	 *     The body is unknown to react as a whole, instead its shape, {@link VoxelWorldShape}, has a hook
	 *     to retrieve bodies dynamically from a simulation.
	 * </p>
	 * @param shape The shape
	 */
	public void createAndSetVoxelBody(final VoxelWorldShape shape) {
		if (body != null) {
			throw new IllegalStateException("The voxel body of a simulation may only be constructed once");
		}
		final Matrix3x3 inertiaTensor = new Matrix3x3();
		shape.computeLocalInertiaTensor(inertiaTensor, 0);
		body = new VoxelBody(new Transform(new Vector3(0, 0, 0), new Quaternion(0,0,0,0)), inertiaTensor, shape);
	}
}
