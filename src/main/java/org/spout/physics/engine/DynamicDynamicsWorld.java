package org.spout.physics.engine;

import java.util.HashSet;
import java.util.Set;

import org.spout.physics.body.ImmobileRigidBody;
import org.spout.physics.engine.dynamic.DynamicWorldInfo;
import org.spout.physics.math.Vector3;

public class DynamicDynamicsWorld extends DynamicsWorld {
	/* The info for this world */
	private final DynamicWorldInfo info;

	/* The bodies dynamically added each physics tick in the DynamicPhase */
	private final Set<ImmobileRigidBody> dynamicBodies = new HashSet<ImmobileRigidBody>();

	public DynamicDynamicsWorld(Vector3 gravity, final DynamicWorldInfo info) {
		super(gravity);
		this.info = info;
	}

	/**
	 * Returns the {@link org.spout.physics.engine.dynamic.DynamicWorldInfo} of this world
	 * @return the dynamic info
	 */
	public DynamicWorldInfo getDynamicInfo() {
		return info;
	}
}
