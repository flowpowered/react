package org.spout.physics.collision.dynamicphase;

import java.util.Set;

import org.spout.physics.body.ImmobileRigidBody;
import org.spout.physics.body.MobileRigidBody;
import org.spout.physics.engine.DynamicDynamicsWorld;
import org.spout.physics.math.Vector3;

/**
 * A phase of the physics tick where bodies is dynamically added via the {@link org.spout.physics.engine.DynamicDynamicsWorld}'s {@link org.spout.physics.engine.dynamic.DynamicWorldInfo}.
 */
public class DynamicPhase {
	private final DynamicDynamicsWorld dynamicWorld;

	public DynamicPhase(final DynamicDynamicsWorld dynamicWorld) {
		this.dynamicWorld = dynamicWorld;
	}

	public Set<ImmobileRigidBody> getBodiesInRange(final MobileRigidBody body) {
		final Vector3 pos = body.getTransform().getPosition();

		return null;
	}
}
