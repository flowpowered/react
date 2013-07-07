package org.spout.physics.collision.dynamicphase;

import org.spout.physics.engine.DynamicDynamicsWorld;

/**
 * A phase of the physics tick where bodies is dynamically added via the {@link org.spout.physics.engine.DynamicDynamicsWorld}'s {@link org.spout.physics.engine.dynamic.DynamicWorldInfo}.
 */
public class DynamicPhase {
	private final DynamicDynamicsWorld dynamicWorld;

	public DynamicPhase(final DynamicDynamicsWorld dynamicWorld) {
		this.dynamicWorld = dynamicWorld;
	}
}
