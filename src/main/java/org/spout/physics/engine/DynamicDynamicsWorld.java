package org.spout.physics.engine;

import java.util.Collection;
import java.util.HashSet;
import java.util.Iterator;
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

	@Override
	public void update() {
		super.update();
		destroyAndClear();
	}

	@Override
	public void forceUpdate(float dt) {
		super.forceUpdate(dt);

		destroyAndClear();
	}

	@Override
	public void forceUpdate() {
		super.forceUpdate();

		destroyAndClear();
	}

	/**
	 * Returns the {@link org.spout.physics.engine.dynamic.DynamicWorldInfo} of this world
	 * @return the dynamic info
	 */
	public DynamicWorldInfo getDynamicInfo() {
		return info;
	}

	/**
	 * Adds {@link ImmobileRigidBody}s to this world. These will be cleared at the end of the physics tick
	 * @param bodies bodies to add
	 */
	public void addBodies(final Collection<ImmobileRigidBody> bodies) {
		dynamicBodies.addAll(bodies);
	}

	/**
	 * Clears all bodies tracked in the world
	 */
	public void destroyAndClear() {
		final Iterator<ImmobileRigidBody> bodiesIterator = dynamicBodies.iterator();
		while (bodiesIterator.hasNext()) {
			destroyRigidBody(bodiesIterator.next());
		}
		dynamicBodies.clear();
	}
}
