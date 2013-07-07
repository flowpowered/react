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
