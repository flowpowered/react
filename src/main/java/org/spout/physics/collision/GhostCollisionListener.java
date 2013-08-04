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
package org.spout.physics.collision;

import org.spout.physics.body.CollisionBody;
import org.spout.physics.body.GhostImmobileRigidBody;
import org.spout.physics.body.GhostMobileRigidBody;

/**
 * Simple {@link CollisionListener} where bodies which are instances of either {@link GhostImmobileRigidBody} or {@link GhostMobileRigidBody} automatically cease collision adjustments. <p> While
 * completely possible to implement CollisionListener directly, implementations are encouraged to use this class to honor ghost objects as well as extend it to inform their callbacks that a collision
 * occurred (and call this method in the superclass while doing so).
 */
public class GhostCollisionListener implements CollisionListener {
	@Override
	public boolean onCollide(CollisionBody body1, CollisionBody body2, ContactInfo contactInfo) {
		if (body1 instanceof GhostMobileRigidBody || body1 instanceof GhostImmobileRigidBody || body2 instanceof GhostImmobileRigidBody || body2 instanceof GhostMobileRigidBody) {
			return true;
		}
		return false;
	}
}
