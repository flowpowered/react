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
import org.spout.physics.constraint.ContactPoint.ContactPointInfo;

/**
 * Reports the collisions between two bodies as they are occurring during the narrow-phase and allows for cancellation of the collision. Each collision will result in a call of {@link
 * #onCollide(org.spout.physics.body.CollisionBody, org.spout.physics.body.CollisionBody, ContactPointInfo)}. Implement this interface to listen to them. Use {@link
 * org.spout.physics.engine.CollisionWorld#addListener(CollisionListener)} to add a listener.
 */
public interface CollisionListener {
    /**
     * This method is called when two bodies are about to collide in a collision detection. Both bodies are provided, including the contact information, which includes the contact points on each body,
     * the penetration depth, and the normal. The method returns a boolean. If it is true, the collision will be canceled.
     *
     * @param body1 The first body
     * @param body2 The second body
     * @param contactInfo The contact information
     * @return Whether or not the collision should be canceled
     */
    public boolean onCollide(CollisionBody body1, CollisionBody body2, ContactPointInfo contactInfo);
}
