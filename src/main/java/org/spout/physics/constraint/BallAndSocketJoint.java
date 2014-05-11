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
package org.spout.physics.constraint;

import org.spout.physics.body.RigidBody;

/**
 * This class represents a ball-and-socket joint that allows arbitrary rotation between two bodies.
 */
public class BallAndSocketJoint extends Constraint {
    /**
     * Constructs a new ball and socket joint from the two bodies, the activity status and the type of constraint.
     *
     * @param body1 The first body
     * @param body2 The second body
     * @param active True if this constraint is active, false if not
     * @param type The type of this constraint
     */
    public BallAndSocketJoint(RigidBody body1, RigidBody body2, boolean active, ConstraintType type) {
        super(body1, body2, active, type);
    }
}
