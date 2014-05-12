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

import org.spout.physics.math.Vector3;

/**
 * This class represents a ball-and-socket joint that allows arbitrary rotation between two bodies.
 */
public class BallAndSocketJoint extends Constraint {
    private Vector3 mFirstLocalAnchorPoint;
    private Vector3 mSecondLocalAnchorPoint;

    /**
     * Constructs a new ball and socket joint from provided ball and socket joint info.
     *
     * @param jointInfo The joint info
     */
    public BallAndSocketJoint(BallAndSocketJointInfo jointInfo) {
        super(jointInfo);
    }

    public static class BallAndSocketJointInfo extends ConstraintInfo {
        private Vector3 anchorPoint;

        public BallAndSocketJointInfo() {
            super(ConstraintType.BALLSOCKETJOINT);
        }
    }
}
