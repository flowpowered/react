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
package org.spout.physics.body;

import org.spout.physics.ReactDefaults;

/**
 * Represents a material for a rigid body. The material has the restitution and friction coefficients. Altering these constants for a material will alter the constants for all bodies.
 */
public class RigidBodyMaterial {
    private float mRestitution = ReactDefaults.DEFAULT_RESTITUTION_COEFFICIENT;
    private float mFriction = ReactDefaults.DEFAULT_FRICTION_COEFFICIENT;

    /**
     * Constructs a new rigid body material using {@link ReactDefaults#DEFAULT_RESTITUTION_COEFFICIENT} and {@link ReactDefaults#DEFAULT_FRICTION_COEFFICIENT} as the default restitution and friction
     * coefficients.
     */
    public RigidBodyMaterial() {
    }

    /**
     * Constructs a new rigid body material from the provided restitution and friction coefficients.
     *
     * @param restitution The restitution coefficient
     * @param friction The friction coefficient
     */
    public RigidBodyMaterial(float restitution, float friction) {
        mRestitution = restitution;
        mFriction = friction;
    }

    /**
     * Gets the restitution coefficient.
     *
     * @return The restitution
     */
    public float getRestitution() {
        return mRestitution;
    }

    /**
     * Sets the restitution coefficient.
     *
     * @param restitution The coefficient to set
     */
    public void setRestitution(float restitution) {
        if (restitution < 0 || restitution > 1) {
            throw new IllegalArgumentException("restitution must be between 0 and 1 inclusively");
        }
        mRestitution = restitution;
    }

    /**
     * Gets the friction coefficient.
     *
     * @return The friction
     */
    public float getFriction() {
        return mFriction;
    }

    /**
     * Sets the friction coefficient.
     *
     * @param friction The coefficient to set
     */
    public void setFriction(float friction) {
        mFriction = friction;
    }

    /**
     * Returns a new unmodifiable rigid body material. Unmodifiable means that the {@link #setRestitution(float)} and {@link #setFriction(float)} methods will throw an {@link
     * UnsupportedOperationException}. This is does not use a wrapper class, and the original material is not linked to the one returned. That is, changes to the original material are not reflected by
     * the returned material.
     *
     * @param material The material to make an unmodifiable copy of
     * @return A new unmodifiable rigid body material.
     */
    public static RigidBodyMaterial asUnmodifiableMaterial(RigidBodyMaterial material) {
        return new UnmodifiableRigidBodyMaterial(material);
    }

    private static class UnmodifiableRigidBodyMaterial extends RigidBodyMaterial {
        private UnmodifiableRigidBodyMaterial(RigidBodyMaterial material) {
            super(material.getRestitution(), material.getFriction());
        }

        @Override
        public void setRestitution(float restitution) {
            throw new UnsupportedOperationException("you cannot alter this material");
        }

        @Override
        public void setFriction(float friction) {
            throw new UnsupportedOperationException("you cannot alter this material");
        }
    }
}
