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

import org.spout.physics.ReactDefaults;

/**
 * Represents a material for a rigid body. The material has the restitution and friction coefficients. Altering these constants for a material will alter the constants for all bodies.
 */
public class Material {
    private float mFrictionCoefficient;
    private float mBounciness;

    /**
     * Constructs a new rigid body material using {@link ReactDefaults#DEFAULT_BOUNCINESS} as the default bounciness and {@link ReactDefaults#DEFAULT_FRICTION_COEFFICIENT} as the default friction
     * coefficient.
     */
    public Material() {
        mBounciness = ReactDefaults.DEFAULT_BOUNCINESS;
        mFrictionCoefficient = ReactDefaults.DEFAULT_FRICTION_COEFFICIENT;
    }

    /**
     * Copy constructor.
     *
     * @param material The material to copy
     */
    public Material(Material material) {
        mBounciness = material.mBounciness;
        mFrictionCoefficient = material.mFrictionCoefficient;
    }

    /**
     * Constructs a new rigid body material from the provided bounciness and friction coefficient.
     *
     * @param bounciness The bounciness
     * @param frictionCoefficient The friction coefficient
     */
    public Material(float bounciness, float frictionCoefficient) {
        mBounciness = bounciness;
        mFrictionCoefficient = frictionCoefficient;
    }

    /**
     * Gets the bounciness.
     *
     * @return The bounciness
     */
    public float getBounciness() {
        return mBounciness;
    }

    /**
     * Sets the bounciness. The bounciness should be a value between 0 and 1. The value 1 is used for a very bouncy body and zero is used for a body that is not bouncy at all.
     *
     * @param bounciness The bounciness
     */
    public void setBounciness(float bounciness) {
        if (bounciness < 0 || bounciness > 1) {
            throw new IllegalArgumentException("Bounciness must be between 0 and 1 inclusively");
        }
        mBounciness = bounciness;
    }

    /**
     * Gets the friction coefficient.
     *
     * @return The friction
     */
    public float getFrictionCoefficient() {
        return mFrictionCoefficient;
    }

    /**
     * Sets the friction coefficient. The friction coefficient has to be a positive value. The value zero is used for no friction at all.
     *
     * @param frictionCoefficient The coefficient to set
     */
    public void setFrictionCoefficient(float frictionCoefficient) {
        if (frictionCoefficient < 0) {
            throw new IllegalArgumentException("Friction coefficient must be greater or equal to 0");
        }
        mFrictionCoefficient = frictionCoefficient;
    }

    /**
     * Sets the material to be same as the provided one.
     *
     * @param material The material to assign
     */
    public void set(Material material) {
        mBounciness = material.mBounciness;
        mFrictionCoefficient = material.mFrictionCoefficient;
    }

    /**
     * Returns a new unmodifiable rigid body material. Unmodifiable means that the {@link #setBounciness(float)} (float)} and {@link #setFrictionCoefficient(float)} (float)} methods will throw an
     * {@link UnsupportedOperationException}. This is does not use a wrapper class, and the original material is not linked to the one returned. That is, changes to the original material are not
     * reflected by the returned material.
     *
     * @param material The material to make an unmodifiable copy of
     * @return A new unmodifiable rigid body material.
     */
    public static Material asUnmodifiableMaterial(Material material) {
        return new UnmodifiableMaterial(material);
    }

    private static class UnmodifiableMaterial extends Material {
        private UnmodifiableMaterial(Material material) {
            super(material);
        }

        @Override
        public void setBounciness(float bounciness) {
            throw new UnsupportedOperationException("You cannot alter this material");
        }

        @Override
        public void setFrictionCoefficient(float frictionCoefficient) {
            throw new UnsupportedOperationException("You cannot alter this material");
        }
    }
}
