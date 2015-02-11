/*
 * This file is part of React, licensed under the MIT License (MIT).
 *
 * Copyright (c) 2013 Flow Powered <https://flowpowered.com/>
 * Original ReactPhysics3D C++ library by Daniel Chappuis <http://danielchappuis.ch>
 * React is re-licensed with permission from ReactPhysics3D author.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
package com.flowpowered.react.engine;

import com.flowpowered.react.ReactDefaults;

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
