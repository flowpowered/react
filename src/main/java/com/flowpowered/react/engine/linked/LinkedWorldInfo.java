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
package com.flowpowered.react.engine.linked;

import com.flowpowered.react.body.RigidBody;

/**
 * A simple class made for implementations with dynamic world planes (ex. Voxel generation) where no assumptions can be made about the world. <p> It is left up to the implementation of this class to
 * provide the body to be used at the x, y, z provided.
 */
public interface LinkedWorldInfo {
    /**
     * Fetches the {@link RigidBody} at the x, y, z in world space. <p> Implementations of this method are expected to generate a body based on data stored for the 3D coordinate in world
     * space. </p>
     *
     * @param x The x coordinate in world space
     * @param y The y coordinate in world space
     * @param z The z coordinate in world space
     * @return The constructed body
     */
    public RigidBody getBody(int x, int y, int z);
}
