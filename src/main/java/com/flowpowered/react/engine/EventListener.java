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

import com.flowpowered.react.constraint.ContactPoint.ContactPointInfo;

/**
 * This class can be used to receive event callbacks from the physics engine. In order to receive callbacks, you need to create a new class that inherits from this one and you must override the
 * methods you need. Then, you need to register your new event listener class to the physics world using the {@link DynamicsWorld#setEventListener(EventListener)} method.
 */
public interface EventListener {
    /**
     * Called when a new contact point is found between two bodies that were separated before.
     *
     * @param contactInfo The info for the contact
     */
    void beginContact(ContactPointInfo contactInfo);

    /**
     * Called when a new contact point is found between two bodies.
     *
     * @param contactInfo The info for the contact
     */
    void newContact(ContactPointInfo contactInfo);
}
