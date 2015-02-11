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
package com.flowpowered.react.body;

/**
 * This is the base class for a body in the physics engine.
 */
public class Body {
    protected final int mID;
    protected boolean mIsAlreadyInIsland;
    protected boolean mIsAllowedToSleep;
    protected boolean mIsActive;
    protected boolean mIsSleeping;
    protected float mSleepTime;

    /**
     * Construct a new body from its ID.
     *
     * @param id The body's ID
     */
    public Body(int id) {
        mID = id;
        mIsAlreadyInIsland = false;
        mIsAllowedToSleep = true;
        mIsActive = true;
        mIsSleeping = false;
        mSleepTime = 0;
    }

    /**
     * Gets the body's unique ID.
     *
     * @return The body's ID
     */
    public int getID() {
        return mID;
    }

    /**
     * Returns true if the body has already been added in an island (for the sleeping technique).
     *
     * @return Whether or not the body is already in an island
     */
    public boolean isAlreadyInIsland() {
        return mIsAlreadyInIsland;
    }

    /**
     * Sets the value of to know if the body has already been added into an island.
     *
     * @param isAlreadyInIsland Whether or not the body is in an island
     */
    public void setIsAlreadyInIsland(boolean isAlreadyInIsland) {
        mIsAlreadyInIsland = isAlreadyInIsland;
    }

    /**
     * Returns whether or not the body is allowed to sleep.
     *
     * @return Whether or not the body can sleep
     */
    public boolean isAllowedToSleep() {
        return mIsAllowedToSleep;
    }

    /**
     * Set whether or not the body is allowed to go to sleep.
     *
     * @param isAllowedToSleep Whether or not the body should be able to sleep
     */
    public void setIsAllowedToSleep(boolean isAllowedToSleep) {
        mIsAllowedToSleep = isAllowedToSleep;
        if (!mIsAllowedToSleep) {
            setIsSleeping(false);
        }
    }

    /**
     * Returns true if the body is active, false if not.
     *
     * @return Whether or not the body is active
     */
    public boolean isActive() {
        return mIsActive;
    }

    /**
     * Sets the activity for this body. True for active, false for inactive.
     *
     * @param mIsActive True if this body is active, false if not
     */
    public void setActive(boolean mIsActive) {
        this.mIsActive = mIsActive;
    }

    /**
     * Returns whether or not the body is sleeping.
     *
     * @return Whether or not the body is sleeping
     */
    public boolean isSleeping() {
        return mIsSleeping;
    }

    /**
     * Set the variable to know whether or not the body is sleeping.
     *
     * @param isSleeping Whether or not the body is sleeping
     */
    public void setIsSleeping(boolean isSleeping) {
        if (isSleeping) {
            mSleepTime = 0;
        } else {
            if (mIsSleeping) {
                mSleepTime = 0;
            }
        }
        mIsSleeping = isSleeping;
    }

    /**
     * Returns the sleep time.
     *
     * @return The sleep time
     */
    public float getSleepTime() {
        return mSleepTime;
    }

    /**
     * Sets the sleep time.
     *
     * @param sleepTime The sleep time
     */
    public void setSleepTime(float sleepTime) {
        mSleepTime = sleepTime;
    }

    /**
     * Returns true this body's ID is smaller than the other body's ID, false if not.
     *
     * @param other The body to compare with
     * @return True if this body is smaller, false if not
     */
    public boolean isSmallerThan(Body other) {
        return mID < other.getID();
    }

    /**
     * Returns true this body's ID is greater than the other body's ID, false if not.
     *
     * @param other The body to compare with
     * @return True if this body is greater, false if not
     */
    public boolean isGreaterThan(Body other) {
        return mID > other.getID();
    }

    /**
     * Returns true this body's ID is equal than the other body's ID, false if not.
     *
     * @param other The body to compare with
     * @return True if this body is equal, false if not
     */
    public boolean isEqualTo(Body other) {
        return mID == other.getID();
    }

    /**
     * Returns true this body's ID is not equal than the other body's ID, false if not.
     *
     * @param other The body to compare with
     * @return True if this body is not equal, false if not
     */
    public boolean isNotEqualTo(Body other) {
        return mID != other.getID();
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) {
            return true;
        }
        if (!(o instanceof Body)) {
            return false;
        }
        final Body body = (Body) o;
        return mID == body.mID;
    }

    @Override
    public int hashCode() {
        return mID;
    }
}
