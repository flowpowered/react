/*
 * This file is part of Flow React, licensed under the MIT License (MIT).
 *
 * Copyright (c) 2013 Flow Powered <https://flowpowered.com/>
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
package com.flowpowered.react.engine;

/**
 * This class takes care of the time in the physics engine. It uses {@link System#nanoTime()} to get the current time.
 */
public class Timer {
    private double mTimeStep;
    private double mLastUpdateTime;
    private double mDeltaTime;
    private double mAccumulator;
    private boolean mIsRunning = false;

    /**
     * Constructs a new timer from the time step.
     *
     * @param timeStep The time step
     */
    public Timer(double timeStep) {
        if (timeStep <= 0) {
            throw new IllegalArgumentException("time step cannot be smaller or equal to zero");
        }
        mTimeStep = timeStep;
    }

    /**
     * Gets the time step of the physics engine.
     *
     * @return The time step
     */
    public double getTimeStep() {
        return mTimeStep;
    }

    /**
     * Sets the time step of the physics engine.
     *
     * @param timeStep The time step to set
     */
    public void setTimeStep(double timeStep) {
        if (timeStep <= 0) {
            throw new IllegalArgumentException("time step must be greater than zero");
        }
        mTimeStep = timeStep;
    }

    /**
     * Gets the current time.
     *
     * @return The current time
     */
    public double getPhysicsTime() {
        return mLastUpdateTime;
    }

    /**
     * Returns true if the timer is running, false if not.
     *
     * @return Whether or not the timer is running
     */
    public boolean isRunning() {
        return mIsRunning;
    }

    /**
     * Start the timer.
     */
    public void start() {
        if (!mIsRunning) {
            mLastUpdateTime = getCurrentSystemTime();
            mAccumulator = 0;
            mIsRunning = true;
        }
    }

    /**
     * Stop the timer.
     */
    public void stop() {
        mIsRunning = false;
    }

    /**
     * Returns true if it's possible to take a new step, false if not.
     *
     * @return Whether or not a new step is possible
     */
    public boolean isPossibleToTakeStep() {
        return mAccumulator >= mTimeStep;
    }

    /**
     * Takes a new step: updates the timer by adding the timeStep value to the current time.
     */
    public void nextStep() {
        if (!mIsRunning) {
            throw new IllegalStateException("Timer is not running");
        }
        mAccumulator -= mTimeStep;
    }

    /**
     * Compute the interpolation factor for the time step.
     *
     * @return The interpolation factor
     */
    public float computeInterpolationFactor() {
        return (float) (mAccumulator / mTimeStep);
    }

    /**
     * Compute the time since the last update call and add it to the accumulator.
     */
    public void update() {
        final double currentTime = getCurrentSystemTime();
        mDeltaTime = currentTime - mLastUpdateTime;
        mLastUpdateTime = currentTime;
        mAccumulator += mDeltaTime;
    }

    private static double getCurrentSystemTime() {
        return System.nanoTime() / 1e9d;
    }
}
