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

/**
 * This class takes care of the time in the physics engine. It uses {@link System#nanoTime()} to get
 * the current time.
 */
public class Timer {
	private double mTimeStep;
	private double mTime;
	private double mLastUpdateTime;
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
	public double getTime() {
		return mTime;
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
		mTime = 0;
		mLastUpdateTime = System.nanoTime() / 1000000000d;
		mAccumulator = 0;
		mIsRunning = true;
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
		mTime += mTimeStep;
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
		final double currentTime = System.nanoTime() / 1000000000d;
		final double mDeltaTime = currentTime - mLastUpdateTime;
		mLastUpdateTime = currentTime;
		mAccumulator += mDeltaTime;
	}
}
