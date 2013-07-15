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

import java.util.HashSet;
import java.util.Set;
import java.util.Vector;

import gnu.trove.map.TObjectIntMap;
import gnu.trove.map.hash.TObjectIntHashMap;

import org.spout.physics.ReactDefaults;
import org.spout.physics.Utilities.IntPair;
import org.spout.physics.body.ImmobileRigidBody;
import org.spout.physics.body.MobileRigidBody;
import org.spout.physics.body.RigidBody;
import org.spout.physics.collision.BroadPhasePair;
import org.spout.physics.collision.ContactInfo;
import org.spout.physics.collision.shape.CollisionShape;
import org.spout.physics.constraint.Constraint;
import org.spout.physics.constraint.ContactPoint;
import org.spout.physics.math.Matrix3x3;
import org.spout.physics.math.Quaternion;
import org.spout.physics.math.Transform;
import org.spout.physics.math.Vector3;

/**
 * This class represents a dynamics world. This class inherits from the CollisionWorld class. In a
 * dynamics world bodies can collide and their movements are simulated using the laws of physics.
 */
public class DynamicsWorld extends CollisionWorld {
	private final Timer mTimer;
	private final ContactSolver mContactSolver;
	private final Set<RigidBody> mRigidBodies = new HashSet<RigidBody>();
	private final Vector<ContactManifold> mContactManifolds = new Vector<ContactManifold>();
	private final Vector<Constraint> mConstraints = new Vector<Constraint>();
	private final Vector3 mGravity;
	private boolean mIsGravityOn = true;
	private final Vector<Vector3> mConstrainedLinearVelocities = new Vector<Vector3>();
	private final Vector<Vector3> mConstrainedAngularVelocities = new Vector<Vector3>();
	private final TObjectIntMap<RigidBody> mMapBodyToConstrainedVelocityIndex = new TObjectIntHashMap<RigidBody>();

	/**
	 * Constructs a new dynamics world from the gravity and the default time step.
	 *
	 * @param gravity The gravity
	 */
	public DynamicsWorld(Vector3 gravity) {
		this(gravity, ReactDefaults.DEFAULT_TIMESTEP);
	}

	/**
	 * Constructs a new dynamics world from the gravity and the time step.
	 *
	 * @param gravity The gravity
	 * @param timeStep The time step
	 */
	public DynamicsWorld(Vector3 gravity, float timeStep) {
		mTimer = new Timer(timeStep);
		mGravity = gravity;
		mContactSolver = new ContactSolver(
				this,
				mConstrainedLinearVelocities,
				mConstrainedAngularVelocities,
				mMapBodyToConstrainedVelocityIndex);
	}

	/**
	 * Starts the physics simulation.
	 */
	public void start() {
		mTimer.start();
	}

	/**
	 * Stops the physics simulation.
	 */
	public void stop() {
		mTimer.stop();
	}

	/**
	 * Sets the number of iterations for the constraint solver.
	 *
	 * @param nbIterations The number of iterations to do
	 */
	public void setNbIterationsSolver(int nbIterations) {
		mContactSolver.setNbIterationsSolver(nbIterations);
	}

	/**
	 * Activates or deactivates the split impulses for contacts.
	 *
	 * @param isActive True if the split impulses are active, false if not
	 */
	public void setSplitImpulseActive(boolean isActive) {
		mContactSolver.setSplitImpulseActive(isActive);
	}

	/**
	 * Activates or deactivates the solving of friction constraints at the center of the contact
	 * manifold instead of solving them at each contact point.
	 *
	 * @param isActive Whether or not to solve the friction constraint at the center of the manifold
	 */
	public void setSolveFrictionAtContactManifoldCenterActive(boolean isActive) {
		mContactSolver.setSolveFrictionAtContactManifoldCenterActive(isActive);
	}

	/**
	 * Adds a constraint in the physics world.
	 *
	 * @param constraint The constraint to add
	 */
	public void addConstraint(Constraint constraint) {
		if (constraint == null) {
			throw new IllegalArgumentException("constraint cannot be null");
		}
		mConstraints.add(constraint);
	}

	/**
	 * Removes a constraint.
	 *
	 * @param constraint The constraint to remove
	 */
	public void removeConstraint(Constraint constraint) {
		if (constraint == null) {
			throw new IllegalArgumentException("constraint cannot be null");
		}
		mConstraints.remove(constraint);
	}

	/**
	 * Returns the gravity vector for the world.
	 *
	 * @return The gravity vector
	 */
	public Vector3 getGravity() {
		return mGravity;
	}

	/**
	 * Returns true if the gravity is on.
	 *
	 * @return Whether or not the gravity is on
	 */
	public boolean isGravityOn() {
		return mIsGravityOn;
	}

	/**
	 * Sets the gravity on if true, off is false.
	 *
	 * @param gravityOn True to turn on the gravity, false to turn it off
	 */
	public void setGravityOn(boolean gravityOn) {
		mIsGravityOn = gravityOn;
	}

	/**
	 * Gets the number of rigid bodies in the world.
	 *
	 * @return The number of rigid bodies in the world
	 */
	public int getNbRigidBodies() {
		return mRigidBodies.size();
	}

	/**
	 * Gets the set of bodies of the physics world.
	 *
	 * @return The rigid bodies
	 */
	public Set<RigidBody> getRigidBodies() {
		return mRigidBodies;
	}

	/**
	 * Gets the number of contact manifolds in the world.
	 *
	 * @return The number of contact manifolds in the world
	 */
	public int getNbContactManifolds() {
		return mContactManifolds.size();
	}

	/**
	 * Gets the constraint list.
	 *
	 * @return The constraints
	 */
	public Vector<Constraint> getConstraints() {
		return mConstraints;
	}

	/**
	 * Gets the contact manifolds list.
	 *
	 * @return The contact manifolds
	 */
	public Vector<ContactManifold> getContactManifolds() {
		return mContactManifolds;
	}

	/**
	 * Updates the physics simulation. The elapsed time is determined by the timer. A step is only
	 * actually taken if enough time has passed. If a lot of time has passed, more than twice the time
	 * step, multiple steps will be taken, to catch up.
	 */
	public void update() {
		if (!mTimer.isRunning()) {
			throw new IllegalStateException("timer must be running");
		}
		mTimer.update();
		final float dt = (float) mTimer.getTimeStep();
		while (mTimer.isPossibleToTakeStep()) {
			tick(dt);
		}
		applyGravity();
		setInterpolationFactorToAllBodies();
	}

	/**
	 * Ticks the simulation by the provided time delta. Note that this method should only be called
	 * externally if the simulation is stopped; to prevent issues with the timer and the {@link
	 * #update()} method. This method doesn't reapply gravity to bodies or set their interpolation
	 * factor. This is accomplished by the {@link #update()} method at the end of the update.
	 *
	 * @param dt The time delta
	 */
	protected void tick(float dt) {
		mContactManifolds.clear();
		mCollisionDetection.computeCollisionDetection();
		initConstrainedVelocitiesArray(dt);
		if (!mContactManifolds.isEmpty()) {
			mContactSolver.solve(dt);
		}
		mTimer.nextStep();
		resetBodiesMovementVariable();
		updateRigidBodiesPositionAndOrientation(dt);
		mContactSolver.cleanup();
		cleanupConstrainedVelocitiesArray();
	}

	// Resets the boolean movement variable for each body.
	private void resetBodiesMovementVariable() {
		for (RigidBody rigidBody : getRigidBodies()) {
			rigidBody.setHasMoved(false);
		}
	}

	// Updates the position and orientation of the rigid bodies using the provided time delta.
	private void updateRigidBodiesPositionAndOrientation(float dt) {
		for (RigidBody rigidBody : getRigidBodies()) {
			if (!(rigidBody instanceof MobileRigidBody)) {
				continue;
			}
			final MobileRigidBody mobileBody = (MobileRigidBody) rigidBody;
			if (mobileBody.isMotionEnabled()) {
				mobileBody.snapshotTransform();
				final int indexArray = mMapBodyToConstrainedVelocityIndex.get(mobileBody);
				final Vector3 newLinVelocity = mConstrainedLinearVelocities.get(indexArray);
				final Vector3 newAngVelocity = mConstrainedAngularVelocities.get(indexArray);
				mobileBody.setLinearVelocity(newLinVelocity);
				mobileBody.setAngularVelocity(newAngVelocity);
				if (mContactSolver.isConstrainedBody(mobileBody)) {
					newLinVelocity.add(mContactSolver.getSplitLinearVelocityOfBody(mobileBody));
					newAngVelocity.add(mContactSolver.getSplitAngularVelocityOfBody(mobileBody));
				}
				final Vector3 currentPosition = mobileBody.getTransform().getPosition();
				final Quaternion currentOrientation = mobileBody.getTransform().getOrientation();
				final Vector3 newPosition = Vector3.add(currentPosition, Vector3.multiply(newLinVelocity, dt));
				final Quaternion newOrientation = Quaternion.add(
						currentOrientation,
						Quaternion.multiply(
								Quaternion.multiply(
										new Quaternion(newAngVelocity.getX(), newAngVelocity.getY(), newAngVelocity.getZ(), 0),
										currentOrientation),
								0.5f * dt));
				final Transform newTransform = new Transform(newPosition, newOrientation.getUnit());
				mobileBody.setTransform(newTransform);
				mobileBody.updateAABB();
			}
		}
	}

	// Computes and set the interpolation factor for all bodies,
	// using the interpolation factor from the timer.
	private void setInterpolationFactorToAllBodies() {
		setInterpolationFactorToAllBodies(mTimer.computeInterpolationFactor());
	}

	// Computes and set the interpolation factor for all bodies.
	private void setInterpolationFactorToAllBodies(float factor) {
		if (factor < 0 && factor > 1) {
			throw new IllegalStateException("interpolation factor must be greater or equal to zero"
					+ " and smaller or equal to one");
		}
		for (RigidBody rigidBody : getRigidBodies()) {
			if (rigidBody == null) {
				throw new IllegalStateException("rigid body cannot be null");
			}
			rigidBody.setInterpolationFactor(factor);
		}
	}

	// Initializes the constrained velocities array using the provided time delta.
	private void initConstrainedVelocitiesArray(float dt) {
		mConstrainedLinearVelocities.clear();
		mConstrainedLinearVelocities.ensureCapacity(mRigidBodies.size());
		mConstrainedAngularVelocities.clear();
		mConstrainedAngularVelocities.ensureCapacity(mRigidBodies.size());
		int i = 0;
		for (RigidBody rigidBody : mRigidBodies) {
			mMapBodyToConstrainedVelocityIndex.put(rigidBody, i);
			mConstrainedLinearVelocities.add(i, Vector3.add(
					rigidBody.getLinearVelocity(),
					Vector3.multiply(dt * rigidBody.getMassInverse(), rigidBody.getExternalForce())));
			mConstrainedAngularVelocities.add(i, Vector3.add(
					rigidBody.getAngularVelocity(),
					Matrix3x3.multiply(
							Matrix3x3.multiply(dt, rigidBody.getInertiaTensorInverseWorld()),
							rigidBody.getExternalTorque())));
			i++;
		}
	}

	// Cleans up the constrained velocities array at each step.
	private void cleanupConstrainedVelocitiesArray() {
		mConstrainedLinearVelocities.clear();
		mConstrainedAngularVelocities.clear();
		mMapBodyToConstrainedVelocityIndex.clear();
	}

	// Applies the gravity force to all bodies of the physics world.
	private void applyGravity() {
		if (mIsGravityOn) {
			for (RigidBody rigidBody : getRigidBodies()) {
				if (rigidBody == null) {
					throw new IllegalStateException("rigid body cannot be null");
				}
				rigidBody.setExternalForce(Vector3.multiply(rigidBody.getMass(), mGravity));
			}
		}
	}

	/**
	 * Creates an immobile rigid body and adds it to the physics world. The inertia tensor will be
	 * computed from the shape and mass.
	 *
	 * @param transform The transform (position and orientation) of the body
	 * @param mass The mass of the body
	 * @param collisionShape The collision shape
	 * @return The new rigid body
	 */
	public ImmobileRigidBody createImmobileRigidBody(Transform transform, float mass, CollisionShape collisionShape) {
		final Matrix3x3 inertiaTensor = new Matrix3x3();
		collisionShape.computeLocalInertiaTensor(inertiaTensor, mass);
		return createImmobileRigidBody(transform, mass, inertiaTensor, collisionShape);
	}

	/**
	 * Creates an immobile rigid body and adds it to the physics world.
	 *
	 * @param transform The transform (position and orientation) of the body
	 * @param mass The mass of the body
	 * @param inertiaTensorLocal The local inertia tensor
	 * @param collisionShape The collision shape
	 * @return The new rigid body
	 */
	public ImmobileRigidBody createImmobileRigidBody(Transform transform, float mass, Matrix3x3 inertiaTensorLocal,
													 CollisionShape collisionShape) {
		final ImmobileRigidBody immobileBody = new ImmobileRigidBody(transform, mass, inertiaTensorLocal,
				collisionShape, getNextFreeID());
		addRigidBody(immobileBody);
		return immobileBody;
	}

	/**
	 * Creates a mobile rigid body and adds it to the physics world. The inertia tensor will be
	 * computed from the shape and mass.
	 *
	 * @param transform The transform (position and orientation) of the body
	 * @param mass The mass of the body
	 * @param collisionShape The collision shape
	 * @return The new rigid body
	 */
	public MobileRigidBody createMobileRigidBody(Transform transform, float mass, CollisionShape collisionShape) {
		final Matrix3x3 inertiaTensor = new Matrix3x3();
		collisionShape.computeLocalInertiaTensor(inertiaTensor, mass);
		return createMobileRigidBody(transform, mass, inertiaTensor, collisionShape);
	}

	/**
	 * Creates a mobile rigid body and adds it to the physics world.
	 *
	 * @param transform The transform (position and orientation) of the body
	 * @param mass The mass of the body
	 * @param inertiaTensorLocal The local inertia tensor
	 * @param collisionShape The collision shape
	 * @return The new rigid body
	 */
	public MobileRigidBody createMobileRigidBody(Transform transform, float mass, Matrix3x3 inertiaTensorLocal,
												 CollisionShape collisionShape) {
		final MobileRigidBody mobileBody = new MobileRigidBody(transform, mass, inertiaTensorLocal,
				collisionShape, getNextFreeID());
		addRigidBody(mobileBody);
		return mobileBody;
	}

	// Adds a rigid body to the body collections and the collision detection.
	public void addRigidBody(RigidBody body) {
		mBodies.add(body);
		mRigidBodies.add(body);
		mCollisionDetection.addBody(body);
	}

	/**
	 * Destroys a rigid body.
	 *
	 * @param rigidBody The rigid body to destroy
	 */
	public void destroyRigidBody(RigidBody rigidBody) {
		mCollisionDetection.removeBody(rigidBody);
		mFreeBodiesIDs.push(rigidBody.getID());
		mBodies.remove(rigidBody);
		mRigidBodies.remove(rigidBody);
	}

	/**
	 * Removes all constraints in the physics world.
	 */
	public void removeAllConstraints() {
		mConstraints.clear();
	}

	@Override
	public void notifyAddedOverlappingPair(BroadPhasePair addedPair) {
		final IntPair indexPair = addedPair.getBodiesIndexPair();
		final OverlappingPair newPair = new OverlappingPair(addedPair.getFirstBody(), addedPair.getSecondBody());
		final OverlappingPair oldPair = mOverlappingPairs.put(indexPair, newPair);
		if (oldPair != null) {
			throw new IllegalStateException("overlapping pair was already in the overlapping pairs map");
		}
	}

	@Override
	public void updateOverlappingPair(BroadPhasePair pair) {
		final IntPair indexPair = pair.getBodiesIndexPair();
		final OverlappingPair overlappingPair = mOverlappingPairs.get(indexPair);
		overlappingPair.update();
	}

	@Override
	public void notifyRemovedOverlappingPair(BroadPhasePair removedPair) {
		final IntPair indexPair = removedPair.getBodiesIndexPair();
		mOverlappingPairs.remove(indexPair);
	}

	@Override
	public void notifyNewContact(BroadPhasePair broadPhasePair, ContactInfo contactInfo) {
		final RigidBody rigidBody1 = (RigidBody) broadPhasePair.getFirstBody();
		final RigidBody rigidBody2 = (RigidBody) broadPhasePair.getSecondBody();
		if (rigidBody1 == null) {
			throw new IllegalArgumentException("first body of the broad phase pair cannot be null");
		}
		if (rigidBody2 == null) {
			throw new IllegalArgumentException("second body of the broad phase pair cannot be null");
		}
		final ContactPoint contact = new ContactPoint(rigidBody1, rigidBody2, contactInfo);
		final IntPair indexPair = broadPhasePair.getBodiesIndexPair();
		final OverlappingPair overlappingPair = mOverlappingPairs.get(indexPair);
		if (overlappingPair == null) {
			throw new IllegalArgumentException("broadphase pair is not in the overlapping pairs");
		}
		overlappingPair.addContact(contact);
		mContactManifolds.add(overlappingPair.getContactManifold());
	}
}
