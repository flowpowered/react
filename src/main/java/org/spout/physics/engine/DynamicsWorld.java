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
import java.util.Iterator;
import java.util.Set;
import java.util.Vector;

import gnu.trove.map.TObjectIntMap;
import gnu.trove.map.hash.TObjectIntHashMap;

import org.spout.physics.Configuration;
import org.spout.physics.Utilities.IntPair;
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
	private final boolean mIsDeactivationActive;
	private final Set<RigidBody> mRigidBodies = new HashSet<RigidBody>();
	private final Vector<ContactManifold> mContactManifolds = new Vector<ContactManifold>();
	private final Vector<Constraint> mConstraints = new Vector<Constraint>();
	private final Vector3 mGravity;
	private boolean mIsGravityOn;
	private final Vector<Vector3> mConstrainedLinearVelocities = new Vector<Vector3>();
	private final Vector<Vector3> mConstrainedAngularVelocities = new Vector<Vector3>();
	private final TObjectIntMap<RigidBody> mMapBodyToConstrainedVelocityIndex = new TObjectIntHashMap<RigidBody>();

	/**
	 * Constructs a new dynamics world from the gravity and the default time step.
	 *
	 * @param gravity The gravity
	 */
	public DynamicsWorld(Vector3 gravity) {
		this(gravity, Configuration.DEFAULT_TIMESTEP);
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
		mIsGravityOn = true;
		mContactSolver = new ContactSolver(
				this,
				mConstrainedLinearVelocities,
				mConstrainedAngularVelocities,
				mMapBodyToConstrainedVelocityIndex);
		mIsDeactivationActive = Configuration.DEACTIVATION_ENABLED;
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
		System.out.println("Stop Simulation");
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
	public void setIsSplitImpulseActive(boolean isActive) {
		mContactSolver.setIsSplitImpulseActive(isActive);
	}

	/**
	 * Activates or deactivates the solving of friction constraints at the center of the contact
	 * manifold instead of solving them at each contact point.
	 *
	 * @param isActive Whether or not to solve the friction constraint at the center of the manifold
	 */
	public void setIsSolveFrictionAtContactManifoldCenterActive(boolean isActive) {
		mContactSolver.setIsSolveFrictionAtContactManifoldCenterActive(isActive);
	}

	// Resets the boolean movement variable for each body.
	private void resetBodiesMovementVariable() {
		for (Iterator<RigidBody> iterator = getRigidBodiesBeginIterator(); iterator.hasNext(); ) {
			iterator.next().setHasMoved(false);
		}
	}

	@Override
	public void updateOverlappingPair(BroadPhasePair pair) {
		final IntPair indexPair = pair.getBodiesIndexPair();
		final OverlappingPair overlappingPair = mOverlappingPairs.get(indexPair);
		overlappingPair.update();
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
	public boolean getIsGravityOn() {
		return mIsGravityOn;
	}

	/**
	 * Sets the gravity on if true, off is false.
	 *
	 * @param isGravityOn True to turn on the gravity, false to turn it off
	 */
	public void setIsGravityOn(boolean isGravityOn) {
		mIsGravityOn = isGravityOn;
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
	 * Gets an iterator to the beginning of the bodies of the physics world.
	 *
	 * @return The iterator for the rigid bodies
	 */
	public Iterator<RigidBody> getRigidBodiesBeginIterator() {
		return mRigidBodies.iterator();
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
	 * Gets the iterator on the constraint list.
	 *
	 * @return The iterator for the constraints
	 */
	public Iterator<Constraint> getConstraintsBeginIterator() {
		return mConstraints.iterator();
	}

	/**
	 * Gets the iterator on the contact manifolds list.
	 *
	 * @return The iterator for the contact manifolds
	 */
	public Iterator<ContactManifold> getContactManifoldsBeginIterator() {
		return mContactManifolds.iterator();
	}

	/**
	 * Updates the physics simulation.
	 */
	public void update() {
		if (!mTimer.getIsRunning()) {
			throw new IllegalStateException("time must be running");
		}
		mTimer.update();
		applyGravity();
		while (mTimer.isPossibleToTakeStep()) {
			mContactManifolds.clear();
			mCollisionDetection.computeCollisionDetection();
			initConstrainedVelocitiesArray();
			if (!mContactManifolds.isEmpty()) {
				mContactSolver.solve((float) mTimer.getTimeStep());
			}
			mTimer.nextStep();
			resetBodiesMovementVariable();
			updateRigidBodiesPositionAndOrientation();
			mContactSolver.cleanup();
			cleanupConstrainedVelocitiesArray();
		}
		setInterpolationFactorToAllBodies();
	}

	// Updates the position and orientation of the rigid bodies.
	private void updateRigidBodiesPositionAndOrientation() {
		final float dt = (float) mTimer.getTimeStep();
		RigidBody rigidBody;
		for (Iterator<RigidBody> iterator = getRigidBodiesBeginIterator(); iterator.hasNext(); ) {
			rigidBody = iterator.next();
			if (rigidBody == null) {
				throw new IllegalStateException("rigid body cannot be null");
			}
			if (rigidBody.getIsMotionEnabled()) {
				rigidBody.updateOldTransform();
				final int indexArray = mMapBodyToConstrainedVelocityIndex.get(rigidBody);
				final Vector3 newLinVelocity = mConstrainedLinearVelocities.get(indexArray);
				final Vector3 newAngVelocity = mConstrainedAngularVelocities.get(indexArray);
				rigidBody.setLinearVelocity(newLinVelocity);
				rigidBody.setAngularVelocity(newAngVelocity);
				if (mContactSolver.isConstrainedBody(rigidBody)) {
					newLinVelocity.add(mContactSolver.getSplitLinearVelocityOfBody(rigidBody));
					newAngVelocity.add(mContactSolver.getSplitAngularVelocityOfBody(rigidBody));
				}
				final Vector3 currentPosition = rigidBody.getTransform().getPosition();
				final Quaternion currentOrientation = rigidBody.getTransform().getOrientation();
				final Vector3 newPosition = Vector3.add(currentPosition, Vector3.multiply(newLinVelocity, dt));
				final Quaternion newOrientation = Quaternion.add(
						currentOrientation,
						Quaternion.multiply(
								Quaternion.multiply(
										new Quaternion(newAngVelocity.getX(), newAngVelocity.getY(), newAngVelocity.getZ(), 0),
										currentOrientation),
								0.5f * dt));
				final Transform newTransform = new Transform(newPosition, newOrientation.getUnit());
				rigidBody.setTransform(newTransform);
				rigidBody.updateAABB();
			}
		}
	}

	// Computes and set the interpolation factor for all bodies.
	private void setInterpolationFactorToAllBodies() {
		final float factor = mTimer.computeInterpolationFactor();
		if (factor < 0 && factor > 1.0) {
			throw new IllegalStateException("interpolation factor must be greater or equal to zero"
					+ " and smaller or equal to one");
		}
		RigidBody rigidBody;
		for (Iterator<RigidBody> iterator = getRigidBodiesBeginIterator(); iterator.hasNext(); ) {
			rigidBody = iterator.next();
			if (rigidBody == null) {
				throw new IllegalStateException("rigid body cannot be null");
			}
			rigidBody.setInterpolationFactor(factor);
		}
	}

	// Initializes the constrained velocities array at each step.
	private void initConstrainedVelocitiesArray() {
		mConstrainedLinearVelocities.clear();
		mConstrainedLinearVelocities.ensureCapacity(mRigidBodies.size());
		mConstrainedAngularVelocities.clear();
		mConstrainedAngularVelocities.ensureCapacity(mRigidBodies.size());
		final float dt = (float) mTimer.getTimeStep();
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
		RigidBody rigidBody;
		for (Iterator<RigidBody> iterator = getRigidBodiesBeginIterator(); iterator.hasNext(); ) {
			rigidBody = iterator.next();
			if (rigidBody == null) {
				throw new IllegalStateException("rigid body cannot be null");
			}
			if (mIsGravityOn) {
				rigidBody.setExternalForce(Vector3.multiply(rigidBody.getMass(), mGravity));
			}
		}
	}

	/**
	 * Creates a rigid body and adds it to the physics world.
	 *
	 * @param transform The transform (position and orientation) of the body
	 * @param mass The mass of the body
	 * @param inertiaTensorLocal The local inertia tensor
	 * @param collisionShape The collision shape
	 * @return The new rigid body
	 */
	public RigidBody createRigidBody(Transform transform, float mass, Matrix3x3 inertiaTensorLocal,
									 CollisionShape collisionShape) {
		final int bodyID = computeNextAvailableBodyID();
		if (bodyID >= Integer.MAX_VALUE) {
			throw new IllegalStateException("body id cannot be larger or equal to the largest interger");
		}
		final RigidBody rigidBody = new RigidBody(transform, mass, inertiaTensorLocal, collisionShape, bodyID);
		mBodies.add(rigidBody);
		mRigidBodies.add(rigidBody);
		mCollisionDetection.addBody(rigidBody);
		return rigidBody;
	}

	/**
	 * Destroys a rigid body.
	 *
	 * @param rigidBody The rigid body to destroy
	 */
	public void destroyRigidBody(RigidBody rigidBody) {
		mCollisionDetection.removeBody(rigidBody);
		mFreeBodiesIDs.add(rigidBody.getID());
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
