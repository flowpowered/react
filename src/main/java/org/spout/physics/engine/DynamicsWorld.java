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

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import gnu.trove.map.TObjectIntMap;
import gnu.trove.map.hash.TObjectIntHashMap;

import org.spout.physics.ReactDefaults;
import org.spout.physics.ReactDefaults.ContactsPositionCorrectionTechnique;
import org.spout.physics.Utilities.IntPair;
import org.spout.physics.body.RigidBody;
import org.spout.physics.collision.BroadPhasePair;
import org.spout.physics.collision.shape.CollisionShape;
import org.spout.physics.constraint.BallAndSocketJoint;
import org.spout.physics.constraint.BallAndSocketJoint.BallAndSocketJointInfo;
import org.spout.physics.constraint.Constraint;
import org.spout.physics.constraint.Constraint.ConstraintInfo;
import org.spout.physics.constraint.ConstraintSolver;
import org.spout.physics.constraint.ContactPoint;
import org.spout.physics.constraint.ContactPoint.ContactPointInfo;
import org.spout.physics.constraint.FixedJoint;
import org.spout.physics.constraint.FixedJoint.FixedJointInfo;
import org.spout.physics.constraint.HingeJoint;
import org.spout.physics.constraint.HingeJoint.HingeJointInfo;
import org.spout.physics.constraint.SliderJoint;
import org.spout.physics.constraint.SliderJoint.SliderJointInfo;
import org.spout.physics.math.Matrix3x3;
import org.spout.physics.math.Quaternion;
import org.spout.physics.math.Transform;
import org.spout.physics.math.Vector3;

/**
 * This class represents a dynamics world. This class inherits from the CollisionWorld class. In a dynamics world bodies can collide and their movements are simulated using the laws of physics.
 */
public class DynamicsWorld extends CollisionWorld {
    private final Timer mTimer;
    private final ContactSolver mContactSolver;
    private final ConstraintSolver mConstraintSolver;
    private int mNbVelocitySolverIterations;
    private int mNbPositionSolverIterations;
    private boolean mIsDeactivationActive;
    private final Set<RigidBody> mRigidBodies = new HashSet<>();
    private final List<ContactManifold> mContactManifolds = new ArrayList<>();
    private final Set<Constraint> mJoints = new HashSet<>();
    private final Vector3 mGravity;
    private boolean mIsGravityOn = true;
    private final ArrayList<Vector3> mConstrainedLinearVelocities = new ArrayList<>();
    private final ArrayList<Vector3> mConstrainedAngularVelocities = new ArrayList<>();
    private final ArrayList<Vector3> mConstrainedPositions = new ArrayList<>();
    private final ArrayList<Quaternion> mConstrainedOrientations = new ArrayList<>();
    private final TObjectIntMap<RigidBody> mMapBodyToConstrainedVelocityIndex = new TObjectIntHashMap<>();
    private boolean isTicking = false;
    // Tick cache
    private final Set<RigidBody> mRigidBodiesToAddCache = new HashSet<>();
    private final Set<RigidBody> mRigidBodiesToDeleteCache = new HashSet<>();

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
        mContactSolver = new ContactSolver(mContactManifolds, mConstrainedLinearVelocities, mConstrainedAngularVelocities, mMapBodyToConstrainedVelocityIndex);
        mConstraintSolver = new ConstraintSolver(mJoints, mConstrainedLinearVelocities, mConstrainedAngularVelocities, mConstrainedPositions, mConstrainedOrientations,
                mMapBodyToConstrainedVelocityIndex);
        mNbVelocitySolverIterations = ReactDefaults.DEFAULT_VELOCITY_SOLVER_NB_ITERATIONS;
        mNbPositionSolverIterations = ReactDefaults.DEFAULT_POSITION_SOLVER_NB_ITERATIONS;
        mIsDeactivationActive = ReactDefaults.DEACTIVATION_ENABLED;
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
     * Sets the number of iterations for the velocity constraint solver.
     *
     * @param nbIterations The number of iterations to do
     */
    public void setNbIterationsVelocitySolver(int nbIterations) {
        mNbVelocitySolverIterations = nbIterations;
    }

    /**
     * Sets the number of iterations for the position constraint solver.
     *
     * @param nbIterations The number of iterations to do
     */
    public void setNbIterationsPositionSolver(int nbIterations) {
        mNbPositionSolverIterations = nbIterations;
    }

    /**
     * Sets the position correction technique used for contacts.
     *
     * @param technique The technique to use
     */
    public void setContactsPositionCorrectionTechnique(ContactsPositionCorrectionTechnique technique) {
        if (technique == ContactsPositionCorrectionTechnique.BAUMGARTE_CONTACTS) {
            mContactSolver.setIsSplitImpulseActive(false);
        } else {
            mContactSolver.setIsSplitImpulseActive(true);
        }
    }

    /**
     * Activates or deactivates the solving of friction constraints at the center of the contact manifold instead of solving them at each contact point.
     *
     * @param isActive Whether or not to solve the friction constraint at the center of the manifold
     */
    public void setSolveFrictionAtContactManifoldCenterActive(boolean isActive) {
        mContactSolver.setSolveFrictionAtContactManifoldCenterActive(isActive);
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
     * Returns the number of joints in the world.
     *
     * @return The number of joints
     */
    public int getNbJoints() {
        return mJoints.size();
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
     * Returns the current physics time (in seconds)
     *
     * @return The current physics time
     */
    public double getPhysicsTime() {
        return mTimer.getPhysicsTime();
    }

    /**
     * Updates the physics simulation. The elapsed time is determined by the timer. A step is only actually taken if enough time has passed. If a lot of time has passed, more than twice the time step,
     * multiple steps will be taken, to catch up.
     */
    public void update() {
        if (!mTimer.isRunning()) {
            throw new IllegalStateException("timer must be running");
        }
        mTimer.update();
        isTicking = true;
        while (mTimer.isPossibleToTakeStep()) {
            mContactManifolds.clear();
            mCollisionDetection.computeCollisionDetection();
            integrateRigidBodiesVelocities();
            resetBodiesMovementVariable();
            mTimer.nextStep();
            solveContactsAndConstraints();
            integrateRigidBodiesPositions();
            solvePositionCorrection();
            updateRigidBodiesAABB();
            mContactSolver.cleanup();
            cleanupConstrainedVelocitiesArray();
        }
        isTicking = false;
        setInterpolationFactorToAllBodies();
        disperseCache();
    }

    // Resets the boolean movement variable for each body.
    private void resetBodiesMovementVariable() {
        for (RigidBody rigidBody : mRigidBodies) {
            rigidBody.setHasMoved(false);
        }
    }

    // Integrates the position and orientation of the rigid bodies using the provided time delta.
    // The positions and orientations of the bodies are integrated using the symplectic Euler time stepping scheme.
    private void integrateRigidBodiesPositions() {
        final float dt = (float) mTimer.getTimeStep();
        for (RigidBody rigidBody : mRigidBodies) {
            if (rigidBody.getIsMotionEnabled()) {
                final int indexArray = mMapBodyToConstrainedVelocityIndex.get(rigidBody);
                final Vector3 newLinVelocity = mConstrainedLinearVelocities.get(indexArray);
                final Vector3 newAngVelocity = mConstrainedAngularVelocities.get(indexArray);
                rigidBody.setLinearVelocity(newLinVelocity);
                rigidBody.setAngularVelocity(newAngVelocity);
                if (mContactSolver.isConstrainedBody(rigidBody) && mContactSolver.isSplitImpulseActive()) {
                    newLinVelocity.add(mContactSolver.getSplitLinearVelocityOfBody(rigidBody));
                    newAngVelocity.add(mContactSolver.getSplitAngularVelocityOfBody(rigidBody));
                }
                final Vector3 currentPosition = rigidBody.getTransform().getPosition();
                final Quaternion currentOrientation = rigidBody.getTransform().getOrientation();
                final Vector3 newPosition = Vector3.add(currentPosition, Vector3.multiply(newLinVelocity, dt));
                final Quaternion newOrientation = Quaternion.add(currentOrientation, Quaternion.multiply(Quaternion.multiply(new Quaternion(0, newAngVelocity), currentOrientation), 0.5f * dt));
                final Transform newTransform = new Transform(newPosition, newOrientation.getUnit());
                rigidBody.setTransform(newTransform);
            }
        }
    }

    // Updates the AABBs of the bodies
    private void updateRigidBodiesAABB() {
        for (RigidBody rigidBody : mRigidBodies) {
            if (rigidBody.getHasMoved()) {
                rigidBody.updateAABB();
            }
        }
    }

    // Computes and set the interpolation factor for all bodies.
    private void setInterpolationFactorToAllBodies() {
        final float factor = mTimer.computeInterpolationFactor();
        if (factor < 0 && factor > 1) {
            throw new IllegalStateException("interpolation factor must be greater or equal to zero"
                    + " and smaller or equal to one");
        }
        for (RigidBody rigidBody : mRigidBodies) {
            if (rigidBody == null) {
                throw new IllegalStateException("rigid body cannot be null");
            }
            rigidBody.setInterpolationFactor(factor);
        }
    }

    // Integrates the constrained velocities array using the provided time delta.
    // This method only sets the temporary velocities and does not update
    // the actual velocities of the bodies. The velocities updated in this method
    // might violate the constraints and will be corrected in the constraint and
    // contact solver.
    private void integrateRigidBodiesVelocities() {
        mConstrainedLinearVelocities.clear();
        mConstrainedLinearVelocities.ensureCapacity(mRigidBodies.size());
        mConstrainedAngularVelocities.clear();
        mConstrainedAngularVelocities.ensureCapacity(mRigidBodies.size());
        final float dt = (float) mTimer.getTimeStep();
        int i = 0;
        for (RigidBody rigidBody : mRigidBodies) {
            mMapBodyToConstrainedVelocityIndex.put(rigidBody, i);
            if (rigidBody.getIsMotionEnabled()) {
                mConstrainedLinearVelocities.add(i, Vector3.add(
                        rigidBody.getLinearVelocity(),
                        Vector3.multiply(dt * rigidBody.getMassInverse(), rigidBody.getExternalForce())));
                mConstrainedAngularVelocities.add(i, Vector3.add(
                        rigidBody.getAngularVelocity(),
                        Matrix3x3.multiply(
                                Matrix3x3.multiply(dt, rigidBody.getInertiaTensorInverseWorld()),
                                rigidBody.getExternalTorque())
                ));
                if (rigidBody.isGravityEnabled() && mIsGravityOn) {
                    mConstrainedLinearVelocities.get(i).add(Vector3.multiply(dt * rigidBody.getMassInverse() * rigidBody.getMass(), mGravity));
                }
                rigidBody.updateOldTransform();
            } else {
                mConstrainedLinearVelocities.add(i, new Vector3(0, 0, 0));
                mConstrainedAngularVelocities.add(i, new Vector3(0, 0, 0));
            }
            i++;
        }
        if (mMapBodyToConstrainedVelocityIndex.size() != mRigidBodies.size()) {
            throw new IllegalStateException("The size of the map from body to constrained velocity index should be the same as the number of rigid bodies");
        }
    }

    // Solves the contacts and constraints
    private void solveContactsAndConstraints() {
        final float dt = (float) mTimer.getTimeStep();
        final boolean isConstraintsToSolve = !mJoints.isEmpty();
        final boolean isContactsToSolve = !mContactManifolds.isEmpty();
        if (!isConstraintsToSolve && !isContactsToSolve) {
            return;
        }
        if (isContactsToSolve) {
            mContactSolver.initialize(dt);
            mContactSolver.warmStart();
        }
        if (isConstraintsToSolve) {
            mConstraintSolver.initialize(dt);
        }
        for (int i = 0; i < mNbVelocitySolverIterations; i++) {
            if (isConstraintsToSolve) {
                mConstraintSolver.solveVelocityConstraints();
            }
            if (isContactsToSolve) {
                mContactSolver.solve();
            }
        }
        if (isContactsToSolve) {
            mContactSolver.storeImpulses();
        }
    }

    public void solvePositionCorrection() {
        if (mJoints.isEmpty()) {
            return;
        }
        // TODO : Use better memory allocation here
        mConstrainedPositions.clear();
        mConstrainedPositions.ensureCapacity(mRigidBodies.size());
        mConstrainedOrientations.clear();
        mConstrainedOrientations.ensureCapacity(mRigidBodies.size());
        for (int i = 0; i < mRigidBodies.size(); i++) {
            mConstrainedPositions.add(null);
            mConstrainedOrientations.add(null);
        }
        for (RigidBody rigidBody : mRigidBodies) {
            if (mConstraintSolver.isConstrainedBody(rigidBody)) {
                final int index = mMapBodyToConstrainedVelocityIndex.get(rigidBody);
                final Transform transform = rigidBody.getTransform();
                mConstrainedPositions.set(index, new Vector3(transform.getPosition()));
                mConstrainedOrientations.set(index, new Quaternion(transform.getOrientation()));
            }
        }
        for (int i = 0; i < mNbPositionSolverIterations; i++) {
            mConstraintSolver.solvePositionConstraints();
        }
        for (RigidBody rigidBody : mRigidBodies) {
            if (mConstraintSolver.isConstrainedBody(rigidBody)) {
                final int index = mMapBodyToConstrainedVelocityIndex.get(rigidBody);
                final Vector3 newPosition = mConstrainedPositions.get(index);
                final Quaternion newOrientation = mConstrainedOrientations.get(index);
                final Transform newTransform = new Transform(newPosition, newOrientation.getUnit());
                rigidBody.setTransform(newTransform);
            }
        }
    }

    // Cleans up the constrained velocities array at each step.
    private void cleanupConstrainedVelocitiesArray() {
        mConstrainedLinearVelocities.clear();
        mConstrainedAngularVelocities.clear();
        mMapBodyToConstrainedVelocityIndex.clear();
    }

    /**
     * Creates a mobile rigid body and adds it to the physics world. The inertia tensor will be computed from the shape and mass.
     *
     * @param transform The transform (position and orientation) of the body
     * @param mass The mass of the body
     * @param collisionShape The collision shape
     * @return The new rigid body
     */
    public RigidBody createRigidBody(Transform transform, float mass, CollisionShape collisionShape) {
        final Matrix3x3 inertiaTensor = new Matrix3x3();
        collisionShape.computeLocalInertiaTensor(inertiaTensor, mass);
        return createRigidBody(transform, mass, inertiaTensor, collisionShape);
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
    public RigidBody createRigidBody(Transform transform, float mass, Matrix3x3 inertiaTensorLocal, CollisionShape collisionShape) {
        final CollisionShape newCollisionShape = createCollisionShape(collisionShape);
        final RigidBody mobileBody = new RigidBody(transform, mass, inertiaTensorLocal, newCollisionShape, getNextFreeID());
        addRigidBody(mobileBody);
        return mobileBody;
    }

    /**
     * Adds a rigid body to the body collections and the collision detection, even if a tick is in progress.
     *
     * @param body The body to add
     */
    protected void addRigidBodyIgnoreTick(RigidBody body) {
        mBodies.add(body);
        mRigidBodies.add(body);
        mCollisionDetection.addBody(body);
    }

    /**
     * Adds a rigid body to the body collections and the collision detection. If a tick is in progress, the body is added at the end.
     *
     * @param body The body to add
     */
    public void addRigidBody(RigidBody body) {
        if (!isTicking) {
            mBodies.add(body);
            mRigidBodies.add(body);
            mCollisionDetection.addBody(body);
        } else {
            mRigidBodiesToAddCache.add(body);
        }
    }

    /**
     * Destroys a rigid body and all the joints to which it belongs.
     *
     * @param rigidBody The rigid body to destroy
     */
    public void destroyRigidBody(RigidBody rigidBody) {
        if (!isTicking) {
            mCollisionDetection.removeBody(rigidBody);
            mFreeBodiesIDs.push(rigidBody.getID());
            mBodies.remove(rigidBody);
            mRigidBodies.remove(rigidBody);
            removeCollisionShape(rigidBody.getCollisionShape());
            final int idToRemove = rigidBody.getID();
            for (Constraint joint : mJoints) {
                if (joint.getFirstBody().getID() == idToRemove || joint.getSecondBody().getID() == idToRemove) {
                    destroyJoint(joint);
                }
            }
        } else {
            mRigidBodiesToDeleteCache.add(rigidBody);
        }
    }

    /**
     * Creates a joint between two bodies in the world and returns the new joint.
     *
     * @param jointInfo The information to use for creating the joint
     * @return The new joint
     */
    public Constraint createJoint(ConstraintInfo jointInfo) {
        final Constraint newJoint;
        switch (jointInfo.getType()) {
            case BALLSOCKETJOINT: {
                final BallAndSocketJointInfo info = (BallAndSocketJointInfo) jointInfo;
                newJoint = new BallAndSocketJoint(info);
                break;
            }
            case SLIDERJOINT: {
                final SliderJointInfo info = (SliderJointInfo) jointInfo;
                newJoint = new SliderJoint(info);
                break;
            }
            case HINGEJOINT: {
                final HingeJointInfo info = (HingeJointInfo) jointInfo;
                newJoint = new HingeJoint(info);
                break;
            }
            case FIXEDJOINT: {
                final FixedJointInfo info = (FixedJointInfo) jointInfo;
                newJoint = new FixedJoint(info);
                break;
            }
            default:
                throw new IllegalArgumentException("Unsupported joint type +" + jointInfo.getType());
        }
        if (!jointInfo.isCollisionEnabled()) {
            mCollisionDetection.addNoCollisionPair(jointInfo.getFirstBody(), jointInfo.getSecondBody());
        }
        mJoints.add(newJoint);
        return newJoint;
    }

    /**
     * Destroys a joint.
     *
     * @param joint The joint to destroy
     */
    public void destroyJoint(Constraint joint) {
        if (joint == null) {
            throw new IllegalArgumentException("Joint cannot be null");
        }
        if (!joint.isCollisionEnabled()) {
            mCollisionDetection.removeNoCollisionPair(joint.getFirstBody(), joint.getSecondBody());
        }
        mJoints.remove(joint);
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
    public void notifyNewContact(BroadPhasePair broadPhasePair, ContactPointInfo contactInfo) {
        final ContactPoint contact = new ContactPoint(contactInfo);
        final IntPair indexPair = broadPhasePair.getBodiesIndexPair();
        final OverlappingPair overlappingPair = mOverlappingPairs.get(indexPair);
        if (overlappingPair == null) {
            throw new IllegalArgumentException("broad phase pair is not in the overlapping pairs");
        }
        overlappingPair.addContact(contact);
        mContactManifolds.add(overlappingPair.getContactManifold());
    }

    /**
     * Returns if the world is currently undertaking a tick
     *
     * @return True if ticking, false if not
     */
    public boolean isTicking() {
        return isTicking;
    }

    /**
     * Disperses the cache of bodies added/removed during the physics tick.
     */
    public void disperseCache() {
        mRigidBodiesToAddCache.removeAll(mRigidBodiesToDeleteCache);
        for (RigidBody body : mRigidBodiesToDeleteCache) {
            destroyRigidBody(body);
        }
        for (RigidBody body : mRigidBodiesToAddCache) {
            addRigidBody(body);
        }
        mRigidBodiesToAddCache.clear();
        mRigidBodiesToDeleteCache.clear();
    }
}
