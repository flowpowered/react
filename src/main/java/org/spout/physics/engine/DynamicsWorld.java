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
import org.spout.physics.body.CollisionBody;
import org.spout.physics.body.RigidBody;
import org.spout.physics.collision.BroadPhasePair;
import org.spout.physics.collision.shape.CollisionShape;
import org.spout.physics.constraint.BallAndSocketJoint;
import org.spout.physics.constraint.BallAndSocketJoint.BallAndSocketJointInfo;
import org.spout.physics.constraint.Constraint;
import org.spout.physics.constraint.Constraint.ConstraintInfo;
import org.spout.physics.constraint.Constraint.JointListElement;
import org.spout.physics.constraint.ConstraintSolver;
import org.spout.physics.constraint.ContactPoint;
import org.spout.physics.constraint.ContactPoint.ContactPointInfo;
import org.spout.physics.constraint.FixedJoint;
import org.spout.physics.constraint.FixedJoint.FixedJointInfo;
import org.spout.physics.constraint.HingeJoint;
import org.spout.physics.constraint.HingeJoint.HingeJointInfo;
import org.spout.physics.constraint.SliderJoint;
import org.spout.physics.constraint.SliderJoint.SliderJointInfo;
import org.spout.physics.engine.ContactManifold.ContactManifoldListElement;
import org.spout.physics.math.Mathematics;
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
    private boolean mIsSleepingEnabled;
    private final Set<RigidBody> mRigidBodies = new HashSet<>();
    private final List<ContactManifold> mContactManifolds = new ArrayList<>();
    private final Set<Constraint> mJoints = new HashSet<>();
    private final Vector3 mGravity;
    private boolean mIsGravityOn;
    private Vector3[] mConstrainedLinearVelocities;
    private Vector3[] mConstrainedAngularVelocities;
    private Vector3[] mSplitLinearVelocities;
    private Vector3[] mSplitAngularVelocities;
    private final ArrayList<Vector3> mConstrainedPositions = new ArrayList<>();
    private final ArrayList<Quaternion> mConstrainedOrientations = new ArrayList<>();
    private final TObjectIntMap<RigidBody> mMapBodyToConstrainedVelocityIndex = new TObjectIntHashMap<>();
    private int mNbIslands;
    private int mNbIslandsCapacity;
    private Island[] mIslands;
    private int mNbBodiesCapacity;
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
        mIsGravityOn = true;
        mConstrainedLinearVelocities = null;
        mConstrainedAngularVelocities = null;
        mContactSolver = new ContactSolver(mMapBodyToConstrainedVelocityIndex);
        mConstraintSolver = new ConstraintSolver(mConstrainedPositions, mConstrainedOrientations, mMapBodyToConstrainedVelocityIndex);
        mNbVelocitySolverIterations = ReactDefaults.DEFAULT_VELOCITY_SOLVER_NB_ITERATIONS;
        mNbPositionSolverIterations = ReactDefaults.DEFAULT_POSITION_SOLVER_NB_ITERATIONS;
        mIsSleepingEnabled = ReactDefaults.SLEEPING_ENABLED;
        mSplitLinearVelocities = null;
        mSplitAngularVelocities = null;
        mNbIslands = 0;
        mNbIslandsCapacity = 0;
        mIslands = null;
        mNbBodiesCapacity = 0;
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
     * Returns a reference to the contact manifolds of the world.
     *
     * @return The contact manifolds
     */
    public List<ContactManifold> getContactManifolds() {
        return mContactManifolds;
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
            resetContactManifoldListsOfBodies();
            mCollisionDetection.computeCollisionDetection();
            integrateRigidBodiesVelocities();
            resetBodiesMovementVariable();
            mTimer.nextStep();
            computeIslands();
            solveContactsAndConstraints();
            integrateRigidBodiesPositions();
            solvePositionCorrection();
            updateRigidBodiesAABB();
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
                final Vector3 newLinVelocity = mConstrainedLinearVelocities[indexArray];
                final Vector3 newAngVelocity = mConstrainedAngularVelocities[indexArray];
                rigidBody.setLinearVelocity(newLinVelocity);
                rigidBody.setAngularVelocity(newAngVelocity);
                if (mContactSolver.isSplitImpulseActive()) {
                    newLinVelocity.add(mSplitLinearVelocities[indexArray]);
                    newAngVelocity.add(mSplitAngularVelocities[indexArray]);
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

    // Initialize the bodies velocities arrays for the next simulation step.
    private void initVelocityArrays() {
        final int nbBodies = mRigidBodies.size();
        if (mNbBodiesCapacity != nbBodies && nbBodies > 0) {
            mNbBodiesCapacity = nbBodies;
            mSplitLinearVelocities = new Vector3[mNbBodiesCapacity];
            mSplitAngularVelocities = new Vector3[mNbBodiesCapacity];
            mConstrainedLinearVelocities = new Vector3[mNbBodiesCapacity];
            mConstrainedAngularVelocities = new Vector3[mNbBodiesCapacity];
        }
        for (int i = 0; i < mNbBodiesCapacity; i++) {
            mSplitLinearVelocities[i] = new Vector3(0, 0, 0);
            mSplitAngularVelocities[i] = new Vector3(0, 0, 0);
        }
    }

    // Integrates the constrained velocities array using the provided time delta.
    // This method only sets the temporary velocities and does not update
    // the actual velocities of the bodies. The velocities updated in this method
    // might violate the constraints and will be corrected in the constraint and
    // contact solver.
    private void integrateRigidBodiesVelocities() {
        initVelocityArrays();
        final float dt = (float) mTimer.getTimeStep();
        int i = 0;
        mMapBodyToConstrainedVelocityIndex.clear();
        for (RigidBody rigidBody : mRigidBodies) {
            mMapBodyToConstrainedVelocityIndex.put(rigidBody, i);
            if (rigidBody.getIsMotionEnabled()) {
                mConstrainedLinearVelocities[i] = Vector3.add(
                        rigidBody.getLinearVelocity(),
                        Vector3.multiply(dt * rigidBody.getMassInverse(), rigidBody.getExternalForce()));
                mConstrainedAngularVelocities[i] = Vector3.add(
                        rigidBody.getAngularVelocity(),
                        Matrix3x3.multiply(
                                Matrix3x3.multiply(dt, rigidBody.getInertiaTensorInverseWorld()),
                                rigidBody.getExternalTorque()));
                if (rigidBody.isGravityEnabled() && mIsGravityOn) {
                    mConstrainedLinearVelocities[i].add(Vector3.multiply(dt * rigidBody.getMassInverse() * rigidBody.getMass(), mGravity));
                }
                final float linDampingFactor = rigidBody.getLinearDamping();
                final float angDampingFactor = rigidBody.getAngularDamping();
                final float linearDamping = Mathematics.clamp(1 - dt * linDampingFactor, 0, 1);
                final float angularDamping = Mathematics.clamp(1 - dt * angDampingFactor, 0, 1);
                mConstrainedLinearVelocities[i].multiply(Mathematics.clamp(linearDamping, 0, 1));
                mConstrainedAngularVelocities[i].multiply(Mathematics.clamp(angularDamping, 0, 1));
                rigidBody.updateOldTransform();
            } else {
                mConstrainedLinearVelocities[i] = new Vector3(0, 0, 0);
                mConstrainedAngularVelocities[i] = new Vector3(0, 0, 0);
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
        mContactSolver.setSplitVelocitiesArrays(mSplitLinearVelocities, mSplitAngularVelocities);
        mContactSolver.setConstrainedVelocitiesArrays(mConstrainedLinearVelocities, mConstrainedAngularVelocities);
        mConstraintSolver.setConstrainedVelocitiesArrays(mConstrainedLinearVelocities, mConstrainedAngularVelocities);
        for (int islandIndex = 0; islandIndex < mNbIslands; islandIndex++) {
            final boolean isConstraintsToSolve = mIslands[islandIndex].getNbJoints() > 0;
            final boolean isContactsToSolve = mIslands[islandIndex].getNbContactManifolds() > 0;
            if (!isConstraintsToSolve && !isContactsToSolve) {
                continue;
            }
            if (isContactsToSolve) {
                mContactSolver.initializeForIsland(dt, mIslands[islandIndex]);
                mContactSolver.warmStart();
            }
            if (isConstraintsToSolve) {
                mConstraintSolver.initializeForIsland(dt, mIslands[islandIndex]);
            }
            for (int i = 0; i < mNbVelocitySolverIterations; i++) {
                if (isConstraintsToSolve) {
                    mConstraintSolver.solveVelocityConstraints(mIslands[islandIndex]);
                }
                if (isContactsToSolve) {
                    mContactSolver.solve();
                }
            }
            if (isContactsToSolve) {
                mContactSolver.storeImpulses();
                mContactSolver.cleanup();
            }
        }
    }

    public void solvePositionCorrection() {
        if (mJoints.isEmpty()) {
            return;
        }
        mConstrainedPositions.clear();
        mConstrainedPositions.ensureCapacity(mRigidBodies.size());
        mConstrainedOrientations.clear();
        mConstrainedOrientations.ensureCapacity(mRigidBodies.size());
        for (int i = 0; i < mRigidBodies.size(); i++) {
            mConstrainedPositions.add(null);
            mConstrainedOrientations.add(null);
        }
        for (int islandIndex = 0; islandIndex < mNbIslands; islandIndex++) {
            final RigidBody[] bodies = mIslands[islandIndex].getBodies();
            for (int b = 0; b < mIslands[islandIndex].getNbBodies(); b++) {
                final int index = mMapBodyToConstrainedVelocityIndex.get(bodies[b]);
                final Transform transform = bodies[b].getTransform();
                mConstrainedPositions.set(index, new Vector3(transform.getPosition()));
                mConstrainedOrientations.set(index, new Quaternion(transform.getOrientation()));
            }
            for (int i = 0; i < mNbPositionSolverIterations; i++) {
                mConstraintSolver.solvePositionConstraints(mIslands[islandIndex]);
            }
            for (int b = 0; b < mIslands[islandIndex].getNbBodies(); b++) {
                final int index = mMapBodyToConstrainedVelocityIndex.get(bodies[b]);
                final Vector3 newPosition = mConstrainedPositions.get(index);
                final Quaternion newOrientation = mConstrainedOrientations.get(index);
                final Transform newTransform = new Transform(newPosition, newOrientation.getUnit());
                bodies[b].setTransform(newTransform);
            }
        }
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
            rigidBody.resetContactManifoldsList();
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
        addJointToBody(newJoint);
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
        joint.getFirstBody().removeJointFromJointsList(joint);
        joint.getSecondBody().removeJointFromJointsList(joint);
    }

    /**
     * Adds the joint to the list of joints of the two bodies involved in the joint.
     *
     * @param joint The joint to add
     */
    public void addJointToBody(Constraint joint) {
        if (joint == null) {
            throw new IllegalArgumentException("Joint cannot be null");
        }
        final JointListElement jointListElement1 = new JointListElement(joint, joint.getFirstBody().getJointsList());
        joint.getFirstBody().setJointsList(jointListElement1);
        final JointListElement jointListElement2 = new JointListElement(joint, joint.getSecondBody().getJointsList());
        joint.getSecondBody().setJointsList(jointListElement2);
    }

    /**
     * Adds a contact manifold to the linked list of contact manifolds of the two bodies involved in the corresponding contact.
     *
     * @param contactManifold The contact manifold to add
     * @param body1 The first body in the manifold
     * @param body2 The second body in the manifold
     */
    public void addContactManifoldToBody(ContactManifold contactManifold, CollisionBody body1, CollisionBody body2) {
        if (contactManifold == null) {
            throw new IllegalArgumentException("The contact manifold cannot be null");
        }
        final ContactManifoldListElement listElement1 = new ContactManifoldListElement(contactManifold, body1.getContactManifoldsLists());
        body1.setContactManifoldsList(listElement1);
        final ContactManifoldListElement listElement2 = new ContactManifoldListElement(contactManifold, body2.getContactManifoldsLists());
        body2.setContactManifoldsList(listElement2);
    }

    /**
     * Resets all the contact manifolds linked list of each body.
     */
    public void resetContactManifoldListsOfBodies() {
        for (RigidBody rigidBody : mRigidBodies) {
            rigidBody.resetContactManifoldsList();
        }
    }

    // Computes the islands of awake bodies.
    // An island is an isolated group of rigid bodies that have constraints (joints or contacts)
    // between each other. This method computes the islands at each time step as follows: For each
    // awake rigid body, we run a Depth First Search (DFS) through the constraint graph of that body
    // (graph where nodes are the bodies and where the edges are the constraints between the bodies) to
    // find all the bodies that are connected with it (the bodies that share joints or contacts with
    // it). Then, we create an island with this group of connected bodies.
    private void computeIslands() {
        final int nbBodies = mRigidBodies.size();
        for (int i = 0; i < mNbIslands; i++) {
            mIslands[i] = null;
        }
        if (mNbIslandsCapacity != nbBodies && nbBodies > 0) {
            mNbIslandsCapacity = nbBodies;
            mIslands = new Island[mNbIslandsCapacity];
        }
        mNbIslands = 0;
        for (RigidBody rigidBody : mRigidBodies) {
            rigidBody.setIsAlreadyInIsland(false);
        }
        for (ContactManifold contactManifold : mContactManifolds) {
            contactManifold.setIsAlreadyInIsland(false);
        }
        for (Constraint joint : mJoints) {
            joint.setIsAlreadyInIsland(false);
        }
        final RigidBody[] stackBodiesToVisit = new RigidBody[nbBodies];
        int idIsland = 0;
        for (RigidBody body : mRigidBodies) {
            if (body.isAlreadyInIsland()) {
                continue;
            }
            if (!body.getIsMotionEnabled()) {
                continue;
            }
            if (body.isSleeping()) {
                continue;
            }
            int stackIndex = 0;
            stackBodiesToVisit[stackIndex] = body;
            stackIndex++;
            body.setIsAlreadyInIsland(true);
            mIslands[mNbIslands] = new Island(idIsland, nbBodies, mContactManifolds.size(), mJoints.size());
            idIsland++;
            while (stackIndex > 0) {
                stackIndex--;
                final RigidBody bodyToVisit = stackBodiesToVisit[stackIndex];
                bodyToVisit.setIsSleeping(false);
                mIslands[mNbIslands].addBody(bodyToVisit);
                if (!bodyToVisit.getIsMotionEnabled()) {
                    continue;
                }
                ContactManifoldListElement contactElement;
                for (contactElement = bodyToVisit.getContactManifoldsLists(); contactElement != null;
                     contactElement = contactElement.getNext()) {
                    final ContactManifold contactManifold = contactElement.getContactManifold();
                    if (contactManifold.isAlreadyInIsland()) {
                        continue;
                    }
                    mIslands[mNbIslands].addContactManifold(contactManifold);
                    contactManifold.setIsAlreadyInIsland(true);
                    final RigidBody body1 = (RigidBody) contactManifold.getFirstBody();
                    final RigidBody body2 = (RigidBody) contactManifold.getSecondBody();
                    final RigidBody otherBody = (body1.getID() == bodyToVisit.getID()) ? body2 : body1;
                    if (otherBody.isAlreadyInIsland()) {
                        continue;
                    }
                    stackBodiesToVisit[stackIndex] = otherBody;
                    stackIndex++;
                    otherBody.setIsAlreadyInIsland(true);
                }
                JointListElement jointElement;
                for (jointElement = bodyToVisit.getJointsList(); jointElement != null;
                     jointElement = jointElement.getNext()) {
                    final Constraint joint = jointElement.getJoint();
                    if (joint.isAlreadyInIsland()) {
                        continue;
                    }
                    mIslands[mNbIslands].addJoint(joint);
                    joint.setIsAlreadyInIsland(true);
                    final RigidBody body1 = joint.getFirstBody();
                    final RigidBody body2 = joint.getSecondBody();
                    final RigidBody otherBody = (body1.getID() == bodyToVisit.getID()) ? body2 : body1;
                    if (otherBody.isAlreadyInIsland()) {
                        continue;
                    }
                    stackBodiesToVisit[stackIndex] = otherBody;
                    stackIndex++;
                    otherBody.setIsAlreadyInIsland(true);
                }
            }
            for (int i = 0; i < mIslands[mNbIslands].getNbBodies(); i++) {
                if (!mIslands[mNbIslands].getBodies()[i].getIsMotionEnabled()) {
                    mIslands[mNbIslands].getBodies()[i].setIsAlreadyInIsland(false);
                }
            }
            mNbIslands++;
        }
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
        addContactManifoldToBody(overlappingPair.getContactManifold(), overlappingPair.getFirstBody(), overlappingPair.getSecondBody());
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
