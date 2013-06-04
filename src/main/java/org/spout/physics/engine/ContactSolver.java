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
import java.util.Map;
import java.util.Set;
import java.util.Vector;

import org.spout.physics.Configuration;
import org.spout.physics.body.RigidBody;
import org.spout.physics.constraint.ContactPoint;
import org.spout.physics.math.Matrix3x3;
import org.spout.physics.math.Vector3;

/**
 * Represents the contact solver that is used to solve rigid bodies contacts. The constraint solver
 * is based on the "Sequential Impulse" technique described by Erin Catto in his GDC slides
 * (http://code.google.com/p/box2d/downloads/list).
 * <p/>
 * A constraint between two bodies is represented by a function C(x) which is equal to zero when the
 * constraint is satisfied. The condition C(x)=0 describes a valid position and the condition
 * dC(x)/dt=0 describes a valid velocity. We have dC(x)/dt = Jv + b = 0 where J is the Jacobian
 * matrix of the constraint, v is a vector that contains the velocity of both bodies and b is the
 * constraint bias. We are looking for a force F_c that will act on the bodies to keep the
 * constraint satisfied. Note that from the virtual work principle, we have F_c = J^t * lambda where
 * J^t is the transpose of the Jacobian matrix and lambda is a Lagrange multiplier. Therefore,
 * finding the force F_c is equivalent to finding the Lagrange multiplier lambda.
 * <p/>
 * An impulse P = F * dt where F is a force and dt is the timestep. We can apply impulses to a body
 * to change its velocity. The idea of the Sequential Impulse technique is to apply impulses to the
 * bodies of each constraints in order to keep the constraint satisfied.
 * <p/>
 * --- Step 1 ---
 * <p/>
 * First, we integrate the applied force F_a acting on each rigid body (like gravity, ...) and we
 * obtain some new velocities v2' that tends to violate the constraints.
 * <p/>
 * v2' = v1 + dt * M^-1 * F_a
 * <p/>
 * where M is a matrix that contains mass and inertia tensor information.
 * <p/>
 * --- Step 2 ---
 * <p/>
 * During the second step, we iterate over all the constraints for a certain number of iterations
 * and for each constraint we compute the impulse to apply to the bodies needed so that the new
 * velocity of the bodies satisfy Jv + b = 0. From the Newton law, we know that M * deltaV = P_c
 * where M is the mass of the body, deltaV is the difference of velocity and P_c is the constraint
 * impulse to apply to the body. Therefore, we have v2 = v2' + M^-1 * P_c. For each constraint, we
 * can compute the Lagrange multiplier lambda using: lambda = -m_c (Jv2' + b) where m_c = 1 / (J *
 * M^-1 * J^t). Now that we have the Lagrange multiplier lambda, we can compute the impulse P_c =
 * J^t * lambda * dt to apply to the bodies to satisfy the constraint.
 * <p/>
 * --- Step 3 ---
 * <p/>
 * In the third step, we integrate the new position x2 of the bodies using the new velocities v2
 * computed in the second step with: x2 = x1 + dt * v2.
 * <p/>
 * Note that in the following code (as it is also explained in the slides from Erin Catto), the
 * value lambda is not only the lagrange multiplier but is the multiplication of the Lagrange
 * multiplier with the timestep dt. Therefore, in the following code, when we use lambda, we mean
 * (lambda * dt).
 * <p/>
 * This implementation uses the accumulated impulse technique that is also described in the slides
 * from Erin Catto.
 * <p/>
 * This implementation also uses warm starting. The idea is to warm start the solver at the
 * beginning of each step by applying the last impulses for the constraints from the previous step.
 * This allows the iterative solver to converge faster towards the solution.
 * <p/>
 * For contact constraints, this implementation also uses split impulses so that the position
 * correction, which uses Baumgarte stabilization, does not change the momentum of the bodies.
 * <p/>
 * There are two ways to apply the friction constraints. Either the friction constraints are applied
 * at each contact point, or they are applied only at the center of the contact manifold between two
 * bodies. If we solve the friction constraints at each contact point, we need two constraints (two
 * tangential friction directions), but if we solve the friction constraints at the center of the
 * contact manifold, we need two constraints for tangential friction and also another twist friction
 * constraint to prevent the body from spinning around the contact manifold center.
 */
public class ContactSolver {
	private static final float BETA = 0.2f;
	private static final float BETA_SPLIT_IMPULSE = 0.2f;
	private static final float SLOP = 0.01f;
	private final DynamicsWorld mWorld;
	private int mNbIterations;
	private Vector3[] mSplitLinearVelocities;
	private Vector3[] mSplitAngularVelocities;
	private float mTimeStep;
	private ContactManifoldSolver[] mContactConstraints;
	private int mNbContactManifolds;
	private final Set<RigidBody> mConstraintBodies = new HashSet<RigidBody>();
	private final Vector<Vector3> mConstrainedLinearVelocities;
	private final Vector<Vector3> mConstrainedAngularVelocities;
	private final Map<RigidBody, Integer> mMapBodyToConstrainedVelocityIndex;
	private final boolean mIsWarmStartingActive;
	private boolean mIsSplitImpulseActive;
	private boolean mIsSolveFrictionAtContactManifoldCenterActive;

	/**
	 * Constructs a new contact solver from the dynamics world, the constrained linear and angular
	 * velocities, and the body to velocity index map.
	 *
	 * @param world The dynamics world
	 * @param constrainedLinearVelocities The constrained linear velocities
	 * @param constrainedAngularVelocities The constrained angular velocities
	 * @param mapBodyToVelocityIndex The body to velocity index map
	 */
	public ContactSolver(DynamicsWorld world, Vector<Vector3> constrainedLinearVelocities,
						 Vector<Vector3> constrainedAngularVelocities, Map<RigidBody, Integer> mapBodyToVelocityIndex) {
		mWorld = world;
		mNbIterations = Configuration.DEFAULT_CONSTRAINTS_SOLVER_NB_ITERATIONS;
		mSplitLinearVelocities = null;
		mSplitAngularVelocities = null;
		mContactConstraints = null;
		mConstrainedLinearVelocities = constrainedLinearVelocities;
		mConstrainedAngularVelocities = constrainedAngularVelocities;
		mMapBodyToConstrainedVelocityIndex = mapBodyToVelocityIndex;
		mIsWarmStartingActive = true;
		mIsSplitImpulseActive = true;
		mIsSolveFrictionAtContactManifoldCenterActive = true;
	}

	/**
	 * Returns true if the body is in at least one constraint.
	 *
	 * @param body The body to check
	 * @return Whether or not the body is in at least one constraint
	 */
	public boolean isConstrainedBody(RigidBody body) {
		return mConstraintBodies.contains(body);
	}

	/**
	 * Gets the split linear velocity for the body.
	 *
	 * @param body The to get the split linear velocity for
	 * @return The split linear velocity
	 */
	public Vector3 getSplitLinearVelocityOfBody(RigidBody body) {
		if (!isConstrainedBody(body)) {
			throw new IllegalArgumentException("body must be a constrained body");
		}
		final int indexBody = mMapBodyToConstrainedVelocityIndex.get(body);
		return mSplitLinearVelocities[indexBody];
	}

	/**
	 * Gets the split angular velocity.
	 *
	 * @param body The to get the split angular velocity for
	 * @return The split angular velocity
	 */
	public Vector3 getSplitAngularVelocityOfBody(RigidBody body) {
		if (!isConstrainedBody(body)) {
			throw new IllegalArgumentException("body must be a constrained body");
		}
		final int indexBody = mMapBodyToConstrainedVelocityIndex.get(body);
		return mSplitAngularVelocities[indexBody];
	}

	/**
	 * Sets the number of iterations for the constraint solver.
	 *
	 * @param nbIterations The number of iterations to do
	 */
	public void setNbIterationsSolver(int nbIterations) {
		mNbIterations = nbIterations;
	}

	/**
	 * Activates or deactivates the split impulses for contacts.
	 *
	 * @param isActive True if the split impulses are active, false if not
	 */
	public void setIsSplitImpulseActive(boolean isActive) {
		mIsSplitImpulseActive = isActive;
	}

	/**
	 * Activates or deactivates the solving of friction constraints at the center of the contact
	 * manifold instead of solving them at each contact point.
	 *
	 * @param isActive Whether or not to solve the friction constraint at the center of the manifold
	 */
	public void setIsSolveFrictionAtContactManifoldCenterActive(boolean isActive) {
		mIsSolveFrictionAtContactManifoldCenterActive = isActive;
	}

	// Computes the collision restitution factor from the restitution factor of each body.
	private float computeMixedRestitutionFactor(RigidBody body1, RigidBody body2) {
		final float restitution1 = body1.getRestitution();
		final float restitution2 = body2.getRestitution();
		return (restitution1 > restitution2) ? restitution1 : restitution2;
	}

	// Computes the mixed friction coefficient from the friction coefficient of each body.
	private float computeMixedFrictionCoefficient(RigidBody body1, RigidBody body2) {
		return (float) Math.sqrt(body1.getFrictionCoefficient() * body2.getFrictionCoefficient());
	}

	// Computes a penetration constraint impulse.
	private Impulse computePenetrationImpulse(float deltaLambda, ContactPointSolver contactPoint) {
		return new Impulse(
				Vector3.multiply(Vector3.negate(contactPoint.normal), deltaLambda),
				Vector3.multiply(Vector3.negate(contactPoint.r1CrossN), deltaLambda),
				Vector3.multiply(contactPoint.normal, deltaLambda),
				Vector3.multiply(contactPoint.r2CrossN, deltaLambda));
	}

	// Computes the first friction constraint impulse.
	private Impulse computeFriction1Impulse(float deltaLambda, ContactPointSolver contactPoint) {
		return new Impulse(
				Vector3.multiply(Vector3.negate(contactPoint.frictionVector1), deltaLambda),
				Vector3.multiply(Vector3.negate(contactPoint.r1CrossT1), deltaLambda),
				Vector3.multiply(contactPoint.frictionVector1, deltaLambda),
				Vector3.multiply(contactPoint.r2CrossT1, deltaLambda));
	}

	// Computes the second friction constraint impulse.
	private Impulse computeFriction2Impulse(float deltaLambda, ContactPointSolver contactPoint) {
		return new Impulse(
				Vector3.multiply(Vector3.negate(contactPoint.frictionVector2), deltaLambda),
				Vector3.multiply(Vector3.negate(contactPoint.r1CrossT2), deltaLambda),
				Vector3.multiply(contactPoint.frictionVector2, deltaLambda),
				Vector3.multiply(contactPoint.r2CrossT2, deltaLambda));
	}

	// Initializes the constraint solver.
	private void initialize() {
		mContactConstraints = new ContactManifoldSolver[mWorld.getNbContactManifolds()];
		mNbContactManifolds = 0;
		ContactManifold externalManifold;
		for (Iterator<ContactManifold> iterator = mWorld.getContactManifoldsBeginIterator(); iterator.hasNext(); ) {
			externalManifold = iterator.next();
			if (mContactConstraints[mNbContactManifolds] == null) {
				mContactConstraints[mNbContactManifolds] = new ContactManifoldSolver();
			}
			final ContactManifoldSolver internalManifold = mContactConstraints[mNbContactManifolds];
			if (externalManifold.getNbContactPoints() <= 0) {
				throw new IllegalStateException("external manifold must have at least one contact point");
			}
			final RigidBody body1 = externalManifold.getContactPoint(0).getBody1();
			final RigidBody body2 = externalManifold.getContactPoint(0).getBody2();
			mConstraintBodies.add(body1);
			mConstraintBodies.add(body2);
			final Vector3 x1 = body1.getTransform().getPosition();
			final Vector3 x2 = body2.getTransform().getPosition();
			internalManifold.indexBody1 = mMapBodyToConstrainedVelocityIndex.get(body1);
			internalManifold.indexBody2 = mMapBodyToConstrainedVelocityIndex.get(body2);
			internalManifold.inverseInertiaTensorBody1.set(body1.getInertiaTensorInverseWorld());
			internalManifold.inverseInertiaTensorBody2.set(body2.getInertiaTensorInverseWorld());
			internalManifold.isBody1Moving = body1.getIsMotionEnabled();
			internalManifold.isBody2Moving = body2.getIsMotionEnabled();
			internalManifold.massInverseBody1 = body1.getMassInverse();
			internalManifold.massInverseBody2 = body2.getMassInverse();
			internalManifold.nbContacts = externalManifold.getNbContactPoints();
			internalManifold.restitutionFactor = computeMixedRestitutionFactor(body1, body2);
			internalManifold.frictionCoefficient = computeMixedFrictionCoefficient(body1, body2);
			internalManifold.externalContactManifold = externalManifold;
			if (mIsSolveFrictionAtContactManifoldCenterActive) {
				internalManifold.frictionPointBody1.set(new Vector3(0, 0, 0));
				internalManifold.frictionPointBody2.set(new Vector3(0, 0, 0));
			}
			for (int c = 0; c < externalManifold.getNbContactPoints(); c++) {
				if (internalManifold.contacts[c] == null) {
					internalManifold.contacts[c] = new ContactPointSolver();
				}
				final ContactPointSolver contactPoint = internalManifold.contacts[c];
				final ContactPoint externalContact = externalManifold.getContactPoint(c);
				final Vector3 p1 = externalContact.getWorldPointOnBody1();
				final Vector3 p2 = externalContact.getWorldPointOnBody2();
				contactPoint.externalContact = externalContact;
				contactPoint.normal.set(externalContact.getNormal());
				contactPoint.r1.set(Vector3.subtract(p1, x1));
				contactPoint.r2.set(Vector3.subtract(p2, x2));
				contactPoint.penetrationDepth = externalContact.getPenetrationDepth();
				contactPoint.isRestingContact = externalContact.getIsRestingContact();
				externalContact.setIsRestingContact(true);
				contactPoint.oldFrictionVector1.set(externalContact.getFrictionVector1());
				contactPoint.oldFrictionVector2.set(externalContact.getFrictionVector2());
				contactPoint.penetrationImpulse = 0;
				contactPoint.friction1Impulse = 0;
				contactPoint.friction2Impulse = 0;
				if (mIsSolveFrictionAtContactManifoldCenterActive) {
					internalManifold.frictionPointBody1.add(p1);
					internalManifold.frictionPointBody2.add(p2);
				}
			}
			if (mIsSolveFrictionAtContactManifoldCenterActive) {
				internalManifold.frictionPointBody1.divide(internalManifold.nbContacts);
				internalManifold.frictionPointBody2.divide(internalManifold.nbContacts);
				internalManifold.r1Friction.set(Vector3.subtract(internalManifold.frictionPointBody1, x1));
				internalManifold.r2Friction.set(Vector3.subtract(internalManifold.frictionPointBody2, x2));
				internalManifold.oldFrictionVector1.set(externalManifold.getFrictionVector1());
				internalManifold.oldFrictionVector2.set(externalManifold.getFrictionVector2());
				if (mIsWarmStartingActive) {
					internalManifold.friction1Impulse = externalManifold.getFrictionImpulse1();
					internalManifold.friction2Impulse = externalManifold.getFrictionImpulse2();
					internalManifold.frictionTwistImpulse = externalManifold.getFrictionTwistImpulse();
				} else {
					internalManifold.friction1Impulse = 0;
					internalManifold.friction2Impulse = 0;
					internalManifold.frictionTwistImpulse = 0;
				}
			}
			mNbContactManifolds++;
		}
		mSplitLinearVelocities = new Vector3[mWorld.getNbRigidBodies()];
		mSplitAngularVelocities = new Vector3[mWorld.getNbRigidBodies()];
		if (mConstraintBodies.size() <= 0) {
			throw new IllegalStateException("the number of constraint bodies must be greater than zero");
		}
		if (mMapBodyToConstrainedVelocityIndex.size() < mConstraintBodies.size()) {
			throw new IllegalStateException("the number of constrained velocities must be greater"
					+ " or equal to the number of constraint bodies");
		}
		if (mConstrainedLinearVelocities.size() < mConstraintBodies.size()) {
			throw new IllegalStateException("the number of constrained linear velocities must be greater"
					+ " or equal to the number of constraint bodies");
		}
		if (mConstrainedAngularVelocities.size() < mConstraintBodies.size()) {
			throw new IllegalStateException("the number of constrained angular velocities must be greater"
					+ " or equal to the number of constraint bodies");
		}
		initializeSplitImpulseVelocities();
	}

	// Initializes the split impulse velocities.
	private void initializeSplitImpulseVelocities() {
		for (RigidBody rigidBody : mConstraintBodies) {
			if (rigidBody == null) {
				throw new IllegalStateException("the rigid body cannot be null");
			}
			final int bodyNumber = mMapBodyToConstrainedVelocityIndex.get(rigidBody);
			mSplitLinearVelocities[bodyNumber] = new Vector3(0, 0, 0);
			mSplitAngularVelocities[bodyNumber] = new Vector3(0, 0, 0);
		}
	}

	// Initializes the contact constraints before solving the system.
	private void initializeContactConstraints() {
		for (int c = 0; c < mNbContactManifolds; c++) {
			final ContactManifoldSolver manifold = mContactConstraints[c];
			final Matrix3x3 I1 = manifold.inverseInertiaTensorBody1;
			final Matrix3x3 I2 = manifold.inverseInertiaTensorBody2;
			if (mIsSolveFrictionAtContactManifoldCenterActive) {
				manifold.normal.set(new Vector3(0, 0, 0));
			}
			final Vector3 v1 = mConstrainedLinearVelocities.get(manifold.indexBody1);
			final Vector3 w1 = mConstrainedAngularVelocities.get(manifold.indexBody1);
			final Vector3 v2 = mConstrainedLinearVelocities.get(manifold.indexBody2);
			final Vector3 w2 = mConstrainedAngularVelocities.get(manifold.indexBody2);
			for (int i = 0; i < manifold.nbContacts; i++) {
				final ContactPointSolver contactPoint = manifold.contacts[i];
				final ContactPoint externalContact = contactPoint.externalContact;
				final Vector3 deltaV = Vector3.subtract(
						Vector3.subtract(Vector3.add(v2, w2.cross(contactPoint.r2)), v1),
						w1.cross(contactPoint.r1));
				contactPoint.r1CrossN.set(contactPoint.r1.cross(contactPoint.normal));
				contactPoint.r2CrossN.set(contactPoint.r2.cross(contactPoint.normal));
				float massPenetration = 0;
				if (manifold.isBody1Moving) {
					massPenetration += manifold.massInverseBody1
							+ Matrix3x3.multiply(I1, contactPoint.r1CrossN).cross(contactPoint.r1).dot(contactPoint.normal);
				}
				if (manifold.isBody2Moving) {
					massPenetration += manifold.massInverseBody2
							+ Matrix3x3.multiply(I2, contactPoint.r2CrossN).cross(contactPoint.r2).dot(contactPoint.normal);
				}
				contactPoint.inversePenetrationMass = massPenetration > 0 ? 1 / massPenetration : 0;
				if (!mIsSolveFrictionAtContactManifoldCenterActive) {
					computeFrictionVectors(deltaV, contactPoint);
					contactPoint.r1CrossT1.set(contactPoint.r1.cross(contactPoint.frictionVector1));
					contactPoint.r1CrossT2.set(contactPoint.r1.cross(contactPoint.frictionVector2));
					contactPoint.r2CrossT1.set(contactPoint.r2.cross(contactPoint.frictionVector1));
					contactPoint.r2CrossT2.set(contactPoint.r2.cross(contactPoint.frictionVector2));
					float friction1Mass = 0;
					float friction2Mass = 0;
					if (manifold.isBody1Moving) {
						friction1Mass += manifold.massInverseBody1
								+ Matrix3x3.multiply(I1, contactPoint.r1CrossT1).cross(contactPoint.r1).dot(contactPoint.frictionVector1);
						friction2Mass += manifold.massInverseBody1
								+ Matrix3x3.multiply(I1, contactPoint.r1CrossT2).cross(contactPoint.r1).dot(contactPoint.frictionVector2);
					}
					if (manifold.isBody2Moving) {
						friction1Mass += manifold.massInverseBody2
								+ Matrix3x3.multiply(I2, contactPoint.r2CrossT1).cross(contactPoint.r2).dot(contactPoint.frictionVector1);
						friction2Mass += manifold.massInverseBody2
								+ Matrix3x3.multiply(I2, contactPoint.r2CrossT2).cross(contactPoint.r2).dot(contactPoint.frictionVector2);
					}
					contactPoint.inverseFriction1Mass = friction1Mass > 0 ? 1 / friction1Mass : 0;
					contactPoint.inverseFriction2Mass = friction2Mass > 0 ? 1 / friction2Mass : 0;
				}
				contactPoint.restitutionBias = 0;
				final float deltaVDotN = deltaV.dot(contactPoint.normal);
				if (deltaVDotN < Configuration.RESTITUTION_VELOCITY_THRESHOLD) {
					contactPoint.restitutionBias = manifold.restitutionFactor * deltaVDotN;
				}
				if (mIsWarmStartingActive) {
					contactPoint.penetrationImpulse = externalContact.getCachedLambda(0);
					contactPoint.friction1Impulse = externalContact.getCachedLambda(1);
					contactPoint.friction2Impulse = externalContact.getCachedLambda(2);
				}
				contactPoint.penetrationSplitImpulse = 0;
				if (mIsSolveFrictionAtContactManifoldCenterActive) {
					manifold.normal.add(contactPoint.normal);
				}
			}
			if (mIsSolveFrictionAtContactManifoldCenterActive) {
				manifold.normal.normalize();
				final Vector3 deltaVFrictionPoint = Vector3.subtract(
						Vector3.subtract(Vector3.add(v2, w2.cross(manifold.r2Friction)), v1),
						w1.cross(manifold.r1Friction));
				computeFrictionVectors(deltaVFrictionPoint, manifold);
				manifold.r1CrossT1.set(manifold.r1Friction.cross(manifold.frictionVector1));
				manifold.r1CrossT2.set(manifold.r1Friction.cross(manifold.frictionVector2));
				manifold.r2CrossT1.set(manifold.r2Friction.cross(manifold.frictionVector1));
				manifold.r2CrossT2.set(manifold.r2Friction.cross(manifold.frictionVector2));
				float friction1Mass = 0;
				float friction2Mass = 0;
				if (manifold.isBody1Moving) {
					friction1Mass += manifold.massInverseBody1
							+ Matrix3x3.multiply(I1, manifold.r1CrossT1).cross(manifold.r1Friction).dot(manifold.frictionVector1);
					friction2Mass += manifold.massInverseBody1
							+ Matrix3x3.multiply(I1, manifold.r1CrossT2).cross(manifold.r1Friction).dot(manifold.frictionVector2);
				}
				if (manifold.isBody2Moving) {
					friction1Mass += manifold.massInverseBody2
							+ Matrix3x3.multiply(I2, manifold.r2CrossT1).cross(manifold.r2Friction).dot(manifold.frictionVector1);
					friction2Mass += manifold.massInverseBody2
							+ Matrix3x3.multiply(I2, manifold.r2CrossT2).cross(manifold.r2Friction).dot(manifold.frictionVector2);
				}
				final float frictionTwistMass =
						manifold.normal.dot(Matrix3x3.multiply(manifold.inverseInertiaTensorBody1, manifold.normal))
								+ manifold.normal.dot(Matrix3x3.multiply(manifold.inverseInertiaTensorBody2, manifold.normal));
				manifold.inverseFriction1Mass = friction1Mass > 0 ? 1 / friction1Mass : 0;
				manifold.inverseFriction2Mass = friction2Mass > 0 ? 1 / friction2Mass : 0;
				manifold.inverseTwistFrictionMass = frictionTwistMass > 0 ? 1 / frictionTwistMass : 0;
			}
		}
	}

	// Warm start the solver.
	// For each constraint, we apply the previous impulse (from the previous step) at the beginning.
	// With this technique, we will converge faster towards the solution for the linear system.
	private void warmStart() {
		for (int c = 0; c < mNbContactManifolds; c++) {
			final ContactManifoldSolver contactManifold = mContactConstraints[c];
			boolean atLeastOneRestingContactPoint = false;
			for (int i = 0; i < contactManifold.nbContacts; i++) {
				final ContactPointSolver contactPoint = contactManifold.contacts[i];
				if (contactPoint.isRestingContact) {
					atLeastOneRestingContactPoint = true;
					// --------- Penetration --------- //
					final Impulse impulsePenetration = computePenetrationImpulse(contactPoint.penetrationImpulse, contactPoint);
					applyImpulse(impulsePenetration, contactManifold);
					if (!mIsSolveFrictionAtContactManifoldCenterActive) {
						final Vector3 oldFrictionImpulse = Vector3.add(
								Vector3.multiply(contactPoint.friction1Impulse, contactPoint.oldFrictionVector1),
								Vector3.multiply(contactPoint.friction2Impulse, contactPoint.oldFrictionVector2));
						contactPoint.friction1Impulse = oldFrictionImpulse.dot(contactPoint.frictionVector1);
						contactPoint.friction2Impulse = oldFrictionImpulse.dot(contactPoint.frictionVector2);
						// --------- Friction 1 --------- //
						final Impulse impulseFriction1 = computeFriction1Impulse(contactPoint.friction1Impulse, contactPoint);
						applyImpulse(impulseFriction1, contactManifold);
						// --------- Friction 2 --------- //
						final Impulse impulseFriction2 = computeFriction2Impulse(contactPoint.friction2Impulse, contactPoint);
						applyImpulse(impulseFriction2, contactManifold);
					}
				} else {
					contactPoint.penetrationImpulse = 0;
					contactPoint.friction1Impulse = 0;
					contactPoint.friction2Impulse = 0;
				}
			}
			if (mIsSolveFrictionAtContactManifoldCenterActive && atLeastOneRestingContactPoint) {
				final Vector3 oldFrictionImpulse = Vector3.add(
						Vector3.multiply(contactManifold.friction1Impulse, contactManifold.oldFrictionVector1),
						Vector3.multiply(contactManifold.friction2Impulse, contactManifold.oldFrictionVector2));
				contactManifold.friction1Impulse = oldFrictionImpulse.dot(contactManifold.frictionVector1);
				contactManifold.friction2Impulse = oldFrictionImpulse.dot(contactManifold.frictionVector2);
				// ------ First friction constraint at the center of the contact manifold ------ //
				Vector3 linearImpulseBody1 = Vector3.multiply(Vector3.negate(contactManifold.frictionVector1), contactManifold.friction1Impulse);
				Vector3 angularImpulseBody1 = Vector3.multiply(Vector3.negate(contactManifold.r1CrossT1), contactManifold.friction1Impulse);
				Vector3 linearImpulseBody2 = Vector3.multiply(contactManifold.frictionVector1, contactManifold.friction1Impulse);
				Vector3 angularImpulseBody2 = Vector3.multiply(contactManifold.r2CrossT1, contactManifold.friction1Impulse);
				final Impulse impulseFriction1 = new Impulse(
						linearImpulseBody1, angularImpulseBody1,
						linearImpulseBody2, angularImpulseBody2);
				applyImpulse(impulseFriction1, contactManifold);
				// ------ Second friction constraint at the center of the contact manifold ----- //
				linearImpulseBody1 = Vector3.multiply(Vector3.negate(contactManifold.frictionVector2), contactManifold.friction2Impulse);
				angularImpulseBody1 = Vector3.multiply(Vector3.negate(contactManifold.r1CrossT2), contactManifold.friction2Impulse);
				linearImpulseBody2 = Vector3.multiply(contactManifold.frictionVector2, contactManifold.friction2Impulse);
				angularImpulseBody2 = Vector3.multiply(contactManifold.r2CrossT2, contactManifold.friction2Impulse);
				final Impulse impulseFriction2 = new Impulse(
						linearImpulseBody1, angularImpulseBody1,
						linearImpulseBody2, angularImpulseBody2);
				applyImpulse(impulseFriction2, contactManifold);
				// ------ Twist friction constraint at the center of the contact manifold ------ //
				linearImpulseBody1 = new Vector3(0, 0, 0);
				angularImpulseBody1 = Vector3.multiply(Vector3.negate(contactManifold.normal), contactManifold.frictionTwistImpulse);
				linearImpulseBody2 = new Vector3(0, 0, 0);
				angularImpulseBody2 = Vector3.multiply(contactManifold.normal, contactManifold.frictionTwistImpulse);
				final Impulse impulseTwistFriction = new Impulse(
						linearImpulseBody1, angularImpulseBody1,
						linearImpulseBody2, angularImpulseBody2);
				applyImpulse(impulseTwistFriction, contactManifold);
			} else {
				contactManifold.friction1Impulse = 0;
				contactManifold.friction2Impulse = 0;
				contactManifold.frictionTwistImpulse = 0;
			}
		}
	}

	// Solves the contact constraints by applying sequential impulses.
	private void solveContactConstraints() {
		float deltaLambda;
		float lambdaTemp;
		for (int iter = 0; iter < mNbIterations; iter++) {
			for (int c = 0; c < mNbContactManifolds; c++) {
				ContactManifoldSolver contactManifold = mContactConstraints[c];
				float sumPenetrationImpulse = 0;
				final Vector3 v1 = mConstrainedLinearVelocities.get(contactManifold.indexBody1);
				final Vector3 w1 = mConstrainedAngularVelocities.get(contactManifold.indexBody1);
				final Vector3 v2 = mConstrainedLinearVelocities.get(contactManifold.indexBody2);
				final Vector3 w2 = mConstrainedAngularVelocities.get(contactManifold.indexBody2);
				for (int i = 0; i < contactManifold.nbContacts; i++) {
					final ContactPointSolver contactPoint = contactManifold.contacts[i];
					// --------- Penetration --------- //
					Vector3 deltaV = Vector3.subtract(
							Vector3.subtract(Vector3.add(v2, w2.cross(contactPoint.r2)), v1),
							w1.cross(contactPoint.r1));
					final float deltaVDotN = deltaV.dot(contactPoint.normal);
					float Jv = deltaVDotN;
					final float beta = mIsSplitImpulseActive ? BETA_SPLIT_IMPULSE : BETA;
					float biasPenetrationDepth = 0;
					if (contactPoint.penetrationDepth > SLOP) {
						biasPenetrationDepth = -(beta / mTimeStep) * Math.max(0, contactPoint.penetrationDepth - SLOP);
					}
					final float b = biasPenetrationDepth + contactPoint.restitutionBias;
					if (mIsSplitImpulseActive) {
						deltaLambda = -(Jv + contactPoint.restitutionBias) * contactPoint.inversePenetrationMass;
					} else {
						deltaLambda = -(Jv + b) * contactPoint.inversePenetrationMass;
					}
					lambdaTemp = contactPoint.penetrationImpulse;
					contactPoint.penetrationImpulse = Math.max(contactPoint.penetrationImpulse + deltaLambda, 0);
					deltaLambda = contactPoint.penetrationImpulse - lambdaTemp;
					final Impulse impulsePenetration = computePenetrationImpulse(deltaLambda, contactPoint);
					applyImpulse(impulsePenetration, contactManifold);
					sumPenetrationImpulse += contactPoint.penetrationImpulse;
					if (mIsSplitImpulseActive) {
						final Vector3 v1Split = mSplitLinearVelocities[contactManifold.indexBody1];
						final Vector3 w1Split = mSplitAngularVelocities[contactManifold.indexBody1];
						final Vector3 v2Split = mSplitLinearVelocities[contactManifold.indexBody2];
						final Vector3 w2Split = mSplitAngularVelocities[contactManifold.indexBody2];
						final Vector3 deltaVSplit = Vector3.subtract(
								Vector3.subtract(Vector3.add(v2Split, w2Split.cross(contactPoint.r2)), v1Split),
								w1Split.cross(contactPoint.r1));
						final float JvSplit = deltaVSplit.dot(contactPoint.normal);
						final float deltaLambdaSplit = -(JvSplit + biasPenetrationDepth) * contactPoint.inversePenetrationMass;
						final float lambdaTempSplit = contactPoint.penetrationSplitImpulse;
						contactPoint.penetrationSplitImpulse = Math.max(contactPoint.penetrationSplitImpulse + deltaLambdaSplit, 0);
						deltaLambda = contactPoint.penetrationSplitImpulse - lambdaTempSplit;
						final Impulse splitImpulsePenetration = computePenetrationImpulse(deltaLambdaSplit, contactPoint);
						applySplitImpulse(splitImpulsePenetration, contactManifold);
					}
					if (!mIsSolveFrictionAtContactManifoldCenterActive) {
						// --------- Friction 1 --------- //
						deltaV = Vector3.subtract(
								Vector3.subtract(Vector3.add(v2, w2.cross(contactPoint.r2)), v1),
								w1.cross(contactPoint.r1));
						Jv = deltaV.dot(contactPoint.frictionVector1);
						deltaLambda = -Jv;
						deltaLambda *= contactPoint.inverseFriction1Mass;
						float frictionLimit = contactManifold.frictionCoefficient * contactPoint.penetrationImpulse;
						lambdaTemp = contactPoint.friction1Impulse;
						contactPoint.friction1Impulse = Math.max(-frictionLimit,
								Math.min(contactPoint.friction1Impulse + deltaLambda, frictionLimit));
						deltaLambda = contactPoint.friction1Impulse - lambdaTemp;
						final Impulse impulseFriction1 = computeFriction1Impulse(deltaLambda, contactPoint);
						applyImpulse(impulseFriction1, contactManifold);
						// --------- Friction 2 --------- //
						deltaV = Vector3.subtract(
								Vector3.subtract(Vector3.add(v2, w2.cross(contactPoint.r2)), v1),
								w1.cross(contactPoint.r1));
						Jv = deltaV.dot(contactPoint.frictionVector2);
						deltaLambda = -Jv;
						deltaLambda *= contactPoint.inverseFriction2Mass;
						frictionLimit = contactManifold.frictionCoefficient * contactPoint.penetrationImpulse;
						lambdaTemp = contactPoint.friction2Impulse;
						contactPoint.friction2Impulse = Math.max(-frictionLimit,
								Math.min(contactPoint.friction2Impulse + deltaLambda, frictionLimit));
						deltaLambda = contactPoint.friction2Impulse - lambdaTemp;
						final Impulse impulseFriction2 = computeFriction2Impulse(deltaLambda, contactPoint);
						applyImpulse(impulseFriction2, contactManifold);
					}
				}
				if (mIsSolveFrictionAtContactManifoldCenterActive) {
					// ------ First friction constraint at the center of the contact manifold ------ //
					Vector3 deltaV = Vector3.subtract(
							Vector3.subtract(Vector3.add(v2, w2.cross(contactManifold.r2Friction)), v1),
							w1.cross(contactManifold.r1Friction));
					float Jv = deltaV.dot(contactManifold.frictionVector1);
					deltaLambda = -Jv * contactManifold.inverseFriction1Mass;
					float frictionLimit = contactManifold.frictionCoefficient * sumPenetrationImpulse;
					lambdaTemp = contactManifold.friction1Impulse;
					contactManifold.friction1Impulse = Math.max(-frictionLimit,
							Math.min(contactManifold.friction1Impulse + deltaLambda, frictionLimit));
					deltaLambda = contactManifold.friction1Impulse - lambdaTemp;
					Vector3 linearImpulseBody1 = Vector3.multiply(Vector3.negate(contactManifold.frictionVector1), deltaLambda);
					Vector3 angularImpulseBody1 = Vector3.multiply(Vector3.negate(contactManifold.r1CrossT1), deltaLambda);
					Vector3 linearImpulseBody2 = Vector3.multiply(contactManifold.frictionVector1, deltaLambda);
					Vector3 angularImpulseBody2 = Vector3.multiply(contactManifold.r2CrossT1, deltaLambda);
					final Impulse impulseFriction1 = new Impulse(
							linearImpulseBody1, angularImpulseBody1,
							linearImpulseBody2, angularImpulseBody2);
					applyImpulse(impulseFriction1, contactManifold);
					// ------ Second friction constraint at the center of the contact manifold ----- //
					deltaV = Vector3.subtract(
							Vector3.subtract(Vector3.add(v2, w2.cross(contactManifold.r2Friction)), v1),
							w1.cross(contactManifold.r1Friction));
					Jv = deltaV.dot(contactManifold.frictionVector2);
					deltaLambda = -Jv * contactManifold.inverseFriction2Mass;
					frictionLimit = contactManifold.frictionCoefficient * sumPenetrationImpulse;
					lambdaTemp = contactManifold.friction2Impulse;
					contactManifold.friction2Impulse = Math.max(-frictionLimit,
							Math.min(contactManifold.friction2Impulse + deltaLambda, frictionLimit));
					deltaLambda = contactManifold.friction2Impulse - lambdaTemp;
					linearImpulseBody1 = Vector3.multiply(Vector3.negate(contactManifold.frictionVector2), deltaLambda);
					angularImpulseBody1 = Vector3.multiply(Vector3.negate(contactManifold.r1CrossT2), deltaLambda);
					linearImpulseBody2 = Vector3.multiply(contactManifold.frictionVector2, deltaLambda);
					angularImpulseBody2 = Vector3.multiply(contactManifold.r2CrossT2, deltaLambda);
					final Impulse impulseFriction2 = new Impulse(
							linearImpulseBody1, angularImpulseBody1,
							linearImpulseBody2, angularImpulseBody2);
					applyImpulse(impulseFriction2, contactManifold);
					// ------ Twist friction constraint at the center of the contact manifold ------ //
					deltaV = Vector3.subtract(w2, w1);
					Jv = deltaV.dot(contactManifold.normal);
					deltaLambda = -Jv * (contactManifold.inverseTwistFrictionMass);
					frictionLimit = contactManifold.frictionCoefficient * sumPenetrationImpulse;
					lambdaTemp = contactManifold.frictionTwistImpulse;
					contactManifold.frictionTwistImpulse = Math.max(-frictionLimit,
							Math.min(contactManifold.frictionTwistImpulse + deltaLambda, frictionLimit));
					deltaLambda = contactManifold.frictionTwistImpulse - lambdaTemp;
					linearImpulseBody1 = new Vector3(0, 0, 0);
					angularImpulseBody1 = Vector3.multiply(Vector3.negate(contactManifold.normal), deltaLambda);
					linearImpulseBody2 = new Vector3(0, 0, 0);
					angularImpulseBody2 = Vector3.multiply(contactManifold.normal, deltaLambda);
					final Impulse impulseTwistFriction = new Impulse(
							linearImpulseBody1, angularImpulseBody1,
							linearImpulseBody2, angularImpulseBody2);
					applyImpulse(impulseTwistFriction, contactManifold);
				}
			}
		}
	}

	/**
	 * Solves the constraints
	 *
	 * @param timeStep The time step for the solving
	 */
	public void solve(float timeStep) {
		mTimeStep = timeStep;
		initialize();
		initializeContactConstraints();
		if (mIsWarmStartingActive) {
			warmStart();
		}
		solveContactConstraints();
		storeImpulses();
	}

	// Stores the computed impulses to use them to warm-start the solver for the next iteration.
	private void storeImpulses() {
		for (int c = 0; c < mNbContactManifolds; c++) {
			final ContactManifoldSolver manifold = mContactConstraints[c];
			for (int i = 0; i < manifold.nbContacts; i++) {
				final ContactPointSolver contactPoint = manifold.contacts[i];
				contactPoint.externalContact.setCachedLambda(0, contactPoint.penetrationImpulse);
				contactPoint.externalContact.setCachedLambda(1, contactPoint.friction1Impulse);
				contactPoint.externalContact.setCachedLambda(2, contactPoint.friction2Impulse);
				contactPoint.externalContact.setFrictionVector1(contactPoint.frictionVector1);
				contactPoint.externalContact.setFrictionVector2(contactPoint.frictionVector2);
			}
			manifold.externalContactManifold.setFrictionImpulse1(manifold.friction1Impulse);
			manifold.externalContactManifold.setFrictionImpulse2(manifold.friction2Impulse);
			manifold.externalContactManifold.setFrictionTwistImpulse(manifold.frictionTwistImpulse);
			manifold.externalContactManifold.setFrictionVector1(manifold.frictionVector1);
			manifold.externalContactManifold.setFrictionVector2(manifold.frictionVector2);
		}
	}

	// Applies an impulse to the two bodies of a constraint.
	private void applyImpulse(Impulse impulse, ContactManifoldSolver manifold) {
		if (manifold.isBody1Moving) {
			mConstrainedLinearVelocities.get(manifold.indexBody1).add(Vector3.multiply(manifold.massInverseBody1, impulse.linearImpulseBody1));
			mConstrainedAngularVelocities.get(manifold.indexBody1).add(Matrix3x3.multiply(manifold.inverseInertiaTensorBody1, impulse.angularImpulseBody1));
		}
		if (manifold.isBody2Moving) {
			mConstrainedLinearVelocities.get(manifold.indexBody2).add(Vector3.multiply(manifold.massInverseBody2, impulse.linearImpulseBody2));
			mConstrainedAngularVelocities.get(manifold.indexBody2).add(Matrix3x3.multiply(manifold.inverseInertiaTensorBody2, impulse.angularImpulseBody2));
		}
	}

	// Applies an impulse to the two bodies of a constraint.
	private void applySplitImpulse(Impulse impulse, ContactManifoldSolver manifold) {
		if (manifold.isBody1Moving) {
			mSplitLinearVelocities[manifold.indexBody1].add(Vector3.multiply(manifold.massInverseBody1, impulse.linearImpulseBody1));
			mSplitAngularVelocities[manifold.indexBody1].add(Matrix3x3.multiply(manifold.inverseInertiaTensorBody1, impulse.angularImpulseBody1));
		}
		if (manifold.isBody2Moving) {
			mSplitLinearVelocities[manifold.indexBody2].add(Vector3.multiply(manifold.massInverseBody2, impulse.linearImpulseBody2));
			mSplitAngularVelocities[manifold.indexBody2].add(Matrix3x3.multiply(manifold.inverseInertiaTensorBody2, impulse.angularImpulseBody2));
		}
	}

	// Computes the two unit orthogonal vectors "t1" and "t2" that span the tangential friction plane
	// for a contact point. The two vectors have to be such that : t1 x t2 = contactNormal.
	private void computeFrictionVectors(Vector3 deltaVelocity, ContactPointSolver contactPoint) {
		if (contactPoint.normal.length() <= 0) {
			throw new IllegalArgumentException("the contact point solver normal must be greater than zero");
		}
		final Vector3 normalVelocity = Vector3.multiply(deltaVelocity.dot(contactPoint.normal), contactPoint.normal);
		final Vector3 tangentVelocity = Vector3.subtract(deltaVelocity, normalVelocity);
		final float lengthTangentVelocity = tangentVelocity.length();
		if (lengthTangentVelocity > Configuration.MACHINE_EPSILON) {
			contactPoint.frictionVector1.set(Vector3.divide(tangentVelocity, lengthTangentVelocity));
		} else {
			contactPoint.frictionVector1.set(contactPoint.normal.getOneUnitOrthogonalVector());
		}
		contactPoint.frictionVector2.set(contactPoint.normal.cross(contactPoint.frictionVector1).getUnit());
	}

	// Computes the two unit orthogonal vectors "t1" and "t2" that span the tangential friction plane
	// for a contact manifold. The two vectors have to be such that : t1 x t2 = contactNormal.
	private void computeFrictionVectors(Vector3 deltaVelocity, ContactManifoldSolver contact) {
		if (contact.normal.length() <= 0) {
			throw new IllegalArgumentException("the contact manifold solver normal must be greater than zero");
		}
		final Vector3 normalVelocity = Vector3.multiply(deltaVelocity.dot(contact.normal), contact.normal);
		final Vector3 tangentVelocity = Vector3.subtract(deltaVelocity, normalVelocity);
		final float lengthTangentVelocity = tangentVelocity.length();
		if (lengthTangentVelocity > Configuration.MACHINE_EPSILON) {
			contact.frictionVector1.set(Vector3.divide(tangentVelocity, lengthTangentVelocity));
		} else {
			contact.frictionVector1.set(contact.normal.getOneUnitOrthogonalVector());
		}
		contact.frictionVector2.set(contact.normal.cross(contact.frictionVector1).getUnit());
	}

	/**
	 * Clean up the constraint solver. Clear the last computed data.
	 */
	public void cleanup() {
		mConstraintBodies.clear();
		if (mContactConstraints != null) {
			mContactConstraints = null;
		}
		if (mSplitLinearVelocities != null) {
			mSplitLinearVelocities = null;
		}
		if (mSplitAngularVelocities != null) {
			mSplitAngularVelocities = null;
		}
	}

	// Represents an impulse that we can apply to bodies in the contact or constraint solver.
	private static class Impulse {
		public final Vector3 linearImpulseBody1;
		public final Vector3 linearImpulseBody2;
		public final Vector3 angularImpulseBody1;
		public final Vector3 angularImpulseBody2;

		private Impulse(
				Vector3 linearImpulseBody1, Vector3 angularImpulseBody1,
				Vector3 linearImpulseBody2, Vector3 angularImpulseBody2) {
			this.linearImpulseBody1 = linearImpulseBody1;
			this.angularImpulseBody1 = angularImpulseBody1;
			this.linearImpulseBody2 = linearImpulseBody2;
			this.angularImpulseBody2 = angularImpulseBody2;
		}
	}

	// Contact solver internal data structure that to store all the information relative to a contact point.
	private static class ContactPointSolver {
		private float penetrationImpulse;
		private float friction1Impulse;
		private float friction2Impulse;
		private float penetrationSplitImpulse;
		private final Vector3 normal = new Vector3();
		private final Vector3 frictionVector1 = new Vector3();
		private final Vector3 frictionVector2 = new Vector3();
		private final Vector3 oldFrictionVector1 = new Vector3();
		private final Vector3 oldFrictionVector2 = new Vector3();
		private final Vector3 r1 = new Vector3();
		private final Vector3 r2 = new Vector3();
		private final Vector3 r1CrossT1 = new Vector3();
		private final Vector3 r1CrossT2 = new Vector3();
		private final Vector3 r2CrossT1 = new Vector3();
		private final Vector3 r2CrossT2 = new Vector3();
		private final Vector3 r1CrossN = new Vector3();
		private final Vector3 r2CrossN = new Vector3();
		private float penetrationDepth;
		private float restitutionBias;
		private float inversePenetrationMass;
		private float inverseFriction1Mass;
		private float inverseFriction2Mass;
		private boolean isRestingContact;
		private ContactPoint externalContact;
	}

	// Contact solver internal data structure to store all the information relative to a contact manifold.
	private static class ContactManifoldSolver {
		private int indexBody1;
		private int indexBody2;
		private float massInverseBody1;
		private float massInverseBody2;
		private final Matrix3x3 inverseInertiaTensorBody1 = new Matrix3x3();
		private final Matrix3x3 inverseInertiaTensorBody2 = new Matrix3x3();
		private boolean isBody1Moving;
		private boolean isBody2Moving;
		private final ContactPointSolver[] contacts = new ContactPointSolver[ContactManifold.MAX_CONTACT_POINTS_IN_MANIFOLD];
		private int nbContacts;
		private float restitutionFactor;
		private float frictionCoefficient;
		private ContactManifold externalContactManifold;
		private final Vector3 normal = new Vector3();
		private final Vector3 frictionPointBody1 = new Vector3();
		private final Vector3 frictionPointBody2 = new Vector3();
		private final Vector3 r1Friction = new Vector3();
		private final Vector3 r2Friction = new Vector3();
		private final Vector3 r1CrossT1 = new Vector3();
		private final Vector3 r1CrossT2 = new Vector3();
		private final Vector3 r2CrossT1 = new Vector3();
		private final Vector3 r2CrossT2 = new Vector3();
		private float inverseFriction1Mass;
		private float inverseFriction2Mass;
		private float inverseTwistFrictionMass;
		private final Vector3 frictionVector1 = new Vector3();
		private final Vector3 frictionVector2 = new Vector3();
		private final Vector3 oldFrictionVector1 = new Vector3();
		private final Vector3 oldFrictionVector2 = new Vector3();
		private float friction1Impulse;
		private float friction2Impulse;
		private float frictionTwistImpulse;
	}
}
