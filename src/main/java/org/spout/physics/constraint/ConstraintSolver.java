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
package org.spout.physics.constraint;

import java.util.List;
import java.util.Set;

import gnu.trove.map.TObjectIntMap;

import org.spout.physics.body.RigidBody;
import org.spout.physics.math.Vector3;

/**
 * This class represents the constraint solver that is used to solve constraints between the rigid bodies. The constraint solver is based on the "Sequential Impulse" technique described by Erin Catto
 * in his GDC slides (http://code.google.com/p/box2d/downloads/list).
 * <p/>
 * A constraint between two bodies is represented by a function C(x) which is equal to zero when the constraint is satisfied. The condition C(x)=0 describes a valid position and the condition
 * dC(x)/dt=0 describes a valid velocity. We have dC(x)/dt = Jv + b = 0 where J is the Jacobian matrix of the constraint, v is a vector that contains the velocity of both bodies and b is the
 * constraint bias. We are looking for a force F_c that will act on the bodies to keep the constraint satisfied. Note that from the virtual work principle, we have F_c = J^t * lambda where J^t is the
 * transpose of the Jacobian matrix and lambda is a Lagrange multiplier. Therefore, finding the force F_c is equivalent to finding the Lagrange multiplier lambda.
 * <p/>
 * An impulse P = F * dt where F is a force and dt is the timestep. We can apply impulses a body to change its velocity. The idea of the Sequential Impulse technique is to apply impulses to bodies of
 * each constraints in order to keep the constraint satisfied.
 * <p/>
 * --- Step 1 ---
 * <p/>
 * First, we integrate the applied force F_a acting of each rigid body (like gravity, ...) and we obtain some new velocities v2' that tends to violate the constraints.
 * <p/>
 * v2' = v1 + dt * M^-1 * F_a
 * <p/>
 * where M is a matrix that contains mass and inertia tensor information.
 * <p/>
 * --- Step 2 ---
 * <p/>
 * During the second step, we iterate over all the constraints for a certain number of iterations and for each constraint we compute the impulse to apply to the bodies needed so that the new velocity
 * of the bodies satisfies Jv + b = 0. From the Newton law, we know that M * deltaV = P_c where M is the mass of the body, deltaV is the difference of velocity and P_c is the constraint impulse to
 * apply to the body. Therefore, we have v2 = v2' + M^-1 * P_c. For each constraint, we can compute the Lagrange multiplier lambda using : lambda = -m_c (Jv2' + b) where m_c = 1 / (J * M^-1 * J^t).
 * Now that we have the Lagrange multiplier lambda, we can compute the impulse P_c = J^t * lambda * dt to apply to the bodies to satisfy the constraint.
 * <p/>
 * --- Step 3 ---
 * <p/>
 * In the third step, we integrate the new position x2 of the bodies using the new velocities v2 computed in the second step with : x2 = x1 + dt * v2.
 * <p/>
 * Note that in the following code (as it is also explained in the slides from Erin Catto), the value lambda is not only the lagrange multiplier but is the multiplication of the Lagrange multiplier
 * with the timestep dt. Therefore, in the following code, when we use lambda, we mean (lambda * dt).
 * <p/>
 * We are using the accumulated impulse technique that is also described in the slides from Erin Catto.
 * <p/>
 * We are also using warm starting. The idea is to warm start the solver at the beginning of each step by applying the last impulses for the constraints that we already existing at the previous step.
 * This allows the iterative solver to converge faster towards the solution.
 * <p/>
 * For contact constraints, we are also using split impulses so that the position correction that uses Baumgarte stabilization does not change the momentum of the bodies.
 * <p/>
 * There are two ways to apply the friction constraints. Either the friction constraints are applied at each contact point or they are applied only at the center of the contact manifold between two
 * bodies. If we solve the friction constraints at each contact point, we need two constraints (two tangential friction directions) and if we solve the friction constraints at the center of the
 * contact manifold, we need two constraints for tangential friction but also another twist friction constraint to prevent spin of the body around the contact manifold center.
 */
public class ConstraintSolver {
    private Set<Constraint> mJoints;
    private List<Vector3> mConstrainedLinearVelocities;
    private List<Vector3> mConstrainedAngularVelocities;
    private TObjectIntMap<RigidBody> mMapBodyToConstrainedVelocityIndex;
    private int mNbIterations;
    private float mTimeStep;

    public ConstraintSolver(Set<Constraint> joints, List<Vector3> constrainedLinearVelocities, List<Vector3> constrainedAngularVelocities, TObjectIntMap<RigidBody> mapBodyToConstrainedVelocityIndex) {
        mJoints = joints;
        mConstrainedLinearVelocities = constrainedLinearVelocities;
        mConstrainedAngularVelocities = constrainedAngularVelocities;
        mMapBodyToConstrainedVelocityIndex = mapBodyToConstrainedVelocityIndex;
    }
}
