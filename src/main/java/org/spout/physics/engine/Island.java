package org.spout.physics.engine;

import org.spout.physics.body.RigidBody;
import org.spout.physics.constraint.Constraint;

/**
 * An island represent an isolated group of awake bodies that are connected with each other by some constraints (contacts or joints).
 */
public class Island {
    private int mID;
    private RigidBody[] mBodies;
    private ContactManifold[] mContactManifolds;
    private Constraint[] mJoints;
    private int mNbBodies;
    private int mNbContactManifolds;
    private int mNbJoints;

    /**
     * Constructs a new island from it's ID, the maximum number of bodies, the maximum number of contact manifolds and the maximum number of joints.
     *
     * @param id The ID
     * @param nbMaxBodies The maximum number of bodies
     * @param nbMaxContactManifolds The maximum number of contact manifolds
     * @param nbMaxJoints The maximum number of joints
     */
    public Island(int id, int nbMaxBodies, int nbMaxContactManifolds, int nbMaxJoints) {
        mID = id;
        mNbBodies = 0;
        mNbContactManifolds = 0;
        mNbJoints = 0;
        mBodies = new RigidBody[nbMaxBodies];
        mContactManifolds = new ContactManifold[nbMaxContactManifolds];
        mJoints = new Constraint[nbMaxJoints];
    }

    /**
     * Adds a body into the island
     *
     * @param body The body
     */
    public void addBody(RigidBody body) {
        assert (!body.isSleeping());
        mBodies[mNbBodies] = body;
        mNbBodies++;
    }

    /**
     * Adds a contact manifold into the island
     *
     * @param contactManifold The contact manifold
     */
    public void addContactManifold(ContactManifold contactManifold) {
        mContactManifolds[mNbContactManifolds] = contactManifold;
        mNbContactManifolds++;
    }

    /**
     * Adds a joint into the island.
     *
     * @param joint The joint
     */
    public void addJoint(Constraint joint) {
        mJoints[mNbJoints] = joint;
        mNbJoints++;
    }

    /**
     * Returns the number of bodies in the island.
     *
     * @return The number of bodies
     */
    public int getNbBodies() {
        return mNbBodies;
    }

    /**
     * Returns the number of contact manifolds in the island.
     *
     * @return The number of contact manifolds
     */
    public int getNbContactManifolds() {
        return mNbContactManifolds;
    }

    /**
     * Returns the number of joints in the island.
     *
     * @return The number of joints
     */
    public int getNbJoints() {
        return mNbJoints;
    }

    /**
     * Returns the array of bodies.
     *
     * @return The array of bodies
     */
    public RigidBody[] getBodies() {
        return mBodies;
    }

    /**
     * Returns the array of contact manifolds.
     *
     * @return The array of contact manifold
     */
    public ContactManifold[] getContactManifold() {
        return mContactManifolds;
    }

    /**
     * Returns the array of joints.
     *
     * @return The array of joints
     */
    public Constraint[] getJoints() {
        return mJoints;
    }
}
