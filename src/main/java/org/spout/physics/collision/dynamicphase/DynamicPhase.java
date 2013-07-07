package org.spout.physics.collision.dynamicphase;

import java.util.HashSet;
import java.util.Set;

import org.spout.physics.ReactDefaults;
import org.spout.physics.body.ImmobileRigidBody;
import org.spout.physics.body.MobileRigidBody;
import org.spout.physics.collision.shape.AABB;
import org.spout.physics.engine.DynamicDynamicsWorld;
import org.spout.physics.math.Vector3;

/**
 * A phase of the physics tick where bodies is dynamically added via the {@link org.spout.physics.engine.DynamicDynamicsWorld}'s {@link org.spout.physics.engine.dynamic.DynamicWorldInfo}.
 */
public class DynamicPhase {
	private final DynamicDynamicsWorld dynamicWorld;

	public DynamicPhase(final DynamicDynamicsWorld dynamicWorld) {
		this.dynamicWorld = dynamicWorld;
	}

	public Set<ImmobileRigidBody> getBodiesInRange(final MobileRigidBody body) {
		final AABB aabb = body.getAABB();
		//Grab object coords
		final Vector3 extents = Vector3.subtract(aabb.getMax(), aabb.getMin()).divide(2);
		final Vector3 max = extents;
		final Vector3 min = Vector3.negate(max);
		//Scale coords
		max.multiply(ReactDefaults.AABB_SCALAR);
		min.multiply(ReactDefaults.AABB_SCALAR);
		//Grab world coords
		max.add(aabb.getCenter());
		min.add(aabb.getCenter());

		final int startx = (int) Math.ceil(min.getX());
		final int starty = (int) Math.ceil(min.getY());
		final int startz = (int) Math.ceil(min.getZ());

		final int endx = (int) Math.floor(max.getX());
		final int endy = (int) Math.floor(max.getY());
		final int endz = (int) Math.floor(max.getZ());

		final Set<ImmobileRigidBody> foundBodies = new HashSet<ImmobileRigidBody>();

		for (int xx = startx; xx <= endx; xx++) {
			for (int yy = starty; yy <= endy; yy++) {
				for (int zz = startz; zz <= endz; zz++) {
					final ImmobileRigidBody immobile = dynamicWorld.getDynamicInfo().getBody(xx, yy, zz);
					if (immobile == null) {
						continue;
					}
					foundBodies.add(immobile);
				}
			}
		}
		return foundBodies;
	}
}
