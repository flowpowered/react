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
package org.spout.physics.collision.shape;

import org.spout.physics.ReactDefaults;
import org.spout.physics.math.Matrix3x3;
import org.spout.physics.math.Transform;
import org.spout.physics.math.Vector3;

/**
 * Represents the collision shape associated with a body that is used during the narrow-phase collision detection.
 */
public abstract class CollisionShape {
    protected final CollisionShapeType mType;
    private int mNbSimilarCreatedShapes;

    /**
     * Constructs a new collision shape from its type.
     *
     * @param type The type of the collision shape
     */
    protected CollisionShape(CollisionShapeType type) {
        mType = type;
        mNbSimilarCreatedShapes = 0;
    }

    protected CollisionShape(CollisionShape shape) {
        mType = shape.getType();
        mNbSimilarCreatedShapes = shape.mNbSimilarCreatedShapes;
    }

    /**
     * Gets the type of collision shape associated to this shape.
     *
     * @return The collision shape type
     */
    public CollisionShapeType getType() {
        return mType;
    }

    /**
     * Gets a local support point in a given direction with the object margin.
     *
     * @param direction The desired direction
     * @return The local support point as a vector3
     */
    public abstract Vector3 getLocalSupportPointWithMargin(Vector3 direction);

    /**
     * Gets a local support point in a given direction without the object margin.
     *
     * @param direction The desired direction
     * @return The local support point as a vector3
     */
    public abstract Vector3 getLocalSupportPointWithoutMargin(Vector3 direction);

    /**
     * Gets the local extents in x,y and z direction with the desired margin .
     *
     * @param margin The required margin
     * @return The local extents
     */
    public abstract Vector3 getLocalExtents(float margin);

    /**
     * Gets the margin distance around the shape.
     *
     * @return The margin for the shape
     */
    public abstract float getMargin();

    /**
     * Computes the local inertia tensor of the collision shape for the mass. Stores the results in the passed matrix3x3.
     *
     * @param tensor The matrix3x3 in which the tensor should be stored
     * @param mass The mass of the shape
     */
    public abstract void computeLocalInertiaTensor(Matrix3x3 tensor, float mass);

    /**
     * Allocates and returns a copy of the object.
     *
     * @return A copy of the objects
     */
    @Override
    public abstract CollisionShape clone();

    /**
     * Tests equality between two collision shapes of the same type (same derived classes).
     *
     * @param otherCollisionShape The shape to test for equality
     */
    public abstract boolean isEqualTo(CollisionShape otherCollisionShape);

    /**
     * Update the AABB of a body using its collision shape.
     *
     * @param aabb The AABB to update
     * @param transform The AABB's transform
     */
    public void updateAABB(AABB aabb, Transform transform) {
        final Vector3 extents = getLocalExtents(ReactDefaults.OBJECT_MARGIN);
        final Matrix3x3 worldAxis = transform.getOrientation().getMatrix().getAbsoluteMatrix();
        final Vector3 worldExtents = new Vector3(
                worldAxis.getColumn(Matrix3x3.FIRST_COLUMN).dot(extents),
                worldAxis.getColumn(Matrix3x3.SECOND_COLUMN).dot(extents),
                worldAxis.getColumn(Matrix3x3.THIRD_COLUMN).dot(extents));
        final Vector3 minCoordinates = Vector3.subtract(transform.getPosition(), worldExtents);
        final Vector3 maxCoordinates = Vector3.add(transform.getPosition(), worldExtents);
        aabb.setMin(minCoordinates);
        aabb.setMax(maxCoordinates);
    }

    /**
     * Returns the number of similar created shapes.
     *
     * @return The number of similar created shapes
     */
    public int getNbSimilarCreatedShapes() {
        return mNbSimilarCreatedShapes;
    }

    /**
     * Increments the number of similar allocated collision shapes.
     */
    public void incrementNbSimilarCreatedShapes() {
        mNbSimilarCreatedShapes++;
    }

    /**
     * Decrements the number of similar allocated collision shapes.
     */
    public void decrementNbSimilarCreatedShapes() {
        mNbSimilarCreatedShapes--;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) {
            return true;
        }
        if (!(o instanceof CollisionShape)) {
            return false;
        }
        final CollisionShape that = (CollisionShape) o;
        return mType == that.mType && that.isEqualTo(this);
    }

    @Override
    protected void finalize() throws Throwable {
        try {
            if (mNbSimilarCreatedShapes != 0) {
                // Thrown exceptions are ignored, so we need to print instead
                System.err.println("The number of similar created shapes should be 0");
            }
        } finally {
            super.finalize();
        }
    }

    /**
     * An enumeration of the possible collision shape (box, sphere, cone and cylinder).
     */
    public static enum CollisionShapeType {
        BOX,
        SPHERE,
        CONE,
        CYLINDER
    }
}
