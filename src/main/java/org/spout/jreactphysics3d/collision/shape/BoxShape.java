/*
 * This file is part of JReactPhysics3D.
 *
 * Copyright (c) 2013 Spout LLC <http://www.spout.org/>
 * JReactPhysics3D is licensed under the Spout License Version 1.
 *
 * JReactPhysics3D is free software: you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * In addition, 180 days after any changes are published, you can use the
 * software, incorporating those changes, under the terms of the MIT license,
 * as described in the Spout License Version 1.
 *
 * JReactPhysics3D is distributed in the hope that it will be useful, but WITHOUT ANY
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
package org.spout.jreactphysics3d.collision.shape;

import org.spout.jreactphysics3d.Configuration;
import org.spout.jreactphysics3d.mathematics.Matrix3x3;
import org.spout.jreactphysics3d.mathematics.Vector3;

/**
 * This class represents a 3D box shape. Those axis are unit length.
 * The three extents are half-widths of the box along the three
 * axis x, y, z local axis. The "transform" of the corresponding
 * rigid body gives an orientation and a position to the box.
 */
public class BoxShape extends CollisionShape {
	private Vector3 mExtent;

	/**
	 * Constructs a BoxShape using Vector extents
	 * 
	 * @param extent 
	 */
	public BoxShape(Vector3 extent){
	        super(CollisionShape.CollisionShapeType.BOX);
	        mExtent = extent;
	}

	/**
	 * Gets BoxShape extents
         * 
	 * @return The Vector3 that representing BoxShape extents
	 */
	public Vector3 getExtent(){
 		return mExtent;
 	}
    
	/**
	 * Sets BoxShape extents to Vector3 extent
         * 
	 * @param extent 
	 */
	public void set(Vector3 extent){
	        mExtent = extent;
	}

	@Override
	public Vector3 getLocalSupportPointWithMargin(Vector3 direction) {
        	float margin = getMargin();

        	if(margin < 0.0) {            
			throw new IllegalArgumentException("Error HERE");
		}
		return new Vector3(direction.getX() < 0.0 ? - mExtent.getX() - margin : mExtent.getX() + margin,
				direction.getX() < 0.0 ? - mExtent.getX() - margin : mExtent.getX() + margin,
				direction.getX() < 0.0 ? - mExtent.getX() - margin : mExtent.getX() + margin);
	}

	@Override
	public Vector3 getLocalSupportPointWithoutMargin(Vector3 direction) {
		return new Vector3(direction.getX() < 0.0 ? - mExtent.getX() : mExtent.getX(),
				direction.getX() < 0.0 ? - mExtent.getX() : mExtent.getX(),
				direction.getX() < 0.0 ? - mExtent.getX() : mExtent.getX());
	}

	@Override
	public Vector3 getLocalExtents(float margin) {
		return new Vector3(getMargin(), getMargin(), getMargin()).add(mExtent);
	}
	
	@Override
	public float getMargin() {
		return Configuration.OBJECT_MARGIN;
	}

	@Override
	public void computeLocalInertiaTensor(Matrix3x3 tensor, float mass) {
		float factor = (1.0f/3.0f) * mass;
		float xSquare = mExtent.getX() * mExtent.getX();
		float ySquare = mExtent.getY() * mExtent.getY();
		float zSquare = mExtent.getZ() * mExtent.getZ();
		tensor.setAllValues(factor, (ySquare + zSquare), 0.0f, 
				0.0f, factor * (xSquare + zSquare), 0.0f, 
				0.0f, 0.0f, factor * (xSquare + xSquare));
	}
}
