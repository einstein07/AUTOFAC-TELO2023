/*
 * @(#) Model.h   1.0   Feb 8, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2013 Andrea Maesani
 *
 * Laboratory of Intelligent Systems, EPFL
 *
 * This file is part of the ROBOGEN Framework.
 *
 * The ROBOGEN Framework is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License (GPL)
 * as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @(#) $Id$
 */
#ifndef ROBOGEN_MODEL_H_
#define ROBOGEN_MODEL_H_

#include <osg/PositionAttitudeTransform>
#include <iostream>
#include <map>
#include <stdexcept>
#include <boost/enable_shared_from_this.hpp>

#include "Robogen.h"
#include "SimpleBody.h"
#include "Joint.h"

namespace robogen {

/**
 * \brief Class to describe a Model
 *
 * A slot center must always be place on the external surface of the slot
 */
class Model : public boost::enable_shared_from_this<Model> {

public:

	/**
	 * \brief Constructor
	 */
	Model(dWorldID odeWorld, dSpaceID odeSpace,std::string id);
	/**
	 *
	 * \brief Constructor takes both an individual robot's space and the whole world space
	 *
	 * Each robot has its own collision space - SM
	 */
	Model(dWorldID odeWorld, dSpaceID odeSpace,  dSpaceID robotSpace, std::string id);

	/**
	 * \brief Destructor
	 */
	virtual ~Model();

	/**
	 * \brief Returns true if the model initialization completed successfully
	 */
	virtual bool initModel() = 0;

	/**
	 * \brief Returns id string of the part
	 */
	const std::string &getId();

	/**
	 * \brief Returns body corresponding to the selected slot
	 */
	virtual boost::shared_ptr<SimpleBody> getSlot(unsigned int i) = 0;

	/**
	 * \brief Returns slot position, in world coordinates
	 */
	virtual osg::Vec3 getSlotPosition(unsigned int i) = 0;

	/**
	 * \brief Returns slot axis (normal to the slot)
	 */
	virtual osg::Vec3 getSlotAxis(unsigned int i) = 0;

	/**
	 * \brief Returns tangent to the slot surface, defines the zero rotation orientation
	 * on that slot
	 */
	virtual osg::Vec3 getSlotOrientation(unsigned int i) = 0;

	/**
	 * \brief Returns root body
	 */
	virtual boost::shared_ptr<SimpleBody> getRoot() = 0;

	/**
	 * \brief Returns position of the root part
	 */
	osg::Vec3 getRootPosition();

	/**
	 * \brief Returns attitude of the root part (a quaternion)
	 */
	osg::Quat getRootAttitude();

	/**
	 * \brief Returns position of the specified body
	 */
	osg::Vec3 getBodyPosition(int id);

	/**
	 * \brief Returns attitude of the specified body
	 */
	osg::Quat getBodyAttitude(int id);

	/**
	 * \brief Sets the position of the root part.
	 * Will force all the other bodies to translate accordingly.
	 */
	void setRootPosition(const osg::Vec3& pos);

	/**
	 * \brief Translate the root part of the specified amount
	 * Will force all the other bodies to translate accordingly.
	 */
	void translateRootPosition(const osg::Vec3& translation);

	/**
	 * \brief Sets the attitude of the root part (a quaternion).
	 * Will force all the other bodies to rotate accordingly.
	 */
	void setRootAttitude(const osg::Quat& quat);

	/**
	 * \brief Returns the physics world
	 */
	dWorldID getPhysicsWorld();

	/**
	 * \brief Returns robot collision space - SM
	 */
	dSpaceID getODECollisionSpace();


	/**
	 * \brief Returns robot collision space - SM
	 */
	dSpaceID getRobotCollisionSpace();

	/**
	 * @return the collision space
	 */
	/**dSpaceID getCollisionSpace();*/

	/**
	 * \brief Returns specified body
	 */
	boost::shared_ptr<SimpleBody> getBody(int id);

	/**
	 * \brief Returns all the bodies belonging to this model
	 */
	std::vector<boost::shared_ptr<SimpleBody> > getBodies();

	/**
	 * \brief Returns all the ids belonging to this model
	 */
	std::vector<int> getIDs();

	/**
	 * \brief Create a box geometry for the body
	 * @param mass
	 * @param pos
	 * @param lengthX
	 * @param lengthY
	 * @param lengthZ
	 * @param label
	 */
	boost::shared_ptr<SimpleBody> addBox(float mass,
			const osg::Vec3& pos, float lengthX, float lengthY, float lengthZ,
			int label=-1);

	/**
	 * \brief Create a capsule geometry for the body
	 * @param mass
	 * @param pos
	 * @param direction
	 * @param radius
	 * @param height
	 * @param label
	 */
	boost::shared_ptr<SimpleBody> addCapsule(float mass, const osg::Vec3& pos,
			int direction, float radius, float height, int label=-1);

	/**
	 * \brief Create a cylinder geometry for the body
	 * @param mass
	 * @param pos
	 * @param direction
	 * @param radius
	 * @param height
	 * @param label
	 */
	boost::shared_ptr<SimpleBody> addCylinder(float mass, const osg::Vec3& pos,
			int direction, float radius, float height, int label=-1);

	/**
	 * \brief Fix bodies together
	 * @param b1 first body
	 * @param b2 second body
	 */
	void fixBodies(boost::shared_ptr<SimpleBody> b1,
			boost::shared_ptr<SimpleBody> b2);

	/**
	 * Fix bodies together
	 * @param bodies vector of bodies to be fixed together
	 */
	void fixBodies(std::vector<boost::shared_ptr<AbstractBody> >bodies);


	/**
	 * \brief Attach with hinge
	 * @param b1 first body
	 * @param b2 second body
	 * @param axis hinge's rotation axis
	 * @param anchor hinge's anchor location
	 */
	boost::shared_ptr<Joint> attachWithHinge(boost::shared_ptr<SimpleBody> b1,
			boost::shared_ptr<SimpleBody> b2, osg::Vec3 axis, osg::Vec3 anchor);

	/**
	 * \brief Attach with universal joint
	 * @param b1 first body
	 * @param b2 second body
	 * @param axis1 universal's first rotation axis
	 * @param axis2 universal's second rotation axis
	 * @param anchor universal's anchor location
	 */
	boost::shared_ptr<Joint> attachWithUniversal(boost::shared_ptr<SimpleBody> b1,
			boost::shared_ptr<SimpleBody> b2, osg::Vec3 axis,osg::Vec3 axis2,
			osg::Vec3 anchor);


	/**
	 * \brief Set orientation to parent slot with increments of 90 degrees
	 * @param orientation integer between 0 and 3, specifying the amount of
	 * additional 90 degree increments after attaching the part to its parent.
	 */
	bool setOrientationToParentSlot(int orientation);

	/**
	 * \brief Get orientation to parent slot.
	 * @return a number specifying the amount of additional 90 degree increments
	 * after attaching the part to its parent.
	 */
	int getOrientationToParentSlot();

	/**
	 * \brief Specify parent's orientation relative to root (increments of 90 degrees)
	 * @param orientation integer between 0 and 3, specifying the amount of
	 * 90 degree increments of parent relative to root
	 * Needed to enforce "planarity" of bricks
	 */
	bool setParentOrientation(int orientation);

	/**
	 * \brief Returns orientation to root
	 */
	int getOrientationToRoot();

	/**
	 * \brief Get the internal joints of the model
	 */
	inline const std::vector< boost::shared_ptr<Joint> > &getJoints() {
		return joints_;
	}
	/**
	 * \brief Removes specified Joint
	 */
	void removeJoint(boost::shared_ptr<Joint> joint);

protected:

	/**
	 * Add a body to the model
	 * @param body
	 */
	void addBody(boost::shared_ptr<SimpleBody>, int id);


private:

	std::set<boost::shared_ptr<AbstractBody> > bodiesToMove();

	/**
	 * ODE world
	 */
	dWorldID odeWorld_;

	/**
	 * ODE collision space
	 */
	dSpaceID odeSpace_;

	/**
	 * SM added - Robot collision space
	 */
	dSpaceID robotSpace_;

	/**
	 * User-defined identifier of the part
	 */
	const std::string id_;

	/**
	 * ODE Bodies composing the model
	 */
	std::map<int, boost::shared_ptr<SimpleBody> > bodies_;

	/**
	 * Orientation at parent slot: 0-3, where the number stands for
	 * increments of 90 degrees when attaching to the parent part.
	 */
	int orientationToParentSlot_;

	int orientationToRoot_;

	/**
	 * Internal joints of the model
	 *
	 */
	std::vector< boost::shared_ptr<Joint> > joints_;

};

}

#endif /* ROBOGEN_MODEL_H_ */
