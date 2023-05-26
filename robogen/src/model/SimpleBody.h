/*
 * @(#) SimpleBody.h   1.0   September 16, 2015
 *
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2015 Joshua Auerbach
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

#ifndef ROBOGEN_SIMPLE_BODY_H_
#define ROBOGEN_SIMPLE_BODY_H_

#include "Robogen.h"
#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include "AbstractBody.h"

namespace robogen {

class Model;
/**
 * \brief Class to describe a SimpleBody
 */
class SimpleBody : public AbstractBody {

public:
	/**
	 * \brief Constructor
	 */
	SimpleBody(boost::shared_ptr<Model> model, dMass mass,
			dGeomID geom, const osg::Vec3& pos,
			const osg::Quat& attitude = osg::Quat());

	/**
	 * \brief Destructor
	 */
	inline ~SimpleBody() {
		/*if(geom_) {
			printf("destroying geom!!!\n");
			std::cout << geom_ << std::endl;
			dGeomDestroy(geom_);
			geom_ = NULL;
		}*/
		joints_.clear();
	}
	/**
	 * \brief Adds a Joint
	 */
	void addJoint(boost::shared_ptr<Joint> joint);
	/**
	 * \brief Clears Joints
	 */
	void clearJoints();
	/**
	 * \brief Removes specified Joint
	 */
	void removeJoint(boost::shared_ptr<Joint> joint);
	/**
	 * \brief Returns Joints
	 */
	inline const std::vector<boost::shared_ptr<Joint> > &getJoints() {
		return joints_;
	}

	/**
	 * \brief Returns position
	 */
	osg::Vec3 getPosition();
	/**
	 * \brief Returns attitude
	 */
	osg::Quat getAttitude();
	/**
	 * \brief Returns local position
	 */
	osg::Vec3 getLocalPosition();
	/**
	 * \brief Returns local attitude
	 */
	osg::Quat getLocalAttitude();
	/**
	 * \brief Returns Geom
	 */
	inline const dGeomID& getGeom() {
		return geom_;
	}
	/**
	 * \brief Returns mass
	 */
	inline const dMass& getMass() {
		return mass_;
	}
	/**
	 * \brief Returns Mass
	 */
	const boost::weak_ptr<Model>& getModel();
	/**
	 * \brief Returns specified position
	 */
	inline const osg::Vec3& getSpecifiedPosition() {
		return specifiedPosition_;
	}
	/**
	 * \brief Returns specified attitude
	 */
	inline const osg::Quat& getSpecifiedAttitude() {
		return specifiedAttitude_;
	}
	/**
	 * \brief Sets specified position
	 */
	inline void setSpecifiedPosition(osg::Vec3 specifiedPosition) {
		specifiedPosition_ = specifiedPosition;
	}
	/**
	 * \brief Sets specified attitude
	 */
	inline void setSpecifiedAttitude(osg::Quat specifiedAttitude) {
		specifiedAttitude_ = specifiedAttitude;
	}
	/**
	 * \brief overwrite parent methods to also update specified position / attitude
	 */
	inline void setPosition(osg::Vec3 position) {
		AbstractBody::setPosition(position);
		setSpecifiedPosition(position);
	}
	/**
	 * \brief Sets attitude
	 */
	inline void setAttitude(osg::Quat attitude) {
		AbstractBody::setAttitude(attitude);
		setSpecifiedAttitude(attitude);
	}



private:

	boost::weak_ptr<Model> model_;
	dMass mass_;
	dGeomID geom_;

	osg::Vec3 specifiedPosition_;
	osg::Quat specifiedAttitude_;

	std::vector<boost::shared_ptr<Joint> > joints_;
};

}

#endif /* ROBOGEN_SIMPLE_BODY_H_ */
