/*
 * @(#) Robot.h   1.0   Mar 4, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 * Titus Cieslewski (dev@titus-c.ch)
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
#ifndef ROBOGEN_ROBOT_H_
#define ROBOGEN_ROBOT_H_

#include <boost/shared_ptr.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/copy.hpp>
#include <map>
#include <vector>

#include "Robogen.h"
#include "robogen.pb.h"
#include "model/Connection.h"

#include "model/CompositeBody.h"

extern "C" {
#include "brain/NeuralNetwork.h"
}

namespace robogen {

class Model;
class Motor;
class Sensor;
/**
 * \brief Struct to describe BodyEdgeDescriptorTag
 */
struct BodyEdgeDescriptorTag {
	/**
	 * \brief Edge property tag
	 */
	typedef boost::edge_property_tag kind;
};
typedef boost::property<BodyEdgeDescriptorTag, boost::shared_ptr<Connection> >
BodyEdgeProperty;
typedef boost::adjacency_list<boost::listS, boost::vecS, boost::directedS,
		boost::no_property, BodyEdgeProperty> BodyGraph;
typedef boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS,
		boost::no_property, BodyEdgeProperty> BodyUndirectedGraph;
typedef boost::graph_traits<BodyGraph>::edge_descriptor BodyEdge;

/**
 * \brief Class that describes a ROBOGEN robot
 */
class Robot {

public:
	/**
	 * \brief Error-less constructor.
	 */
	Robot();

	/**
	 * \brief Initializes a Robogen robot from a robogen message
	 * @param odeWorld
	 * @param odeSpace
	 * @param robotSpec
	 */
	bool init(dWorldID odeWorld, dSpaceID odeSpace,
				const robogenMessage::Robot& robotSpec,
				bool printInfo=false, bool printInitErrors=true);
	/**
	 * \brief Modified - to allow constructor initialization with two
	 * collision spaces - one for the whole word and the other
	 * specific to this robot - SM
	 */

	bool init(dWorldID odeWorld, dSpaceID odeSpace,
			dSpaceID robotSpace, int& nbAlive, const robogenMessage::Robot& robotSpec,
				bool printInfo=false, bool printInitErrors=true);

	/**
	 * \brief Destructor
	 */
	virtual ~Robot();

	/**
	 * \brief Returns body parts
	 *
	 *  @return  the body parts of which the robot is composed of
	 */
	const std::vector<boost::shared_ptr<Model> >& getBodyParts();

	/**
	 * \brief Returns ANN
	 *
	 * @return the neural network that controls the robot
	 */
	const boost::shared_ptr<NeuralNetwork>& getBrain() const;

	/**
	 * \brief Returns sensors
	 *
	 * @return the sensors of the robot
	 */
	const std::vector<boost::shared_ptr<Sensor> >& getSensors() const;

	/**
	 * \brief Returns motors
	 *
	 * @return the motors of the robot
	 */
	const std::vector<boost::shared_ptr<Motor> >& getMotors();

	/**
	 * \brief Returns core component
	 *
	 * @return the core component of the robot
	 */
	boost::shared_ptr<Model> getCoreComponent();

	/**
	 * \brief Translate the robot
	 *
	 * @param pos Amount of translation
	 */
	void translateRobot(const osg::Vec3& translation);

	/**
	 * \brief Rotate the robot
	 *
	 * @param rot rotation Quaternion
	 */
	void rotateRobot(const osg::Quat &rot);

	/**
	 * \brief Returns the robot axis-aligned bounding box
	 */
	void getAABB(double& minX, double& maxX, double& minY, double& maxY,
			double& minZ, double& maxZ);

	/**
	 * \brief Returns robot id
	 *
	 * @return the robot ID
	 */
	int getId() const;

	/**
	 * \brief Returns specified body part
	 */
	boost::shared_ptr<Model> getBodyPart(std::string id);

	/**
	 * \brief Returns robogen message
	 *
	 * @return message that generated the robot
	 */
	const robogenMessage::Robot& getMessage();
	/**
	 * \brief Returns body connections
	 */
	const std::vector<boost::shared_ptr<Connection> >& getBodyConnections()const;
	/**
	 * \brief Traverse body
	 */
	void traverseBody(const std::vector<boost::shared_ptr<Model> >,
			const std::vector<boost::shared_ptr<Connection> >);
	/**
	 * \brief Returns root
	 */
	int getRoot();
	/**
	 * \brief Add joint
	 */
	inline void addJoint(boost::shared_ptr<Joint> joint) {
		joints_.push_back(joint);
	}

	/**
	 * \brief Merges bodies connected with fixed joints into complex bodies
	 */
	void optimizePhysics();
        
	/**
	 * \brief Returns the status of this robot vis-a-vis resources - SM
	 * @return returns the status of this robot with regard to
	 * resources in the environment
	 */
	const bool isBoundToResource();
        
	/**
	 * \brief Sets the status of this robot vis-a-vis resources in the environment - SM
	 * @param isBoundToResource sets the status of this robot with regard to
	 * resources in the environment
	 */
	void setBoundToResource(bool isBoundToResource);

	/**
	 * \brief Sets the id of the resource bound to this robot
	 */
	void setBoundResourceId(int resourceId){resourceId_ = resourceId;}

	/**
	 * \brief Gets the id of the resource this robot is bound to
	 */
	int getBoundResourceId(){return resourceId_;}

	/**
	 * \brief CH - Brain-body complexity of robot
	 */
	float complexity_;

	/**
	 * \brief CH - Flag to indicate if complexity cost should be imposed or not
	 */
	int complexityCost_;

	/**
	 * \brief BK - for novelty score, to record end position achieved during evaluation
	 */
	float endPosX_;
	/**
	 * \brief BK - for novelty score, to record end position achieved during evaluation
	 */
	float endPosY_;

	/**
	 * \brief Set robot state
	 */
	 void setAlive( bool value ) { isAlive_ = value; }
	 /**
	  * \brief Returns robot status
	  */
	 bool isAlive() { return isAlive_; }
private:

	/**
	 * Decodes the body of the robot
	 * @param robotBody
	 * @return true if the operation completed successfully
	 */
	bool decodeBody(const robogenMessage::Body& robotBody);

	/**
	 * Decodes the brain of the robot
	 * @param robotBrain
	 * @return true if the operation completed successfully
	 */
	bool decodeBrain(const robogenMessage::Brain& robotBrain);

	/**
	 * Connects all body parts to the root part
	 */
	void reconnect();

	/**
	 * ODE physics world
	 */
	dWorldID odeWorld_;

	/**
	 * ODE collision space
	 */
	dSpaceID odeSpace_;

	/**
	 * SM Added
	 * Per robot collision space
	 */
	dSpaceID robotSpace_;

	/**
	 * Robot body parts
	 */
	std::vector<boost::shared_ptr<Model> > bodyParts_;

	/**
	 * Connections between the body parts.
	 */
	std::vector<boost::shared_ptr<Connection> > bodyConnections_;

	/**
	 * Mapping from body part to associated sensors
	 */
	std::map<unsigned int, std::vector<boost::shared_ptr<Sensor> > > bodyPartsToSensors_;

	/**
	 * Mapping from body part to associated motors
	 */
	std::map<unsigned int, std::vector<boost::shared_ptr<Motor> > > bodyPartsToMotors_;

	/**
	 * Robot sensors
	 */
	std::vector<boost::shared_ptr<Sensor> > sensors_;

	/**
	 * Robot motors
	 */
	std::vector<boost::shared_ptr<Motor> > motors_;

	/**
	 * Neural network
	 */
	boost::shared_ptr<NeuralNetwork> neuralNetwork_;

	/**
	 * Maps the identifier of a body part with the body part in the bodyParts_ vector
	 */
	std::map<std::string, unsigned int> bodyPartsMap_;

	/**
	 * The core component of the robot
	 */
	boost::shared_ptr<Model> coreComponent_;

	/**
	 * Robot ID
	 */
	int id_;

	/**
	 * Root part index of the robot
	 */
	int rootNode_;

	/**
	 * Boost graph of body tree.
	 */
	boost::shared_ptr<BodyGraph> bodyTree_;

	/**
	 * Joint group of connections between parts
	 */
	dJointGroupID connectionJointGroup_;

	/**
	 * Contains the robot message
	 */

	const robogenMessage::Robot* robotMessage_;

	bool printInfo_;

	bool printInitErrors_;

	// store joints created by connecting components
	// not a set because we don't want iteration order to depend on address,
	// but we will use a set to check for uniqueness while iterating
	std::vector<boost::shared_ptr<Joint> > joints_;

	// store the composite bodies formed by replacing fixed joints
	std::vector<boost::shared_ptr<CompositeBody> > composites_;
        
	/**
	 * SM - to indicate when this robot is bound to a resource
	 */
	bool isBoundToResource_;

	/**
	 * SM Added - to indicate the id of the resource currently attached to
	 */
	int resourceId_;

	/**
	 * SM - Object information used by sensor to identify the nature of this object
	 */
	ObjectData data_;

	/**
	 * SM - robot behavior is executed *only* if robot is alive. (default is: alive)
	 */
    bool isAlive_;

};

}

#endif /* ROBOGEN_ROBOT_H_ */
