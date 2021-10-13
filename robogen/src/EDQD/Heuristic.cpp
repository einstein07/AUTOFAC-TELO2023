/*
 * @(#) Heuristic.cpp   1.0   Sep 28, 2021
 *
 * Sindiso Mkhatshwa (mkhsin035@myuct.ac.za)
 *
 * Nitschke Laboratory, UCT
 *
 * @(#) $Id$
 */
#include "Heuristic.h"

namespace robogen{
	Heuristic::Heuristic(boost::shared_ptr<Robot> robot): robot_(robot), priority_(0){}
	Heuristic::~Heuristic(){}

	/**
	 * Set the priority of this heuristic
	 */
	void Heuristic::setPriority(int priority){
		priority_ = priority;
	}

	/**
	 * Returns motor signals that will drive the agent towards the desired
	 * target position
	 * @param targetPosition The position of the target in local coordinates
	 * @return The heuristic motor signals
	 */
	osg::Vec2d Heuristic::driveToTargetPosition(osg::Vec2d targetPosition){
		double left, right;
		//osg::Vec3d coreGlobalPos = robot_->getCoreComponent()->getRoot()->getPosition();
		osg::Vec3d calcLocalPos = getLocalPoint(osg::Vec3d(targetPosition.x(), targetPosition.y(), 0));
		/**osg::Vec3d lx1 = getLocalPoint(osg::Vec3d(20, 20, 0));
		osg::Vec3d lx2 = getLocalPoint(osg::Vec3d(-20, 20, 0));
		osg::Vec3d lx3 = getLocalPoint(osg::Vec3d(-20, -20, 0));
		osg::Vec3d lx4 = getLocalPoint(osg::Vec3d(20, -20, 0));
		std::cout << "X1 in local coordinates: (" << lx1.x() << ", " << lx1.y() << ")" << std::endl;
		std::cout << "X2 in local coordinates: (" << lx2.x() << ", " << lx2.y() << ")" << std::endl;
		std::cout << "X3 in local coordinates: (" << lx3.x() << ", " << lx3.y() << ")" << std::endl;
		std::cout << "X4 in local coordinates: (" << lx4.x() << ", " << lx4.y() << ")" << std::endl;*/
		double y = calcLocalPos.y();// - coreGlobalPos.y();
		double x = calcLocalPos.x();// - coreGlobalPos.x();
		double targetAngle = atan2 (y, x);
		osg::Vec3d coreGlobalPos = robot_->getCoreComponent()->getRoot()->getPosition();
		//osg::Vec3d coreGlobalPos2 = robot_->getCoreComponent()->getRootPosition();
		osg::Vec3d robbotCalcLocalPos = getLocalPoint(robot_->getCoreComponent()->getRoot()->getPosition());
		//osg::Vec3d retrievedLocalPos = robot_->getCoreComponent()->getRoot()->getLocalPosition();*/

		//std::cout << "Robot global position (simple body): (" << coreGlobalPos.x() << ", " << coreGlobalPos.y() << ", " << coreGlobalPos.z() <<")." << std::endl;
		//std::cout << "Robot global position (model): (" << coreGlobalPos2.x() << ", " << coreGlobalPos2.y() << ", " << coreGlobalPos2.z() <<")." << std::endl;
		//std::cout << "Robot calculated local position: (" << robbotCalcLocalPos.x() << ", " << robbotCalcLocalPos.y() << ", " << robbotCalcLocalPos.z() <<")." << std::endl;
		//std::cout << "Robot retrieved local position: (" << retrievedLocalPos.x() << ", " << retrievedLocalPos.y() << ", " << retrievedLocalPos.z() <<")." << std::endl;*/
		//std::cout << "Target area in global coordinates: (" << targetPosition.x() << ", " << targetPosition.y() << ")" << std::endl;
		//std::cout << "Target area in local coordinates: (" << calcLocalPos.x() << ", " << calcLocalPos.y() << ")" << std::endl;
		//std::cout << "Angle to target area: " << targetAngle * 180 / M_PI << " degrees" << std::endl;
		if(std::abs(targetAngle) > M_PI){
			if(signbit(targetAngle))
				targetAngle = M_PI*(-1);
			else
				targetAngle = M_PI;
		}

		// Different response for each of four quadrants
		if (targetAngle >= 0) {
			if (targetAngle < Heuristic::HALF_PI) {
				// First
				left = (Heuristic::HALF_PI - targetAngle) / Heuristic::HALF_PI;
				right = 1;
				//std::cout << "First quadrant. Left: " << left << ", Right: " << right << std::endl;
			} else {
				// Second
				left = -(targetAngle - Heuristic::HALF_PI) / Heuristic::HALF_PI;
				right = -1;
				//std::cout << "Second quadrant. Left: " << left << ", Right: " << right << std::endl;
			}
		} else {
			if (targetAngle < -Heuristic::HALF_PI) {
				// Third
				left = 1;//-1;
				right = -(targetAngle + Heuristic::HALF_PI) / Heuristic::HALF_PI;
				//std::cout << "Third quadrant. Left: " << left << ", Right: " << right << std::endl;
			} else {
				// Fourth
				left = 1;
				right = (Heuristic::HALF_PI + targetAngle) / Heuristic::HALF_PI;
				//std::cout << "Fourth quadrant. Left: " << left << ", Right: " << right << std::endl;
			}
		}

		return osg::Vec2d(left, right);

	}

	osg::Vec3 Heuristic::getLocalPoint(osg::Vec3 globalPoint){
	    dVector3 localPoint;
	    dBodyID dRobotID = robot_->getCoreComponent()->getRoot()->getBody();
	    dBodyGetPosRelPoint (
	    					dRobotID,
	                        globalPoint.x(),
	                        globalPoint.y(),
	                        globalPoint.z(),
	                        localPoint
	                    );
	    return osg::Vec3(localPoint[0], localPoint[1], localPoint[2]);
	}
}

