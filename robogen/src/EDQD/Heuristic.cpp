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

	Heuristic::Heuristic(boost::shared_ptr<Robot> robot, boost::shared_ptr<Scenario> scenario):
			robot_(robot), scenario_(scenario), priority_(0){}

	Heuristic::~Heuristic(){}

	void Heuristic::setPriority(int priority){
		priority_ = priority;
	}

	osg::Vec2d Heuristic::driveToTargetPosition(osg::Vec2d targetPosition){
		double left, right;
		osg::Vec3d calcLocalPos = getLocalPoint(osg::Vec3d(targetPosition.x(), targetPosition.y(), 0));

		double y = calcLocalPos.y();// - coreGlobalPos.y();
		double x = calcLocalPos.x();// - coreGlobalPos.x();
		double targetAngle = atan2 (y, x);
		osg::Vec3d coreGlobalPos = robot_->getCoreComponent()->getRoot()->getPosition();
		osg::Vec3d robbotCalcLocalPos = getLocalPoint(robot_->getCoreComponent()->getRoot()->getPosition());

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
			} else {
				// Second
				left = -(targetAngle - Heuristic::HALF_PI) / Heuristic::HALF_PI;
				right = -1;
			}
		} else {
			if (targetAngle < -Heuristic::HALF_PI) {
				// Third
				left = -1;
				right = -(targetAngle + Heuristic::HALF_PI) / Heuristic::HALF_PI;
			} else {
				// Fourth
				left = 1;
				right = (Heuristic::HALF_PI + targetAngle) / Heuristic::HALF_PI;
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

	osg::Vec3d Heuristic::getGlobalPoint(osg::Vec3d localPoint){
		dVector3 result;
		dBodyID dRobotID = robot_->getCoreComponent()->getRoot()->getBody();
		dBodyGetRelPointPos (
				dRobotID,
						localPoint.x(),
						localPoint.y(),
						localPoint.z(),
						result
					);

		return osg::Vec3d(result[0], result[1], result[2]);

	}

}

