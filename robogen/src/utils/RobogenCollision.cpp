/*
 * @(#) RobogenCollision.cpp   1.0   March 21, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2014 Andrea Maesani, Joshua Auerbach
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
#include "utils/RobogenCollision.h"
#include "config/RobogenConfig.h"

#include "Robot.h"
#include "model/SimpleBody.h"

#include <algorithm>
#include <boost/smart_ptr/shared_ptr.hpp>

// ODE World
extern dWorldID odeWorld;

// Container for collisions
extern dJointGroupID odeContactGroup;

namespace robogen {

const int MAX_CONTACTS = 32; // maximum number of contact points per body


//CollisionData::CollisionData(boost::shared_ptr<Scenario> scenario) :
//		scenario_(scenario), hasObstacleCollisions_(false) {
//
//	//numCulled = 0;
//
//	for (size_t i=0; i<scenario->getRobot()->getBodyParts().size(); ++i) {
//		boost::shared_ptr<Model> model =
//				scenario->getRobot()->getBodyParts()[i];
//		for(size_t j=0; j<model->getBodies().size(); ++j) {
//			geomModelMap_[model->getBodies()[j]->getGeom()] = model;
//		}
//
//	}
//}
CollisionData::CollisionData(boost::shared_ptr<Scenario> scenario) :
		scenario_(scenario), hasObstacleCollisions_(false) {

    for(size_t i = 0; i < scenario->getRobots().size(); ++i){
		for (size_t j=0; j<scenario->getRobot(i)->getBodyParts().size(); ++j) {
			/**boost::shared_ptr<Model> model =
					scenario->getRobot(i)->getBodyParts()[j];
			std::map<dGeomID, boost::shared_ptr<Model> > geomap;
			for(size_t k = 0; k < model->getBodies().size(); ++k) {
				geomap[model->getBodies()[k]->getGeom()]
									= model;
			}
			geomModelMap_.push_back(geomap);*/

			boost::shared_ptr<Model> model =
						scenario->getRobot(i)->getBodyParts()[j];
			for(size_t k=0; k<model->getBodies().size(); ++k) {
				if (geomModelMap_.count(model->getBodies()[k]->getGeom())){
					std::cout << "Already exists!" << std::endl;
				}
				else{
					geomModelMap_[model->getBodies()[k]->getGeom()] = model;
				}

			}
		}
		//std::cout << "Done initializing model map for one robot. Size of model map for the one robot: "<< geomModelMap_.size() << std::endl;
    }
    //std::cout << "Done initializing CollisionData constructor. Final size of model map: "<< geomModelMap_.size() << std::endl;
}

bool CollisionData::ignoreCollision(dGeomID o1, dGeomID o2) {

	if (geomModelMap_.count(o1) == 0 || geomModelMap_.count(o2) == 0 )
		return false;
	return ( geomModelMap_[o1] == geomModelMap_[o2]);

}

/**bool CollisionData::ignoreCollision(dGeomID o1, dGeomID o2) {

//	if (geomModelMap_.count(o1) == 0 || geomModelMap_.count(o2) == 0 )
//		return false;
//	return ( geomModelMap_[o1] == geomModelMap_[o2]);
    //SM added
    bool inModelMap;
    for (size_t i = 0; i < geomModelMap_.size(); ++i){
        if (geomModelMap_[i].count(o1) == 0 || geomModelMap_[i].count(o2) == 0 )
            inModelMap = false;
        else{
            inModelMap = geomModelMap_[i][o1] == geomModelMap_[i][o2];
            break;
        }
    }
    return inModelMap;
}*/

bool CollisionData::isPartOfBody(dGeomID o1) {
    return geomModelMap_.count(o1);
    /**int count;
    for(size_t i  = 0; i < geomModelMap_.size(); ++i){
        count = geomModelMap_[i].count(o1);
    }
    return count;*/
}

void CollisionData::testObstacleCollisons(dGeomID o1, dGeomID o2) {
    /**for(size_t i  = 0; i < geomModelMap_.size(); ++i){
        if (	(isPartOfBody(o1) && dGeomGetClass(o2) == dBoxClass)
			||
			(isPartOfBody(o2) && dGeomGetClass(o1) == dBoxClass)) {
		//std::cout << "colliding with obstacle!!!" << std::endl;
		hasObstacleCollisions_ = true;
	}
    }*/
	if (	(isPartOfBody(o1) && dGeomGetClass(o2) == dBoxClass)
			||
			(isPartOfBody(o2) && dGeomGetClass(o1) == dBoxClass)) {
		//std::cout << "colliding with obstacle!!!" << std::endl;
		hasObstacleCollisions_ = true;
	}
}


/**void odeCollisionCallback(void *data, dGeomID o1, dGeomID o2) {
	//std::cout << "*****odeCollisionCallback - Begins*****" << std::endl;
	CollisionData *collisionData = static_cast<CollisionData*>(data);

	// Since we are now using complex bodies, just because two bodies
	// are connected with a joint does not mean we should ignore their
	// collision.  Instead we need to use the ignoreCollision method define
	// above, which will check if the two geoms are part of the same
	// model, in which case we can ignore.
	// TODO can we make this more efficient?

	//std::cout << "*****odeCollisionCallback - get body ids for both geoms: "<< o1<<" "<< o2 <<" *****" << std::endl;
	dBodyID b1 = dGeomGetBody(o1);
	dBodyID b2 = dGeomGetBody(o2);
	//std::cout << "*****odeCollisionCallback - done getting body ids for "<< o1<<" "<< o2 <<" *****" << std::endl;
	//if (b1 && b2 && dAreConnectedExcluding (b1,b2,dJointTypeContact)) {
	if (collisionData->ignoreCollision(o1, o2) ) {
		//collisionData->numCulled++;
		return;
	}


	dContact contact[MAX_CONTACTS];
	for (int i = 0; i < MAX_CONTACTS; i++) {
		contact[i].surface.slip1 = 0.01;
		contact[i].surface.slip2 = 0.01;
		contact[i].surface.mode = dContactSoftERP |
					dContactSoftCFM |
					dContactApprox1 |
					dContactSlip1 | dContactSlip2;
		// TODO use different value for self collisions and/or obstacles?
		contact[i].surface.mu = collisionData->getScenario()->getRobogenConfig(
									)->getTerrainConfig()->getFriction();
		contact[i].surface.soft_erp = 0.96;
		contact[i].surface.soft_cfm = 0.01;



	}
	//std::cout << "*****odeCollisionCallback - colliding geoms: "<< o1<<" and "<< o2 <<" *****" << std::endl;
	int collisionCounts = dCollide(o1, o2, MAX_CONTACTS, &contact[0].geom,
		sizeof(dContact));
	//std::cout << "*****odeCollisionCallback - done colliding geoms "<< o1<<" and "<< o2 <<" *****" << std::endl;

	if (collisionCounts > 0) {
		collisionData->testObstacleCollisons(o1, o2);
	}


	//std::cout << "*****odeCollisionCallback - creating contact joints for geaoms "<< o1<<" and "<< o2 <<" *****" << std::endl;
	for (int i = 0; i < collisionCounts; i++) {

		dJointID c = dJointCreateContact(odeWorld, odeContactGroup,
				contact + i);
		dJointAttach(c, b1, b2);


	}
	//std::cout << "*****odeCollisionCallback - done creating contact joints for geaoms "<< o1<<" and "<< o2 <<" *****" << std::endl;
	//std::cout << "*****odeCollisionCallback - Ends*****" << std::endl;
}*/
/**
 * Commenting this out for now because it is not working with the sensors.
 * TODO: Come back later and revise this code, and perhaps try make it work with the sensors?
 */
void odeCollisionCallback(void *data, dGeomID o1, dGeomID o2) {

	CollisionData *collisionData = static_cast<CollisionData*>(data);

	// Since we are now using complex bodies, just because two bodies
	// are connected with a joint does not mean we should ignore their
	// collision.  Instead we need to use the ignoreCollision method define
	// above, which will check if the two geoms are part of the same
	// model, in which case we can ignore.
	// TODO can we make this more efficient?

        if (dGeomIsSpace (o1) || dGeomIsSpace (o2)) {  
            // colliding a space with something :
            dSpaceCollide2 (o1, o2, data, odeCollisionCallback);  
            // collide all geoms internal to the space(s)
            if (dGeomIsSpace (o1))
                dSpaceCollide ((dSpaceID)o1, data, odeCollisionCallback);
            if (dGeomIsSpace (o2))
                dSpaceCollide ((dSpaceID)o2, data, odeCollisionCallback); 
        }

        else {
            // colliding two non-space geoms, so generate contact
            // points between o1 and o2
            dBodyID b1 = dGeomGetBody(o1);
            dBodyID b2 = dGeomGetBody(o2);
            if (collisionData->ignoreCollision(o1, o2) ) {
            	return;
            }

            dContact contact[MAX_CONTACTS];
            for (int i = 0; i < MAX_CONTACTS; i++) {
                contact[i].surface.slip1 = 0.01;
                contact[i].surface.slip2 = 0.01;
                contact[i].surface.mode = dContactSoftERP |
                                        dContactSoftCFM |
                                        dContactApprox1 |
                                        dContactSlip1 | dContactSlip2;
                // TODO use different value for self collisions and/or obstacles?
                contact[i].surface.mu = collisionData->getScenario()->getRobogenConfig(
                                                                        )->getTerrainConfig()->getFriction();
                contact[i].surface.soft_erp = 0.96;
                contact[i].surface.soft_cfm = 0.01;
            }
            int collisionCounts = dCollide(
            								o1,
											o2,
											MAX_CONTACTS,
											&contact[0].geom,
            								sizeof(dContact));

            if (collisionCounts > 0) {
                collisionData->testObstacleCollisons(o1, o2);
            }

            for (int i = 0; i < collisionCounts; i++) {
                dJointID c = dJointCreateContact(odeWorld, odeContactGroup,
                                contact + i);
                dJointAttach(c, b1, b2);
            }
        }
}
}
