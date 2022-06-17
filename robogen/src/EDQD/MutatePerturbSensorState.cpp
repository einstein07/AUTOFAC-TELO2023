/*
 * @(#) MutatePertubSensorState.cpp   1.0   Jun 14, 2022
 *
 * Sindiso Mkhatshwa (mkhsin035@myuct.ac.za)
 *
 * Nitschke Laboratory, UCT
 *
 * @(#) $Id$
 */
#include "MutatePerturbSensorState.h"
#include "SensorMorphology.h"

namespace robogen{
	MutatePerturbSensorState::MutatePerturbSensorState(double sigma):sigma_(sigma){}
	MutatePerturbSensorState::~MutatePerturbSensorState(){}

	void MutatePerturbSensorState::mutateSensorGroup(std::vector< boost::shared_ptr<Robot> > robots){
		double delta = 0.0;
		for (unsigned int i = 0; i < robots.size(); i++){
			for (unsigned int j = 0; j < robots[i]->getSensors().size(); j++){
				if (boost::dynamic_pointer_cast<SensorElement>(robots[i]-> getSensors()[j])){
				if ( 	boost::dynamic_pointer_cast< SensorElement>(robots[i]-> getSensors()[j])-> getType() == SensorElement::Type::RESOURCET1 ||
						boost::dynamic_pointer_cast< SensorElement>(robots[i]-> getSensors()[j])-> getType() == SensorElement::Type::RESOURCET2 ||
						boost::dynamic_pointer_cast< SensorElement>(robots[i]-> getSensors()[j])-> getType() == SensorElement::Type::RESOURCET3 ||
						boost::dynamic_pointer_cast< SensorElement>(robots[i]-> getSensors()[j])-> getType() == SensorElement::Type::RESOURCET4 ||
						boost::dynamic_pointer_cast< SensorElement>(robots[i]-> getSensors()[j])-> getType() == SensorElement::Type::RESOURCET1){
						double newValue = boost::dynamic_pointer_cast< SensorElement>(robots[i]-> getSensors()[j])-> getValue() + delta;
						boost::dynamic_pointer_cast< SensorElement>(robots[i]-> getSensors()[j])-> setValue(newValue);
					}
				}
			}
		}
	}
}
