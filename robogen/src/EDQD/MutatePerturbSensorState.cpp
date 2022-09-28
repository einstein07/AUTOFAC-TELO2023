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
#include "model/sensors/SensorMorphology.h"

namespace robogen{
	MutatePerturbSensorState::MutatePerturbSensorState(double sigma):sigma_(sigma), minValue_(0.0), maxValue_(0.3){
		srand((unsigned) time(NULL));
	}
	MutatePerturbSensorState::~MutatePerturbSensorState(){}

	void MutatePerturbSensorState::mutateSensorGroup(std::vector< boost::shared_ptr<Robot> > robots){
		mutateSigmaValue();
		double delta = randgaussian() * sigma_;
		double normalisedVal = 0;
		int sensor =  randomSensor(engine);//1 + (rand()%5);
		//std::cout << "Sensor val: " <<  sensor << std::endl;
		bool isNormalised = false;
		for (unsigned int i = 0; i < robots.size(); i++){
			for (unsigned int j = 0; j < robots[i]->getSensors().size(); j++){
				if (boost::dynamic_pointer_cast<SensorElement>(robots[i]-> getSensors()[j])){
				if ( 	boost::dynamic_pointer_cast< SensorElement>(robots[i]-> getSensors()[j])-> getType() == sensor){

						double newValue = boost::dynamic_pointer_cast< SensorElement>(robots[i]-> getSensors()[j])-> getValue() + delta;
						if (!isNormalised){
							normalisedVal = normaliseValue(newValue, minValue_, maxValue_);
							isNormalised = true;
						}

						boost::dynamic_pointer_cast< SensorElement>(robots[i]-> getSensors()[j])-> setValue(normalisedVal);
					}
				}
			}
		}
		//std::cout << "New sensor "<< sensor <<" state value: " << normalisedVal << std::endl;
	}
	void MutatePerturbSensorState::mutateSigmaValue(){
		float dice = float( randint() % 100) / 100.0;

		if ( dice <= EDQD::Parameters::pMutation ){

			dice = float(randint() %100) / 100.0;
			if ( dice < 0.5 ){

				sigma_ = sigma_ * ( 1 + EDQD::Parameters::updateSigmaStep ); // increase sigma

				if (sigma_ > EDQD::Parameters::sigmaMax){
					sigma_ = EDQD::Parameters::sigmaMax;
				}
			}
			else{
				sigma_ = sigma_ * ( 1 - EDQD::Parameters::updateSigmaStep ); // decrease sigma

				if ( sigma_ < EDQD::Parameters::sigmaMin ){
					sigma_ = EDQD::Parameters::sigmaMin;
				}
			}
		}
	}
}
