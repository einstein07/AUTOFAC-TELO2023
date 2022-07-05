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
	MutatePerturbSensorState::MutatePerturbSensorState(double sigma):sigma_(sigma), _minValue(0.0), _maxValue(1.0){srand((unsigned) time(NULL));}
	MutatePerturbSensorState::~MutatePerturbSensorState(){}

	void MutatePerturbSensorState::mutateSensorGroup(std::vector< boost::shared_ptr<Robot> > robots){

		double delta = randgaussian() * sigma_;
		double normalisedVal = 0;
		int sensor = 1 + (rand()%5);
		bool isNormalised = false;
		for (unsigned int i = 0; i < robots.size(); i++){
			for (unsigned int j = 0; j < robots[i]->getSensors().size(); j++){
				if (boost::dynamic_pointer_cast<SensorElement>(robots[i]-> getSensors()[j])){
				if ( 	boost::dynamic_pointer_cast< SensorElement>(robots[i]-> getSensors()[j])-> getType() == sensor){

						double newValue = boost::dynamic_pointer_cast< SensorElement>(robots[i]-> getSensors()[j])-> getValue() + delta;
						if (!isNormalised){
							normalisedVal = normaliseValue(newValue);
							isNormalised = true;
						}

						boost::dynamic_pointer_cast< SensorElement>(robots[i]-> getSensors()[j])-> setValue(normalisedVal);
					}
				}
			}
		}
		std::cout << "New sensor "<< sensor <<" state value: " << normalisedVal << std::endl;
	}

	double MutatePerturbSensorState::normaliseValue(double newValue){
		double value = newValue;
		// bouncing upper/lower bounds
		if ( value < _minValue )
		{
			double range = _maxValue - _minValue;
			double overflow = - ( (double)value - _minValue );
			overflow = overflow - 2*range * (int)( overflow / (2*range) );
			if ( overflow < range )
				value = _minValue + overflow;
			else // overflow btw range and range*2
				value = _minValue + range - (overflow-range);
		}
		else if ( value > _maxValue )
		{
			double range = _maxValue - _minValue;
			double overflow = (double)value - _maxValue;
			overflow = overflow - 2*range * (int)( overflow / (2*range) );
			if ( overflow < range )
				value = _maxValue - overflow;
			else // overflow btw range and range*2
				value = _maxValue - range + (overflow-range);
		}
		return value;
	}
}
