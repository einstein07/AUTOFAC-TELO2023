/*
 * @(#) MutatePerturbSensorState.h   1.0   Jun 14, 2022
 *
 * Sindiso Mkhatshwa (mkhsin035@myuct.ac.za)
 *
 * Nitschke Laboratory, UCT
 *
 * @(#) $Id$
 */

#ifndef EDQD_MUTATEPERTURBSENSORSTATE_H_
#define EDQD_MUTATEPERTURBSENSORSTATE_H_
#include "Robot.h"

namespace robogen{
class MutatePerturbSensorState{
private:
	double sigma_;
	double _minValue;
	double _maxValue;
public:
	MutatePerturbSensorState(double sigma);
	~MutatePerturbSensorState();
	void mutateSensorGroup(std::vector< boost::shared_ptr<Robot> > robots);
	double normaliseValue(double newValue);
};
}
#endif /*EDQD_MUTATEPERTURBSENSORSTATE_H_ */
