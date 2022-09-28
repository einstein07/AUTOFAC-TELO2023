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
#include "EDQD/Parameters.h"


namespace robogen{
class MutatePerturbSensorState{
private:
	double sigma_;
	double minValue_;
	double maxValue_;
public:
	MutatePerturbSensorState(double sigma);
	~MutatePerturbSensorState();
	void mutateSigmaValue();
	void mutateSensorGroup(std::vector< boost::shared_ptr<Robot> > robots);

};
}
#endif /*EDQD_MUTATEPERTURBSENSORSTATE_H_ */
