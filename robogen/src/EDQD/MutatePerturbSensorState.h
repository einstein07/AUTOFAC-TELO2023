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
/**
 * \brief Class that describes the mutation of sensors
 */
class MutatePerturbSensorState{
private:
	/** Sigma value */
	double sigma_;
	/** Minimum sensor value */
	double minValue_;
	/** Maximum sensor value */
	double maxValue_;
public:
	/**
	 * \brief Class constructor
	 * @param double sigma value
	 */
	MutatePerturbSensorState(double sigma);
	/**
	 * \brief Destructor
	 */
	~MutatePerturbSensorState();
	/**
	 * \brief Mutate sigma value
	 */
	void mutateSigmaValue();
	/**
	 * \brief Mutate chosen sensors across robot population
	 * @param std::vector< boost::shared_ptr<Robot> > all robots in the simulation
	 */
	void mutateSensorGroup(std::vector< boost::shared_ptr<Robot> > robots);

};
}
#endif /*EDQD_MUTATEPERTURBSENSORSTATE_H_ */
