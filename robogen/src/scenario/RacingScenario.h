/*
 * @(#) RacingScenario.h   1.0   Mar 13, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
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
#ifndef ROBOGEN_RACING_SCENARIO_H_
#define ROBOGEN_RACING_SCENARIO_H_

#include <osg/Vec3>
#include "scenario/Scenario.h"
#include <vector>

namespace robogen {

/**
 * \brief Class to describe a RacingScenario.
 *
 *
 * The robot that can cover the longer distance in the given simulation time wins.
 * The distance is computed as the euclidean distance from the starting to the ending position computed using as
 * reference the core component.
 */
class RacingScenario: public Scenario {

public:

	/**
	 * \brief Initializes a RacingScenario
	 */
	RacingScenario(boost::shared_ptr<RobogenConfig> robogenConfig);

	/**
	 * \brief Destructor
	 */
	virtual ~RacingScenario();

	/**
	 * Method inherited from {@link #Scenario}
	 */
	virtual bool setupSimulation();
	/**
	 * \brief Method inherited from {@link #Scenario}
	 */
	virtual bool afterSimulationStep();
	/**
	 * \brief Method inherited from {@link #Scenario}
	 */
	virtual bool endSimulation();
	/**
	 * \brief Method inherited from {@link #Scenario}
	 */
	virtual double getFitness();
	/**
	 * \brief Method inherited from {@link #Scenario}
	 */
	virtual std::vector<float> getEndPosition();
	/**
	 * \brief Method inherited from {@link #Scenario}
	 */
	virtual bool remainingTrials();
	/**
	 * \brief Method inherited from {@link #Scenario}
	 */
	virtual int getCurTrial() const;

private:

	std::vector<osg::Vec2> startPosition_;
	//added by Brooke
	osg::Vec2 endPosition_;
	std::vector<osg::Vec2> endPositions_;
	std::vector<double> distances_;
	unsigned int curTrial_;

};

}

#endif /* ROBOGEN_RACING_SCENARIO_H_ */
