/*
 * @(#) EvolverConfiguration.h 1.0   Sep 2, 2013
 *
 * Titus Cieslewski (dev@titus-c.ch)
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2013-2016 Titus Cieslewski, Joshua Auerbach
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

#ifndef EVOLVERCONFIGURATION_H_
#define EVOLVERCONFIGURATION_H_

#include <string>
#include <vector>
#include <utility>
#include <stdexcept>
#include "evolution/neat/Parameters.h"


namespace robogen {

/**
 * \brief Struct to describe evolution config parameters
 *
 * The choice is made to use a struct as we want to easily write access to new
 * parameters of the configuration class
 */
typedef struct EvolverConfiguration {
	/**
	 * \brief Types of replacement strategies
	 */
	enum ReplacementTypes{
		PLUS_REPLACEMENT, COMMA_REPLACEMENT
	};

	/**
	 * \brief Types of selection strategies
	 */
	enum SelectionTypes{
		DETERMINISTIC_TOURNAMENT
	};

	/**
	 * \brief Modes of evolution
	 */
	enum EvolutionModes {
		BRAIN_EVOLVER, FULL_EVOLVER, EMBODIED
	};

	/**
	 * \brief Evolationary Algorithm
	 * TODO add other possibilities (e.g. CMA-ES)
	 */
	enum EvolutionaryAlgorithms {
		BASIC, HYPER_NEAT, EDQD
	};


	/**
	 * \brief Types of body mutation.
	 * Last entry is used to get the amount of operators
	 */
	enum BodyMutationOperators{
		SUBTREE_REMOVAL, SUBTREE_DUPLICATION, SUBTREE_SWAPPING,
		NODE_INSERTION, NODE_REMOVAL, PARAMETER_MODIFICATION,
		/** \brief ORIENTATION_CHANGE, SENSOR_SWAP, LINK_CHANGE, ACTIVE_PASSIVE,*/
		NUM_BODY_OPERATORS
	};

	static const std::string BodyMutationOperatorsProbabilityCodes[];

	/**
	 * \brief File name of this configuration
	 */
	std::string confFileName;

	/**
	 * \brief Parses a configuration from a proper configuration file
	 */
	bool init(std::string configFileName);

	// GENERAL EVOLUTION PARAMS
	// ========================================================================

	/**
	 * \brief File for reference robot
	 */
	std::string referenceRobotFile;

	/**
	 * \brief Configuration file for simulator
	 */
	std::string simulatorConfFile;

	/**
	 * \brief Population size
	 */
	unsigned int mu;

	/**
	 * \brief Offspring size
	 */
	unsigned int lambda;

	/**
	 * \brief Amount of generations to be simulated
	 */
	unsigned int numGenerations;

	/**
	 * \brief Employed selection strategy
	 */
	int selection;

	/**
	 * \brief Tournament size for tournaments
	 */
	unsigned int tournamentSize;

	/**
	 * \brief Employed replacement strategy
	 */
	int replacement;

	/**
	 * \brief Employed replacement strategy
	 */
	unsigned int evolutionMode;

	/**
	 *  \brief Flag to continue evolving from provided brain instead of re-initialing
	 *  params randomly
	 */
	bool useBrainSeed;

	/**
	 * \brief Sockets to be used to connect to the server
	 */
	std::vector<std::pair<std::string, int> > sockets;

	// BRAIN EVOLUTION PARAMS
	// ========================================================================

	/**
	 * \brief Probability of mutation for any single brain parameter
	 */
	double pBrainMutate;

	/**
	 * \brief Sigma of brain weight mutation
	 */
	double brainWeightSigma;

	/**
	 * \brief Sigma of brain bias mutation
	 */
	double brainBiasSigma;

	/**
	 * \brief Lower bound for brain weights
	 */
	double minBrainWeight;

	/**
	 * \brief Upper bound for brain weights
	 */
	double maxBrainWeight;

	/**
	 * \brief Lower bound for biases
	 */
	double minBrainBias;

	/**
	 * \brief Upper bound for biases
	 */
	double maxBrainBias;

	/**
	 * \brief PARAM FOR OTHER BRAIN PARAMETERS\n
	 * 	Required if using relevant neuron types  TODO -- validate this
	 */

	double brainTauSigma;
	/**
	 * \brief PARAM FOR OTHER BRAIN PARAMETERS\n
	 * 	Required if using relevant neuron types  TODO -- validate this
	 */
	double brainPeriodSigma;
	/**
	 * \brief PARAM FOR OTHER BRAIN PARAMETERS\n
	 * 	Required if using relevant neuron types  TODO -- validate this
	 */
	double brainPhaseOffsetSigma;
	/**
	 * \brief PARAM FOR OTHER BRAIN PARAMETERS\n
	 * 	Required if using relevant neuron types  TODO -- validate this
	 */
	double brainAmplitudeSigma;
	/**
	 * \brief PARAM FOR OTHER BRAIN PARAMETERS\n
	 * 	Required if using relevant neuron types  TODO -- validate this
	 */
	double minBrainTau;
	/**
	 * \brief PARAM FOR OTHER BRAIN PARAMETERS\n
	 * 	Required if using relevant neuron types  TODO -- validate this
	 */
	double maxBrainTau;
	/**
	 * \brief PARAM FOR OTHER BRAIN PARAMETERS\n
	 * 	Required if using relevant neuron types  TODO -- validate this
	 */
	double minBrainPeriod;
	/**
	 * \brief PARAM FOR OTHER BRAIN PARAMETERS\n
	 * 	Required if using relevant neuron types  TODO -- validate this
	 */
	double maxBrainPeriod;

	/**
	 * \brief PARAM FOR OTHER BRAIN PARAMETERS\n
	 * 	Required if using relevant neuron types  TODO -- validate this
	 * 	defaults to -1, 1
	 */
	double minBrainPhaseOffset;
	/**
	 * \brief PARAM FOR OTHER BRAIN PARAMETERS\n
	 * 	Required if using relevant neuron types  TODO -- validate this
	 * 	defaults to -1, 1
	 */
	double maxBrainPhaseOffset;
	/**
	 * \brief PARAM FOR OTHER BRAIN PARAMETERS\n
	 * 	Required if using relevant neuron types  TODO -- validate this
	 * 	defaults to 0, 1
	 */
	double minBrainAmplitude;
	/**
	 * \brief PARAM FOR OTHER BRAIN PARAMETERS\n
	 * 	Required if using relevant neuron types  TODO -- validate this
	 * 	defaults to 0, 1
	 */
	double maxBrainAmplitude;


	/**
	 * \brief Probability of crossover among brains
	 */
	double pBrainCrossover;

	// BODY EVOLUTION PARAMS
	// ========================================================================

	/**
	 * \brief Allowed body part types
	 */
	std::vector<char> allowedBodyPartTypes;

	/**
	 * \brief Probabilities for the various body operators
	 */
	double bodyOperatorProbability[NUM_BODY_OPERATORS];


	/**
	 * \brief Max body mutation attempts per reproduction events\n
	 * 	Mutator will give up trying to mutate the body tree after
	 * 	this many failed attempts
	 */
	unsigned int maxBodyMutationAttempts;

	/**
	 * \brief Maximum number of allowed body parts
	 */
	unsigned int maxBodyParts;

	/**
	 * \brief Minimum number of body parts in individuals in the initial population
	 */
	unsigned int minNumInitialParts;

	/**
	 * \brief Maximum number of body parts in individuals in the initial population
	 */
	unsigned int maxNumInitialParts;

	/**
	 * \brief Std dev of body param mutations
	 */
	double bodyParamSigma;

	/**
	 * \brief Evolutionary algorithm
	 */
	unsigned int evolutionaryAlgorithm;

	/**
	 * \brief NEAT/HyperNEAT parameters
	 */
	NEAT::Parameters neatParams;

	/**
	 * \brief File for NEAT/HyperNEAT parameters
	 */
	std::string neatParamsFile;
	/**
	 * \brief Probability to mutate oscillator neuron
	 */
	double pOscillatorNeuron;
	/**
	 * \brief Probability to add a hidden neuron
	 */
	double pAddHiddenNeuron;

	/**
	 * \brief BK attempt to add noveltysearch optional param
	 */
	bool noveltySearch;

} EvolverConfiguration;

} /* namespace robogen */
#endif /* EVOLVERCONFIGURATION_H_ */
