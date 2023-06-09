/*
 * @(#) NeuralNetwork.h   1.0   March 5, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 * Joshua Auerbach (joshua.auerbach@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2015 Andrea Maesani, Joshua Auerbach
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
#ifndef ROBOGEN_NEURAL_NETWORK_H_
#define ROBOGEN_NEURAL_NETWORK_H_

#define MAX_INPUT_NEURONS /**20*/80 //SM modified
#define MAX_OUTPUT_NEURONS 8

/**
 * set arbitrarily
 */
#define MAX_HIDDEN_NEURONS 20

/**
 * max is either (bias, tau, gain) or (phase offset, period, gain)
 */
#define MAX_PARAMS 3


/* HEADER_FOOTER_BREAK */


/**
 * No namespace here on purpose ;-)
 */

/**
 * \brief Neuron type
 *
 * Copied from NeuronRepresentation.h
 */
enum neuronType{
		SIMPLE, /* corresponds to inputs */
		SIGMOID,
		CTRNN_SIGMOID,
		OSCILLATOR,
		SUPG
};

/**		\brief Struct to describe Neural Network representation
 *
 *
	 * Given m input neurons and n output neurons
	 * m <= MAX_INPUT_NEURONS
	 * n <= MAX_OUTPUT_NEURONS
	 *
	 * One weight for each input-output connection (w_ij, input neuron i, 0 <= i <= m, output neuron j, 0 <= j <= n
	 * One weight for each output-output connection (wr_ij, output neuron i,j, 0 <= i,j <= n )
	 *
	 * The weights are saved as the concatenation by row of the following:
	 * w_00 w_01 ... w_0n
	 * w_10 w_11 ... w_1n
	 * ...  ...  ... ...
	 * w_m0 w_m1 ... w_mn
	 *
	 * wo_00 wo_01 ... wo_0n
	 * wo_10 wo_11 ... wo_1n
	 * ...  ...  ... ....
	 * wo_n0 wo_n1 ... wo_nn
	 */
typedef struct {

	#ifndef ARDUINO
	float weight[(MAX_INPUT_NEURONS + MAX_OUTPUT_NEURONS + MAX_HIDDEN_NEURONS)
	             * (MAX_OUTPUT_NEURONS + MAX_HIDDEN_NEURONS)];
	#endif
	/**
	 * \brief Params for hidden and output neurons, quantity depends on the type of
	 * neuron
	 */
	#ifndef ARDUINO
	float params[MAX_PARAMS * (MAX_OUTPUT_NEURONS + MAX_HIDDEN_NEURONS)];
	#endif

	/**
	 * \brief One state for each output and hidden neuron
	 * activations will be used to temporarily store summed inputs before updating states
	 */
	float state[(MAX_OUTPUT_NEURONS + MAX_HIDDEN_NEURONS)];
	float activations[(MAX_OUTPUT_NEURONS + MAX_HIDDEN_NEURONS)];

	/**
	 * \brief One input state for each input neuron
	 */
	float input[MAX_INPUT_NEURONS];


	/**
	 * \brief Type of each non-input neuron
	 */
	#ifndef ARDUINO
	unsigned int types[(MAX_OUTPUT_NEURONS + MAX_HIDDEN_NEURONS)];
	#endif
	/**
	 * \brief The number of inputs
	 */
	unsigned int nInputs;

	/**
	 * \brief The number of outputs
	 */
	unsigned int nOutputs;

	/**
	 * \brief The number of hidden units
	 */
	unsigned int nHidden;

	/**
	 * \brief The number of non-inputs (i.e. nOutputs + nHidden)
	 */
	unsigned int nNonInputs;

} NeuralNetwork;

/**
 * \brief Initializes a NeuralNetwork data structure
 * @param network the neural network
 * @param nInputs the number of inputs of the neural network
 * @param nOutputs the number of outputs of the neural network
 * @param weight weights of the neural network. Weights must be provided in the same order as
 *               specified in the NeuralNetwork structure
 * @param bias the bias of each output neuron
 * @param gain the gain of each output neuron
 */
void initNetwork(NeuralNetwork* network, unsigned int nInputs,
		unsigned int nOutputs, unsigned int nHidden,
		const float *weights, const float* params,
		const unsigned int *types);

/**
 * \brief Feed the neural network with input values
 * @param network the neural network
 * @param input the input values, must be an array of m inputs
 */
void feed(NeuralNetwork* network, const float *input);

/**
 * \brief Step the neural network of 1 timestep
 * @param network the neural network
 * @param time, amount of time elapsed since brain turned on
 * 				(needed for oscillators)
 */
void step(NeuralNetwork* network, float time);

/**
 * \brief Read the output of the neural network
 * @param network the neural network
 * @param output the output of the neural network, must point to an area of memory of at least size n
 */
void fetch(const NeuralNetwork* network, float *output);

/**
 * \brief Set the weights of the neural network with new/updated values -SM
 * @param network the neural network
 * @param weights the weight values, must be an array
 */
void setWeights(NeuralNetwork* network, const float *weights);

#endif /* ROBOGEN_NEURAL_NETWORK_H_ */
