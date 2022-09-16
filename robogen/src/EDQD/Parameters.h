/*
 * @(#) Parameters.h   1.0   Sep 05, 2021
 *
 * Sindiso Mkhatshwa (mkhsin035@myuct.ac.za)
 *
 * Nitschke Laboratory, UCT
 *
 * @(#) $Id$
 */

#ifndef EDQD_PARAMETERS_H_
#define EDQD_PARAMETERS_H_
#include "Logger.h"
#include <fstream>

extern std::ofstream gLitelogFile;
extern Logger *gLiteLogger;
namespace EDQD{
	//=============================================================================================
	// Embodied Evolution Parameters
	//=============================================================================================

	class Parameters{
	public:
		//=========================================================================================
		// Members
		//=========================================================================================

		/**
		 * Used with gDynamicSigma defined to true
		 */
		static double sigmaMin;

		/**
		 * Maximal value of sigma
		 */
		static double sigmaMax;

		static int nbHiddenLayers; // default: 1
		static int nbNeuronsPerHiddenLayer; // default: 5
		/**
		 * Bounds of brain weights
		 * Default this to 800 (i.e. weights are in [-400,+400] as done in previous work)
		 */
		static int weightRange;
		static int nbOfPhysicalObjectGroups;


		/**
		 * Step used in the decrease or increase of the value of sigma
		 */
		static double updateSigmaStep;

		/**
		 * Step used in the decrease or increase of the value of sigma
		 */
		static double sigmaRef;

		/**
		 * Probability of transmitting the current genome mutated with sigma ref
		 */
		static double pMutation;

		/**
		 * Probability of mutating sensor state
		 */
		static double pMutateSensorState;

		/**
		 * Switch sensor on/off
		 */
		static double sensorState;

		/**
		 * Theoretical duration of a generation (ie. maximum time a controller
		 * will be evaluated on a robot)
		 */
		static unsigned int evaluationTime;

		/**
		 * Maximum number of iterations
		 */
		static unsigned int maxIterations;

		/**
		 * If set to false, a robot will restart its controller as soon as it has no more energy.
		 * If set to true, the robot without energy will wait and reload its controller at the
		 * same time as every other robots.
		 */
		static bool synchronization;

		/**
		 * Defines the number of dimensions of a map
		 * NB: needs to be set at compile time
		 */
		static const int nbOfDimensions = 2;

		/**
		 * Defines the number of intervals per dimension of a map
		 * NB: needs to be set at compile time
		 */
		static const int nbOfIntervals = 10;//2;///*15*/5;

		/**
		 * Used to decide whether to discard or keep an elite in the map
		 */
		static double fitEpsilon;

		/**
		 * Controller type (0: MLP, 1: Perceptron, 2: Elman)
		 */
		static int controllerType;

		/**
		 * Max. nb of genome transmission per individual per lifetime.
		 */
		static int maxNbGenomeTransmission;

		/**
		 * Enable/disable limiting transmission
		 */
		static bool limitGenomeTransmission;

		/**
		 * -1: infinite ; 0: no delay ; >0: delay
		 */
		static int notListeningStateDelay;

		/**
		 * -1: infinite ; 0: no delay ; >0: delay (ignored if gNotListeningStateDelay=-1)
		 */
		static int listeningStateDelay;

		static double individualMutationRate;

		/**
		 * 0: uniform, 1: gaussian/normal distribution
		 */
		static int mutationOperator;

		/**
		 * Value of sigma
		 */
		static double sigma;

		/**
		 * Keep maps for only one generation
		 */
		static bool onlyKeepMapsForGeneration;

		static bool EDQDMapSelection;
		static int selectionMethod;

		/**
		 * Maximum time a robot can stay attached to a resource before eventually
		 * getting forced to drop it off for other robots to pick it up.
		 */
		static int maxTimeResourceBound;

		static std::ofstream gEOGLogFile;
		static Logger* gEOGLogger;

		static std::ofstream gMapsLogFile;
		static Logger* gMapsLogger;

		static std::ofstream gResourcesLogFile;
		static Logger* gResourcesLogger;

		static std::ofstream gMorphLogFile;
		static Logger* gMorphLogger;

		// =========================================================================================
		// Constructors
		// =========================================================================================
		// Default constructor
		Parameters();

		// =========================================================================================
		// Destructor
		// =========================================================================================

		~Parameters();

		//==========================================================================================
		// Methods
		//==========================================================================================

		// Load the parameters from a file
		// returns 0 on success
		int Load(const char* filename);
		// Load the parameters from an already opened file for reading
		int Load(std::ifstream& a_DataFile);

		void Save(const char* filename);
		// Saves the parameters to an already opened file for writing
		void Save(FILE* a_fstream);


		// resets the parameters to built-in defaults
		void Reset();
	};
}



#endif /* EDQD_PARAMETERS_H_ */
