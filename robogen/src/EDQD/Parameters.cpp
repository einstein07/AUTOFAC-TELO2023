/*
 * @(#) Parameters.cpp   1.0   Sep 05, 2021
 *
 * Sindiso Mkhatshwa (mkhsin035@myuct.ac.za)
 *
 * Nitschke Laboratory, UCT
 *
 * @(#) $Id$
 */

#include "EDQD/Parameters.h"
std::ofstream gLitelogFile;
Logger *gLiteLogger = NULL;
namespace EDQD{

	//============================================================================================
	// Parameters class implementation
	//============================================================================================
	double Parameters::sigmaMin = 0.001; // From EDQD-GECCO2018
	// maximal value of sigma
	double Parameters::sigmaMax = 0.5; // From EDQD-GECCO2018
	int Parameters::nbHiddenLayers = 1;
	int Parameters::nbNeuronsPerHiddenLayer = 20;
	int Parameters::weightRange = 800; // From EDQD-GECCO2018 [-400, +400]
	int Parameters::nbOfPhysicalObjectGroups = 5; // 5 resource types in this work
	double Parameters::pMutation = 0.34; // From EDQD-GECCO2018

	double Parameters::pMutateSensorState = 0.34;

	double Parameters::updateSigmaStep = 0.35; // From EDQD-GECCO2018
	// reference value of sigma
	double Parameters::sigmaRef = 0.1; // From EDQD-GECCO2018
	// how long a controller will be evaluated on a robot
	unsigned int Parameters::evaluationTime = 10000;//3200;//800; // From EDQD-GECCO2018*/
	// default to value used in previous work
	unsigned int Parameters::maxIterations = 1000000;//804000;// 40000; // From EDQD-GECCO2018*/

	bool Parameters::synchronization = true;

	//const int EDQDSharedData::gEDQDNbOfDimensions = 2; // commented out as needs to be set at compile time. only use header declaration.

	//const int EDQDSharedData::gEDQDNbOfIntervals = 5; // commented out as needs to be set at compile time. only use header declaration.

	double Parameters::fitEpsilon = 1.0;
	// default: arbitrarily set to 65535 as in previous work.
	int Parameters::maxNbGenomeTransmission = 65535;
	int Parameters::controllerType = 1;
	// default: do not limit.
	bool Parameters::limitGenomeTransmission = false;
	// -1: infinite ; 0: no delay ; >0: delay
	int Parameters::notListeningStateDelay = 0;
	// -1: infinite ; 0: no delay ; >0: delay (ignored if gNotListeningStateDelay=-1)
	int Parameters::listeningStateDelay = -1;
	double Parameters::individualMutationRate = 1.0;
	// # 0: uniform, 1: gaussian with evolved sigma, 2: gaussian with fixed sigma
	int Parameters::mutationOperator = 1;

	// 0.01 is just some random value
	double Parameters::sigma = 0.01;

	bool Parameters::onlyKeepMapsForGeneration = true;
	bool Parameters::EDQDMapSelection = true;
	bool Parameters::EDQDMultiBCMap = false;
	int Parameters::selectionMethod = 2;

	int Parameters::maxTimeResourceBound = 8000;//1000;

	std::ofstream Parameters::gEOGLogFile;
	Logger* Parameters::gEOGLogger = NULL;

	std::ofstream Parameters::gMapsLogFile;
	Logger* Parameters::gMapsLogger = NULL;

	std::ofstream Parameters::gMorphMapsLogFile;
	Logger* Parameters::gMorphMapsLogger = NULL;

	std::ofstream Parameters::gResourcesLogFile;
	Logger* Parameters::gResourcesLogger = NULL;

	std::ofstream Parameters::gMorphLogFile;
	Logger* Parameters::gMorphLogger = NULL;

	Parameters::Parameters(){
	}
	Parameters::~Parameters(){}
	int Parameters::Load(std::ifstream& a_DataFile){
	    std::string s,tf;
	    do
	    {
	        a_DataFile >> s;
	    }
	    while (s != "EDQD_ParametersStart");

	    while(s != "EDQD_ParametersEnd")
	    {
	        a_DataFile >> s;

	        if (s == "sigmaMin")
	            a_DataFile >> sigmaMin;

	        if (s == "sigmaMax")
	            a_DataFile >> sigmaMax;

	        if (s == "weightRange")
				a_DataFile >> weightRange;

	        if (s == "updateSigmaStep")
	            a_DataFile >> updateSigmaStep;

	        if (s == "sigmaRef")
				a_DataFile >> sigmaRef;

	        if (s == "pMutation")
				a_DataFile >> pMutation;

	        if (s == "evaluationTime")
				a_DataFile >> evaluationTime;

	        if (s == "synchronization"){
	        	a_DataFile >> tf;
				if (tf == "true" || tf == "1" || tf == "1.0")
					synchronization = true;
				else
					synchronization = false;
	        }

	        /**
	         * Compiler compails about this code
	        if (s == "nbOfDimensions")
				a_DataFile >> nbOfDimensions;

	        if (s == "nbOfIntervals")
				a_DataFile >> nbOfIntervals;*/

	        if (s == "fitEpsilon")
				a_DataFile >> fitEpsilon;

	        if (s == "maxNbGenomeTransmission")
				a_DataFile >> maxNbGenomeTransmission;

	        if (s == "limitGenomeTransmission"){
	        	a_DataFile >> tf;
				if (tf == "true" || tf == "1" || tf == "1.0")
					limitGenomeTransmission = true;
				else
					limitGenomeTransmission = false;
	        }

	        if (s == "notListeningStateDelay")
				a_DataFile >> notListeningStateDelay;

	        if (s == "listeningStateDelay")
				a_DataFile >> listeningStateDelay;

	        if (s == "mutationOperator")
				a_DataFile >> mutationOperator;

	        if (s == "sigma")
				a_DataFile >> sigma;
	    }
	    return 0;
	}

	int Parameters::Load(const char* a_FileName){

	    std::ifstream data(a_FileName);
	    if (!data.is_open())
	        return 0;

	    int result = Load(data);
	    data.close();
	    return result;
	}

	void Parameters::Save(const char* filename){

		FILE* f = fopen(filename, "w");
	    Save(f);
	    fclose(f);
	}

	void Parameters::Save(FILE* a_fstream){

	    fprintf(a_fstream, "EDQD_ParametersStart\n");

	    fprintf(a_fstream, "sigmaMin %3.20f\n", sigmaMin);
	    fprintf(a_fstream, "sigmaMax %3.20f\n", sigmaMax);
	    fprintf(a_fstream, "weightRange %d\n", weightRange);
	    fprintf(a_fstream, "updateSigmaStep %3.20f\n", updateSigmaStep);
	    fprintf(a_fstream, "sigmaRef %3.20f\n", sigmaRef);
	    fprintf(a_fstream, "pMutation %3.20f\n", pMutation);
	    fprintf(a_fstream, "evaluationTime %d\n", evaluationTime);
	    fprintf(a_fstream, "maxIterations %d\n", maxIterations);
	    fprintf(a_fstream, "synchronization %s\n", synchronization==true?"true":"false");
	    fprintf(a_fstream, "nbOfDimensions %d\n", nbOfDimensions);
	    fprintf(a_fstream, "nbOfIntervals %d\n", nbOfIntervals);
	    fprintf(a_fstream, "fitEpsilon %3.20f\n", fitEpsilon);
	    fprintf(a_fstream, "maxNbGenomeTransmission %d\n", maxNbGenomeTransmission);
	    fprintf(a_fstream, "limitGenomeTransmission %s\n", limitGenomeTransmission==true?"true":"false");
	    fprintf(a_fstream, "notListeningStateDelay %d\n", notListeningStateDelay);
	    fprintf(a_fstream, "listeningStateDelay %d\n", listeningStateDelay);
	    fprintf(a_fstream, "mutationOperator %d\n", mutationOperator);
	    fprintf(a_fstream, "sigma %3.20f\n", sigma);

	    fprintf(a_fstream, "EDQD_ParametersEnd\n");
	}

}
