/*
 * @(#) Map.h   1.0   Sep 02, 2021
 *
 * Sindiso Mkhatshwa (mkhsin035@myuct.ac.za)
 *
 * Nitschke Laboratory, UCT
 *
 * @(#) $Id$
 */
#include "EDQDRobot.h"
#include <EDQD/contrib/neuralnetworks/MLP.h>
#include <EDQD/contrib/neuralnetworks/Perceptron.h>
#include <EDQD/contrib/neuralnetworks/Elman.h>
//#define DEBUG_EDQD
//#define DEBUG_STEP_EV
//#define DEBUG_EDQD_LOAD_GEN
namespace robogen{

	//==========================================================================================
	// EDQDRobot class implementation
	//==========================================================================================
	EDQDRobot::EDQDRobot(){
		Robot();
	}
	bool EDQDRobot::initialise(
						dWorldID odeWorld,
						dSpaceID odeSpace, dSpaceID robotSpace, int& nbALive,
						boost::shared_ptr<RobogenConfig> configuration,
						const robogenMessage::Robot& robotSpec, boost::shared_ptr<Scenario> scenario){

		bool status = this->init(odeWorld, odeSpace, robotSpace, nbALive, robotSpec);

		isNull = true;

		resourceLogFilename = gLogDirectoryname + "/resources/robot_"+ std::to_string(getId()) +"_resourcelog_" + gStartTime + "_" + getpidAsReadableString() + ".csv";
		resourceLogFile.open(resourceLogFilename.c_str());
		if(!resourceLogFile) {
			std::cout << "[error] Cannot open log file " << std::endl;
			exit (-1);
		}
		resourceLogger = new Logger();
		resourceLogger->setLoggerFile(resourceLogFile);
		resourceLogger -> write("generation, A, B, C, D, E, Total-Collected");
		resourceLogger -> write("\n");
		resourceLogger -> flush();

		if (EDQD::Parameters::evolveSensors || EDQD::Parameters::EDQDMultiBCMap){
			sensorLogFilename = gLogDirectoryname + "/sensors/robot_"+ std::to_string(getId()) +"_sensorlog_" + gStartTime + "_" + getpidAsReadableString() + ".csv";
			sensorLogFile.open(sensorLogFilename.c_str());
			if(!sensorLogFile) {
				std::cout << "[error] Cannot open log file " << std::endl;
				exit (-1);
			}
			sensorLogger = new Logger();
			sensorLogger -> setLoggerFile(sensorLogFile);
			sensorLogger -> write("generation, sensor-type, range, isActive");
			sensorLogger -> write("\n");
			sensorLogger -> flush();
		}

		this->configuration = configuration;

		map_ = new EDQDMap();
		mergedMap_ = new EDQDMap();

		if (EDQD::Parameters::EDQDMultiBCMap){
			morphMap_ = new EDQDMap();
			morphMergedMap_ = new EDQDMap();

			sensorMinValue_ = -1.0;
			sensorMaxValue_ = 1.0;

		}

		currentSigma_ = EDQD::Parameters::sigmaRef;
		minValue_ = -1.0;
		maxValue_ = 1.0;

		motorOutputs = osg::Vec2d(0, 0);
		// behaviour
		birthdate_ = 0;

		ancestor_ = std::make_pair(getId(), getBirthdate());

		isListening_ = true;
		notListeningDelay_ = EDQD::Parameters::notListeningStateDelay;
		listeningDelay_ = EDQD::Parameters::listeningStateDelay;

		nbGenomeTransmission_ = 0;

		dSumTravelled_ = 0.0;
		dMaxTravelled_ = 0.0;
		dPotMaxTravelled_ = 0.0;
		nbComTx_ = 0;
		nbComRx_ = 0;
		currentCellId_ = 0;
		step_ = configuration->getTimeStepLength();
		timeResourceBound_ = 0;

		scenario_ = scenario;
		reset();
		resetFitness();


		setAlive(true);
		targetArea_ = osg::Vec2d(
				0,-2.5);
		return status;
	}

	EDQDRobot::~EDQDRobot(){

		//	delete _map;
		parameters_.clear();
		delete nbNeuronsPerHiddenLayer_;
		delete nn;
		nn = NULL;

	}

	void EDQDRobot::step(boost::mutex& queueMutex){

		// step evolution
	    stepEvolution(queueMutex);
	    // update fitness
	    if ( isAlive() ){
	    	stepController();
	    	//updateFitness();
			if (iterations == EDQD::Parameters::maxIterations-1){
				writeMapToFile(map_-> getMap(), gLogDirectoryname + "/robot-maps/behavior/robot"+ boost::lexical_cast<std::string> ( getId()) + gStartTime + "_" + getpidAsReadableString()  + std::string(".csv"),
									"");
				if (EDQD::Parameters::EDQDMultiBCMap)
					writeMapToFile(morphMap_-> getMap(), gLogDirectoryname + "/robot-maps/morph/robot"+ boost::lexical_cast<std::string> ( getId()) + gStartTime + "_" + getpidAsReadableString()  + std::string(".csv"),
									"");
			}

	    }
	    else{
			assert ( notListeningDelay_ >= -1 ); // -1 means infinity
			if ( notListeningDelay_ > 0 ){
				notListeningDelay_--;
				if ( notListeningDelay_ == 0 ){
					listeningDelay_ = EDQD::Parameters::listeningStateDelay;
					if ( listeningDelay_ > 0 || listeningDelay_ == -1 ){
						isListening_ = true;
						std::string sLog = std::string("");
						sLog += "" + std::to_string(robogen::iterations) + "," + std::to_string(getId()) +
								"::" + std::to_string(birthdate_) + ",status,listening\n";
					}
					else{
						std::string sLog = std::string("");
						sLog += "" + std::to_string(robogen::iterations) + ", " + std::to_string(getId()) +
								"::" + std::to_string(birthdate_) + ", status, inactive\n"; // never listen again.
					}
				}
			}
			else{
				if ( notListeningDelay_ != -1 && listeningDelay_ > 0 ){
					assert ( isListening_ == true );
					listeningDelay_--;
					if ( listeningDelay_ == 0 ){
						isListening_ = false;

						std::string sLog = std::string("");
						sLog += "" + std::to_string(robogen::iterations) + ", " + std::to_string(getId()) +
								"::" + std::to_string(birthdate_) + ", status, inactive\n"; // never listen again.

						// agent will not be able to be active anymore
						notListeningDelay_ = -1;

						// destroy then create a new NN
						reset();

						setAlive(false);

					}
				}
			}
		}

	}

	std::vector<double> EDQDRobot::getSensorInputs(){
		std::vector<double> inputs;
		double floor_sensor = 0;

		/**
		 * 1: distance/max_range
		 * 2: agent
		 * 3: resource type 1
		 * 4: resource type 2
		 * 5: resource type 3
		 * 6: resource type 4
		 * 7: resource type 5
		 * 8: wall
		 * 9: target area
		 */
		for (unsigned int  i = 0; i < getSensors().size(); ++i){

			if (boost::dynamic_pointer_cast< IrSensorElement>(getSensors()[i])) {

				inputs.push_back(boost::dynamic_pointer_cast< IrSensorElement>(getSensors()[i])->
						read()/SensorMorphology::SENSOR_RANGE);

				if (boost::dynamic_pointer_cast< SensorElement>(getSensors()[i + SensorElement::ROBOT])) {
					if ( boost::dynamic_pointer_cast< SensorElement>(getSensors()[i + SensorElement::ROBOT])->
							read()){
						// This is an agent
						inputs.push_back( 1 );
					}
					else{
						inputs.push_back( 0 );
					}
				}

				if (boost::dynamic_pointer_cast<SensorElement>(getSensors()[i + SensorElement::RESOURCET1])) {
					if(boost::dynamic_pointer_cast< SensorElement>(getSensors()[i + SensorElement::RESOURCET1])->
							isActive()){
						if ( boost::dynamic_pointer_cast< SensorElement>(getSensors()[i + SensorElement::RESOURCET1])->
								read()){ // This is a type 1 resource
							inputs.push_back( 1 );
						}
						else{ // This is not a type 1 resource
							inputs.push_back( 0 );
						}
					}
					else{ // not active
						inputs.push_back( 0 );
					}

				}
				if (boost::dynamic_pointer_cast<SensorElement>(getSensors()[i + SensorElement::RESOURCET2])) {
					if(boost::dynamic_pointer_cast< SensorElement>(getSensors()[i + SensorElement::RESOURCET2])->
							isActive()){
						if ( boost::dynamic_pointer_cast< SensorElement>(getSensors()[i + SensorElement::RESOURCET2])->
								read()){ // This is a type 2 resource
							inputs.push_back( 1 );
						}
						else{ // This is not a type 2 resource
							inputs.push_back( 0 );
						}
					}
					else{ // not active
						inputs.push_back( 0 );
					}

				}
				if (boost::dynamic_pointer_cast<SensorElement>(getSensors()[i + SensorElement::RESOURCET3])) {
					if(boost::dynamic_pointer_cast< SensorElement>(getSensors()[i + SensorElement::RESOURCET3])->
							isActive()){
						if ( boost::dynamic_pointer_cast< SensorElement>(getSensors()[i + SensorElement::RESOURCET3])->
								read()){ // This is a type 3 resource
							inputs.push_back( 1 );
						}
						else{ // This is not a type 3 resource
							inputs.push_back( 0 );
						}
					}
					else{ // not active
							inputs.push_back( 0 );
						}

				}
				if (boost::dynamic_pointer_cast<SensorElement>(getSensors()[i + SensorElement::RESOURCET4])) {
					if(boost::dynamic_pointer_cast< SensorElement>(getSensors()[i + SensorElement::RESOURCET4])->
							isActive()){
						if ( boost::dynamic_pointer_cast< SensorElement>(getSensors()[i + SensorElement::RESOURCET4])->
								read()){ // This is a type 4 resource
							inputs.push_back( 1 );
						}
						else{ // This is not a type 4 resource
							inputs.push_back( 0 );
						}
					}
					else{ // not active
						inputs.push_back( 0 );
					}

				}
				if (boost::dynamic_pointer_cast<SensorElement>(getSensors()[i + SensorElement::RESOURCET5])) {
					if(boost::dynamic_pointer_cast< SensorElement>(getSensors()[i + SensorElement::RESOURCET5])->
							isActive()){
						if ( boost::dynamic_pointer_cast< SensorElement>(getSensors()[i + SensorElement::RESOURCET5])->
								read()){ // This is a type 5 resource
							inputs.push_back( 1 );
						}
						else{ // This is not a type 5 resource
							inputs.push_back( 0 );
						}
					}
					else{ // not active
						inputs.push_back( 0 );
					}

				}
				if (boost::dynamic_pointer_cast< SensorElement>(getSensors()[ i + SensorElement::WALL])) {
					if ( boost::dynamic_pointer_cast< SensorElement>(getSensors()[i + SensorElement::WALL])->
							read()){ // This is a wall
						inputs.push_back( 1 );
					}
					else{
						inputs.push_back( 0 );
					}
				}

			}
			else if (boost::dynamic_pointer_cast< TargetAreaDetectorElement >(getSensors()[i])){
				if ( boost::dynamic_pointer_cast< TargetAreaDetectorElement >(getSensors()[i])->
											read()){
					floor_sensor = 1;
				}
			}
		}
		inputs.push_back(floor_sensor);
		return inputs;
	}

	void EDQDRobot::stepController(){

	    // ---- compute and read out ----

	    nn->setWeights(parameters_); // set-up NN
	    std::vector<double> inputs = getSensorInputs(); // Build list of inputs (check properties file for extended/non-extended input values
	    nn->setInputs(inputs);

	    nn->step();
	    motorOutputs.set(nn->readOut()[0], nn->readOut()[1]);
	   }

	void EDQDRobot::createNN(){

	    setIOcontrollerSize(); // compute #inputs and #outputs
	    if ( !isNull ) // useless: delete will anyway check if nn is NULL or not.
	    	delete nn;
	    isNull = false;
	    switch ( EDQD::Parameters::controllerType )
	    {
	        case 0:
	        {
	            // MLP
	            nn = new Neural::MLP(parameters_, nbInputs_, nbOutputs_, *(nbNeuronsPerHiddenLayer_));
	            break;
	        }
	        case 1:
	        {
	            // PERCEPTRON
	            nn = new Neural::Perceptron(parameters_, nbInputs_, nbOutputs_);
	            break;
	        }
	        case 2:
	        {
	            // ELMAN
	            nn = new Neural::Elman(parameters_, nbInputs_, nbOutputs_, *(nbNeuronsPerHiddenLayer_));
	            break;
	        }
	        default: // default: no controller
	            std::cerr << "[ERROR] gController type unknown (value: " << EDQD::Parameters::controllerType << ").\n";
	            exit(-1);
	    };
	}


	unsigned int EDQDRobot::computeRequiredNumberOfWeights(){
	    unsigned int res = nn->getRequiredNumberOfWeights();
	    return res;
	}


	void EDQDRobot::stepEvolution(boost::mutex& queueMutex){
		{
			boost::lock_guard<boost::mutex> lock(queueMutex);
			if( robogen::iterations > 0 && robogen::iterations % EDQD::Parameters::evaluationTime == 0 ){

				/**
				 * lifetime ended: replace genome (if possible)
				 * add needs to happen before new genome is loaded to ensure the map has an entry.
				 * moved to agent observer in previous work - why???
				 */
				if (map_->add(getId(), this, currentGenome_, currentSigma_ )){
					std::string sLog = std::string("");
					sLog += "" + std::to_string(robogen::iterations) + ", " + std::to_string(getId()) +
							"::" + std::to_string(birthdate_) + ", genome added to behavior map\n";
					//logger->write(sLog);
					//logger->flush();
				}
				else{
					std::string sLog = std::string("");
					sLog += "" + std::to_string(robogen::iterations) + ", " + std::to_string(getId()) +
							"::" + std::to_string(birthdate_) + ", genome NOT added to map\n";
					//logger->write(sLog);
					//logger->flush();
				}
				if (EDQD::Parameters::EDQDMultiBCMap){
					if (morphMap_ -> morphAdd( getId(), this, currentGenome_, currentSigma_, perSensorTypeRange_ ) ){
						std::string sLog = std::string("");
						sLog += "" + std::to_string(robogen::iterations) + ", " + std::to_string(getId()) +
								"::" + std::to_string(birthdate_) + ", genome added to morphology map\n";
						//logger->write(sLog);
						//logger->flush();
					}
					else{
						std::string sLog = std::string("");
						sLog += "" + std::to_string(robogen::iterations) + ", " + std::to_string(getId()) +
								"::" + std::to_string(birthdate_) + ", genome NOT added to map\n";
						//logger->write(sLog);
						//logger->flush();
					}
				}

				loadNewGenome();
				nbGenomeTransmission_ = 0;

				resetFitness();
				/**if (EDQD::Parameters::EDQDMultiBCMap){
					resetSensorInfo();
					updateSensorInfo();
				}*/

	#ifdef DEBUG_STEP_EV
				std::cout 	<< "*****Robot ID: "<< getId() << " Done resetting fitness*****" << std::endl;
	#endif
			}
			else{
				/* broadcasting genome : robot broadcasts its genome to all neighbors
				 * (contact-based wrt proximity sensors)
				 * note: no broadcast if last iteration before replacement --
				 * this is enforced as roborobo update method is random-asynchroneous.
				 * This means that robots broadcasting may transmit genomes to robot
				 * already at the next generation depending on the update ordering (should be avoided).
				 */

				// We now exchange maps with all agents within the population
				// That is, environment pressure is not applied.
				//if ( isAlive() /*&& gRadioNetwork*/ ){
				//    broadcastMap();
				//}

				osg::Vec3 currPos = getCoreComponent()->getRootPosition();
				int tmpMaxDistance = sqrt(std::pow(currPos.x() - Xinit_,2) + pow(currPos.y() - Yinit_, 2));
				dSumTravelled_ += sqrt(pow(currPos.x() - Xlast_,2) + pow(currPos.y() - Ylast_, 2));
				Xlast_ = currPos.x();
				Ylast_ = currPos.y();
				if ( dMaxTravelled_ < tmpMaxDistance ) {
					dMaxTravelled_ = tmpMaxDistance;
				}
			}

			// check for new NN parameters
			if ( getNewGenomeStatus() ){
				mapGenotypeToPhenotype();
				mapMorphPhenotype();
				setNewGenomeStatus(false);
			}
		}
	}

	void EDQDRobot::mapGenotypeToPhenotype(){
		parameters_.clear();
		parameters_.reserve(currentGenome_.size());
		parameters_ = currentGenome_;

	}

	void EDQDRobot::performVariation(){
		// global mutation rate (whether this genome will get any mutation or not) - default: always
			//if ( EDQD::Parameters::individualMutationRate > random() ) {
			switch ( EDQD::Parameters::mutationOperator ){
				case 0:
					mutateUniform();
					break;
				case 1:

					/*
					 * self-contained sigma. mutated before use (assume: performVariation is called after selection of
					 * new current genome)
					 * original MEDEA [ppsn2010], as used before year 2015
					 *
					 */
					mutateSigmaValue();
					mutateGaussian(currentSigma_);
					break;
				case 2:

					/*
					 * fixed mutation rate
					 */
					mutateGaussian(EDQD::Parameters::sigma);
					break;
				default:
					std::cerr 	<< "[ERROR] unknown variation method (gMutationOperator = "
								<< EDQD::Parameters::mutationOperator << ")\n";
					exit(-1);
			}
//		}
	}

	/*
	 * If called, assume genomeList.size() > 0
	 */
	void EDQDRobot::selectRandomGenomeFromMergedMap(){

	    std::map<int, EDQDMap* >::iterator it = mapList_.begin();
	    while ( it != mapList_.end() ) {
	    	it->second->mergeInto(*mergedMap_);
	    	it ++;
	    }
	    behav_index_t index;
	    //Useful when environment pressure is active - i.e. size of map list depends on how much an agent moved around in the environment.
	    int tries = 0;
	    if (mergedMap_->getNumFilledCells() > 0){
			do {
				index = mergedMap_->getRandomIndex();
				if (tries > 99)
					break;
				tries++;
			} while ( mergedMap_->get(index)->genome_.size() == 0 );
			if (tries < 100){
				currentGenome_ = mergedMap_->get(index)->genome_;
				currentSigma_ = mergedMap_->get(index)->sigma_;
				birthdate_ = robogen::iterations;

				ancestor_ = mergedMap_->get(index)->id_;

				setNewGenomeStatus(true);
			}
	    }
	}

	/*
	 * If called, assume genomeList.size() > 0
	 */
	void EDQDRobot::selectRandomGenomeFromMorphMergedMap(){

		std::map<int, EDQDMap* >::iterator it = morphMapList_.begin();
		while ( it != morphMapList_.end() ) {
			it->second->mergeInto(*morphMergedMap_);
			it ++;
		}
		behav_index_t index;
		//Useful when environment pressure is active - i.e. size of map list depends on how much an agent moved around in the environment.
		int tries = 0;
		if (morphMergedMap_->getNumFilledCells() > 0){
			do {
				index = morphMergedMap_->getRandomIndex();
				if (tries > 99)
					break;
				tries++;
			} while ( morphMergedMap_->get(index)->genome_.size() == 0 );
			if (tries < 100){
				morphBirthdate_ = robogen::iterations;
				morphAncestor_ = morphMergedMap_->get(index)->id_;
				perSensorTypeRange_ = morphMergedMap_->get(index)-> perSensorTypeRange_;
				setNewGenomeStatus(true);
			}
		}
	}

	/*
	 * if called, assume genomeList.size()>0
	*/
	void EDQDRobot::selectRandomGenome(){

	    int randomIndex = randint() % genomesList_.size();

	    std::map<std::pair<int,int>, std::vector<double> >::iterator it = genomesList_.begin();
	    while (randomIndex !=0 ){
	        it ++;
	        randomIndex --;
	    }

	    currentGenome_ = (*it).second;
	    currentSigma_ = sigmaList_[(*it).first];
	    birthdate_ = robogen::iterations;

	    ancestor_ = (*it).first;

	    setNewGenomeStatus(true);
	}

	/*
	 * used for debugging
	 * if called, assume genomeList.size()>0
	 */
	void EDQDRobot::selectFirstGenome(){
	    currentGenome_ = (*genomesList_.begin()).second;
	    currentSigma_ = sigmaList_[(*genomesList_.begin()).first];
	    birthdate_ = robogen::iterations;

	    ancestor_ = (*genomesList_.begin()).first;

	    setNewGenomeStatus(true);
	}

	void EDQDRobot::selectBestGenome(){
	    std::pair<int,int> bestId;

	    std::map<std::pair<int,int>, float >::iterator fitnessesIt = fitnessValuesList_.begin();

	    float bestFitnessValue = (*fitnessesIt).second;
	    bestId = (*fitnessesIt).first;

	    ++fitnessesIt;

	    int nbSimilar = 0;

	    for ( int i = 1 ; fitnessesIt != fitnessValuesList_.end(); ++fitnessesIt, i++)
	    {
	        if ( (*fitnessesIt).second >= bestFitnessValue )
	        {
	            if ( (*fitnessesIt).second > bestFitnessValue )
	            {
	                bestFitnessValue = (*fitnessesIt).second;
	                bestId = (*fitnessesIt).first;
	                nbSimilar = 0;
	            }
	            else
	            {
	                nbSimilar++;
	            }
	        }
	    }

	    if ( nbSimilar > 0 ) // >1 genomes have the same fitness best value. Pick randomly among them
	    {
	        int count = 0;
	        int randomPick = randint() % ( nbSimilar + 1 );

	        if ( randomPick != 0 ) // not already stored (i.e. not the first one)
	        {
	            fitnessesIt = fitnessValuesList_.begin();
	            for ( int i = 0 ; ; ++fitnessesIt, i++)
	            {
	                if ( (*fitnessesIt).second == bestFitnessValue )
	                {
	                    if ( count == randomPick )
	                    {
	                        bestId = (*fitnessesIt).first;
	                        break;
	                    }
	                    count++;
	                }
	            }
	        }
	    }

	    birthdate_ = robogen::iterations;

	    currentGenome_ = genomesList_[bestId];
	    currentSigma_ = sigmaList_[bestId];

	    ancestor_ = bestId;

	    setNewGenomeStatus(true);
	}

	void EDQDRobot::selectFitProp()
	{
	    std::pair<int,int> selectId;

	    // compute sum of fitness

	    float sumOfFit = 0;

	    std::map<std::pair<int,int>, float >::iterator fitnessesIt = fitnessValuesList_.begin();

	    for ( ; fitnessesIt != fitnessValuesList_.end(); ++fitnessesIt )
	    {
	        sumOfFit += (*fitnessesIt).second;
	    }

	    // randomly draw a value in [0,sum_of_fitness] -- assume maximisation

	    float fitnessTarget = random()*sumOfFit;

	    // find the parent

	    float currentSum = 0;

	    fitnessesIt = fitnessValuesList_.begin();

	    for ( ; fitnessesIt != fitnessValuesList_.end(); ++fitnessesIt )
	    {
	        currentSum += (*fitnessesIt).second;
	        if ( currentSum >= fitnessTarget )
	        {
	            selectId = (*fitnessesIt).first;
	            break;
	        }
	    }

	    // update current genome with selected parent (mutation will be done elsewhere)

	    birthdate_ = robogen::iterations;

	    currentGenome_ = genomesList_[selectId];
	    currentSigma_ = sigmaList_[selectId];

	    ancestor_ = selectId;

	    setNewGenomeStatus(true);
	}

	bool EDQDRobot::storeMap(EDQDMap* map, int senderId){
	    if ( !isListening_ ){
	    	// current agent is not listening: do nothing.
	    	return false;
	    }
	    else{
	        nbComRx_++;
	        std::map<int, EDQDMap* >::const_iterator it = mapList_.find(senderId);
	        /*
	         * This exact agent's map is already stored.
	         * Exact means: same robot, same generation.
	         * Then: update fitness value (the rest in unchanged) - however, fitness is not really updated, why???
	         */
	        if ( it != mapList_.end() ){
	        	/*Sender already stored in map list.*/
	            return false;
	        }
	        else{
	        	mapList_[senderId] = map;
	            return true;
	        }
	    }
	}

	bool EDQDRobot::storeMaps(EDQDMap* map, EDQDMap* morphMap, int senderId){
		if ( !isListening_ ){
			// current agent is not listening: do nothing.
			return false;
		}
		else{
			nbComRx_++;
			std::map<int, EDQDMap* >::const_iterator it = mapList_.find(senderId);
			/*
			 * This exact agent's map is already stored.
			 * Exact means: same robot, same generation.
			 * Then: update fitness value (the rest in unchanged) - however, fitness is not really updated, why???
			 */
			if ( it != mapList_.end() ){
				/*Sender already stored in map list.*/
				return false;
			}
			else{
				mapList_[senderId] = map;
				morphMapList_[senderId] = morphMap;
				return true;
			}
		}
	}

	bool EDQDRobot::storeGenome(std::vector<double> genome, std::pair<int,int> senderId, float sigma, float fitness, EDQDMap* map){ // fitness is optional (default: 0)

	    if ( !isListening_ )
	    {
	        return false; // current agent is not listening: do nothing.
	    }
	    else
	    {
	    	nbComRx_++;

	        std::map<std::pair<int,int>, std::vector<double> >::const_iterator it = genomesList_.find(senderId);

	        if ( it != genomesList_.end() ) // this exact agent's genome is already stored. Exact means: same robot, same generation. Then: update fitness value (the rest in unchanged)
	        {
	            fitnessValuesList_[senderId] = fitness; // update with most recent fitness (IMPLEMENTATION CHOICE) [!n]
	            return false;
	        }
	        else
	        {
	        	genomesList_[senderId].clear();
	            genomesList_[senderId] = genome;
	            sigmaList_[senderId] = sigma;
	            fitnessValuesList_[senderId] = fitness;

	            mapList_[senderId.first] = map;
	            return true;
	        }
	    }
	}

	void EDQDRobot::mutateGaussian(float sigma){

	    currentSigma_ = sigma;

	    for (unsigned int i = 0 ; i < currentGenome_.size() ; ++i ){

	        double value = currentGenome_[i] + randgaussian() * currentSigma_;
	        // bouncing upper/lower bounds
	        if ( value < minValue_ ){

	            double range = maxValue_ - minValue_;
	            double overflow = - ( (double)value - minValue_ );
	            overflow = overflow - 2*range * (int)( overflow / (2*range) );

	            if ( overflow < range ){
	                value = minValue_ + overflow;
	            }
	            else{
	            	// overflow btw range and range*2
	                value = minValue_ + range - (overflow-range);
	            }
	        }
	        else if ( value > maxValue_ ){

	            double range = maxValue_ - minValue_;
	            double overflow = (double)value - maxValue_;
	            overflow = overflow - 2*range * (int)( overflow / (2*range) );
	            if ( overflow < range ){
	                value = maxValue_ - overflow;
	            }
	            else{
	            	// overflow between range and range * 2
	                value = maxValue_ - range + (overflow-range);
	            }
	        }

	        currentGenome_[i] = value;
	    }

	}

	void EDQDRobot::mutateUniform(){

	    for (unsigned int i = 0 ; i != currentGenome_.size() ; i++ ){
	        float randomValue = float(randint()%100) / 100.0; // in [0,1]
	        double range = maxValue_ - minValue_;
	        double value = randomValue * range + minValue_;

	        currentGenome_[i] = value;
	    }
	}


	void EDQDRobot::setIOcontrollerSize(){
		// wrt inputs

		//TODO: Calculate this dynamically
		nbInputs_ = 33;

		// wrt outputs

		nbOutputs_ = 2;
	}

	void EDQDRobot::initController(){
	    nbHiddenLayers_ = EDQD::Parameters::nbHiddenLayers;
	    nbNeuronsPerHiddenLayer_ = new std::vector<unsigned int>(nbHiddenLayers_);
	    for(unsigned int i = 0; i < nbHiddenLayers_; i++)
	        (*nbNeuronsPerHiddenLayer_)[i] = EDQD::Parameters::nbNeuronsPerHiddenLayer;
	    createNN();
	    unsigned int const nbGene = computeRequiredNumberOfWeights();
	    currentGenome_.clear();


	    // Intialize genome

	    for ( unsigned int i = 0 ; i != nbGene ; i++ )
	    {
	        currentGenome_.push_back((double)(randint()%EDQD::Parameters::weightRange)/(EDQD::Parameters::weightRange/2)-1.0); // weights: random init between -1 and +1
	    }
	    setNewGenomeStatus(true);
	    clearReservoir(); // will contain the genomes received from other robots
	}

	void EDQDRobot::mapMorphPhenotype(){
		for (unsigned int j = 0; j < getSensors().size(); j++){
			if ( boost::dynamic_pointer_cast<SensorElement>(getSensors()[j]) ){
				if ( boost::dynamic_pointer_cast< SensorElement>(getSensors()[j])-> getType() == SensorElement::RESOURCET1 ){
					boost::dynamic_pointer_cast< SensorElement>(getSensors()[j])-> updateSensorRange(perSensorTypeRange_.at(SensorElement::RESOURCET1));
				}
				else if ( boost::dynamic_pointer_cast< SensorElement>(getSensors()[j])-> getType() == SensorElement::RESOURCET2 ){
					boost::dynamic_pointer_cast< SensorElement>(getSensors()[j])-> updateSensorRange(perSensorTypeRange_.at(SensorElement::RESOURCET2));
				}
				else if ( boost::dynamic_pointer_cast< SensorElement>(getSensors()[j])-> getType() == SensorElement::RESOURCET3 ){
					boost::dynamic_pointer_cast< SensorElement>(getSensors()[j])-> updateSensorRange(perSensorTypeRange_.at(SensorElement::RESOURCET3));
				}
				else if ( boost::dynamic_pointer_cast< SensorElement>(getSensors()[j])-> getType() == SensorElement::RESOURCET4 ){
					boost::dynamic_pointer_cast< SensorElement>(getSensors()[j])-> updateSensorRange(perSensorTypeRange_.at(SensorElement::RESOURCET4));
				}
				else if ( boost::dynamic_pointer_cast< SensorElement>(getSensors()[j])-> getType() == SensorElement::RESOURCET5 ){
					boost::dynamic_pointer_cast< SensorElement>(getSensors()[j])-> updateSensorRange(perSensorTypeRange_.at(SensorElement::RESOURCET5));
				}
			}
		}
		updateSensorInfo();
	}

	void EDQDRobot::clearReservoir(){
		sigmaList_.clear();
		fitnessValuesList_.clear();
		genomesList_.clear();
		// always clear _mapList, as results have been merged.
		mapList_.clear();
		if(EDQD::Parameters::EDQDMultiBCMap){
			morphMapList_.clear();
		}
		if ( EDQD::Parameters::onlyKeepMapsForGeneration||robogen::iterations == 0 ){
				delete mergedMap_;
				mergedMap_ = new EDQDMap();
				if(EDQD::Parameters::EDQDMultiBCMap){
					delete morphMergedMap_;
					morphMergedMap_ = new EDQDMap();
				}
		}

	}

	void EDQDRobot::reset(){
		initController();
		updateSensorInfo();

	}

	void EDQDRobot::mutateSigmaValue(){
	    float dice = float( randint() % 100) / 100.0;

	    if ( dice <= EDQD::Parameters::pMutation ){

	        dice = float(randint() %100) / 100.0;
	        if ( dice < 0.5 ){

	            currentSigma_ = currentSigma_ * ( 1 + EDQD::Parameters::updateSigmaStep ); // increase sigma

	            if (currentSigma_ > EDQD::Parameters::sigmaMax){
	                currentSigma_ = EDQD::Parameters::sigmaMax;
	            }
	        }
	        else{
	            currentSigma_ = currentSigma_ * ( 1 - EDQD::Parameters::updateSigmaStep ); // decrease sigma

	            if ( currentSigma_ < EDQD::Parameters::sigmaMin ){
	                currentSigma_ = EDQD::Parameters::sigmaMin;
	            }
	        }
	    }
	}


	void EDQDRobot::broadcastMap(){
	    // TODO: From previous work: limiting genome transmission is sensitive to sensor order. (but: assume ok)

	    if ( isAlive()
	    	&&( EDQD::Parameters::limitGenomeTransmission == false
			|| ( EDQD::Parameters::limitGenomeTransmission == true
			&& nbGenomeTransmission_ < EDQD::Parameters::maxNbGenomeTransmission ))){

	    	for ( unsigned int i = 0 ; i < getSensors().size(); ++i){

				ColorSensorElement::Type colorType;
				if (boost::dynamic_pointer_cast< IrSensorElement>(getSensors()[i])) {
					if (boost::dynamic_pointer_cast< SensorElement>(getSensors()[i + SensorElement::ROBOT])->read()){

					// sensor ray bumped into a robot : communication is possible

					int targetIndex = boost::dynamic_pointer_cast<SensorElement>(getSensors()[i + SensorElement::ROBOT])
											->getObjectId(); // convert image registering index into robot id.

					if ( targetIndex >= 0 && targetIndex < scenario_ -> getRobots().size()){
						if (boost::dynamic_pointer_cast<EDQDRobot>(scenario_->getRobot(targetIndex))){
							if ( boost::dynamic_pointer_cast<EDQDRobot>(scenario_->getRobot(targetIndex))->isListening() ){

								bool success = false;
								// other agent stores a genome from my map
								success = boost::dynamic_pointer_cast<EDQDRobot>(scenario_->getRobot(targetIndex))->storeMap(
																map_,
																getId()
																);
								if ( success == true ){
									// count unique transmissions (ie. nb of different genomes stored).
									nbGenomeTransmission_++;
								}
								nbComTx_++;
							}
						}
						else{
							std::cerr 	<< "Error from robot " << getId() << " : the observer of robot " << targetIndex
										<< " is not compatible." << std::endl;
						}
					}
					}
				}
	    	}
		}
	}


	void EDQDRobot::performSelection(){
		if (EDQD::Parameters::EDQDMapSelection) {

			// Include Local Map For Selection Merge
			mapList_[getId()] = map_;
			//if (selectRandomGenomeFromMergedMap()){
			selectRandomGenomeFromMergedMap();
			mergedMap_->mergeInto(*map_);
			// Logging: track descendance
			std::string sLog = std::string("");
			sLog += "" + std::to_string(robogen::iterations) + ", " + std::to_string(getId()) +
					"::" + std::to_string(birthdate_) + ", descendsFrom, " + std::to_string(ancestor_.first) +
					"::" + std::to_string(ancestor_.second) + "\n";

			if (EDQD::Parameters::EDQDMultiBCMap){
				// Include Local Map For Selection Merge
				morphMapList_[getId()] = morphMap_;
				//if (selectRandomGenomeFromMergedMap()){
				selectRandomGenomeFromMorphMergedMap();
				morphMergedMap_->mergeInto(*morphMap_);
				// Logging: track descendance
				std::string sLog = std::string("");
				sLog += "" + std::to_string(robogen::iterations) + ", " + std::to_string(getId()) +
						"::" + std::to_string(birthdate_) + ", descendsFrom, " + std::to_string(ancestor_.first) +
						"::" + std::to_string(ancestor_.second) + "\n";
			}
		}
		else{
			switch ( EDQD::Parameters::selectionMethod ){
				case 0:
					selectRandomGenome();
					break;
				case 1:
					selectFirstGenome();
					break;
				case 2:
					selectBestGenome();
					break;
				case 3:
					selectFitProp();
					break;
				default:
					std::cerr << "[ERROR] unknown selection method (gSelectionMethod = " << EDQD::Parameters::selectionMethod << ")\n";
					exit(-1);
			}
		}

	}

	void EDQDRobot::loadNewGenome(){
		if (isAlive()){
			logCurrentState();
	        if ( mapList_.size() > 0 ){

	        	performSelection();
	        	performVariation();
	        	if (EDQD::Parameters::EDQDMultiBCMap){
					float dice = float(randint()%100) / 100.0;
					if ( dice <= EDQD::Parameters::pMutateSensorState ){
						mutateSensors();
					}

				}
	        	clearReservoir();
	        	setAlive(true);

				std::string sLog = std::string("");
				sLog += "" + std::to_string(robogen::iterations) + ", " + std::to_string(getId()) + "::" + std::to_string(birthdate_) + ", status, active\n";

	            osg::Vec3 currPos = getCoreComponent()->getRootPosition();
				Xinit_ = currPos.x();
				Yinit_ = currPos.y();
	            Xlast_ = Xinit_;
				Ylast_ = Yinit_;
				dSumTravelled_ = 0.0;
				dMaxTravelled_ = 0.0;
				nbComTx_ = 0;
				nbComRx_ = 0;
				currentCellId_ = 0;

				if ( isAlive() ){
					// Logging: full genome
					std::string sLog = std::string("");
					sLog += "" + std::to_string(robogen::iterations) + ", " + std::to_string(getId()) +
							"::" + std::to_string(birthdate_) + ", genome, ";

					sLog += "(...)"; // do not write genome

					sLog += "\n";
				}
	        }
	        else{
	        	if ( isAlive() == true){
	        		// Logging: "no genome"
					std::string sLog = std::string("");
					sLog += "" + std::to_string(robogen::iterations) + "," + std::to_string(getId()) +
							"::" + std::to_string(birthdate_) + ", genome, n/a.\n";

	        		reset();

	        		setAlive(false);

	        		// ie. -1 (infinite,dead) or >0 (temporary,mute)
	        		if ( EDQD::Parameters::notListeningStateDelay != 0 ){

						isListening_ = false;

						notListeningDelay_ = EDQD::Parameters::notListeningStateDelay;
						listeningDelay_ = EDQD::Parameters::listeningStateDelay;

						std::string sLog = std::string("");
						sLog += "" + std::to_string(robogen::iterations) + ", " + std::to_string(getId()) + "::" + std::to_string(birthdate_) + ", status, inactive\n";

					}
	        		else{

	        			listeningDelay_ = EDQD::Parameters::listeningStateDelay;

						if (listeningDelay_ > 0 || listeningDelay_ == -1 ){
							isListening_ = true;
							std::string sLog = std::string("");
							sLog += "" + std::to_string(robogen::iterations) + ", " + std::to_string(getId()) + "::" + std::to_string(birthdate_) + ", status, listening\n";

						}
						else{
							std::string sLog = std::string("");
							sLog += "" + std::to_string(robogen::iterations) + ", " + std::to_string(getId()) + "::" + std::to_string(birthdate_) + ", status, inactive\n";

						}

					}
	        	}
	        }
	    }
	}

	void EDQDRobot::logCurrentState(){
	    // Logging
		int generation = iterations / EDQD::Parameters::evaluationTime;

	    if (EDQD::Parameters::EDQDMultiBCMap || EDQD::Parameters::evolveSensors) {

			// "generation, s-type, range, isActive"
			for(unsigned int c = 0; c < getSensors().size(); c++ ){
				// output
				if (boost::dynamic_pointer_cast<SensorElement>( getSensors()[c])){
					std::string ofs =
							std::to_string(generation) + ","
						+ 	std::to_string( boost::dynamic_pointer_cast< SensorElement>( getSensors()[c])-> getType() ) + ",";
					if(EDQD::Parameters::EDQDMultiBCMap)
						ofs +=	std::to_string( boost::dynamic_pointer_cast< SensorElement>( getSensors()[c])-> getSensorRange() ) + ",";
					else
						ofs +=	std::to_string( boost::dynamic_pointer_cast< SensorElement>( getSensors()[c])-> getValue() ) + ",";
					ofs +=	std::to_string( boost::dynamic_pointer_cast< SensorElement>( getSensors()[c])-> isActive() );
					ofs += "\n";
					sensorLogger->write(std::string(ofs));
					ofs.clear();
					ofs = "";
				}
			}

			sensorLogger->flush();

		}

	    // generation, A, B, C, D, E, Total-Collected
	    int total = 0;
	    std::string ofs = std::to_string(generation);
		for (int i = 1; i <= EDQD::Parameters::nbOfPhysicalObjectGroups; i++) {
			ofs += "," + std::to_string(resourceCounters_[i]);
			total += resourceCounters_[i];
		}
		ofs += "," + std::to_string(total);
		ofs += "\n";
		resourceLogger->write(std::string(ofs));
		resourceLogger->flush();

	}


	void EDQDRobot::writeEOGEntry(Logger* lm){
		/*
		 * generation,iteration,robot,birthday,ancestorId,ancestorBirthday,age,energy,sigma,genomes,fitness,cellsInMap,distMax,distPotMax,distTotal,comRx,comTx,g0,g1
	    */
		std::string sLog = "" + std::to_string(iterations/EDQD::Parameters::evaluationTime)
				+ "," + std::to_string(iterations)
				+ "," + std::to_string(getId())
				+ "," + std::to_string(birthdate_)
				+ "," + std::to_string(ancestor_.first)
				+ "," + std::to_string(ancestor_.second)
				+ "," + std::to_string(currentCellId_)
		    	+ "," + std::to_string(iterations - birthdate_)
		    	+ "," + std::to_string(currentSigma_)
				+ "," + std::to_string(mapList_.size())
				+ "," + std::to_string(genomesList_.size())
		    	+ "," + std::to_string(fitness_)
		    	+ "," + std::to_string(map_->getNumFilledCells())
				+ "," + std::to_string(dMaxTravelled_)
				+ "," + std::to_string(dPotMaxTravelled_)
		    	+ "," + std::to_string(dSumTravelled_)
		    	+ "," + std::to_string(nbComRx_)
		    	+ "," + std::to_string(nbComTx_);

		for (int i = 1; i <= EDQD::Parameters::nbOfPhysicalObjectGroups; i++) {
		    sLog += "," + std::to_string(resourceCounters_[i]);
		}

		sLog += "\n";

		lm->write(sLog);
	}


	double EDQDRobot::getFitness(){
		return fitness_;
	}

	void EDQDRobot::resetFitness()
	{

			fitness_ = 0;
			resetResourceCounter();

	}

	void EDQDRobot::resetPotMaxTravelled() {
		osg::Vec3 currPos = getCoreComponent()->getRootPosition();
		Xinit_ = currPos.x();
		Yinit_ = currPos.y();
		Xlast_ = Xinit_;
		Ylast_ = Yinit_;
		dPotMaxTravelled_ =
				distance(osg::Vec3d(scenario_ ->getEnvironment() -> getTerrain() -> getWidth()/2.0, scenario_ ->getEnvironment() -> getTerrain() -> getWidth()/2.0, 0), osg::Vec3d(Xinit_, Yinit_, 0)) +
				(scenario_ ->getEnvironment() -> getTerrain() -> getWidth()/2); // crude way to get average radius if ellipse is not a circle
	}

	void EDQDRobot::updateFitness(osg::Vec3d dropOffPosition, int pushingRobots, double value){
		double totalDistPossible = (scenario_ ->getEnvironment() -> getTerrain() -> getWidth() - ((scenario_ ->getEnvironment() -> getGatheringZone() -> getSize().y()/2) - 0.5));
		double distResourceMoved = distance(osg::Vec3d(pickUpPosition.x(), dropOffPosition.y(), pickUpPosition.z()), pickUpPosition);
		double fitValue = ((100*value/pushingRobots) * (distResourceMoved/totalDistPossible));
		fitness_ += fitValue;
	}

	void EDQDRobot::updateFitness(int pushingRobots, double value){
		fitness_ += (100*value/pushingRobots) * (1/10);
	}

	int EDQDRobot::updateCellId() {
		behav_index_t index = EDQDMap::computeIndex(resourceCounters_, dMaxTravelled_, dPotMaxTravelled_);
		currentCellId_ = index[0] * EDQD::Parameters::nbOfIntervals  + index[1];

		return currentCellId_;
	}

	/*****************************************************************************************************************
	 * MORPHO-EVO
	 ****************************************************************************************************************/
	void EDQDRobot::mutateSensors(){
		int sensor =  randomSensor(engine);
		double newValue = (((double)20/(double)3))*(perSensorTypeRange_[sensor] - (randgaussian() * currentSigma_));
		perSensorTypeRange_[sensor] = normaliseValue(newValue, sensorMinValue_, sensorMaxValue_);
	}

	void EDQDRobot::updateSensorInfo(){
		resetSensorInfo();
		for (unsigned int i = 0; i < getSensors().size(); ++i){
			if ( boost::dynamic_pointer_cast<SensorElement>(getSensors()[i]) ){
				if ( boost::dynamic_pointer_cast< SensorElement>(getSensors()[i])-> getType() == SensorElement::RESOURCET1 ){
					possibleTotalNumberOfSensors_++;
					perSensorTypeRange_[SensorElement::RESOURCET1] = boost::dynamic_pointer_cast< SensorElement>(getSensors()[i]) -> getSensorRange();
					perSensorTypeMaxRange_[SensorElement::RESOURCET1] = boost::dynamic_pointer_cast< SensorElement>(getSensors()[i]) -> getMaxSensorRange();
					if (  boost::dynamic_pointer_cast< SensorElement>(getSensors()[i])->isActive() ){
						numOfActiveSensorsPerType_[SensorElement::RESOURCET1]++;
						if (isSensorTypeActive_[SensorElement::RESOURCET1] == false){
							isSensorTypeActive_[SensorElement::RESOURCET1] = true;
						}
					}
				}
				else if ( boost::dynamic_pointer_cast< SensorElement>(getSensors()[i])-> getType() == SensorElement::RESOURCET2 ){
					possibleTotalNumberOfSensors_++;
					perSensorTypeMaxRange_[SensorElement::RESOURCET2] = boost::dynamic_pointer_cast< SensorElement>(getSensors()[i]) -> getMaxSensorRange();
					perSensorTypeRange_[SensorElement::RESOURCET2] = boost::dynamic_pointer_cast< SensorElement>(getSensors()[i]) -> getSensorRange();
					if ( boost::dynamic_pointer_cast< SensorElement>(getSensors()[i])->isActive() ){
						numOfActiveSensorsPerType_[SensorElement::RESOURCET2]++;
						if (isSensorTypeActive_[SensorElement::RESOURCET2] == false){
							isSensorTypeActive_[SensorElement::RESOURCET2] = true;
						}
					}
				}
				else if ( boost::dynamic_pointer_cast< SensorElement>(getSensors()[i])-> getType() == SensorElement::RESOURCET3 ){
					possibleTotalNumberOfSensors_++;
					perSensorTypeMaxRange_[SensorElement::RESOURCET3] = boost::dynamic_pointer_cast< SensorElement>(getSensors()[i]) -> getMaxSensorRange();
					perSensorTypeRange_[SensorElement::RESOURCET3] = boost::dynamic_pointer_cast< SensorElement>(getSensors()[i]) -> getSensorRange();
					if ( boost::dynamic_pointer_cast< SensorElement>(getSensors()[i])->isActive() ){
						numOfActiveSensorsPerType_[SensorElement::RESOURCET3]++;
						if (isSensorTypeActive_[SensorElement::RESOURCET3] == false){
							isSensorTypeActive_[SensorElement::RESOURCET3] = true;
						}
					}
				}
				else if ( boost::dynamic_pointer_cast< SensorElement>(getSensors()[i])-> getType() == SensorElement::RESOURCET4 ){
					possibleTotalNumberOfSensors_++;
					perSensorTypeMaxRange_[SensorElement::RESOURCET4] = boost::dynamic_pointer_cast< SensorElement>(getSensors()[i]) -> getMaxSensorRange();
					perSensorTypeRange_[SensorElement::RESOURCET4] = boost::dynamic_pointer_cast< SensorElement>(getSensors()[i]) -> getSensorRange();
					if (boost::dynamic_pointer_cast< SensorElement>(getSensors()[i])->isActive()){
						numOfActiveSensorsPerType_[SensorElement::RESOURCET4]++;
						if ( isSensorTypeActive_[SensorElement::RESOURCET4] == false ){
							isSensorTypeActive_[SensorElement::RESOURCET4] = true;
						}
					}
				}
				else if ( boost::dynamic_pointer_cast< SensorElement>(getSensors()[i])-> getType() == SensorElement::RESOURCET5 ){
					possibleTotalNumberOfSensors_++;
					perSensorTypeMaxRange_[SensorElement::RESOURCET5] = boost::dynamic_pointer_cast< SensorElement>(getSensors()[i]) -> getMaxSensorRange();
					perSensorTypeRange_[SensorElement::RESOURCET5] = boost::dynamic_pointer_cast< SensorElement>(getSensors()[i]) -> getSensorRange();
					if ( boost::dynamic_pointer_cast< SensorElement>(getSensors()[i])->isActive() ){
						numOfActiveSensorsPerType_[SensorElement::RESOURCET5]++;
						if ( isSensorTypeActive_[SensorElement::RESOURCET5] == false ){
							isSensorTypeActive_[SensorElement::RESOURCET5] = true;
						}
					}
				}
			}
		}
	}

	double EDQDRobot::getAverageActiveSensorRange(){
		double av = 0.0;
		int totalActive = 0;
		for (int i = SensorElement::RESOURCET1; i <= SensorElement::RESOURCET5 ; i++){
			if ( isSensorTypeActive_[i] ){
				av += perSensorTypeRange_[i];
				totalActive++;
			}
		}
		averageActiveSensorRange_ = av/((double)totalActive);
		return averageActiveSensorRange_;
	}

	double EDQDRobot::getMaxActiveSensorRangeAverage(){
		double av = 0.0;
		int totalActive = 0;
		for (int i = SensorElement::RESOURCET1; i <= SensorElement::RESOURCET5 ; i++){
			if ( isSensorTypeActive_[i] ){
				av += perSensorTypeMaxRange_[i];
				totalActive++;
			}
		}
		maxActiveSensorRangeAverage_ = av/((double)totalActive);
		return maxActiveSensorRangeAverage_;
	}

    void EDQDRobot::resetSensorInfo() {
    	possibleTotalNumberOfSensors_ = 0;
    	averageActiveSensorRange_ = 0;
    	maxActiveSensorRangeAverage_ = 0;
		/*
		 * 5 sensor-types right now (sizes 1, 2, 3, 4, 5)
		 */
		for (int i = SensorElement::RESOURCET1; i <= SensorElement::RESOURCET5 ; i++){
			isSensorTypeActive_[i] = false;
			numOfActiveSensorsPerType_[i] = 0;
			perSensorTypeMaxRange_[i] = 0;
			perSensorTypeRange_[i] = 0;
			numOfActiveSensorsPerType_[i] = 0;
		}
	}


}
