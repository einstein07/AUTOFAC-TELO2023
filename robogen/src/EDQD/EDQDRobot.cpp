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
		logFilename = "logs/robot_"+ std::to_string(getId()) +"_datalog_" + gStartTime + "_" + getpidAsReadableString() + ".txt";

		robotLogFile.open(logFilename.c_str());


		if(!robotLogFile) {
			std::cout << "[error] Cannot open log file " << std::endl;
			exit (-1);
		}

		logger = new Logger();
		logger->setLoggerFile(robotLogFile);


		/*std::string sLog = std::string("");
		sLog += "***********ROBOT [initialise] "+ getId()+ " initialized successfully.********\n" ;*/


		this->configuration = configuration;

		map_ = new EDQDMap();
		mergedMap_ = new EDQDMap();

		//nbInputs_ = getBrain()->nInputs;
		//nbHidden_ = getBrain()->nHidden;
		//nbOutputs_ = getBrain()->nOutputs;

		//params_ =getBrain()->params; ***compiler complained about this assignment
		//memcpy(params_, getBrain()->params,
		//			sizeof(float) * (nbOutputs_ + nbHidden_) * MAX_PARAMS);
		//types_ = getBrain()->types; ***compiler complained about this assignment
		//memcpy(types_, getBrain()->types,
		//			sizeof(unsigned int) * (nbOutputs_ + nbHidden_));

		currentSigma_ = EDQD::Parameters::sigmaRef;
		minValue_ = -1.0;
		maxValue_ = 1.0;

		motorOutputs[0] = 0;
		motorOutputs[1] = 0;
		// behaviour
		iteration_ = 0;
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
		count_ = 0;
		timeResourceBound_ = 0;

		scenario_ = scenario;
		reset();
		resetFitness();
		// debug
		//fitness_ = 1;
		setAlive(true);
		targetArea_ = osg::Vec2d(
				0,-2.5);
		/**sLog += "Ancestor: " + ancestor_ + "\n"
				"isListening: " + isListening_ + "\n"
				"Starting position: (" + Xinit_+ ", " + Yinit_ +").\n"
				"Fitness: " + getFitness() + "\n"
				"*****************************************************\n";
		logger->write(sLog);
		logger->flush();*/
		/**
		 * Initialize behavioral heuristics
		 */
		//pheuristic_.reset(new PickUpHeuristic(boost::shared_ptr<Robot>(this), targetArea_));
		//cheuristic_.reset(new CollisionAvoidanceHeuristic(boost::shared_ptr<Robot>(this)));
		return status;
	}

	EDQDRobot::~EDQDRobot(){
		//memset(weights_, 0, sizeof(weights_));

		//	delete _map;
		parameters_.clear();
		delete nbNeuronsPerHiddenLayer_;
		delete nn;
		nn = NULL;

	}

	/**
	 * Handles control decision and evolution (but: actual movement
	 * is done in roborobo's main loop)
	 */
	void EDQDRobot::step( double elapsedEvaluationTime ){

		iteration_++;
		count_++;
#ifdef DEBUG_EDQD
		std::cout << "*****Robot ID: " << getId() << " STEPPING EVOLUTION*****" << std::endl;
#endif
	    // step evolution
	    stepEvolution();
#ifdef DEBUG_EDQD
	    std::cout << "*****Robot ID: " << getId() << " DONE STEPPING EVOLUTION*****" << std::endl;
#endif
	    // update fitness
	    if ( isAlive() ){
#ifdef DEBUG_STEP
	    	std::cout << "*****Robot ID: " << getId() << " STEPPING CONTROLLER*****" << std::endl;
#endif
	    	stepController();
#ifdef DEBUG_STEP
	    	std::cout << "*****Robot ID: " << getId() << " DONE STEPPING CONTROLLER*****" << std::endl;
#endif
#ifdef DEBUG_STEP
	    	std::cout << "*****Robot ID: " << getId() << " UPDATING FITNESS*****" << std::endl;
#endif
	        updateFitness();
#ifdef DEBUG_STEP
	        std::cout << "*****Robot ID: " << getId() << " DONE UPDATING FITNESS*****" << std::endl;
#endif
	    }

	    else{
#ifdef DEBUG_STEP
	    	std::cout << "*****Robot ID: " << getId() <<" NOT ALIVE - setting motors to 0 . . .*****" <<std::endl;
#endif
	    	// Set motors to zero (0.00)
			for (unsigned int i = 0; i < getMotors().size(); ++i) {

				if (boost::dynamic_pointer_cast<RotationMotor>(getMotors()[i])) {
					boost::dynamic_pointer_cast<RotationMotor>(getMotors()[i])
							->setDesiredVelocity(
													0.00,
													step_ * configuration->getActuationPeriod()
												);
				}
			}
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
						logger->write(sLog);
						logger->flush();
					}
					else{
						std::string sLog = std::string("");
						sLog += "" + std::to_string(robogen::iterations) + ", " + std::to_string(getId()) +
								"::" + std::to_string(birthdate_) + ", status, inactive\n"; // never listen again.
						logger->write(sLog);
						logger->flush();
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
						logger->write(sLog);
						logger->flush();

						// agent will not be able to be active anymore
						notListeningDelay_ = -1;

						// destroy then create a new NN
						reset();

						setAlive(false);

					}
				}
			}
#ifdef DEBUG_STEP_EV
			std::cout << "*****Robot ID: " << getId() <<" DONE DISABLING ROBOT*****" << std::endl;
#endif
	    }
	    /**std::string sLog = std::string("");
		sLog += "[Step] - done\n"; // never listen again.
		logger->write(sLog);
		logger->flush();*/

	}

	std::vector<double> EDQDRobot::getInputs(){
		std::vector<double> inputs;
		//inputs.reserve(nbInputs_);
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
						read()/ColorSensor::SENSOR_RANGE);

				if (boost::dynamic_pointer_cast< ColorSensorElement>(getSensors()[i+1])) {
					if ( boost::dynamic_pointer_cast< ColorSensorElement>(getSensors()[i+1])->
							read()){
						// This is an agent
						inputs.push_back( 1 );
					}
					else{
						inputs.push_back( 0 );
					}
				}

				if (boost::dynamic_pointer_cast< ColorSensorElement>(getSensors()[i+2])) {
					if ( boost::dynamic_pointer_cast< ColorSensorElement>(getSensors()[i+2])->
							read()){ // This is a resource
						for (int j = 1; j <= EDQD::Parameters::nbOfPhysicalObjectGroups; j++){
							if (boost::dynamic_pointer_cast< ColorSensorElement>(getSensors()[i+3])->
									read() == j){ // check the object type
								inputs.push_back( 1 );
							}
							else{
								inputs.push_back( 0 );
							}
						}

					}
					else{ // This is not a resource, set all resource types to 0
						for (int j = 1; j <= EDQD::Parameters::nbOfPhysicalObjectGroups; j++){
							inputs.push_back( 0 );
						}
					}
				}
				if (boost::dynamic_pointer_cast< ColorSensorElement>(getSensors()[i+3])) {
					if ( boost::dynamic_pointer_cast< ColorSensorElement>(getSensors()[i+3])->
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

		//std::cout << "Inputs size: " << inputs.size() << std::endl;
		return inputs;
	}

	std::vector<double> EDQDRobot::getSensorInputs(){
		std::vector<double> inputs;
		//inputs.reserve(nbInputs_);
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
		std::cout << "Inputs size: " << inputs.size() << std::endl;
		return inputs;
	}
	void EDQDRobot::stepController(){

	    // ---- compute and read out ----

	    nn->setWeights(parameters_); // set-up NN
	//    nn.set_all_weights(_parameters);
	    std::vector<double> inputs = getInputs(); // Build list of inputs (check properties file for extended/non-extended input values

	    nn->setInputs(inputs);

	    nn->step();

	    std::vector<double> outputs = nn->readOut();

	    // std::cout << "[DEBUG] Neural Network :" << nn->toString() << " of size=" << nn->getRequiredNumberOfWeights() << std::endl;

	   motorOutputs[0] = outputs[0];
	   motorOutputs[1] = outputs[1];
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
	//            nn = Mlp<Neuron<PfWSum<>, AfSigmoidNoBias<> >, Connection<> >(
	//            				_nbInputs,
	//							_nbHiddenLayers * EDQDSharedData::gNbNeuronsPerHiddenLayer,
	//							_nbOutputs);
	//            nn.init();
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


	void EDQDRobot::stepEvolution(){
		if( robogen::iterations > 0 && robogen::iterations % EDQD::Parameters::evaluationTime == 0 ){

			/**
			 * lifetime ended: replace genome (if possible)
			 * add needs to happen before new genome is loaded to ensure the map has an entry.
			 * moved to agent observer in previous work - why???
			 */
#ifdef DEBUG_STEP_EV
			std::cout<< "*****Robot ID: "<< getId() << " Adding genome to map*****" << std::endl;
#endif
	    	if (map_->add(getId(), this, currentGenome_, currentSigma_ )){
#ifdef DEBUG_STEP_EV
	    		std::cout<< "*****Robot ID: "<< getId() << " Genome added to map. Loading new genome*****"<< std::endl;
#endif

	    		std::string sLog = std::string("");
				sLog += "" + std::to_string(robogen::iterations) + ", " + std::to_string(getId()) +
						"::" + std::to_string(birthdate_) + ", genome added to map\n";
				logger->write(sLog);
				logger->flush();
	    	}
	    	else{
#ifdef DEBUG_STEP_EV
	    		std::cout<< "*****Robot ID: "<< getId() << " Genome NOT added to map. Loading new genome*****"<< std::endl;
#endif
	    		std::string sLog = std::string("");
				sLog += "" + std::to_string(robogen::iterations) + ", " + std::to_string(getId()) +
						"::" + std::to_string(birthdate_) + ", genome NOT added to map\n";
				logger->write(sLog);
				logger->flush();
	    	}

	    	loadNewGenome();
#ifdef DEBUG_STEP_EV
	    	std::cout<< "*****Robot ID: "<< getId() << " Done loading new genome. Resetting fitness*****" << std::endl;
#endif
	        nbGenomeTransmission_ = 0;
	        resetFitness();
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

	        if ( isAlive() /*&& gRadioNetwork*/ ){
	            broadcastMap();
	        }

	        osg::Vec3 currPos = getCoreComponent()->getRootPosition();
	        int tmpMaxDistance = sqrt(pow(currPos.x() - Xinit_,2) + pow(currPos.y() - Yinit_, 2));
	        dSumTravelled_ += sqrt(pow(currPos.x() - Xlast_,2) + pow(currPos.y() - Ylast_, 2));
	        Xlast_ = currPos.x();
			Ylast_ = currPos.y();
			if ( dMaxTravelled_ < tmpMaxDistance ) {
				dMaxTravelled_ = tmpMaxDistance;
			}
	    }

		// check for new NN parameters
	    if ( getNewGenomeStatus() ){
#ifdef DEBUG_STEP_EV
	    	std::cout<< "*****Robot ID: "<< getId()<< " Mapping genotype to phenotype*****" << std::endl;
#endif
	    	mapGenotypeToPhenotype();
#ifdef DEBUG_STEP_EV
	        std::cout<< "*****Robot ID: "<< getId()	<< " Done mapping genotype to phenotype*****" << std::endl;
#endif
	        setNewGenomeStatus(false);
#ifdef DEBUG_STEP_EV
	        std::cout << "*****Robot ID: " << getId() << " New weights set *****" << std::endl;
#endif
	    }
	}

	void EDQDRobot::mapGenotypeToPhenotype(){
		parameters_.clear();
		parameters_.reserve(currentGenome_.size());
		parameters_ = currentGenome_;

	}

	void EDQDRobot::performVariation(){
		//mutator_->EDQDperformVariation(currentGenome_, currentSigma_);
		// global mutation rate (whether this genome will get any mutation or not) - default: always
			if ( EDQD::Parameters::individualMutationRate > random() ) {
			switch ( EDQD::Parameters::mutationOperator ){
				case 0:
					mutateUniform();
					break;
				case 1:

					/**
					 * self-contained sigma. mutated before use (assume: performVariation is called after selection of
					 * new current genome)
					 * original MEDEA [ppsn2010], as used before year 2015
					 *
					 */
	#ifdef DEBUG_EDQD
					std::cout 	<< "*****Robot ID: " << getId() << " Mutating sigma value *****" << std::endl;
	#endif
					mutateSigmaValue();
	#ifdef DEBUG_EDQD
					std::cout 	<< "*****Robot ID: " << getId() << " Done mutating sigma value. Mutating guassian. . . *****" << std::endl;
	#endif
					mutateGaussian(currentSigma_);
	#ifdef DEBUG_EDQD
					std::cout 	<< "*****Robot ID: " << getId() << " Done mutating guassian*****" << std::endl;
	#endif
					break;
				case 2:

					/**
					 * fixed mutation rate
					 */
					mutateGaussian(EDQD::Parameters::sigma);
					break;
				default:
					std::cerr 	<< "[ERROR] unknown variation method (gMutationOperator = "
								<< EDQD::Parameters::mutationOperator << ")\n";
					exit(-1);
			}
		}
	}


	/**
	 * If called, assume genomeList.size() > 0
	 */
	void EDQDRobot::selectRandomGenomeFromMergedMap(){

	    std::map<int, EDQDMap* >::iterator it = mapList_.begin();
	    while ( it != mapList_.end() ) {
	    	it->second->mergeInto(*mergedMap_);
	    	it ++;
	    }
	    behav_index_t index;
	    int tries = 0;
	    if (mergedMap_->getNumFilledCells() > 0){
			do {
				index = mergedMap_->getRandomIndex();
				if (tries > 99)
					break;
				tries++;
				std::cout << "*****Robot ID: " << getId() << " Selecting random genome from merged map. Map list size: "<<mapList_.size()<< " Num of filled cells: " << mergedMap_->getNumFilledCells()<<" *****" << std::endl;
			} while ( mergedMap_->get(index)->genome_.size() == 0 );
			if (tries < 100){
				currentGenome_ = mergedMap_->get(index)->genome_;
				currentSigma_ = mergedMap_->get(index)->sigma_;
				birthdate_ = robogen::iterations;

				ancestor_ = mergedMap_->get(index)->id_;

				setNewGenomeStatus(true);
			}
	    }
	    //return true;
	}

	/**
	 * Manage storage of a map received from a neighbour
	 * Note that in case of multiple encounters with the same robot (same id, same "birthdate"),
	 * genome is stored only once, and last known fitness value is stored
	 * (i.e. updated at each encounter).
	 * Fitness is optional (default: 0)
	 */
	bool EDQDRobot::storeMap(EDQDMap* map, int senderId){
	    if ( !isListening_ ){
	    	// current agent is not listening: do nothing.
	    	return false;
	    }
	    else{
	        nbComRx_++;
	        std::map<int, EDQDMap* >::const_iterator it = mapList_.find(senderId);
	        /**
	         * This exact agent's map is already stored.
	         * Exact means: same robot, same generation.
	         * Then: update fitness value (the rest in unchanged) - however, fitness is not really updated, why???
	         */
	        if ( it != mapList_.end() ){
	        	/** Sender already stored in map list.*/
	            return false;
	        }
	        else{
	        	mapList_[senderId] = map;
	        	//std::cout << "*" <<std::flush;
	            return true;
	        }
	    }
	}

	/**
	 * mutates within bounds.
	 */
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

	/**
	 * Mutate within bounds.
	 */
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
		nbInputs_ = 64;

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
	//    if ( _currentGenome.size() < nbGene )
	//    {
	//    	_currentGenome.reserve(nbGene);
	//    }

	    // Intialize genome

	    for ( unsigned int i = 0 ; i != nbGene ; i++ )
	    {
	        currentGenome_.push_back((double)(randint()%EDQD::Parameters::weightRange)/(EDQD::Parameters::weightRange/2)-1.0); // weights: random init between -1 and +1
	//        _currentGenome[i] = ((double)(randint()%EDQDSharedData::gNeuronWeightRange)/(EDQDSharedData::gNeuronWeightRange/2)-1.0); // weights: random init between -1 and +1
	    }
	    setNewGenomeStatus(true);
	    clearReservoir(); // will contain the genomes received from other robots
	}

	void EDQDRobot::clearReservoir(){
		//std::cout 	<< "*****Robot ID: " << getId() << " Clear-reservoir: clearing sigmalist. . .*****"
		//																			<< std::endl;
		sigmaList_.clear();
		//std::cout 	<< "*****Robot ID: " << getId() << " Clear-reservoir: clearing fitness values list. . .*****"
		//																			<< std::endl;
		fitnessValuesList_.clear();

		// always clear _mapList, as results have been merged.
		//std::cout 	<< "*****Robot ID: " << getId() << " Clear-reservoir: clearing map list. . .*****"
		//																			<< std::endl;
		mapList_.clear();
		//std::cout 	<< "*****Robot ID: " << getId() << " Clear-reservoir: done clearing map list.*****"
		//																			<< std::endl;

		if ( EDQD::Parameters::onlyKeepMapsForGeneration||robogen::iterations == 0 ){
				//std::cout 	<< "*****Robot ID: " << getId() << " Clear-reservoir: deleting. . .*****"
				//															<< std::endl;
				delete mergedMap_;
				//std::cout 	<< "*****Robot ID: " << getId() << " Clear-reservoir: deleted.*****"
				//																			<< std::endl;
				mergedMap_ = new EDQDMap();
				//std::cout 	<< "*****Robot ID: " << getId() << " Clear-reservoir: leaving. . .*****"
				//																			<< std::endl;
		}
	}

	void EDQDRobot::reset(){
		initController();
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
#ifdef DEBUG_EDQD
	    	std::cout 	<< "*****Robot ID: " << getId() << " Broadcasting map *****" << std::endl;
#endif
	    	for ( unsigned int i = 0 ; i < getSensors().size(); ++i){

				ColorSensorElement::Type colorType;
				if (boost::dynamic_pointer_cast< ColorSensorElement>(getSensors()[i])) {
					if (boost::dynamic_pointer_cast< ColorSensorElement>(getSensors()[i])->read()){
						colorType = boost::dynamic_pointer_cast< ColorSensorElement>(getSensors()[i])->
																	getType();

						// sensor ray bumped into a robot : communication is possible
						if ( colorType == ColorSensorElement::Type::ROBOT ){

							int targetIndex = boost::dynamic_pointer_cast<ColorSensorElement>(getSensors()[i])
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
											//std::cout << "Map broadcasted successfully!" << std::endl;
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
#ifdef DEBUG_EDQD
	    	std::cout << "*****Robot ID: " << getId() << " Done broadcasting map *****"	<< std::endl;
#endif
		}
	}

	/**
	 * Called only if at least 1 genome was stored.
	 */
	void EDQDRobot::performSelection(){
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
			logger->write(sLog);
			logger->flush();
			//return true;
		//}
		// Logging: track descendance
		/**std::string sLog = std::string("");
		sLog += "[Perform selection - could not be performed]\n";
		logger->write(sLog);
		logger->flush();
		return false;*/
	}

	void EDQDRobot::loadNewGenome(){
#ifdef DEBUG_STEP_EV
		std::cout << "*****Robot ID: " << getId() << " EMBODIED-LOAD NEW GENOME: BEGIN*****" << std::endl;
#endif
		if (isAlive()){
			logCurrentState();
	        if ( mapList_.size() > 0 ){
#ifdef DEBUG_STEP_EV
	        	std::cout << "******Robot ID: " << getId() << "Map list size: "<< mapList_.size()<<". Peforming selection. . . *****" << std::endl;
#endif
				performSelection();
#ifdef DEBUG_STEP_EV
				std::cout 	<< "*****Robot ID: " << getId() << "EMBODIED-LOAD NEW GENOME: Done performing selection."
							<< " Performing variation. . .*****" << std::endl;
#endif
				performVariation();
#ifdef DEBUG_STEP_EV
				std::cout 	<< "*****Robot ID: " << getId() << " EMBODIED-LOAD NEW GENOME:  Done performing variation. Clearing reservoir. . .*****"
							<< std::endl;
#endif
				clearReservoir();
#ifdef DEBUG_STEP_EV
				std::cout << "*****Robot ID: " << getId() << " EMBODIED-LOAD NEW GENOME: Done clearing reservoir*****" << std::endl;
#endif
				setAlive(true);

				std::string sLog = std::string("");
				sLog += "" + std::to_string(robogen::iterations) + ", " + std::to_string(getId()) + "::" + std::to_string(birthdate_) + ", status, active\n";
				logger->write(sLog);
				logger->flush();

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

					/*
					 // write genome (takes a lot of disk space)
					 for(unsigned int i=0; i<_genome.size(); i++)
					 {
					 sLog += std::to_string(_genome[i]) + ",";
					 //gLogFile << std::fixed << std::showpoint << _wm->_genome[i] << " ";
					 }
					 */
					sLog += "(...)"; // do not write genome

					sLog += "\n";
					logger->write(sLog);
					logger->flush();
				}
	        }
	        else{
	        	if ( isAlive() == true){
#ifdef DEBUG_STEP_EV
	        		std::cout << "*****Robot ID: " << getId() << " EMBODIED-LOAD NEW GENOME: Map list empty.*****" << std::endl;
#endif
	        		// Logging: "no genome"
					std::string sLog = std::string("");
					sLog += "" + std::to_string(robogen::iterations) + "," + std::to_string(getId()) +
							"::" + std::to_string(birthdate_) + ", genome, n/a.\n";
					logger->write(sLog);
					logger->flush();

	        		reset();

	        		setAlive(false);

	        		// ie. -1 (infinite,dead) or >0 (temporary,mute)
	        		if ( EDQD::Parameters::listeningStateDelay != 0 ){

						isListening_ = false;

						notListeningDelay_ = EDQD::Parameters::notListeningStateDelay;
						listeningDelay_ = EDQD::Parameters::listeningStateDelay;

						std::string sLog = std::string("");
						sLog += "" + std::to_string(robogen::iterations) + ", " + std::to_string(getId()) + "::" + std::to_string(birthdate_) + ", status, inactive\n";
						logger->write(sLog);
						logger->flush();
					}
	        		else{

	        			listeningDelay_ = EDQD::Parameters::listeningStateDelay;

						if (listeningDelay_ > 0 || listeningDelay_ == -1 ){
							isListening_ = true;
							std::string sLog = std::string("");
							sLog += "" + std::to_string(robogen::iterations) + ", " + std::to_string(getId()) + "::" + std::to_string(birthdate_) + ", status, listening\n";
							logger->write(sLog);
							logger->flush();
						}
						else{
							std::string sLog = std::string("");
							sLog += "" + std::to_string(robogen::iterations) + ", " + std::to_string(getId()) + "::" + std::to_string(birthdate_) + ", status, inactive\n";
							logger->write(sLog);
							logger->flush();
						}

					}
	        	}
	        }
	    }
	}

	void EDQDRobot::logCurrentState(){
	    // Logging
	    std::string sLog = "Iteration: " + std::to_string(robogen::iterations) + ", Robot Id: " + std::to_string(getId()) + ", Birthdate: " + std::to_string(birthdate_) +
	    ", age, " + std::to_string(robogen::iterations-birthdate_) +

	    ", rxMapListSize, " + std::to_string(mapList_.size()) +
	    ", rxGenomesListSize ," + std::to_string(genomesList_.size()) +
	    ", sigma, " + std::to_string(currentSigma_) +
	    ", x_init, " + std::to_string(Xinit_) +
	    ", y_init, " + std::to_string(Yinit_) +
	    ", x_current, " + std::to_string( getCoreComponent()->getRootPosition().x()) +
	    ", y_current, " + std::to_string( getCoreComponent()->getRootPosition().y()) +
	    ", dist, " + std::to_string( getEuclideanDistance( Xinit_, Yinit_, getCoreComponent()->getRootPosition().x(),getCoreComponent()->getRootPosition().y() ) ) +
	    ", maxDist, " + std::to_string( dMaxTravelled_ ) +
	    ", fitnessValue, " + std::to_string(getFitness()) +
	    "\n";
	    logger->write(sLog);
	    logger->flush();
	}


	void EDQDRobot::writeEOGEntry(Logger* lm){
		/*
		 * generation,iteration,robot,birthday,ancestorId,ancestorBirthday,age,energy,sigma,genomes,fitness,cellsInMap,distMax,distPotMax,distTotal,comRx,comTx,g0,g1
	    }*/
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
	//	    lm->flush();
	}

	/**
	 * @return fitness The fitness of the robot
	 */
	double EDQDRobot::getFitness(){
		return fitness_;
	}

	/*
	 * note: resetFitness is first called by the Controller's constructor.
	 */
	void EDQDRobot::resetFitness()
	{
		fitness_ = 0;
		resetResourceCounter();
	}

	/**
	 * Updates the fitness of the robot based on the number of resources that have been collected.
	 */
	void EDQDRobot::updateFitness(){
		double sum = 0.00;
		std::map<int, int>::iterator it = resourceCounters_.begin();
		for (; it != resourceCounters_.end(); ++it)
		{
			// The value of each resource is 100/number-of-pushing-robots
			/**
			 * Changed this to just 100 - since all resources are pushed by one robot now
			 * JUst different colors
			 */
			sum += 100 * it->second;// / it->first;
		}
		fitness_ = sum;
	}

	int EDQDRobot::updateCellId() {
		behav_index_t index = EDQDMap::computeIndex(resourceCounters_, dMaxTravelled_, dPotMaxTravelled_);
		currentCellId_ = index[0] * EDQD::Parameters::nbOfIntervals  + index[1];

		return currentCellId_;
	}

}
