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

		this->configuration = configuration;

		map_ = new EDQDMap();
		mergedMap_ = new EDQDMap();

		nbInputs_ = getBrain()->nInputs;
		nbHidden_ = getBrain()->nHidden;
		nbOutputs_ = getBrain()->nOutputs;

		//params_ =getBrain()->params; ***compiler complained about this assignment
		memcpy(params_, getBrain()->params,
					sizeof(float) * (nbOutputs_ + nbHidden_) * MAX_PARAMS);
		//types_ = getBrain()->types; ***compiler complained about this assignment
		memcpy(types_, getBrain()->types,
					sizeof(unsigned int) * (nbOutputs_ + nbHidden_));

		currentSigma_ = EDQD::Parameters::sigmaRef;
		minValue_ = -1.0;
		maxValue_ = 1.0;

		// behaviour
		iteration_ = 0;
		birthdate_ = 0;

		ancestor_ = std::make_pair(getId(), getBirthdate());

		isListening_ = true;
		notListeningDelay_ = EDQD::Parameters::notListeningStateDelay;
		listeningDelay_ = EDQD::Parameters::listeningStateDelay;

		nbGenomeTransmission_ = 0;

		osg::Vec3 currPos = getCoreComponent()->getRootPosition();
		Xinit_ = currPos.x();
		Yinit_ = currPos.y();
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
		targetArea_ = osg::Vec2d(0,-4.25);
		/**
		 * Initialize behavioral heuristics
		 */
		//pheuristic_.reset(new PickUpHeuristic(boost::shared_ptr<Robot>(this), targetArea_));
		//cheuristic_.reset(new CollisionAvoidanceHeuristic(boost::shared_ptr<Robot>(this)));
		return status;
	}

	EDQDRobot::~EDQDRobot(){
		memset(weights_, 0, sizeof(weights_));
		//parameters_.clear();
		//delete _nbNeuronsPerHiddenLayer;
		//delete nn;
		//nn = NULL;

	}

	/**
	 * Handles control decision and evolution (but: actual movement
	 * is done in roborobo's main loop)
	 */
	void EDQDRobot::step(
							//boost::shared_ptr<Environment> env,
							/**boost::shared_ptr<RobogenConfig> configuration,*/
							/**boost::random::mt19937 &rng,*/
							/**int count,*/
							/**double step,*/
							double elapsedEvaluationTime){
		//osg::Vec3d gZone = env->getGatheringZone()->getPosition();

		iteration_++;
		count_++;
		/**std::cout << "*****STEPPING EVOLUTION*****" << std::endl;*/
	    // step evolution
	    stepEvolution();
	    /**std::cout << "*****DONE*****" << std::endl;*/
	    // step controller
	    if ( isAlive() ){
	    	//std::cout << "*****STEPPING CONTROLLER*****" << std::endl;
	    	//stepController(
	    	//				env,
							/**configuration,*/
							/**rng,*/
							/**count,*/
							/**step,*/
			//				elapsedEvaluationTime
	    	//				);
	    	/**std::cout << "*****DONE*****" << std::endl;
	    	std::cout << "*****UPDATING FITNESS*****" << std::endl;*/
	        updateFitness();
	        /**std::cout << "*****DONE*****" << std::endl;*/
	    }

	    else{
	    	//std::cout << "*****NOT ALIVE - setting motors to 0 . . ." <<std::endl;
	    	// Set motors to zero (0.00)
			for (unsigned int i = 0; i < getMotors().size(); ++i) {

				if (boost::dynamic_pointer_cast<RotationMotor>(getMotors()[i])) {
					boost::dynamic_pointer_cast<RotationMotor>(getMotors()[i])
							->setDesiredVelocity(
													0.00,
													step_ * configuration->getActuationPeriod()
												);
				}
				else if (boost::dynamic_pointer_cast<ServoMotor>(getMotors()[i])) {
					boost::dynamic_pointer_cast<ServoMotor>(getMotors()[i])
							->setDesiredPosition(
													0.00,
													step_ * configuration->getActuationPeriod()
												);
				}
			}
			/**std::cout << "*****DONE*****" << std::endl;*/
	    }
	    // updating listening state
	    /**std::cout << "*****UPDATING LISTENING STATE******" << std::endl;*/
	    if ( !isAlive()){
	        assert ( notListeningDelay_ >= -1 ); // -1 means infinity
	        if ( notListeningDelay_ > 0 ){
	            notListeningDelay_--;
	            if ( notListeningDelay_ == 0 ){
	                listeningDelay_ = EDQD::Parameters::listeningStateDelay;
	                if ( listeningDelay_ > 0 || listeningDelay_ == -1 ){
	                    isListening_ = true;
	                }
	            }
	        }
	        else{
	            if ( notListeningDelay_ != -1 && listeningDelay_ > 0 ){
	                assert ( isListening_ == true );
	                listeningDelay_--;
	                if ( listeningDelay_ == 0 ){
	                    isListening_ = false;
	                    // agent will not be able to be active anymore
	                    notListeningDelay_ = -1;
	                    // destroy then create a new NN
	                    //reset();
	                    setAlive(false);
	                }
	            }
	        }
	    }
	    /**std::cout << "*****DONE*****" << std::endl;*/
	}

	void EDQDRobot::stepController(
									boost::shared_ptr<Environment> env,
									/*boost::shared_ptr<RobogenConfig> configuration,*/
									/**boost::random::mt19937 &rng,*/
									/**int count,*/ /**double step,*/ double elapsedEvaluationTime){

		// Set network weights first
		//neuralNetwork_.reset(new NeuralNetwork);
		/**::initNetwork(
						getBrain().get(),
						nbInputs_,
						nbOutputs_,
						nbHidden_,
						&weights_[0],
						&params_[0],
						&types_[0]
					);*/

		float networkInput[MAX_INPUT_NEURONS];
		float networkOutputs[MAX_OUTPUT_NEURONS];

		// Update Sensors
		/**for (unsigned int i = 0; i < getBodyParts().size(); ++i) {
			if (boost::dynamic_pointer_cast<PerceptiveComponent>(getBodyParts()[i])) {
				boost::dynamic_pointer_cast<PerceptiveComponent>(getBodyParts()[i])->
						updateSensors(env);
			}
		}*/


		/**double minDistanceToObject = 0;
		ColorSensorElement::Type colorType;
		IrSensorElement::Type IRtype;
		int objectId;
		for (unsigned int  i = 0; i < getSensors().size(); ++i){
			double distanceToObject = getSensors()[i]->read();
			/**if (distanceToObject < minDistanceToObject){
				minDistanceToObject = distanceToObject;*/
				/**if (boost::dynamic_pointer_cast< ColorSensorElement>(getSensors()[i])) {
					colorType = boost::dynamic_pointer_cast< ColorSensorElement>(getSensors()[i])->
							getType();
					if (colorType == ColorSensorElement::ROBOT){
						std::cout << "Distance to object: " << distanceToObject << ". Type: robot" << std::endl;
					}
					else if (colorType == ColorSensorElement::RESOURCE){
						std::cout << "Distance to object: " << distanceToObject << ". Type: resource" << std::endl;
					}
					else if (colorType == ColorSensorElement::RESOURCESIZE){
						std::cout << "Distance to object: " << distanceToObject << ". Type: resource size" << std::endl;
					}
				}
				else if (boost::dynamic_pointer_cast< IrSensorElement>(getSensors()[i])) {
					IRtype = boost::dynamic_pointer_cast< IrSensorElement>(getSensors()[i])->
							getType();
					if (IRtype == IrSensorElement::IR){
						std::cout << "Distance to object: " << distanceToObject << ". Type: IR Sensor" << std::endl;
					}
				}

			//}
		}*/
		//std::cout << "Stepping heuristic." << std::endl;
		osg::Vec2d signal = osg::Vec2d(-1000, -1000);// cheuristic_->step(env, scenario_);
		//std::cout << "Done." << std::endl;
		if (signal.x() != -1000 || signal.y() != -1000) {
			//std::cout << "Collision Heuristic active." << std::endl;
			networkOutputs[0] = signal.x();
			networkOutputs[1] = signal.y();
		}
		else{
			//signal = pheuristic_->step(env, scenario_);
			if (signal.x() != -1000 || signal.y() != -1000) {
				//std::cout << "Pick up Heuristic active." << std::endl;
				networkOutputs[0] = signal.x();
				networkOutputs[1] = signal.y();
			}
		}
		if(((count_ - 1) % configuration->getActuationPeriod()) == 0) {

			if (signal.x() == -1000 || signal.y() == -1000){

				// Feed neural network
				for (unsigned int i = 0; i < getSensors().size(); ++i) {
					networkInput[i] = getSensors()[i]->read();
					// Add sensor noise: Gaussian with std dev of
					// sensorNoiseLevel * actualValue
					if (configuration->getSensorNoiseLevel() > 0.0) {
						networkInput[i] += (/**normalDistribution(rng)*/randgaussian() *
						configuration->getSensorNoiseLevel() *
						networkInput[i]);
					}
				}

				::feed(getBrain().get(), &networkInput[0]);

				// Step the neural network
				::step(getBrain().get(), elapsedEvaluationTime);

				// Fetch the neural network ouputs
				::fetch(getBrain().get(), &networkOutputs[0]);
			}

			// Send control to motors
			for (unsigned int i = 0; i < getMotors().size(); ++i) {

				// Add motor noise:
				// uniform in range +/- motorNoiseLevel * actualValue
				if(configuration->getMotorNoiseLevel() > 0.0) {
					networkOutputs[i] += (
								((/**uniformDistribution(rng)*/ random() *
								2.0 *
								configuration->getMotorNoiseLevel())
								- configuration->getMotorNoiseLevel())
								* networkOutputs[i]);
				}
				if (boost::dynamic_pointer_cast<RotationMotor>(getMotors()[i])) {
					boost::dynamic_pointer_cast<RotationMotor>(getMotors()[i])
							->setDesiredVelocity(
													networkOutputs[i],
													step_ * configuration->getActuationPeriod()
												);
					std::string id = getMotors()[i]->getId().first;
					/**std::cout << id <<": Desired velocity: " << networkOutputs[i] << ". Step size: "
							<< step_ * configuration->getActuationPeriod()
							<< std::endl;*/
				} else if (boost::dynamic_pointer_cast<ServoMotor>(getMotors()[i])) {
					boost::dynamic_pointer_cast<ServoMotor>(getMotors()[i])
							->setDesiredPosition(
													networkOutputs[i],
													step_ * configuration->getActuationPeriod()
												);
					/**std::cout << "Desired position: " << networkOutputs[i] << ". Step size: "
							<< step_ * configuration->getActuationPeriod()
							<< std::endl;*/
				}

			}

			for (unsigned int i = 0; i < getMotors().size(); ++i) {
				getMotors()[i]->step(step_ ) ; //* configuration->getActuationPeriod() );
				//std::cout<<"MOTORS STEPPED."<<std::endl;
				// TODO find a cleaner way to do this
				// for now will reuse accel cap infrastructure
				/**if (getMotors()[i]->isBurntOut()) {
						std::cout << "Motor burnt out, will terminate now "
										<< std::endl;

				}*/
			}

		}
	}

	unsigned int EDQDRobot::computeRequiredNumberOfWeights(){
		/**
		 * To add function logic here
		 */

		return 0;
	}

	void EDQDRobot::stepEvolution(){
		if( robogen::iterations > 0 && robogen::iterations % EDQD::Parameters::evaluationTime == 0 ){

			/**
			 * lifetime ended: replace genome (if possible)
			 * add needs to happen before new genome is loaded to ensure the map has an entry.
			 * moved to agent observer in previous work - why???
			 */
			//std::cout << "EMBODIED-STEP-EVOLUTION - Robot ID: "
			//		<< getId()
					//<< ". Number of transmitted genomes: "
					//<< nbGenomeTransmission_
					//<< ". Map list size: "
					//<< mapList_.size()
					//<< " Current genome size: "
					//<< currentGenome_.size()
			//		<< std::endl;
	    	map_->add(getId(),
	    			this,
	    			currentGenome_,
					currentSigma_
					);
	    	//std::cout << "EMBODIED-STEP-EVOLUTION: Robot ID: " << getId() << ". Done adding to Map." << std::endl;
	    	//std::cout << "EMBODIED-STEP-EVOLUTION: Robot ID: " << getId() << ". loading new genome. . ." << std::endl;
	    	loadNewGenome();
	    	//std::cout << "EMBODIED-STEP-EVOLUTION: Robot ID: " << getId() << ". Done loading new genome." << std::endl;
	        nbGenomeTransmission_ = 0;
	        //std::cout << "EMBODIED-STEP-EVOLUTION: Robot ID: " << getId() << ". resetting fitness. . ." << std::endl;
	        resetFitness();
	        //std::cout << "EMBODIED-STEP-EVOLUTION: Robot ID: " << getId() << ". Done stepping evolution." << std::endl;
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
	        	//std::cout << "EMBODIED-STEP-EVOLUTION: Broadcasting . . ." << std::endl;
	            broadcastMap();
	            //std::cout << "EMBODIED-STEP-EVOLUTION: Done." << std::endl;
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
		//std::cout << "Max distance travelled: " << dMaxTravelled_ << std::endl;
		// check for new NN parameters
	    if ( getNewGenomeStatus() ){
	    	//std::cout << "EMBODIED-STEP-EVOLUTION: mapping genotype to phenotype. . . ." << std::endl;
	        mapGenotypeToPhenotype();
	        //std::cout << "EMBODIED-STEP-EVOLUTION: done." << std::endl;
	        //std::cout << "EMBODIED-STEP-EVOLUTION: setting nw genotype status. . . " << std::endl;
	        ::setWeights(getBrain().get(), weights_);
	        setNewGenomeStatus(false);
	        /**std::cout 	<< "EMBODIED-STEP-EVOLUTION: Robot ID: "
	        			<< getId()
						<< " New weights set."
						<< std::endl;*/
	    }
	}

	void EDQDRobot::mapGenotypeToPhenotype(){
		// Fill the parameters array with zeros first
		memset(weights_, 0, sizeof(weights_));
		// Intialize genome
		for ( unsigned int i = 0 ; i != currentGenome_.size() ; i++ ){
			// weights: random init between -1 and +1
			weights_[i] = currentGenome_[i];
		}

	}

	void EDQDRobot::performVariation(){
		//mutator_->EDQDperformVariation(currentGenome_, currentSigma_);

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
				mutateSigmaValue();
				mutateGaussian(currentSigma_);
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


	/**
	 * If called, assume genomeList.size() > 0
	 */
	bool EDQDRobot::selectRandomGenomeFromMergedMap(){

	    std::map<int, EDQDMap* >::iterator it = mapList_.begin();
	    while ( it != mapList_.end() ) {
	    	it->second->mergeInto(*mergedMap_);
	    	/**std::cout << "[SelectRandomGenomeFromMergedMap] Robot id: "
	    			<< getId()
	    			<<" :: receivedMap: " << (*it->second).getNumFilledCells() << std::endl << (*it->second) << std::endl;*/

	    	it ++;
	    }
	    behav_index_t index;
	    /**std::cout << "[SelectRandomGenomeFromMergedMap] Robot id: "
					<< getId()
					<< ". do-while loop starts. . . "
					<<std::endl;*/
	    int tries = 0;
		do {
			index = mergedMap_->getRandomIndex();
			if (tries > 100) {
				std::cout 	<< "\nRobot ID - "
							<< getId()
							<< ": Failed to select genome from merged map. Robot will be disabled."
							<< std::endl;
				return false;
			}
			tries++;
		} while ( mergedMap_->get(index)->genome.size() == 0 );
		/**std::cout << "[SelectRandomGenomeFromMergedMap] Robot id: "
					<< getId()
					<< ". do-while loop ends. . . "
					<<std::endl;*/
	    currentGenome_ = mergedMap_->get(index)->genome;
	    currentSigma_ = mergedMap_->get(index)->sigma;
	    birthdate_ = robogen::iterations;

	    ancestor_ = mergedMap_->get(index)->id;

	    setNewGenomeStatus(true);
	    return true;
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
	    	/**std::cout << "ROBOT-"
					<< getId()
					<< " not listening right now."
					<< std::endl;*/
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
	        	/**std::cout << "ROBOT-"
	        			<< getId()
						<< ": sender-"
						<< senderId
						<< " already stored in map list."
						<< std::endl;*/
	            return false;
	        }
	        else{
	        	mapList_[senderId] = map;
	        	std::cout << "*" <<std::flush;
	        	/**std::cout << "ROBOT-"
						<< getId()
						<< ": sender-"
						<< senderId
						<< " map stored successfully in map list."
						<< std::endl;*/
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

	    for (unsigned int i = 0 ; i < currentGenome_.size() ; ++i ){
	        float randomValue = float(randint()%100) / 100.0; // in [0,1]
	        double range = maxValue_ - minValue_;
	        double value = randomValue * range + minValue_;

	        currentGenome_[i] = value;
	    }
	}



	void EDQDRobot::clearReservoir(){

		sigmaList_.clear();
		fitnessValuesList_.clear();

		// always clear _mapList, as results have been merged.
		mapList_.clear();

		if ( EDQD::Parameters::onlyKeepMapsForGeneration||robogen::iterations == 0 ){
			delete mergedMap_;
			mergedMap_ = new EDQDMap();
		}
	}

	void EDQDRobot::reset(){
		currentGenome_.clear();
		// Fill the current genome with zeros first
		//memset(currentGenome_, 0, sizeof(currentGenome_));
		// Intialize genome
		//TODO: Determine exact number of genes
		for ( unsigned int i = 0 ; i != maxGenomeSize ; i++ ){
			// weights: random init between -1 and +1
			currentGenome_.push_back(((double)(randint()%EDQD::Parameters::weightRange)
					/(EDQD::Parameters::weightRange/2)-1.0));
		}

		setNewGenomeStatus(true);

		clearReservoir(); // will contain the genomes received from other robots
		// Now copy the current genome from the neural network
		/*memcpy(
					currentGenome_,
					getBrain()->weight,
					sizeof(float) * ((nInputs + nOutputs + nHidden) * (nOutputs + nHidden))
				);*/
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


				if (boost::dynamic_pointer_cast<IrSensorElement>(getSensors()[i])) {
					if (boost::dynamic_pointer_cast<IrSensorElement>(getSensors()[i])->read() > 0.0){
						double dist = boost::dynamic_pointer_cast<IrSensorElement>(getSensors()[i])->read();
						//std::cout << "Object detected at " << dist << "away." << std::endl;
					}
				}
				ColorSensorElement::Type colorType;
				if (boost::dynamic_pointer_cast< ColorSensorElement>(getSensors()[i])) {
					if (boost::dynamic_pointer_cast< ColorSensorElement>(getSensors()[i])->read()){
						colorType = boost::dynamic_pointer_cast< ColorSensorElement>(getSensors()[i])->
																	getType();

						// sensor ray bumped into a robot : communication is possible
						if ( colorType == ColorSensorElement::Type::ROBOT ){

							int targetIndex = boost::dynamic_pointer_cast<ColorSensorElement>(getSensors()[i])
													->getObjectId(); // convert image registering index into robot id.
							//std::cout << "Robot with id: " << targetIndex << " has been detected by robot: " << getId() << std::endl;
							//boost::shared_ptr<EDQDRobot> targetRobot = static_cast<EDQDRobot>(scenario_->getRobot(targetIndex));


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
								//exit(-1);
							}
						}
					}
				}
	    	}
		}
	}

	/**
	 * Called only if at least 1 genome was stored.
	 */
	bool EDQDRobot::performSelection(){
		// Include Local Map For Selection Merge
		mapList_[getId()] = map_;
		if (selectRandomGenomeFromMergedMap()){
			mergedMap_->mergeInto(*map_);
			return true;
		}
		return false;
	}

	void EDQDRobot::loadNewGenome(){
		//std::cout << "EMBODIED-LOAD NEW GENOME: Robot ID: " << getId() << ". BEGIN." << std::endl;
		if (isAlive()){

	        if ( mapList_.size() > 0 ){
	        	/**std::cout << "EMBODIED-LOAD NEW GENOME: Robot ID: " << getId() << ". alive with map size of: "
	        			<< mapList_.size()
						<< ". Peforming selection. . . "
						<< std::endl;*/
	            if (performSelection()){


					/**std::cout << "EMBODIED-LOAD NEW GENOME: Robot ID: " << getId() << ". Done performing selection."
							<< " Performing variation. . ."
							<< std::endl;*/
					performVariation();
					//std::cout << "EMBODIED-LOAD NEW GENOME: Robot ID: " << getId() << ". Done performing variation. Clearing reservoir. . ." << std::endl;
					clearReservoir();
					//std::cout << "EMBODIED-LOAD NEW GENOME: Robot ID: " << getId() << ". Done clearing reservoir." << std::endl;
					setAlive(true);
	        	}
	            else{
	            	setAlive(false);
	            }
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
	        }
	        else{
	        	if ( isAlive() == true){
	        		/**std::cout << "EMBODIED-LOAD NEW GENOME: Robot ID: " << getId() << ". Alive with map size: "
	        				<< mapList_.size() << ". Resetting. . ."
							<< std::endl;*/
	        		reset();
	        		std::cout << "\nRobot ID - " << getId()
	        				<< ": Received map list empty, will be disabled."
	        				<< std::endl;
	        		setAlive(false);

	        		// ie. -1 (infinite,dead) or >0 (temporary,mute)
	        		if ( EDQD::Parameters::listeningStateDelay != 0 ){

						isListening_ = false;

						notListeningDelay_ = EDQD::Parameters::notListeningStateDelay;
						listeningDelay_ = EDQD::Parameters::listeningStateDelay;

					}
	        		else{

	        			listeningDelay_ = EDQD::Parameters::listeningStateDelay;

						if (listeningDelay_ > 0 || listeningDelay_ == -1 ){
							isListening_ = true;
						}

					}
	        		//std::cout << "EMBODIED-LOAD NEW GENOME: Robot ID: " << getId() << ". Done setting robot alve state to FALSE" << std::endl;
	        	}
	        }
	    }
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
			/**std::cout 		<< "rc.first: "
					 	 	 << it->first
							 << " rc.second: "
							 << it->second
							 <<std::endl;*/
			sum += 100 * it->second / it->first;
		}
		fitness_ = sum;
	}

	int EDQDRobot::updateCellId() {
		behav_index_t index = EDQDMap::computeIndex(resourceCounters_, dMaxTravelled_, dPotMaxTravelled_);
		currentCellId_ = index[0] * EDQD::Parameters::nbOfIntervals  + index[1];

		return currentCellId_;
	}

    /**void EDQDRobot::setWeights(){
    	//neuralNetwork_.reset(new NeuralNetwork);
    	::initNetwork(
    				getBrain().get(),
					nbInputs_,
					nbOutputs_,
					nbHidden_,
					&weights_[0],
					&params_[0],
					&types_[0]
					);
    }*/

}
