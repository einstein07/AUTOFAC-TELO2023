/*
 * @(#) Map.h   1.0   Sep 02, 2021
 *
 * Sindiso Mkhatshwa (mkhsin035@myuct.ac.za)
 *
 * Nitschke Laboratory, UCT
 *
 * @(#) $Id$
 */
#include "EDQDMap.h"

namespace robogen{

	//==========================================================================================
	// EDQDMap class implementation
	//==========================================================================================
	EDQDMap::EDQDMap() {
		mapHasEntry_ = false;
		num_filled_cells_ = 0;
		map_dim_ = EDQD::Parameters::nbOfDimensions;

		for (size_t i = 0; i < map_dim_; ++i) {
			map_shape_[i] = EDQD::Parameters::nbOfIntervals;
		}
		map_.resize(map_shape_);
	}

	EDQDMap::~EDQDMap() {
		for(size_t i = 0; i < map_.num_elements(); i++){
			map_.data()[i].genome_.clear();
			map_.data()[i].pos_.clear();
		}
	}

	behav_index_t EDQDMap::computeIndex(const std::map<int, int>& oc, double distance, double maxDistance, std::vector<double>* pos) {
		double dim1 = 0.5, dim2 = 0.0;
		int oc1 = oc.at(1);
		int	oc2 = oc.at(2);
		int oc3 = oc.at(3);
		int	oc4 = oc.at(4);
		int oc5 = oc.at(5);
		double sumOfToken = ((double)(oc1 + oc2 + oc3 + oc4 + oc5));
		double r_types = 5.0;
		if ( oc.at(5) > 0.0 ) {
			//dim1 = ( oc1 + oc2 + oc3 + oc4 + oc5 ) / sumOfToken;
			dim1 = ((double)(5.0/r_types));
		} else if ( oc.at(4) > 0.0 ) {
			//dim1 = ( oc1 + oc2 + oc3 + oc4 ) / sumOfToken;
			dim1 = ((double)(4.0/r_types));
		}
		else if ( oc.at(3) > 0.0 ) {
			//dim1 = ( oc1 + oc2 + oc3 ) / sumOfToken;
			dim1 = ((double)(3.0/r_types));
		}
		else if ( oc.at(2) > 0.0 ) {
			//dim1 =  ( oc1 + oc2 )  / sumOfToken;
			dim1 = ((double)(2.0/r_types));
		}
		else if ( oc.at(1) > 0.0 ) {
			//dim1 = 1.0;
			//dim1 =  oc1  / sumOfToken;
			dim1 = ((double)(1.0/r_types));
		}
		dim1 *= ((double) EDQD::Parameters::nbOfIntervals - 0.5);

		if ( distance > 0 ) {
			dim2 = ( (distance / ((double)maxDistance)) * ( (double) EDQD::Parameters::nbOfIntervals - 0.5 ) );
			if ( dim2 < 0.0 ) { dim2 = 0; }
			if ( dim2 > EDQD::Parameters::nbOfIntervals - 1 ) { dim2 = (double) EDQD::Parameters::nbOfIntervals - 0.5; }
		}
		//std::cout << dim1 << " " << dim2 << " "  << (long int)dim1 << " " << (long int)dim2 << std::endl; // DEBUG

		if (pos) {
			(*pos)[0] = dim1;
			(*pos)[1] = dim2;
		}
		behav_index_t index = {{(long int)dim1, (long int)dim2}};
		return index;
	}
	behav_index_t EDQDMap::computeMorphIndex(const std::map<int, int>& sensorTypes, int totalNumOfSensors, double averageRange, double maxRange, std::vector<double>* pos){
		double dim1 = 0.5, dim2 = 0.0;

		int s1 = sensorTypes.at(SensorElement::RESOURCET1);
		int	s2 = sensorTypes.at(SensorElement::RESOURCET2);
		int s3 = sensorTypes.at(SensorElement::RESOURCET3);
		int	s4 = sensorTypes.at(SensorElement::RESOURCET4);
		int s5 = sensorTypes.at(SensorElement::RESOURCET5);
		double sumOfActiveSensors = ((double)(s1 + s2 + s3 + s4 + s5));
		dim1 = sumOfActiveSensors/totalNumOfSensors;
		//std::cout << "dim 1: " << dim1 << " active sensors: " << sumOfActiveSensors << " total number of sensors: " << totalNumOfSensors << std::endl;

		dim1 *= ((double) EDQD::Parameters::nbOfIntervals - 0.5);

		if ( averageRange > 0 ) {
			dim2 = ( (averageRange / ((double)maxRange)) * ( (double) EDQD::Parameters::nbOfIntervals - 0.5 ) );
			if ( dim2 < 0.0 ) { dim2 = 0; }
			if ( dim2 > EDQD::Parameters::nbOfIntervals - 1 ) { dim2 = (double) EDQD::Parameters::nbOfIntervals - 0.5; }
		}
		// DEBUG
		//std::cout << "dim1" << dim1 << " s-1" << oc1 << " s-2"  << oc2 << " s-3" << oc3 << " s-4" << oc4 << " s-5" << oc5 << " dim2: " << dim2 << " av-range: " << averageRange << " max-range: " << maxRange << std::endl; // DEBUG
		//std::cout << dim1 << " " << dim2 << " "  << (long int)dim1 << " " << (long int)dim2 << std::endl; // DEBUG
		//std::cout << "dim 2: " << dim2 << " average range: " << averageRange << " max range: " << maxRange << std::endl;
		if (pos) {
			(*pos)[0] = dim1;
			(*pos)[1] = dim2;
		}

		behav_index_t index = {{(long int)dim1, (long int)dim2}};
		return index;
	}

	behav_index_t EDQDMap::getRandomIndex() {
		behav_index_t index = {{(long int)(random() * EDQD::Parameters::nbOfIntervals),
								(long int)(random() * EDQD::Parameters::nbOfIntervals)}};
		return index;
	}

	bool EDQDMap::add( int id, EDQDRobot* robot, std::vector<double> genome, float sigma ) {

		std::vector<double>* pos = new std::vector<double>(2);
		behav_index_t index = computeIndex(
										(*robot).getResourceCounters(),
										(*robot).getMaxTravelled(),
										(*robot).getPotMaxTravelled(),
										pos
										);

		if (map_(index).fitness_ == -1
			|| (robot->getFitness() - map_(index).fitness_) > EDQD::Parameters::fitEpsilon
			|| (fabs(robot->getFitness() - map_(index).fitness_) <= EDQD::Parameters::fitEpsilon
					&& _dist_center((*pos)) < _dist_center(map_(index).pos_))
		) {
			if (map_(index).fitness_ == -1) {
				num_filled_cells_++;
			}

			map_(index).fitness_ = robot->getFitness();
			map_(index).genome_.clear();
			map_(index).genome_ = genome;
			map_(index).id_ = std::make_pair(id,robot->getBirthdate());
			map_(index).sigma_ = sigma;
			map_(index).pos_.clear();
			map_(index).pos_ = (*pos);

			setMapHasEntry(true);
			return true;
		}
		return false;
	}
	/*****************************************************************************************************
	 * Morphology-map genome addition addition
	 ****************************************************************************************************/

	bool EDQDMap::morphAdd( int id, EDQDRobot* robot, std::vector<double> genome, float sigma, std::map<int,double> perSensorTypeRange ) {

			std::vector<double>* pos = new std::vector<double>(2);
			behav_index_t index = computeMorphIndex(
									(*robot).getActiveSensors(),
									(*robot).getPossibleNumberOfSensors(),
									(*robot).getAverageActiveSensorRange(),
									(*robot).getMaxActiveSensorRangeAverage(),
									pos);

			if (map_(index).fitness_ == -1
				|| (robot->getFitness() - map_(index).fitness_) > EDQD::Parameters::fitEpsilon
				|| (fabs(robot->getFitness() - map_(index).fitness_) <= EDQD::Parameters::fitEpsilon
						&& _dist_center((*pos)) < _dist_center(map_(index).pos_))
			) {
				if (map_(index).fitness_ == -1) {
					num_filled_cells_++;
				}

				map_(index).fitness_ = robot->getFitness();
				map_(index).genome_.clear();
				map_(index).genome_ = genome;
				map_(index).id_ = std::make_pair(id,robot->getBirthdate());
				map_(index).sigma_ = sigma;
				map_(index).pos_.clear();
				map_(index).pos_ = (*pos);

				map_(index).perSensorTypeRange_.clear();
				map_(index).perSensorTypeRange_ = perSensorTypeRange;

				setMapHasEntry(true);
				return true;
			}
			return false;
		}

	bool EDQDMap::mergeInto(EDQDMap &m) {
		bool result = false;
		for (size_t i = 0; i < this->map_.num_elements(); i ++) {
			if ( m.getMap().data()[i].fitness_ == -1 ||
					m.getMap().data()[i] < this->map_.data()[i] )
			{
				m.getMap().data()[i] = this->map_.data()[i];
				result = true;
			}
		}
		if (result) {
			m.updateNumFilledCells();
		}
		return result;
	}

}
