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
		//Simple environment - i.e. 1 & 2 pushing robots per resource only
		int oc0 = oc.at(1);
		int	oc1 = oc.at(2);

		if ( oc.at(2) > 0.0 ) {
			dim1 = oc0 / ((double)(oc0 + oc1));
		} else if ( oc.at(1) > 0.0 ) {
			dim1 = 1.0;
		}
		dim1 *= ((double) EDQD::Parameters::nbOfIntervals - 0.5);

		if ( distance > 0 ) {
			dim2 = ( (distance / ((double)maxDistance)) * ( (double) EDQD::Parameters::nbOfIntervals - 0.5 ) );
			if ( dim2 < 0.0 ) { dim2 = 0; }
			if ( dim2 > EDQD::Parameters::nbOfIntervals - 1 ) { dim2 = (double) EDQD::Parameters::nbOfIntervals - 0.5; }
		}
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
	//=========================================================================================
	//*************************STATIC METHODS*************************
	//=========================================================================================
	static void writeMapBin(const EDQDMap& m, const std::string& filename) {
		std::ofstream ofs(filename.c_str());
		assert(ofs.good());
		std::cout << "writing : " << filename << std::endl;
		boost::archive::binary_oarchive oa(ofs);
		oa& BOOST_SERIALIZATION_NVP(m);
		std::cout << "done" << std::endl;
	}

	static EDQDMap loadMap(const std::string& filename) {
		std::ifstream ifs(filename.c_str());
		assert(ifs.good());
		boost::archive::binary_iarchive ia(ifs);
		EDQDMap data;
		ia& BOOST_SERIALIZATION_NVP(data);
		return data;
	}

	static std::string mapToString(const map_t& m, const std::string& prefix = std::string("")) {
		std::stringstream ofs;

		size_t behav_dim = m.dimensionality;
		size_t offset = 0;
		for (const Elite* i = m.data(); i < (m.data() + m.num_elements()); ++i) {
			if (i) {
				behav_index_t index;
				for (unsigned int dim = 0; dim < behav_dim; dim++ ) {
					index[dim] = (offset / m.strides()[dim] % m.shape()[dim] +  m.index_bases()[dim]);
				}

				assert(m(index).fitness_ == (i)->fitness_);

				ofs << prefix << offset << ",";
				for (size_t dim = 0; dim < behav_dim; ++dim) {
					ofs << index[dim] / (float)m.shape()[dim] << ",";
				}
				ofs << m(index).fitness_;
				ofs << std::endl;
			}
			++offset;
		}

		return ofs.str();
	}

	static void writeMapToFile(const map_t& m, const std::string& filename,
		const std::string& note = "")
	{
		std::cout << "writing " << note << " " << filename << std::endl;

		std::ofstream ofs(filename.c_str());

		ofs << mapToString(m);
	}

}
