/*
 * @(#) Map.h   1.0   Sep 01, 2021
 *
 * Sindiso Mkhatshwa (mkhsin035@myuct.ac.za)
 *
 * Nitschke Laboratory, UCT
 *
 * @(#) $Id$
 */

#ifndef MAP_ELITES_MAP_H_
#define MAP_ELITES_MAP_H_
#include <boost/multi_array.hpp>
#include <boost/optional.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/array.hpp>
#include <boost/serialization/split_member.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <fstream>
#include <iostream>
#include <math.h>
#include <map>
#include "EDQD/EDQDRobot.h"
#include "EDQD/Parameters.h"
namespace robogen{
class EDQDRobot;
//================================================================================================
//	A struct that represents an elite stored in the behavior map
//================================================================================================
struct Elite {
	public:
		//========================================================================================
		// Methods
		//========================================================================================
		Elite(){}

		Elite(Elite* o) {
			fitness_ = o->fitness_;
			genome_.clear();
			genome_ = o->genome_;
			id_ = o->id_;
			sigma_ = o->sigma_;
			pos_.clear();
			pos_ = o->pos_;
		}

		Elite(double fitness, std::vector<double> genome, std::pair<int, int> id, float sigma, std::vector<double> pos) {
			fitness_ = fitness;
			genome_.clear();
			genome_ = genome;
			id_ = id;
			sigma_ = sigma;
			pos_.clear();
			pos_ = pos;
		}

		inline bool operator< (const Elite& rhs){
			return this->fitness_ < rhs.fitness_ ||
					( fitness_ == rhs.fitness_ && _dist_center(pos_) > _dist_center(rhs.pos_) ) ;
		}
		inline bool operator> (const Elite& rhs){
			return rhs.fitness_ < fitness_ ||
					( fitness_ == rhs.fitness_ && _dist_center(rhs.pos_) > _dist_center( pos_) );
		}
		inline bool operator<=(const Elite& rhs){
			return !(rhs.fitness_ < fitness_ ||
					( fitness_ == rhs.fitness_ && _dist_center(rhs.pos_) > _dist_center( pos_) ));
		}
		inline bool operator>=(const Elite& rhs){
			return !( fitness_ < rhs.fitness_ ||
					( fitness_ == rhs.fitness_ && _dist_center( pos_) > _dist_center(rhs.pos_) ));
		}
		/**
		 * Write Elite object to standard output stream
		 */
		inline friend std::ostream& operator<<(std::ostream& os, const Elite& obj){
			os << obj.fitness_;
			return os;
		};

		template <class Archive>
		void serialize(Archive& ar, const unsigned int version) {
			ar& BOOST_SERIALIZATION_NVP(fitness_);
			ar& BOOST_SERIALIZATION_NVP(genome_);
			ar& BOOST_SERIALIZATION_NVP(id_);
			ar& BOOST_SERIALIZATION_NVP(sigma_);
			ar& BOOST_SERIALIZATION_NVP(pos_);
			ar& BOOST_SERIALIZATION_NVP(counter_);
		}

		/**
		 * Returns distance to center of behavior descriptor cell
		 */
		float _dist_center(const std::vector<double>& pos) {

			float dist = 0.0;
			for(size_t i = 0; i < pos.size(); ++i) {
				dist += pow(pos[i] - (float)round(pos[i] * (float)(EDQD::Parameters::nbOfIntervals - 1))/(float)(EDQD::Parameters::nbOfIntervals - 1), 2);
			}
			dist=sqrt(dist);
			return dist;
		}

		void incCounter() {
			counter_++;
		}
		//=======================================================================================
		// Members
		//=======================================================================================
		double fitness_ = -1.0;
		std::vector<double> genome_;
		std::pair<int, int> id_;
		float sigma_ = EDQD::Parameters::sigmaRef;
		std::vector<double> pos_;
		int counter_ = 0;

};

typedef boost::multi_array<Elite, EDQD::Parameters::nbOfDimensions> map_t;
typedef map_t::index map_index_t;
typedef boost::array<map_index_t, EDQD::Parameters::nbOfDimensions> behav_index_t;
//===============================================================================================
// Map class
//===============================================================================================
class EDQDMap {

	friend class boost::serialization::access;

	protected:
		//========================================================================================
		// Methods
		//========================================================================================
		void setMapHasEntry(bool mapHasEntry) {
			mapHasEntry_ = mapHasEntry;
		}

		/****************************************************************************************
		 * @return float distance to center of behavior descriptor cell
		 ***************************************************************************************/
		float _dist_center(const std::vector<double>& pos) {

			float dist = 0.0;
			for(size_t i = 0; i < map_dim_; ++i) {
				dist += pow(pos[i] - (float)round(pos[i] * (float)(map_.shape()[i] - 1))
						/(float)(map_.shape()[i] - 1), 2);
			}
			dist=sqrt(dist);
			return dist;
		}

		//=======================================================================================
		// Members
		//=======================================================================================
		map_t map_;
		size_t map_dim_;
		behav_index_t map_shape_;
		bool mapHasEntry_;
		int num_filled_cells_;

	public:
		//=======================================================================================
		// Methods
		//=======================================================================================
		EDQDMap();
		virtual ~EDQDMap();

		static behav_index_t computeIndex(
											const std::map<int, int>& oc,
											double distance,
											double maxDistance,
											std::vector<double>* pos = NULL
										);
		/****************************************************************************************
		 * Morphology index computation
		 ***************************************************************************************/
		static behav_index_t computeMorphIndex(const std::map<int, int>& sensorTypes, int totalNumOfSensors, double averageRange, double maxRange, std::vector<double>* pos);

		//Setter
		Elite& operator()(int i, int j) {
			return map_[i][j];
		}
		//getter
		Elite operator()(int i, int j) const {
			return map_[i][j];
		}

		/**************************************************************************************
		 * Write EDQDMap object to standard output stream
		 *************************************************************************************/
		inline friend std::ostream& operator<<(std::ostream& os, const EDQDMap& obj) {

			int dim1 = obj.map_.shape()[0];
			int dim2 = obj.map_.shape()[2];
			for (map_index_t i = 0; i < dim1; i++){
				os << i;
				for (map_index_t j = 0; j < dim2; j++) {
					os << "," << (obj.map_[i][j]);
				}
				if ( i < dim1 - 1 ){
					os << std::endl;
				}
			}

			return os;
		};

		friend class boost::serialization::access;
		BOOST_SERIALIZATION_SPLIT_MEMBER()

		template <class Archive>
		void load(Archive& ar, const int version) {
			ar& BOOST_SERIALIZATION_NVP(mapHasEntry_);
			ar& BOOST_SERIALIZATION_NVP(map_dim_);
			for (size_t i = 0; i < map_dim_; i++) {
				ar& BOOST_SERIALIZATION_NVP(map_shape_[i]);
			}

			map_.resize(map_shape_);
			ar & boost::serialization::make_array(map_.data(), map_.num_elements());
		}

		template <class Archive>
		void save(Archive& ar, const int version) const {
			ar& BOOST_SERIALIZATION_NVP(mapHasEntry_);
			ar& BOOST_SERIALIZATION_NVP(map_dim_);
			for (size_t i = 0; i < map_dim_; i++) {
				ar& BOOST_SERIALIZATION_NVP(map_shape_[i]);
			}
			ar & boost::serialization::make_array(map_.data(), map_.num_elements());
		}


		bool add(
					int id,
					EDQDRobot* ctrl,
					std::vector<double> genome,
					float sigma
				);
		bool morphAdd(
					int id,
					EDQDRobot* robot,
					std::vector<double> genome,
					float sigma
				);

		Elite* get(behav_index_t index) {
			return &map_(index);
		}

		behav_index_t getRandomIndex();

		bool hasEntry() const {
			return mapHasEntry_;
		}

		map_t& getMap() {
			return map_;
		}

		int getNumFilledCells() const
		{
			return num_filled_cells_;
		}

		bool mergeInto(EDQDMap& m);

		void updateNumFilledCells() {
			num_filled_cells_ = 0;
			for (size_t i = 0; i < map_.num_elements(); i++) {
				if (map_.data()[i].fitness_ > -1.0) {
					num_filled_cells_++;
					setMapHasEntry(true);
				}
			}
		}
	};
}
#endif /*MAP_ELITES_MAP_H_*/
