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
			this->fitness = o->fitness;
			this->genome.clear();
			this->genome = o->genome;
			this->id = o->id;
			this->sigma = o->sigma;
			this->pos.clear();
			this->pos = o->pos;
		}

		Elite(double fitness, std::vector<double> genome, std::pair<int, int> id, float sigma, std::vector<double> pos) {
			this->fitness = fitness;
			this->genome.clear();
			this->genome = genome;
			this->id = id;
			this->sigma = sigma;
			this->pos.clear();
			this->pos = pos;
		}

		inline bool operator< (const Elite& rhs){
			return this->fitness < rhs.fitness ||
					( this->fitness == rhs.fitness && _dist_center(this->pos) > _dist_center(rhs.pos) ) ;
		}
		inline bool operator> (const Elite& rhs){
			return rhs.fitness < this->fitness ||
					( this->fitness == rhs.fitness && _dist_center(rhs.pos) > _dist_center(this->pos) );
		}
		inline bool operator<=(const Elite& rhs){
			return !(rhs.fitness < this->fitness ||
					( this->fitness == rhs.fitness && _dist_center(rhs.pos) > _dist_center(this->pos) ));
		}
		inline bool operator>=(const Elite& rhs){
			return !(this->fitness < rhs.fitness ||
					( this->fitness == rhs.fitness && _dist_center(this->pos) > _dist_center(rhs.pos) ));
		}
		/**
		 * Write Elite object to standard output stream
		 */
		inline friend std::ostream& operator<<(std::ostream& os, const Elite& obj){
			os << obj.fitness;
			return os;
		};

		template <class Archive>
		void serialize(Archive& ar, const unsigned int version) {
			ar& BOOST_SERIALIZATION_NVP(fitness);
			ar& BOOST_SERIALIZATION_NVP(genome);
			ar& BOOST_SERIALIZATION_NVP(id);
			ar& BOOST_SERIALIZATION_NVP(sigma);
			ar& BOOST_SERIALIZATION_NVP(pos);
			ar& BOOST_SERIALIZATION_NVP(counter);
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
			this->counter++;
		}
		//=======================================================================================
		// Members
		//=======================================================================================
		double fitness = -1.0;
		std::vector<double> genome;
		std::pair<int, int> id;
		float sigma = EDQD::Parameters::sigmaRef;
		std::vector<double> pos;
		int counter = 0;

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

		/**
		 * @return float distance to center of behavior descriptor cell
		 */
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

		//Setter
		Elite& operator()(int i, int j) {
			return map_[i][j];
		}
		//getter
		Elite operator()(int i, int j) const {
			return map_[i][j];
		}

		/**
		 * Write EDQDMap object to standard output stream
		 */
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
				if (map_.data()[i].fitness > -1.0) {
					num_filled_cells_++;
					setMapHasEntry(true);
				}
			}
		}
	};
}
#endif /*MAP_ELITES_MAP_H_*/
