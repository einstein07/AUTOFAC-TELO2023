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
/** \brief	A struct that represents an elite stored in the behavior map */
//================================================================================================
struct Elite {
	public:
		//========================================================================================
		// Methods
		//========================================================================================
	/**
	 * \brief Default Constructor
	 */
		Elite(){}

		/*Elite(Elite* o) {
			fitness_ = o->fitness_;
			genome_.clear();
			genome_ = o->genome_;
			id_ = o->id_;
			sigma_ = o->sigma_;
			pos_.clear();
			pos_ = o->pos_;
		}*/
		/**
		 * \brief Constructor
		 */
		Elite(Elite* o) {
			fitness_ = o->fitness_;
			genome_.clear();
			genome_ = o->genome_;
			id_ = o->id_;
			sigma_ = o->sigma_;
			pos_.clear();
			pos_ = o->pos_;
			perSensorTypeRange_.clear();
			perSensorTypeRange_ = o->perSensorTypeRange_;
		}

		/*Elite(double fitness, std::vector<double> genome, std::pair<int, int> id, float sigma, std::vector<double> pos) {
			fitness_ = fitness;
			genome_.clear();
			genome_ = genome;
			id_ = id;
			sigma_ = sigma;
			pos_.clear();
			pos_ = pos;
		}*/
		/**
		 * \brief Constructor
		 */
		Elite(double fitness, std::vector<double> genome, std::pair<int, int> id, float sigma, std::vector<double> pos, std::map<int,double> perSensorTypeRange) {
			fitness_ = fitness;
			genome_.clear();
			genome_ = genome;
			id_ = id;
			sigma_ = sigma;
			pos_.clear();
			pos_ = pos;
			perSensorTypeRange_.clear();
			perSensorTypeRange_ = perSensorTypeRange;
		}
		/**
		 * \brief Less than definition
		 */
		inline bool operator< (const Elite& rhs){
			return this->fitness_ < rhs.fitness_ ||
					( fitness_ == rhs.fitness_ && _dist_center(pos_) > _dist_center(rhs.pos_) ) ;
		}
		/**
		 * \brief Greater than definition
		 */
		inline bool operator> (const Elite& rhs){
			return rhs.fitness_ < fitness_ ||
					( fitness_ == rhs.fitness_ && _dist_center(rhs.pos_) > _dist_center( pos_) );
		}
		/**
		 * \brief Less than or equal to definition
		 */
		inline bool operator<=(const Elite& rhs){
			return !(rhs.fitness_ < fitness_ ||
					( fitness_ == rhs.fitness_ && _dist_center(rhs.pos_) > _dist_center( pos_) ));
		}
		/**
		 * \brief Greater or equal to definition
		 */
		inline bool operator>=(const Elite& rhs){
			return !( fitness_ < rhs.fitness_ ||
					( fitness_ == rhs.fitness_ && _dist_center( pos_) > _dist_center(rhs.pos_) ));
		}
		/**
		 * \brief Write Elite object to standard output stream
		 */
		inline friend std::ostream& operator<<(std::ostream& os, const Elite& obj){
			os << obj.fitness_;
			return os;
		};

		template <class Archive>
		/**
		 * \brief Serializes map object
		 */
		void serialize(Archive& ar, const unsigned int version) {
			ar& BOOST_SERIALIZATION_NVP(fitness_);
			ar& BOOST_SERIALIZATION_NVP(genome_);
			ar& BOOST_SERIALIZATION_NVP(id_);
			ar& BOOST_SERIALIZATION_NVP(sigma_);
			ar& BOOST_SERIALIZATION_NVP(pos_);
			ar& BOOST_SERIALIZATION_NVP(counter_);
		}

		/**
		 * \brief Returns distance to center of behavior descriptor cell
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
		/**
		 * \brief Fitness
		 */
		double fitness_ = -1.0;
		/**
		 * \brief Genome
		 */
		std::vector<double> genome_;
		/**
		 * \brief ID
		 */
		std::pair<int, int> id_;
		/**
		 * \brief Reference sigma value
		 */
		float sigma_ = EDQD::Parameters::sigmaRef;
		/**
		 * \brief Position
		 */
		std::vector<double> pos_;
		/**
		 * \brief Counter
		 */
		int counter_ = 0;

		/**
		* \brief Double-Map EDQD
		*
		* All sensor types are subjected to the same variation ops, i.e. if one type-1 sensor is
		* off, then all type 1 sensors should be off
		*/
		std::map<int,double> perSensorTypeRange_;

};

typedef boost::multi_array<Elite, EDQD::Parameters::nbOfDimensions> map_t;
typedef map_t::index map_index_t;
typedef boost::array<map_index_t, EDQD::Parameters::nbOfDimensions> behav_index_t;
//===============================================================================================
/** \brief Class describing diversity maintenance maps */
//===============================================================================================
class EDQDMap {

	friend class boost::serialization::access;

	protected:
		//========================================================================================
		// Methods
		//========================================================================================
		/**
		 * \brief Updates has-entry flag
		 * @param bool new status
		 */
		void setMapHasEntry(bool mapHasEntry) {
			mapHasEntry_ = mapHasEntry;
		}

		/**
		 * \brief Returns distance to center of behavior descriptor
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
		/** \brief Diversity map */
		map_t map_;
		/** \brief Map dimensions */
		size_t map_dim_;
		/** \brief Map shape according to behavior indexing */
		behav_index_t map_shape_;
		/** \brief Flags whether this map has an entry */
		bool mapHasEntry_;
		/** \brief Number of filled cells */
		int num_filled_cells_;

	public:
		//=======================================================================================
		// Methods
		//=======================================================================================
		/**
		 * \brief Construtor
		 */
		EDQDMap();
		/**
		 * \brief Destructor
		 */
		virtual ~EDQDMap();

		/**
		 * \brief Behavior index computation
		 *
		 * @param std::map<int, int>& map of resource types to number collected
		 * @param double distance traversed
		 * @param double maximum distance possible
		 * @param std::vector<double>* x y position of elite
		 * @return behav_index_t index
		 */
		static behav_index_t computeIndex(
											const std::map<int, int>& oc,
											double distance,
											double maxDistance,
											std::vector<double>* pos = NULL
										);
		/**
		 * \brief Morphology index computation
		 *
		 * @param std::map<int, int>& map of sensor types to number active
		 * @param int total number of sensors
		 * @param double average range
		 * @param double maximum possible range
		 * @param std::vector<double>* x y position of elite
		 * @return behav_index_t index
		 */
		static behav_index_t computeMorphIndex(const std::map<int, int>& sensorTypes, int totalNumOfSensors, double averageRange, double maxRange, std::vector<double>* pos);

		/** \brief Setter */
		Elite& operator()(int i, int j) {
			return map_[i][j];
		}
		/** \brief Getter */
		Elite operator()(int i, int j) const {
			return map_[i][j];
		}

		/**
		 * \brief Write EDQDMap object to standard output stream
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
		/**
		 * \brief Load map archive
		 *
		 * @param Archive archive
		 * @param int version of archive
		 */
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
		/** \brief Save map archive
		 * @param Archive archive
		 * @param int version of archive
		 * */
		void save(Archive& ar, const int version) const {
			ar& BOOST_SERIALIZATION_NVP(mapHasEntry_);
			ar& BOOST_SERIALIZATION_NVP(map_dim_);
			for (size_t i = 0; i < map_dim_; i++) {
				ar& BOOST_SERIALIZATION_NVP(map_shape_[i]);
			}
			ar & boost::serialization::make_array(map_.data(), map_.num_elements());
		}

		/**
		 * \brief Add new elite to behavior map
		 *
		 * @param int robot id
		 * @param EDQDRobot* pointer to EDQDRobot object
		 * @param std::vector<double> genome of elite being added
		 * @param flot sigma of elite being added
		 * @return bool true if elite was added, false if not
		 */
		bool add(
					int id,
					EDQDRobot* ctrl,
					std::vector<double> genome,
					float sigma
				);
		/**
		 * \brief Add new elite to map
		 *
		 * @param int robot id
		 * @param EDQDRobot* pointer to EDQDRobot object
		 * @param std::vector<double> genome of elite being added
		 * @param flot sigma of elite being added
		 * @param std::map<int,double> per sensor type range
		 * @return bool true if elite was added, false if not
		 */
		bool morphAdd(
					int id,
					EDQDRobot* robot,
					std::vector<double> genome,
					float sigma,
					std::map<int,double> perSensorTypeRange
				);
		/**
		 * \brief Access an elite stored at the specified index
		 *
		 * @param behav_index_t index
		 * @return Elite*
		 */
		Elite* get(behav_index_t index) {
			return &map_(index);
		}

		/**
		 * \brief Returns a random map index
		 *
		 * @return behav_index_t
		 */
		behav_index_t getRandomIndex();

		/**
		 * \brief Checks is map has an entry
		 * @return bool
		 */
		bool hasEntry() const {
			return mapHasEntry_;
		}
		/**
		 * \brief Returns a reference to this map
		 * @return map_t&
		 */
		map_t& getMap() {
			return map_;
		}
		/**
		 * \brief Returns number of filled cells in map
		 * @return int
		 */
		int getNumFilledCells() const
		{
			return num_filled_cells_;
		}
		/**
		 * \brief Merge this map into the parameter of this function
		 * @param EDQDMap& map to merge into
		 * @return bool true if merge occurred
		 */
		bool mergeInto(EDQDMap& m);

		/**
		 * \brief Recalculates the number of filled cells within the map
		 */
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
	//=========================================================================================
	//*************************STATIC METHODS*************************
	//=========================================================================================
	/**
	 * \brief Write binary archive of map
	 * @param EDQDMap& map
	 * @param std::string& filename
	 */
	static void writeMapBin(const EDQDMap& m, const std::string& filename) {
		std::ofstream ofs(filename.c_str());
		assert(ofs.good());
		//std::cout << "writing : " << filename << std::endl;
		boost::archive::binary_oarchive oa(ofs);
		oa& BOOST_SERIALIZATION_NVP(m);
		//std::cout << "done" << std::endl;
	}

	/**
	 * \brief Load map from file
	 * @param const std::string& filename
	 */
	static EDQDMap loadMap(const std::string& filename) {
		std::ifstream ifs(filename.c_str());
		assert(ifs.good());
		boost::archive::binary_iarchive ia(ifs);
		EDQDMap data;
		ia& BOOST_SERIALIZATION_NVP(data);
		return data;
	}

	/**
	 * \brief Convert map to a string readable format
	 * @param map_t& map
	 * @param std::string& prefix
	 * @return std::string map in string format
	 */
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
	/**
	 * \brief Write map to file
	 * @param const map_t& map
	 * @param const std::string& filename
	 * @param const std::string& Note
	 */

	static void writeMapToFile(const map_t& m, const std::string& filename,
		const std::string& note = "")
	{
		//std::cout << "writing " << note << " " << filename << std::endl;

		std::ofstream ofs(filename.c_str());

		ofs << mapToString(m);
	}
}
#endif /*MAP_ELITES_MAP_H_*/
