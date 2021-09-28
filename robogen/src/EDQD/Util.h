/*
 * @(#) EDQDController.h   1.0   Sep 07, 2021
 *
 * Sindiso Mkhatshwa (mkhsin035@myuct.ac.za)
 *
 * Nitschke Laboratory, UCT
 *
 * @(#) $Id$
 */

#ifndef EDQD_UTIL_H_
#define EDQD_UTIL_H_

#include <cstdlib> // RAND_MAX
#include <random>
/**
 * According to previous work:
 * "Note that boost/c++11 random is slower (by one order of magnitude), but more precise,
 * than old classic rand()"
 * The same is used here in order to ensure that the algorithm is not any less efficient
 */

extern std::random_device rnd;
extern std::minstd_rand randint; // randint() returns an int with value drawn uniformly in [0,max)
extern std::mt19937 engine;
extern std::uniform_real_distribution<double> disRandom;
extern std::normal_distribution<> disNormal;

#define random() disRandom(engine) // uniform in [0,1), return double
#define randgaussian() disNormal(engine) // normal distribution mean=0 and stddev=1 (use: mean+rand*stddev)


#endif /* EDQD_UTIL_H_ */
