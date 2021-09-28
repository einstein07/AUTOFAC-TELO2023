/*
 * @(#) Util.cpp   1.0  Sep 16, 2021
 *
 * Sindiso Mkhatshwa (mkhsin035@myuct.ac.za)
 *
 * Nitschke Laboratory, UCT
 *
 * @(#) $Id$
 */
#include "Util.h"
#include <string>
#include <random>

// random generator functions, header declaration in common.h (general scope)
std::random_device rnd;
std::minstd_rand randint;
std::mt19937 engine(rnd());
std::uniform_real_distribution<double> disRandom(0.0, 1.0);
std::normal_distribution<> disNormal(0,1);
