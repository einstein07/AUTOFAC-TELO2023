/*
 * @(#) Resource.h   1.0   Jan 09, 2021
 *
 * Sindiso Mkhatshwa (mkhsin035@myuct.ac.za)
 *
 * @(#) $Id$
 */
#ifndef ROBOGEN_RESOURCE_H_
#define ROBOGEN_RESOURCE_H_

#include "model/PositionObservable.h"

namespace robogen {

class Resource : public PositionObservable {

		/**
		 * Remove from world
		 */
		virtual void remove() = 0;

};

}


#endif /* ROBOGEN_RESOURCE_H_ */
