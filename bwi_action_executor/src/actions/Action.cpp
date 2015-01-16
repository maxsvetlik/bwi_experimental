#include "Action.h"

#include <sstream>

using namespace std;

namespace bwi_actexec {
	
	
std::string Action::toASP(unsigned int timeStep) const {
	
	stringstream nameS;
	
	nameS << this->getName() << "(";

	for(int i=0, size=this->getParameters().size(); i<size ; ++i)
		nameS << this->getParameters()[i] << ",";

	nameS << timeStep <<")";
	
	return nameS.str();
	
}
	
}