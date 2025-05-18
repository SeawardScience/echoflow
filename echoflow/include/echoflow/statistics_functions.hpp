#pragma once 

#include <cmath>
#include "package_defs.hpp"

NS_HEAD

/**
 * @brief Compute the circular mean angle on a sample of angle observations. \f$\overline{\alpha}= \textrm{atan2}\biggl(\sum_{j=1}^{n} \sin \alpha_j, \sum_{j=1}^{n} \cos \alpha_j) \biggr)\f$
 * 
 * Function is implemented using 
 * 
 * 
 */
void computeCircularMean();

NS_FOOT
