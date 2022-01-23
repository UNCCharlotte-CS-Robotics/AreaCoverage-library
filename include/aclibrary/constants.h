/**
 * This file is part of the AreaCoverage-library.
 * The file provides constants
 *
 * TODO:
 *
 * @author Saurav Agarwal
 * @contact sagarw10@uncc.edu
 * @contact agr.saurav1@gmail.com
 * Repository: https://github.com/UNCCharlotte-Robotics/AreaCoverage-library
 *
 * Copyright (C) 2020--2022 University of North Carolina at Charlotte.
 * The AreaCoverage-library is owned by the University of North Carolina at Charlotte and is protected by United States copyright laws and applicable international treaties and/or conventions.
 *
 * The AreaCoverage-library is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
 *
 * DISCLAIMER OF WARRANTIES: THE SOFTWARE IS PROVIDED "AS-IS" WITHOUT WARRANTY OF ANY KIND INCLUDING ANY WARRANTIES OF PERFORMANCE OR MERCHANTABILITY OR FITNESS FOR A PARTICULAR USE OR PURPOSE OR OF NON-INFRINGEMENT. YOU BEAR ALL RISK RELATING TO QUALITY AND PERFORMANCE OF THE SOFTWARE OR HARDWARE.
 *
 * SUPPORT AND MAINTENANCE: No support, installation, or training is provided.
 *
 * You should have received a copy of the GNU General Public License along with AreaCoverage-library. If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef ACLIBRARY_CORE_CONSTANTS_H_
#define ACLIBRARY_CORE_CONSTANTS_H_

#include <cstddef>
#include <limits>
#include <aclibrary/cgal_config.h>

namespace aclibrary {
	const double kEps = 1e-12;
	const double kDoubleMax = std::numeric_limits<double>::max();
	const double kDoubleMin = std::numeric_limits<double>::min();
	const size_t kNIL = std::numeric_limits<std::size_t>::max();
	const bool kIsRequired = true;
	const bool kIsNotRequired = false;
	const bool kSuccess = false;
	const bool kFail = true;
	const Point_2 kOrigin(0,0);
}

#endif /* ACLIBRARY_CORE_CONSTANTS_H_ */
