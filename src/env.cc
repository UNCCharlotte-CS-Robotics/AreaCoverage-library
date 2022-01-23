/**
 * This file is part of the AreaCoverage-library.
 * The file contains function definitions for Env class
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
#include <aclibrary/env.h>

namespace aclibrary {


	bool Env::CheckPointInHoles (const Point_2 &p) const {
		for(const auto &h:holes_) {
			if(CGAL::bounded_side_2(h.begin(), h.end(), p) == CGAL::ON_BOUNDED_SIDE) {
				return true;
			}
		}
		return false;
	}

	bool Env::CheckPointOnHoles (const Point_2 &p) const {
		for(const auto &h:holes_) {
			if(CGAL::bounded_side_2(h.begin(), h.end(), p) == CGAL::ON_BOUNDARY) {
				return true;
			}
		}
		return false;
	}

	bool Env::CheckPointOnHole (const Polygon_2 &h, const Point_2 &p) const {
		if(CGAL::bounded_side_2(h.begin(), h.end(), p) == CGAL::ON_BOUNDARY) {
			return true;
		}
		return false;
	}

	bool Env::CheckPointOnHole (size_t i, const Point_2 &p) const {
		auto h = holes_[i];
		return CheckPointOnHole(h, p);
	}

	bool Env::CheckIntersectsHoleEdge(const Point_2 &u, const Point_2 &v) const {
		Segment_2 l(u, v);
		for(const auto &h:holes_) {
			int count = 0;
			for(auto e_it = h.edges_begin(); e_it != h.edges_end(); ++e_it) {
				if(parallel(*e_it, l))
					continue;
				if (do_intersect(*e_it, l) or CheckPointOnHole(h, u) or CheckPointOnHole(h, v) or CGAL::squared_distance(*e_it, u) < 1e-5 or CGAL::squared_distance(*e_it, v) < 1e-5)
					++count;
				if(count > 1)
					return true;
			}
		}
		return false;
	}

	void Env::ComputeFreeSpace() {

		std::list <Polygon_with_holes_2> sum_list;
		for(auto hole:holes_) {
			/* print_polygon(hole); */
			if(CGAL::orientation_2(hole.begin(), hole.end()) == CGAL::CLOCKWISE) {
				hole.reverse_orientation();
			}
			sum_list.push_back(CGAL::minkowski_sum_2(robot_image_, hole));
		}
		std::vector <Polygon_with_holes_2> obstacles;
		CGAL::join(sum_list.begin(), sum_list.end(), std::back_inserter(obstacles));
		holes_.clear();
		for(auto &obstacle:obstacles) {
			holes_.push_back(obstacle.outer_boundary());
		}
		for(auto &hole:holes_) {
			if(CGAL::orientation_2(hole.begin(), hole.end()) == CGAL::COUNTERCLOCKWISE)
				hole.reverse_orientation();
		}
		holes_off_ = holes_;
		auto inner_offset_polygons = CGAL::create_interior_skeleton_and_offset_polygons_2(CGAL::to_double(offset_)/2.,outer_polygon_);
		outer_polygon_input_ = outer_polygon_;
		auto new_poly = *(inner_offset_polygons[0]);
		outer_polygon_off_.clear();
		for(const auto &p:new_poly) {
			outer_polygon_off_.push_back(Point_2(p.x(), p.y()));
		}
		outer_polygon_ = outer_polygon_off_;
	}

}
