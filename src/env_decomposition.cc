/**
 * This file is part of the AreaCoverage-library.
 * The file contains definitions for decomposition related functions in the Env class
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
#include <polygon_coverage_geometry/decomposition.h>
#include <polygon_coverage_geometry/bcd.h>
#include <polygon_coverage_geometry/sweep.h>

namespace aclibrary {

	/* void Env::ComputeArrangement() { */
	/* 	std::vector <Line_2> cv; */
	/* 	std::vector <Segment_2> cv1; */
	/* 	CGAL::insert (arr_, outer_polygon_.edges_begin(), outer_polygon_.edges_end()); */
	/* 	AddNonConvexLines(outer_polygon_, cv); */
	/* 	for(const auto &h:holes_) { */
	/* 		if(holes_[0].size() == 0) { */
	/* 			continue; */
	/* 		} */
	/* 		/1* for(auto e_it = h.edges_begin(); e_it != h.edges_end(); ++e_it) { *1/ */
	/* 		/1* 	if(not CheckIfLineExists(cv, e_it->supporting_line())) { *1/ */
	/* 		/1* 		cv.push_back(e_it->supporting_line()); *1/ */
	/* 		/1* 	} *1/ */
	/* 		/1* } *1/ */
	/* 		CGAL::insert (arr_, h.edges_begin(), h.edges_end()); */
	/* 		AddNonConvexLines(h, cv); */
	/* 	} */

	/* 	/1* AddBCDLines(cv); *1/ */
	/* 	/1* AddBCDLines(cv1); *1/ */
	/* 	CGAL::insert (arr_, cv.begin(), cv.end()); */
	/* 	CGAL::insert (arr_, cv1.begin(), cv1.end()); */
	/* 	GeneratePolygons(arr_, arr_decomposition_); */
	/* } */

	bool Env::CheckEdgeDirs(const std::vector<Direction_2> &dirs, const Direction_2 &dir) const {
		if((dir.dx() == 0 or dir.dx() == -0) and (dir.dy() == 0 or dir.dy() == -0))
			return 1;
		for(auto &d:dirs) {
			if(d == dir)
				return 1;
			if((dir.dx() == 0 or dir.dx() == -0) and (d.dx() == 0 or d.dx() == -0))
				return 1;
			if((dir.dy() == 0 or dir.dy() == -0) and (d.dy() == 0 or d.dy() == -0))
				return 1;
			if((dir.dx() == 0 or dir.dx() == -0))
				continue;
			if((d.dx() == 0 or d.dx() == -0))
				continue;
			if(dir.dy()/dir.dx() == d.dy()/d.dx())
				return 1;
		}
		return 0;
	}

	bool Env::CheckParallelLines(const std::vector<Line_2> &lines, const Line_2 &line) const{
		for(const auto &l:lines) {
			if(CGAL::parallel(l, line)) {
				return 1;
			}
		}
		return 0;
	}

	void Env::GetNonConvexVertices(const Polygon_2 &poly, std::vector <Point_2> &pts) const {
		auto edge_start = poly.edges_circulator();
		auto edge_curr = edge_start;
		do {
			if(IsReflex(*edge_curr, *(std::next(edge_curr)))) {
				pts.push_back(edge_curr->target());
			}
			++edge_curr;
		} while(edge_curr != edge_start);
	}

	void Env::GetNonConvexVertices(const Polygon_2 &poly, std::vector <std::array<Point_2,3>> &pts) const {
		auto edge_start = poly.edges_circulator();
		auto edge_curr = edge_start;
		do {
			if(IsReflex(*edge_curr, *(std::next(edge_curr)))) {
				std::array<Point_2, 3> pt_array;
				pt_array[0] = edge_curr->source();
				pt_array[1] = edge_curr->target();
				pt_array[2] = std::next(edge_curr)->source();
				pts.push_back(pt_array);
			}
			++edge_curr;
		} while(edge_curr != edge_start);
	}

	void Env::AddBCDLines (const Polygon_with_holes_2 &pwh, std::vector <Line_2> &cv) const {
		std::vector <std::array<double, 2>> cell_points;
		/* std::vector <Point_2> pts; */
		std::vector <std::array<Point_2, 3>> pts;
		GetNonConvexVertices(pwh.outer_boundary(), pts);
		/* for(auto it = pwh.holes_begin(); it != pwh.holes_end(); ++it) { */
		/* 	GetNonConvexVertices(*it, pts); */
		/* } */
		std::vector <Line_2> bcd_lines;
		for(auto e_it = pwh.outer_boundary().edges_begin(); e_it != pwh.outer_boundary().edges_end(); ++e_it ) {
			auto l = Line_2(*e_it);
			if(CheckParallelLines(bcd_lines, l) == 0) {
				bcd_lines.push_back(l);
			}
		}
		for(auto h = pwh.holes_begin(); h != pwh.holes_end(); ++h){
			for(auto e_it = h->edges_begin(); e_it != h->edges_end(); ++e_it ) {
				auto l = Line_2(*e_it);
				if(CheckParallelLines(bcd_lines, l) == 0) {
					bcd_lines.push_back(l);
				}
			}
		}
		for(auto &l: bcd_lines) {
			for(auto &pt:pts) {
				if(l.oriented_side(pt[0]) != l.oriented_side(pt[2])) {
					continue;
				}
				auto new_line = Line_2(pt[1], l.direction());
				if(not CheckIfLineExists(cv, new_line)) {
					cv.push_back(new_line);
				}
			}
		}
	}

	void Env::AddBCDLines (std::vector <Segment_2> &cv) const {
		for(auto e_it = outer_polygon_.edges_begin(); e_it != outer_polygon_.edges_end(); ++e_it) {
			auto supporting_line = e_it->supporting_line();
			auto direction = supporting_line.perpendicular(kOrigin).direction();
			std::vector<Polygon_2> bcd = polygon_coverage_planning::computeBCD(env_, direction);
			for(auto &p:bcd) {
				for(auto e_jt = p.edges_begin(); e_jt != p.edges_end(); ++e_jt) {
					auto sup_line = *e_jt;
					cv.push_back(sup_line);
				}
			}
		}
	}
	void Env::FindIntersectingSegmentUni(const Segment_2 &ls, Segment_2 &extension, const bool opposite) const {
		/* std::cout << "FIndInterSeg\n"; */
		/* std::cout << ls << std::endl; */
		Ray_2 ray;
		Point_2 u;
		if(opposite == false) {
			u = ls.target();
			ray = Ray_2(u, ls.to_vector());
		}
		else {
			u = ls.source();
			ray = Ray_2(u, ls.opposite().to_vector());
		}

		Point_2 intersection;
		K::FT dist = kDoubleMax;
		for(auto e_it = outer_polygon_.edges_begin(); e_it != outer_polygon_.edges_end(); ++e_it) {
			Point_2 p;
			if(not GetRayEdgeIntersect(ray, *e_it, p)) {
				auto d = CGAL::squared_distance(p, u);
				if(d < dist and d!=0) {
					dist = d;
					intersection = p;
				}
			}
		}
		for(const auto &h:holes_) {
			for(auto e_it = h.edges_begin(); e_it != h.edges_end(); ++e_it) {
				Point_2 p;
				if(not GetRayEdgeIntersect(ray, *e_it, p)) {
					auto d = CGAL::squared_distance(p, u);
					if(d < dist and d!=0) {
						dist = d;
						intersection = p;
					}
				}
			}
		}
		extension = Segment_2 (u, intersection);
		/* std::cout << extension << std::endl; */
	}


	void Env::FindIntersectingSegment(const Segment_2 &ls, Segment_2 &extension) const {
		Point_2 u = ls.source(); Point_2 v = ls.target();
		Ray_2 ray(v, ls.to_vector());
		Ray_2 ray_opposite(u, ls.opposite().to_vector());
		Point_2 intersection; Point_2 intersection_opposite;
		K::FT dist = kDoubleMax; K::FT dist_opposite = kDoubleMax;
		for(auto e_it = outer_polygon_.edges_begin(); e_it != outer_polygon_.edges_end(); ++e_it) {
			Point_2 p;
			if(not GetRayEdgeIntersect(ray, *e_it, p)) {
				auto d = CGAL::squared_distance(p, v);
				if(d < dist and d!=0) {
					dist = d;
					intersection = p;
				}
			}
			if(not GetRayEdgeIntersect(ray_opposite, *e_it, p)) {
				auto d = CGAL::squared_distance(p, u);
				if(d < dist_opposite and d != 0) {
					dist_opposite = d;
					intersection_opposite = p;
				}
			}
		}

		for(const auto &h:holes_) {
			for(auto e_it = h.edges_begin(); e_it != h.edges_end(); ++e_it) {
				Point_2 p;
				if(not GetRayEdgeIntersect(ray, *e_it, p)) {
					auto d = CGAL::squared_distance(p, v);
					if(d < dist and d!=0) {
						dist = d;
						intersection = p;
					}
				}
				if(not GetRayEdgeIntersect(ray_opposite, *e_it, p)) {
					auto d = CGAL::squared_distance(p, u);
					if(d < dist_opposite and d != 0) {
						dist_opposite = d;
						intersection_opposite = p;
					}
				}
			}
		}
		extension = Segment_2 (intersection, intersection_opposite);
		/* std::cout << "FIndInterSeg\n"; */
		/* std::cout << ls << std::endl; */
		/* std::cout << extension << std::endl; */
	}

	void Env::AddNonConvexLinesHoles(const Polygon_2 &poly, std::list <SegmentPair> &extension_lists) {
		auto edge_start = poly.edges_circulator();
		auto edge_curr = edge_start;
		do {
			if(IsReflex(*edge_curr, *(edge_curr + 1))) {
				Segment_2 extension1, extension2;
				FindIntersectingSegment(*edge_curr, extension1);
				/* FindIntersectingSegmentUni(*edge_curr, extension1); */
				FindIntersectingSegment(*(edge_curr + 1), extension2);
				/* FindIntersectingSegmentUni(*(edge_curr + 1), extension2, true); */
				extension_lists.push_back(SegmentPair(extension1, extension2));
			}
			++edge_curr;
		} while(edge_curr != edge_start);
	}

	void Env::AddNonConvexLines(const Polygon_2 &poly, std::vector <Line_2> &cv) const {
		auto edge_start = poly.edges_circulator();
		auto edge_curr = edge_start;
		do {
			if(IsReflex(*edge_curr, *(edge_curr + 1))) {
				if(not CheckIfLineExists(cv, edge_curr->supporting_line())) {
					cv.push_back(edge_curr->supporting_line());
				}
				if(not CheckIfLineExists(cv, (edge_curr + 1)->supporting_line())) {
					cv.push_back((edge_curr + 1)->supporting_line());
				}
			}
			++edge_curr;
		} while(edge_curr != edge_start);
	}

	void Env::AddNonConvexLines(const Polygon_2 &poly, std::vector <Segment_2> &cv, const bool uni) const {
		auto edge_start = poly.edges_circulator();
		auto edge_curr = edge_start;
		do {
			if(IsReflex(*edge_curr, *(edge_curr + 1))) {
				Segment_2 extension;
				if(uni == false)
					FindIntersectingSegment(*edge_curr, extension);
				else
					FindIntersectingSegmentUni(*edge_curr, extension);
				cv.push_back(extension);
				if(uni == false)
					FindIntersectingSegment(*(edge_curr + 1), extension);
				else
					FindIntersectingSegmentUni(*(edge_curr + 1), extension, true);
				cv.push_back(extension);
			}
			++edge_curr;
		} while(edge_curr != edge_start);
	}

	void Env::AddLinesFromEdges(const Polygon_2 &poly, std::vector <Line_2> &cv, std::vector <Segment_2> &cv1) const {
		for(auto e_it = poly.edges_begin(); e_it != poly.edges_end(); ++e_it) {
			cv1.push_back(*e_it);
			auto supporting_line = e_it->supporting_line();
			bool flag = false;
			for(auto &cv_line:cv) {
				if(CheckLineEquality(cv_line, supporting_line)) {
					flag = true;
					break;
				}
			}
			if(flag == false) {
				cv.push_back(supporting_line);
				cv1.push_back(*e_it);
			}
		}
	}


	double Env::ComputeAltitudeBCD(const Polygon_with_holes_2 &poly, std::vector<Polygon_2> &decomposition, std::vector<Line_2> &dirs) {
		double width;
		polygon_coverage_planning::computeBestBCDFromPolygonWithHoles(poly, decomposition, dirs, width);
		return width;
	}

	double Env::ComputeAltitudeBCD(std::vector<Polygon_2> &decomposition) {
		decomposition.clear();
		double width;
		polygon_coverage_planning::computeBestBCDFromPolygonWithHoles(env_, decomposition, cell_longitudinal_dirs_, width);
		return width;
	}

	double Env::ComputeAltitudeBCD(const Polygon_with_holes_2 &poly) {
		double width;
		std::vector<Polygon_2> decomposition;
		std::vector<Line_2> dirs;
		polygon_coverage_planning::computeBestBCDFromPolygonWithHoles(poly, decomposition, dirs, width);
		return width;
	}

}
