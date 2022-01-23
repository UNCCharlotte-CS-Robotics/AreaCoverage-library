/**
 * This file is part of the AreaCoverage-library.
 * The file provides utility functions for polygons
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

#include <aclibrary/polygon_utilities.h>
namespace aclibrary {

	bool CheckIfLineExists(const std::vector <Line_2> &cv, const Line_2 &l, const double eps) {
		bool flag = false;
		for(auto &cv_line:cv) {
			if(CheckLineEquality(cv_line, l, eps)) {
				flag = true;
				break;
			}
		}
		return flag;
	}

	bool CheckLineEqualityExact(const Line_2 &l1, const Line_2 &l2) {
		if(l1 == l2 or l1 == l2.opposite()) {
			return true;
		} else {
			return false;
		}
	}
	bool CheckLineEquality(const Line_2 &l1, const Line_2 &l2, const double eps) {
		if(CheckLineEqualityExact(l1, l2))
			return true;
		if(eps == 0) {
			return false;
		}
		auto l1a = l1.a(); auto l1b = l1.b(); auto l1c = l1.c();
		auto l2a = l2.a(); auto l2b = l2.b(); auto l2c = l2.c();
		Line_2 l1p(-l1.a(), -l1.b(), -l1.c());
		Line_2 l2p(-l2.a(), -l2.b(), -l2.c());
		bool flag = ((l1 == l2) or (l1p == l2) or (l1 == l2p) or (l1p == l2p));
		if(flag)
			return flag;
		if(l1a == l2a  and l1b == l2b and l1c != l2c)
			return false;
		if((l1.a() == 0 or l1p.a() == 0) and (l2.a() != 0 and l2p.a() != 0))
			return false;
		if((l2.a() == 0 or l2p.a() == 0) and (l1.a() != 0 and l1p.a() != 0))
			return false;
		if((l1a == 0 or -l1a == 0) and (l2a == 0 or -l2a == 0)) {
			if(l1c/l1b == l2c/l2b)
				return true;
			else
				return false;
		}
		if(l1a < 0) {
			l1a = -l1a;
			l1b = -l1b;
			l1c = -l1c;
		}
		if(l2a < 0) {
			l2a = -l2a;
			l2b = -l2b;
			l2c = -l2c;
		}
		if((CGAL::abs(l1c/l1a - l2c/l2a) < eps) and (CGAL::abs(l1b/l1a - l2b/l2a) < eps))
			return true;
		return false;
	}

	bool IsReflex(const Segment_2 &e1, const Segment_2 &e2) {
		auto p1 = e1.source();
		auto p2 = e1.target();
		auto p3 = e2.target();
		Vector_2 v1(p2, p1);
		Vector_2 v2(p2, p3);
		auto cross = v2[0] * v1[1] - v2[1] * v1[0];
		if(cross <= 0)
			return true;
		return false;
	}

	bool RemoveCollinearVertices(Polygon_2 &p) {
		std::cout << "RCV called!!!!!!!!!!!!!!11\n";
		bool g_flag = false, flag = true;
		std::cout << "Inside RCV\n";
		std::cout << p << std::endl;
		while(flag) {
			flag = false;
			if(p.size() == 0) {
				std::cout << "End RCV1\n";
				return g_flag;
			}
			std::cout << "Size: " << p.size() << std::endl;
			auto v_it = p.vertices_circulator();
			for(size_t i = 0; i < p.size(); ++i) {
				while(CGAL::collinear(*v_it, *(std::next(v_it, 1)), *(std::next(v_it, 2)))) {
					flag = true; g_flag = true;
					std::cout << "Erase: " << *(std::next(v_it, 1)) << std::endl;
					v_it = p.erase(std::next(v_it));
					std::cout << "Erased\n";
					std::cout << *v_it << std::endl;
					std::advance(v_it, -1);
					std::cout << *v_it << std::endl;
					std::cout << "Next\n";
				}
				std::cout << "Advancing\n";
				std::advance(v_it, 1);
				std::cout << "Advanced\n";
			}
		}
		std::cout << "End RCV\n";
		return g_flag;
	}

	void RemoveCollinearVertices(Polygon_with_holes_2 &p) {
		bool flag = false;
		auto outer = p.outer_boundary();
		if(outer.size() == 0)
			return;
		flag = RemoveCollinearVertices(outer);
		bool holes_flag = false;
		if(p.holes_begin() != p.holes_end()) {
			std::vector <Polygon_2> new_holes;
			for(auto it = p.holes_begin(); it != p.holes_end(); ++it) {
				auto hole = *it;
				holes_flag =  holes_flag or RemoveCollinearVertices(hole);
				new_holes.push_back(hole);
			}
			if(flag or holes_flag) {
				p = Polygon_with_holes_2(outer, new_holes.begin(), new_holes.end());

			}
		}
		else {
			if(flag)
				p = Polygon_with_holes_2(outer);
		}
	}

	bool IsAdjacent(const Polygon_with_holes_2 &p1, const Polygon_with_holes_2 &p2) {
		const auto p1_outer = p1.outer_boundary();
		const auto p2_outer = p2.outer_boundary();
		for(auto edge_it1 = p1_outer.edges_begin(); edge_it1 != p1_outer.edges_end(); ++edge_it1) {
			auto e1 = *edge_it1;
			for(auto edge_it2 = p2_outer.edges_begin(); edge_it2 != p2_outer.edges_end(); ++edge_it2) {
				auto e2 = *edge_it2;
				if(e1 == e2 or e1.opposite() == e2)
					return true;
				if(CGAL::do_intersect(e1, e2))
					return true;
			}
		}
		return false;
	}

	void PrintPolygon( const Polygon_2 &polygon, std::ofstream &outfile){
		if(polygon.size() == 0) {
			return;
		}
		std::vector <double> x_values;
		std::vector <double> y_values;
		for(const auto &p:polygon) {
			x_values.push_back(CGAL::to_double(p.x()));
			y_values.push_back(CGAL::to_double(p.y()));
		}
		PrintVector(x_values, outfile);
		PrintVector(y_values, outfile);
	}

	int CheckEdgeEdgeIntersect(
			const Segment_2 &edge1,
			const Segment_2 &edge2) {
		CGAL::cpp11::result_of<Intersect_2(K::Segment_2, K::Segment_2)>::type
			result = intersection(edge1, edge2);
		if (result) {
			if (boost::get<Segment_2>(&*result)) {
				/* std::cout << *s << std::endl; */
				return 1;
			} else {
				/* std::cout << *p << std::endl; */
				return 0;
			}
		}
		return 2;
	}

	int GetRayEdgeIntersect(
			const Ray_2 &ray,
			const Segment_2 &edge,
			Point_2 &pt) {
		CGAL::cpp11::result_of<Intersect_2(K::Segment_2, K::Ray_2)>::type
			result = intersection(edge, ray);
		if (result) {
			if (boost::get<Segment_2>(&*result)) {
				/* std::cout << *s << std::endl; */
				return 1;
			} else {
				const Point_2* p = boost::get<Point_2 >(&*result);
				/* std::cout << *p << std::endl; */
				pt = *p;
				return 0;
			}
		}
		return 2;
	}

	int GetLineLineIntersect(
			const Line_2 &l1,
			const Line_2 &l2,
			Point_2 &pt) {
		const auto result = intersection(l1, l2);
		if (result) {
			auto p = boost::get<Point_2>(&*result);
			if (p != nullptr) {
				pt = *p;
				return 0;
			}
		}
		return 1;
	}

	void TracksToPWH(std::vector <std::vector <Point_2> > &tracks_vertices, std::vector <std::vector <std::pair<int, int> > > &parallel_tracks_edges, const Kernel::FT &offset, std::vector <Polygon_2> &poly_list) {
		for(size_t i = 0; i < parallel_tracks_edges.size(); ++i) {
			auto tracks = parallel_tracks_edges[i];
			auto vertices = tracks_vertices[i];
			for(size_t j = 0; j < tracks.size(); ++j) {
				auto vertex_u = vertices[std::get<0>(tracks[j])];
				auto vertex_v = vertices[std::get<1>(tracks[j])];
				poly_list.push_back(LineSegmentToPolygon(vertex_u, vertex_v, offset));
			}
		}
		std::list<Polygon_with_holes_2> res;
		CGAL::join (poly_list.begin(), poly_list.end(), std::back_inserter (res));
	}

	void TracksToPWH(const std::vector <Point_2> &vertices, const std::vector <std::pair<int, int> > &edges, const Kernel::FT &offset, std::vector <Polygon_2> &poly_list, bool line_sweep) {
		for(const auto &edge:edges) {
			auto vertex_u = vertices[std::get<0>(edge)];
			auto vertex_v = vertices[std::get<1>(edge)];
			poly_list.push_back(LineSegmentToPolygon(vertex_u, vertex_v, offset, line_sweep));
		}
	}

	Polygon_2 LineSegmentToPolygon (const Point_2 &u, const Point_2 &v, const Kernel::FT &offset, bool line_sweep, double buffer_eps) {
		Line_2 l(u, v);
		Segment_2 ls(u, v);
		auto a = l.a();
		auto b = l.b();
		auto c = l.c();
		double d = CGAL::sqrt(CGAL::to_double(a * a + b * b)) + buffer_eps;
		Line_2 l1(a, b, c + offset/2 * d);
		Line_2 l2(a, b, c - offset/2 * d);
		auto mid = CGAL::midpoint(u, v);
		double length = CGAL::sqrt(CGAL::to_double(ls.squared_length()))/2.0;
		auto p = l.perpendicular(mid);
		a = p.a();
		b = p.b();
		c = p.c();
		d = CGAL::sqrt(CGAL::to_double(a * a + b * b));
		Line_2 p1, p2;
		if(line_sweep == false) {
			p1 = Line_2(a, b, c + (length + offset/2) * d);
			p2 = Line_2(a, b, c - (length + offset/2) * d);
		} else {
			p1 = Line_2(a, b, c + length * d);
			p2 = Line_2(a, b, c - length * d);
		}
		std::array <Point_2, 4> pt_intersection;
		GetLineLineIntersect(l1, p1, pt_intersection[0]);
		GetLineLineIntersect(p1, l2, pt_intersection[1]);
		GetLineLineIntersect(l2, p2, pt_intersection[2]);
		GetLineLineIntersect(p2, l1, pt_intersection[3]);
		Polygon_2 pgn(pt_intersection.begin(), pt_intersection.end());
		if(pgn.orientation() == CGAL::COUNTERCLOCKWISE) {
			pgn.reverse_orientation();
		}
		return pgn;
		/* Arrangement_2 arr; */
		/* CGAL::insert(arr, l1); CGAL::insert(arr, l2); */
		/* CGAL::insert(arr, p1); CGAL::insert(arr, p2); */
		/* std::vector <Polygon_2> polygon_list; */
		/* GeneratePolygons(arr, polygon_list); */
		/* return polygon_list[0]; */
	}

}
