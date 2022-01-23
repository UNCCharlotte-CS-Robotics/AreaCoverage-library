/**
 * This file is part of the AreaCoverage-library.
 * This file contains utility functions for polygons.
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

#ifndef ACLIBRARY_POLYGON_UTILITIES_H_
#define ACLIBRARY_POLYGON_UTILITIES_H_

#include <string>
#include <fstream>
#include <cmath>
#include <vector>
#include <aclibrary/cgal_config.h>
#include <aclibrary/constants.h>
#include <CGAL/Triangular_expansion_visibility_2.h>
#include <CGAL/Rotational_sweep_visibility_2.h>

namespace aclibrary {

	typedef CGAL::Triangular_expansion_visibility_2<Arrangement_2>  TEV;
	typedef CGAL::Triangular_expansion_visibility_2<Arrangement_2, CGAL::Tag_false> NSPV;
	/* typedef CGAL::Rotational_sweep_visibility_2<Arrangement_2, CGAL::Tag_false> NSPV; */
	bool CheckLineEqualityExact(const Line_2 &l1, const Line_2 &l2);
	bool CheckLineEquality(const Line_2 &l1, const Line_2 &l2, const double eps = 0);
	bool CheckIfLineExists(const std::vector <Line_2> &cv, const Line_2 &l, const double eps = 0);

	bool IsReflex(const Segment_2 &e1, const Segment_2 &e2);
	bool RemoveCollinearVertices(Polygon_2 &p);
	void RemoveCollinearVertices(Polygon_with_holes_2 &p);
	bool IsAdjacent(const Polygon_with_holes_2 &p1, const Polygon_with_holes_2 &p2);
	Polygon_2 LineSegmentToPolygon (const Point_2 &u, const Point_2 &v, const Kernel::FT &offset, bool line_sweep = false, double buffer_eps = 0);
	void TracksToPWH(std::vector <std::vector <Point_2> > &tracks_vertices, std::vector <std::vector <std::pair<int, int> > > &parallel_tracks_edges, const Kernel::FT &offset, std::vector <Polygon_2> &poly_list);
	void TracksToPWH(const std::vector <Point_2> &vertices, const std::vector <std::pair<int, int>>  &edges, const Kernel::FT &offset, std::vector <Polygon_2> &poly_list, bool line_sweep = false);

	inline void PrintVector (const std::vector <double> &values, std::ofstream &outfile) {
		for(const auto v:values) {
			outfile << v << " ";
		}
		outfile << std::endl;
	}

	void PrintPolygon(const Polygon_2 &polygon, std::ofstream &outfile);

	int GetRayEdgeIntersect( const Ray_2 &ray, const Segment_2 &edge, Point_2 &pt);
	int CheckEdgeEdgeIntersect( const Segment_2 &, const Segment_2 &);

	inline int GetLineEdgeIntersection(
			const Line_2 &line,
			const Segment_2 &edge,
			Point_2 &pt)
	{
		CGAL::cpp11::result_of<Intersect_2(Segment_2, Line_2)>::type
			result = intersection(edge, line);
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

	template<class Arrangement>
		void CCBTraversal (typename Arrangement::Ccb_halfedge_const_circulator circ, Polygon_2 &polygon) {
			polygon.clear();
			typename Arrangement::Ccb_halfedge_const_circulator curr = circ;
			typename Arrangement::Halfedge_const_handle he;

			auto pt = curr->source()->point();
			do {
				he = curr;
				pt = he->target()->point();
				polygon.push_back(pt);
				++curr;
			} while (curr != circ);

			return;
		}


	template<class Arrangement>
		void GeneratePolygons (const Arrangement& arr, std::vector <Polygon_2> &polygon_list) {

			CGAL_precondition (arr.is_valid());
			typename Arrangement::Face_const_iterator    fit;
			for (fit = arr.faces_begin(); fit != arr.faces_end(); ++fit) {
				if (fit->is_unbounded()) {
					continue;
				}
				else {
					Polygon_2 polygon;
					CCBTraversal<Arrangement> (fit->outer_ccb(), polygon);
					polygon_list.push_back(polygon);
				}
			}
			return;
		}

	template<class Arrangement>
		void GeneratePolygons (const Arrangement& arr, std::vector <Polygon_with_holes_2> &polygon_list) {

			CGAL_precondition (arr.is_valid());
			typename Arrangement::Face_const_iterator    fit;
			for (fit = arr.faces_begin(); fit != arr.faces_end(); ++fit) {
				if (fit->is_unbounded()) {
					continue;
				}
				else {
					Polygon_2 polygon;
					std::vector <Polygon_2> holes;
					CCBTraversal<Arrangement> (fit->outer_ccb(), polygon);
					for (auto hi = fit->holes_begin(); hi != fit->holes_end(); ++hi) {
						Polygon_2 h;
						CCBTraversal<Arrangement> (*hi, h);
						holes.push_back(h);
					}
					if(holes.size() > 0)
						polygon_list.push_back(Polygon_with_holes_2(polygon, holes.begin(), holes.end()));
					else
						polygon_list.push_back(Polygon_with_holes_2(polygon));
				}
			}
			return;
		}

	inline void RemoveCollinearVerticesFromPolygon(Polygon_2 &poly) {

		std::vector<Polygon_2::Vertex_circulator> pointer_vertices;

		auto circ = poly.vertices_circulator();

		bool flag = false;
		do {
			if (CGAL::collinear(*std::prev(circ), *circ, *std::next(circ))) {
				pointer_vertices.push_back(circ);
			}
		} while (++circ != poly.vertices_circulator());

		for (auto it = pointer_vertices.rbegin(); it != pointer_vertices.rend(); ++it) {
			poly.erase(*it);
			flag = true;
		}
		if(flag) {
			RemoveCollinearVerticesFromPolygon(poly);
		}
	}

	inline void RemoveCollinearVerticesFromPolygon(PolygonWithHoles &pwh) {
		RemoveCollinearVerticesFromPolygon(pwh.outer_boundary());
		for (auto h_it = pwh.holes_begin(); h_it != pwh.holes_end(); ++h_it)
			RemoveCollinearVerticesFromPolygon(*h_it);
	}

	inline bool ComputeVisibilityRegion(const Arrangement_2 &arr, const Point_2 &q, Polygon_2 &visibility_polygon) {
		auto main_face = arr.faces_begin();
		while (main_face->is_unbounded() and main_face != arr.faces_end()) {
			main_face++;
		}
		if(main_face == arr.faces_end()) {
			std::cerr << "No main face\n";
		}

		typename Arrangement_2::Vertex_const_handle vertex_handle_query;
		typename Arrangement_2::Halfedge_const_handle  edge_handle_query;
		typename Arrangement_2::Face_const_handle      face_handle_query;
		// compute non regularized visibility area
		// Define visibiliy object type that computes non-regularized visibility area
		Arrangement_2 non_regular_output;
		NSPV non_regular_visibility(arr);
		// find the face of the query point
		/* Arrangement_2::Face_const_handle * face; */
		CGAL::Arr_naive_point_location<Arrangement_2> pl(arr);
		CGAL::Arr_point_location_result<Arrangement_2>::Type obj = pl.locate(q);
		// The query point locates in the interior of a face

		Arrangement_2::Face_handle face_handle_visibility;
		bool found_query = false;

		if(CGAL::assign(face_handle_query, obj)) {
			if(face_handle_query->is_unbounded()) {
				/* std::cout << "Unbounded face: " << q << std::endl; */
			} else {
				face_handle_visibility = non_regular_visibility.compute_visibility(q, face_handle_query, non_regular_output);
				found_query = true;
			}
		}
		if(found_query == false) {
			if(CGAL::assign(vertex_handle_query, obj)) {
				auto he = arr.halfedges_begin();
				while ((he->target()->point() != vertex_handle_query->point()) || (he->face() != main_face)) {
					he++;
					if (he == arr.halfedges_end()) {
						return true;
					}
				}
				face_handle_visibility = non_regular_visibility.compute_visibility(q, he, non_regular_output);
				found_query = true;

			}
		}
		if(found_query == false) {
			if(CGAL::assign(edge_handle_query, obj)) {
				auto he = edge_handle_query->face() == main_face ? edge_handle_query : edge_handle_query->twin();
				face_handle_visibility = non_regular_visibility.compute_visibility(q, he, non_regular_output);
				found_query = true;
			}
		}

		if(found_query == false) {
			return true;
		}

		if (face_handle_visibility->is_fictitious()) {
			return true;
		}
		if (face_handle_visibility->is_unbounded()) {
			return true;
		}

		auto start = face_handle_visibility->outer_ccb();
		auto curr = start;
		do {
			visibility_polygon.push_back(curr->source()->point());
		} while (++curr != start);

		return false;

	}


}

#endif /* ACLIBRARY_POLYGON_UTILITIES_H_ */
