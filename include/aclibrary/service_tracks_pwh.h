/**
 * This file is part of the AreaCoverage-library.
 * This file contains class for computing service tracks for a polygon with holes (cells)
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

#ifndef ACLIBRARY_SERVICE_TRACKS_PWH_H_
#define ACLIBRARY_SERVICE_TRACKS_PWH_H_

#include <string>
#include <cmath>
#include <list>
#include <algorithm>
#include <aclibrary/cgal_config.h>

namespace aclibrary {

	struct Intersection {
		Point_2 pt;
		bool type;
		Intersection(const Point_2 &pt_in, const bool type_in) : pt(pt_in), type(type_in) {}
	};

	struct EdgeEvent {
		Point_2 source_vertex, target_vertex;
		Segment_2 edge;
		Point_2 curr_vertex;
		Line_2 longitudinal_dir;
		CGAL::Oriented_side orientation;
		bool is_serviced = false;
	};

	struct CmpSource {
		bool operator()(const EdgeEvent &lhs, const EdgeEvent &rhs) const {
			if(lhs.source_vertex == rhs.source_vertex) {
				return CGAL::has_larger_signed_distance_to_line(lhs.longitudinal_dir, lhs.target_vertex, rhs.target_vertex);
			}
			return CGAL::has_larger_signed_distance_to_line(lhs.longitudinal_dir, lhs.source_vertex, rhs.source_vertex);
		}
	};
	struct CmpTarget {
		bool operator()(const EdgeEvent &lhs, const EdgeEvent &rhs) const {
			return CGAL::has_larger_signed_distance_to_line(rhs.longitudinal_dir, lhs.target_vertex, rhs.target_vertex);
		}
	};

	template <class T, class Cmp>
		class Events {
			public:
				std::vector <T> vec_;
				Events() { }
				Events(std::vector <T> &vec) {
					vec_ = vec;
					std::make_heap(vec_.begin(), vec_.end(), Cmp());
				}

				void MakeHeap() {
					std::make_heap(vec_.begin(), vec_.end(), Cmp());
				}

				void Push(T e) {
					vec_.push_back(e);
					std::push_heap(vec_.begin(), vec_.end(), Cmp());
				}

				int Pop() {
					if (vec_.size() < 0)
						return 1;
					std::pop_heap(vec_.begin(), vec_.end(), Cmp());
					vec_.pop_back();
					return 0;
				}

				T Top() {
					return vec_[0];
				}

				int Top(T &top) {
					if(vec_.size() > 0) {
						top = vec_[0];
						return 0;
					}
					return 1;
				}

				bool Empty() {
					if(vec_.size() == 0)
						return true;
					else
						return false;
				}

				size_t Size() {
					return vec_.size();
				}
		};

	class ServiceTracksPWH {
		private:
			Polygon_with_holes_2 pwh_;
			Line_2 longitudinal_dir_;
			Kernel::FT offset_;
			bool linesweep_ = true;
			K::FT offset_sign_ = 1;
			Line_2 support_line_, curr_line_;
			Events <EdgeEvent, CmpSource> events_;
			Events <EdgeEvent, CmpTarget> curr_events_;
			std::vector <Segment_2> service_tracks_;
			size_t previous_service_tracks_idx_ = 0, curr_service_tracks_idx_ = 0;
			std::vector <Segment_2> extra_edges_;
			bool add_obs_edges_;
			Arrangement_2 arr_;
			Polygon_set_2 coverage_set_;
		public:
			ServiceTracksPWH(Polygon_with_holes_2 pwh, Line_2 longitudinal_dir, Kernel::FT offset, bool add_obs_edges) :
				pwh_{pwh},
				longitudinal_dir_{longitudinal_dir},
				offset_{offset},
				add_obs_edges_{add_obs_edges}{ }

			ServiceTracksPWH(Polygon_with_holes_2 pwh, Line_2 longitudinal_dir, Kernel::FT offset, bool linesweep, bool add_obs_edges) :
				pwh_{pwh},
				longitudinal_dir_{longitudinal_dir},
				offset_{offset}, linesweep_{linesweep},
				add_obs_edges_{add_obs_edges}{ }

			void Solve() {
				CGAL::insert (arr_, pwh_.outer_boundary().edges_begin(), pwh_.outer_boundary().edges_end());
				for(auto h_it = pwh_.holes_begin(); h_it != pwh_.holes_end(); ++h_it) {
					CGAL::insert (arr_, h_it->edges_begin(), h_it->edges_end());
				}
				GetEdgeEvents();
				GenerateTracks();
			}

			void GetEdgeEvents(const Polygon_2 &poly) {
				std::vector <EdgeEvent> events_vec;
				for(auto e_it = poly.edges_begin(); e_it != poly.edges_end(); ++e_it) {
					EdgeEvent edge_event;
					auto edge = *e_it;
					if(CGAL::has_smaller_signed_distance_to_line(longitudinal_dir_, edge.target(), edge.source())) { edge = edge.opposite(); }
					edge_event.edge = edge;
					edge_event.source_vertex = edge.source();
					edge_event.target_vertex = edge.target();
					edge_event.curr_vertex = edge.source();
					edge_event.longitudinal_dir = longitudinal_dir_;
					events_.Push(edge_event);
				}
			}

			void GetEdgeEvents() {
				GetEdgeEvents(pwh_.outer_boundary());
				for(auto h_it = pwh_.holes_begin(); h_it != pwh_.holes_end(); ++h_it) {
					GetEdgeEvents(*h_it);
				}
				EdgeEvent first_event;
				events_.Top(first_event);
				support_line_ = Line_2(first_event.source_vertex, longitudinal_dir_.to_vector());
				curr_line_ = support_line_;
				AddOrientation();
			}

			void PrintEdgeEvents() {
				std::cout << "Number of edge events: " << events_.Size() << "\n";
				std::cout << "Longitudinal_dir" << longitudinal_dir_ << "\n";
				auto q = events_;
				while(!q.Empty()) {
					auto event = q.Top();
					std::cout << event.edge << " " << event.orientation << '\n';
					q.Pop();
				}
				std::cout << '\n';
			}

			void AddOrientation() {
				for(auto &event:events_.vec_) {
					event.orientation = support_line_.oriented_side(event.source_vertex);
					if(event.orientation == CGAL::ON_ORIENTED_BOUNDARY) {
						event.orientation = support_line_.oriented_side(event.target_vertex);
					}
				}
			}

			bool CheckIfServiceRequired(const EdgeEvent &event) {
				/* if(add_obs_edges_ == true) { */
				/* 	return true; */
				/* } */
				double x1 = CGAL::to_double(event.source_vertex.x());
				double y1 = CGAL::to_double(event.source_vertex.y());
				double x2 = CGAL::to_double(event.target_vertex.x());
				double y2 = CGAL::to_double(event.target_vertex.y());
				double v1_x = x2 - x1; double v1_y = y2 - y1;
				auto vec = longitudinal_dir_.to_vector().perpendicular(CGAL::POSITIVE);
				double v2_x = CGAL::to_double(vec.x()); double v2_y = CGAL::to_double(vec.y());
				double angle = acos((v1_x * v2_x + v1_y * v2_y) / ( sqrt(v1_x*v1_x + v1_y*v1_y) * sqrt(v2_x*v2_x + v2_y*v2_y)));
				/* double angle = atan((v1_y - v2_y)/(v1_x - v2_x)); */
				if(add_obs_edges_ == true) {
					if(not(std::abs(angle) < 1e-3 )) {
						return true;
					}
					else {return false;}
				}
				if(not(angle >= -M_PI/4. and angle <= M_PI/4.)) {
					return true;
				}
				else {return false;}

			}
			void AddCurrEvents() {
				while(!events_.Empty()) {
					auto event = events_.Top();
					/* std::cout << "event: " << event.edge << std::endl; */

					if(event.orientation != curr_line_.oriented_side(event.source_vertex) and event.orientation != curr_line_.oriented_side(event.target_vertex)) {
						extra_edges_.push_back(event.edge);
						events_.Pop();
						continue;
					}

					if(event.orientation != curr_line_.oriented_side(event.source_vertex)) {
						curr_events_.Push(event);
						events_.Pop();
						if(CheckIfServiceRequired(event)) {
							/* std::cout << "Edge angled: " << event.edge << std::endl; */
							service_tracks_.push_back(event.edge);
						}
					}
					else {break;}
				}
			}

			void DeleteEvents() {
				while(!curr_events_.Empty()) {
					EdgeEvent event;
					curr_events_.Top(event);
					if(event.orientation != curr_line_.oriented_side(event.target_vertex)) {
						curr_events_.Pop();
					}
					else {break;}
				}
			}

			void AddServiceSegments() {
				if(curr_events_.Size() == 0) {
					return;
				}
				std::vector <Intersection> intersection_points;
				for(const auto &event:curr_events_.vec_) {
					if(curr_line_.has_on(event.source_vertex)) {
						intersection_points.push_back(Intersection(event.source_vertex, true));
					}

					if(curr_line_.has_on(event.target_vertex)) {
						intersection_points.push_back(Intersection(event.target_vertex, true));
					}

					Point_2 pt;
					if(GetLineEdgeIntersection(curr_line_, event.edge, pt)) {
						continue;
					}
					intersection_points.push_back(Intersection(pt, false));
				}
				if(intersection_points.size() == 0)
					return;

				Line_2 perp_line = curr_line_.perpendicular(Point_2(0,0));
				std::sort(intersection_points.begin(), intersection_points.end(), [&perp_line](const Intersection &lhs, const Intersection &rhs) { return CGAL::has_smaller_signed_distance_to_line(perp_line, lhs.pt, rhs.pt); });

				bool add = true;
				for(size_t i = 0; i < intersection_points.size() - 1; ++i) {
					/* if(intersection_points[i].type) { */
					/* 	add = not add; */
					/* 	/1* continue; *1/ */
					/* } */
					if(add) {
						if(intersection_points[i].pt == intersection_points[i+1].pt) {
							continue;
						}

						Polygon_2 visibility_polygon;
						if(ComputeVisibilityRegion(arr_, intersection_points[i].pt, visibility_polygon) == true) {
							continue;
						}
						if(CGAL::bounded_side_2(visibility_polygon.begin(), visibility_polygon.end(), intersection_points[i+1].pt) ==  CGAL::ON_UNBOUNDED_SIDE) {
							continue;
						}
						Segment_2 service_track(intersection_points[i].pt, intersection_points[i + 1].pt);
						/* std::cout << "Intersection edge: " << service_track << std::endl; */
						service_tracks_.push_back(service_track);
					}
					/* ++i; */
					add = not add;
				}
			}

			void AddExtraEdgesSimple() {
				for(const auto &edge:extra_edges_) {
					/* std::cout << "Extra edge: " << edge << std::endl; */
					bool is_covered = false;
					for(size_t i = previous_service_tracks_idx_; i < service_tracks_.size(); ++i) {
						auto poly = LineSegmentToPolygon(service_tracks_[i].source(), service_tracks_[i].target(), offset_, linesweep_);
						if(CGAL::bounded_side_2(poly.begin(), poly.end(), edge.source(), K()) == CGAL::ON_UNBOUNDED_SIDE) {
							continue;
						}
						if(CGAL::bounded_side_2(poly.begin(), poly.end(), edge.target(), K()) == CGAL::ON_UNBOUNDED_SIDE) {
							continue;
						}
						is_covered = true;
						break;
					}
					if(not is_covered) {
						/* std::cout << "Almost parallel: " << edge << std::endl; */
						service_tracks_.push_back(edge);
					}
				}
				extra_edges_.clear();
				previous_service_tracks_idx_ = curr_service_tracks_idx_;
				curr_service_tracks_idx_ = service_tracks_.size();
			}
			void AddExtraEdges() {
				/* std::vector <Polygon_2> poly_list; */
				/* for(const auto &ls:service_tracks_) { */
				/* 	poly_list.push_back(LineSegmentToPolygon(ls.source(), ls.target(), offset_, linesweep_, 1e-10)); */
				/* } */
				/* std::vector<Polygon_with_holes_2> res; */
				/* CGAL::join(poly_list.begin(), poly_list.end(), std::back_inserter (res)); */

				for(size_t i = curr_service_tracks_idx_; i < service_tracks_.size(); ++i) {
					auto ls = service_tracks_[i];
					Polygon_2 poly = LineSegmentToPolygon(ls.source(), ls.target(), offset_, linesweep_, 1e-8);
					coverage_set_.join(poly);
				}
				curr_service_tracks_idx_ = service_tracks_.size();
				std::vector<Polygon_with_holes_2> res;
				coverage_set_.polygons_with_holes (std::back_inserter (res));
				std::vector <Arrangement_2> arr_list;
				for(const auto &pwh:res) {
					Arrangement_2 arr;
					CGAL::insert (arr, pwh.outer_boundary().edges_begin(), pwh.outer_boundary().edges_end());
					for(auto h_it = pwh.holes_begin(); h_it != pwh.holes_end(); ++h_it) {
						CGAL::insert (arr, h_it->edges_begin(), h_it->edges_end());
					}
					arr_list.push_back(arr);
				}
				for(const auto &edge:extra_edges_) {
					bool found_flag = false;
					for(size_t i = 0; i < arr_list.size(); ++i) {
						/* if(CGAL::oriented_side(edge.source(), res[i]) == CGAL::NEGATIVE or CGAL::oriented_side(edge.target(), res[i]) == CGAL::NEGATIVE) { */
						/* 	continue; */
						/* } */
						Polygon_2 visibility_polygon;
						if(ComputeVisibilityRegion(arr_list[i], edge.source(), visibility_polygon) == true) {
							continue;
						}
						if(CGAL::bounded_side_2(visibility_polygon.begin(), visibility_polygon.end(), edge.target()) ==  CGAL::ON_UNBOUNDED_SIDE) {
							continue;
						}
						found_flag = true;
						break;
					}
					if(found_flag == false) {
						service_tracks_.push_back(edge);

					}

				}
				extra_edges_.clear();
			}

			void GenerateTracks() {
				/* std::cout << "Long: " << support_line_ << std::endl; */
				previous_service_tracks_idx_ = curr_service_tracks_idx_ = 0;
				auto a = support_line_.a();
				auto b = support_line_.b();
				auto c = support_line_.c();
				K::FT d = CGAL::sqrt(CGAL::to_double(a * a + b * b));
				EdgeEvent key_event;
				key_event = events_.Top();

				offset_sign_ = 1;
				K::FT offset_count = 1; offset_count = offset_count/2;
				curr_line_ = Line_2(a, b, c + K::FT(offset_sign_ * offset_count * offset_ * d));
				if(key_event.orientation != support_line_.oriented_side(curr_line_.point())) {
					offset_sign_ = -1;
					curr_line_ = Line_2(a, b, c + K::FT(offset_sign_ * offset_count * offset_ * d));
				}

				do {
					offset_count += 1;
					/* previous_service_tracks_idx_ = curr_service_tracks_idx_; */
					/* curr_service_tracks_idx_ = service_tracks_.size(); */
					AddCurrEvents();
					DeleteEvents();
					AddServiceSegments();
					if(add_obs_edges_ == false) {
						AddExtraEdges();
					} else {
						AddExtraEdgesSimple();
					}
					curr_line_ = Line_2(a, b, c + K::FT(offset_sign_ * offset_count * offset_ * d));
				} while(!curr_events_.Empty());
				/* if(add_obs_edges_ == true) { */
				/* 	AddExtraEdges(); */
				/* } */

				/* AddExtraEdges(); */
			}

			void GetTracks(std::vector<Point_2> &vertices, std::vector<std::pair<int , int>> &edges, Kernel::FT discretize_length = 0) {

				for(const auto &p:pwh_.outer_boundary()) {
					vertices.push_back(p);
				}
				for(auto h_it = pwh_.holes_begin(); h_it != pwh_.holes_end(); ++h_it) {
					for(const auto &p:*h_it) {
						vertices.push_back(p);
					}
				}
				double disc_len = CGAL::to_double(discretize_length);
				for(const auto &track:service_tracks_) {
					size_t source_idx = vertices.size();
					size_t target_idx = vertices.size() + 1;
					bool source_found = false;
					bool target_found = false;
					auto source_vertex = track.source();
					auto target_vertex = track.target();
					for(size_t i = 0; i < vertices.size(); ++i) {
						if((not source_found) and vertices[i] == source_vertex) {
							source_found = true;
							source_idx = i;
						}
						if((not target_found) and vertices[i] == target_vertex) {
							target_found = true;
							target_idx = i;
						}
						if(source_found and target_found)
							break;
					}
					if(source_found == false) {
						vertices.push_back(source_vertex);
					}
					if(target_found == false) {
						vertices.push_back(target_vertex);
					}
					if(discretize_length == 0 or track.squared_length() <= discretize_length * discretize_length) {
						edges.push_back(std::pair<int, int> (source_idx, target_idx));
					} else {
						auto diff_vec = target_vertex - source_vertex;
						double dist = sqrt(CGAL::to_double(track.squared_length()));
						auto curr_len = disc_len;

						vertices.push_back(source_vertex);
						auto prev_idx = source_idx;
						while(curr_len < dist) {
							auto next_vertex = source_vertex + diff_vec * curr_len/dist;
							vertices.push_back(next_vertex);
							edges.push_back(std::pair<int, int> (prev_idx, vertices.size() - 1));
							prev_idx = vertices.size() - 1;
							curr_len += disc_len;
						}
						edges.push_back(std::pair<int, int> (prev_idx, target_idx));
					}
				}
			}
	};
}
#endif /* ACLIBRARY_SERVICE_TRACKS_PWH_H_ */
