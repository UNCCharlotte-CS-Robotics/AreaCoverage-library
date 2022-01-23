/**
 * This file is part of the AreaCoverage-library.
 * The file contains class for computing service tracks of an environment
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

#ifndef ACLIBRARY_SERVICE_TRACKS_H_
#define ACLIBRARY_SERVICE_TRACKS_H_

#include <aclibrary/cgal_config.h>
#include <aclibrary/env.h>
#include <aclibrary/service_tracks_pwh.h>
#include <aclibrary/polygon_utilities.h>

namespace aclibrary {
	typedef std::pair<Point_2, Point_2> EdgePP;
	class ServiceTracks {
		private:
			Env env_;
			Kernel::FT offset_;
			bool line_sweep_;
			std::list <PWHwithDir> pwh_dir_list_;
			std::vector<Point_2> vertices_;
			std::vector<std::pair<int , int>> edges_;
			std::list<Segment_2> edge_list_;
			std::vector <ServiceTracksPWH> service_tracks_object_list_;
			double service_track_total_length_;
		public:
			ServiceTracks(const Env &env, const double offset, bool line_sweep) {
				offset_ = FT(offset);
				env_ = env;
				offset_ = offset;
				line_sweep_ = line_sweep;
				env_.GetPWHwithDirList(pwh_dir_list_);
				for(const auto &pwh_dir:pwh_dir_list_) {
					ServiceTracksPWH service_tracks_pwh(pwh_dir.pwh, pwh_dir.dir, offset_, line_sweep_, env_.HasRobot());
					service_tracks_pwh.Solve();
					service_tracks_pwh.GetTracks(vertices_, edges_);
					service_tracks_object_list_.push_back(service_tracks_pwh);
				}
				GenerateEdgeList();
				FilterEdges();
				GenerateVerticesEdges();
				/* ComputeServiceTrackLength(); */
			}

			double ComputeServiceTrackLength() {
				service_track_total_length_ = 0;
				for(const auto &ls:edge_list_) {
					auto u = ls.source(); auto v = ls.target();
					double ux = CGAL::to_double(u.x()); double uy = CGAL::to_double(u.y());
					double vx = CGAL::to_double(v.x()); double vy = CGAL::to_double(v.y());
					double delx = ux - vx; double dely = uy - vy;
					service_track_total_length_ += std::sqrt(delx * delx + dely * dely);
				}
				return service_track_total_length_;
			}

			void GetTracks(std::vector<Point_2> &vertices, std::vector<std::pair<int , int>> &edges) {
				vertices.insert(vertices.end(), vertices_.begin(), vertices_.end());
				edges.insert(edges.end(), edges_.begin(), edges_.end());
			}

			void WriteCoverage(const std::string &filename) {
				std::vector <Polygon_2> poly_list;
				TracksToPWH(vertices_, edges_, offset_, poly_list, line_sweep_);
				std::ofstream outfile (filename);
				outfile << std::setprecision(std::numeric_limits<long double>::digits10 + 1);
				for(const auto &p:poly_list) {
					if(p.size() == 0)
						continue;
					PrintPolygon(p, outfile);
				}
				outfile.close();
			}

			void WriteServiceTracks1(std::string filename) {
				std::ofstream outfile (filename);
				for(auto &obj:service_tracks_object_list_) {
					std::vector<Point_2> vertices;
					std::vector<std::pair<int , int>> edges;
					obj.GetTracks(vertices, edges);
					for(const auto &edge:edges) {
						auto vertex_u = vertices[std::get<0>(edge)];
						auto vertex_v = vertices[std::get<1>(edge)];
						outfile << vertex_u.x() << " " << vertex_u.y() << std::endl;
						outfile << vertex_v.x() << " " << vertex_v.y() << std::endl;
					}
					outfile << std::endl;

				}
				outfile.close();
			}
			void WriteServiceTracks(std::string filename) {
				std::ofstream outfile (filename);
					for(const auto &edge:edge_list_) {
						auto vertex_u = edge.source();
						auto vertex_v = edge.target();
						outfile << vertex_u.x() << " " << vertex_u.y() << std::endl;
						outfile << vertex_v.x() << " " << vertex_v.y() << std::endl;
					}
				outfile.close();
			}


			void GenerateEdgeList() {
				edge_list_.clear();
				if(env_.HasRobot()) {
					std::cout << "Adding obstacle edges\n";
					env_.GetEdgesOffset(edge_list_);
				}
				for(auto &obj:service_tracks_object_list_) {
					std::vector<Point_2> vertices;
					std::vector<std::pair<int , int>> edges;
					obj.GetTracks(vertices, edges);
					for(const auto &edge:edges) {
						auto vertex_u = vertices[std::get<0>(edge)];
						auto vertex_v = vertices[std::get<1>(edge)];
						edge_list_.push_back(Segment_2(vertex_u, vertex_v));
					}
				}
			}

			bool CheckEdgeIntersect(Segment_2 &e1, Segment_2 &e2, std::list<Segment_2> &edges) {
				auto u1 = e1.source(); auto v1 = e1.target();
				auto u2 = e2.source(); auto v2 = e2.target();
				bool e1_has_u2 = e1.has_on(u2); bool e1_has_v2 = e1.has_on(v2);
				bool e2_has_u1 = e2.has_on(u1); bool e2_has_v1 = e2.has_on(v1);
				if(e1_has_u2 or e1_has_v2 or e2_has_u1 or e2_has_v1) {
					if(e1_has_u2 and e1_has_v2) {
						edges.push_back(e1);
						return 1;
					}
					if(e2_has_u1 and e2_has_v1) {
						edges.push_back(e2);
						return 1;
					}
					if(e1_has_u2 and CGAL::are_ordered_along_line(u1, v1, v2)) {
						if(v1 == u2) {
							return 0;
						}
						edges.push_back(Segment_2(u1, u2));
						edges.push_back(Segment_2(u2, v1));
						edges.push_back(Segment_2(v1, v2));
						return 1;
					}
					if(e1_has_u2 and CGAL::are_ordered_along_line(v2, u1, v1)) {
						if(u1 == u2) {
							return 0;
						}
						edges.push_back(Segment_2(v2, u1));
						edges.push_back(Segment_2(u1, u2));
						edges.push_back(Segment_2(u2, v1));
						return 1;
					}
					if(e1_has_u2 and CGAL::are_ordered_along_line(v1, u1, v2)) {
						if(u1 == u2) {
							return 0;
						}
						edges.push_back(Segment_2(v1, u2));
						edges.push_back(Segment_2(u2, u1));
						edges.push_back(Segment_2(u1, v2));
						return 1;
					}
					if(e1_has_u2 and CGAL::are_ordered_along_line(v2, v1, u1)) {
						if(v1 == u2) {
							return 0;
						}
						edges.push_back(Segment_2(v2, v1));
						edges.push_back(Segment_2(v1, u2));
						edges.push_back(Segment_2(u2, u1));
						return 1;
					}
					if(e1_has_v2 and CGAL::are_ordered_along_line(u1, v1, u2)) {
						if(v2 == v1) {
							return 0;
						}
						edges.push_back(Segment_2(u1, v2));
						edges.push_back(Segment_2(v2, v1));
						edges.push_back(Segment_2(v1, u2));
						return 1;
					}
					if(e1_has_v2 and CGAL::are_ordered_along_line(u2, u1, v1)) {
						if(u1 == v2) {
							return 0;
						}
						edges.push_back(Segment_2(u2, u1));
						edges.push_back(Segment_2(u1, v2));
						edges.push_back(Segment_2(v2, v1));
						return 1;
					}
					if(e1_has_v2 and CGAL::are_ordered_along_line(v1, u1, u2)) {
						if(v2 == v1) {
							return 0;
						}
						edges.push_back(Segment_2(v1, v2));
						edges.push_back(Segment_2(v2, u1));
						edges.push_back(Segment_2(u1, u2));
						return 1;
					}
					if(e1_has_v2 and CGAL::are_ordered_along_line(u2, v1, u1)) {
						if(v1 == v2) {
							return 0;
						}
						edges.push_back(Segment_2(u2, v1));
						edges.push_back(Segment_2(v1, v2));
						edges.push_back(Segment_2(v2, u1));
						return 1;
					}
				}
				else {
					return 0;
				}
				return 0;

			}

			void FilterEdges() {
				auto it_main = edge_list_.begin();
				while(it_main != edge_list_.end()) {
					bool merge = false;
					auto it = std::next(it_main);
					while(it != edge_list_.end()) {
						std::list <Segment_2> new_edges;
						if(CheckEdgeIntersect(*it_main, *it, new_edges) == false) {
							std::advance(it, 1);
							continue;
						}
						for(auto &e:new_edges) {
							edge_list_.push_back(e);
						}
						edge_list_.erase(it);
						it_main = edge_list_.erase(it_main);
						merge = true;
						break;
					}
					if(merge == false) {
						std::advance(it_main, 1);
					}
				}
			}

			void GenerateVerticesEdges() {
				vertices_.clear();
				edges_.clear();
				env_.GetVertices(vertices_);
				for(const auto &e:edge_list_) {
					size_t source_idx;
					size_t target_idx;
					bool source_found = false;
					bool target_found = false;
					auto u = e.source(); auto v = e.target();
					for(size_t i = 0; i < vertices_.size(); ++i) {
						if((not source_found) and vertices_[i] == u) {
							source_found = true;
							source_idx = i;
						}
						if((not target_found) and vertices_[i] == v) {
							target_found = true;
							target_idx = i;
						}
						if(source_found and target_found)
							break;
					}
					if(not source_found) {
						source_idx = vertices_.size();
						vertices_.push_back(u);
					}
					if(not target_found) {
						target_idx = vertices_.size();
						vertices_.push_back(v);
					}
					edges_.push_back(std::pair<int, int> (source_idx, target_idx));
				}
			}

	};
}
#endif /* ACLIBRARY_SERVICE_TRACKS_H_ */

