/**
 * This file is part of the AreaCoverage-library.
 * The file contains the primary file for describing the environment for the area coverage problem
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

#ifndef ACLIBRARY_ENV_H_
#define ACLIBRARY_ENV_H_

#include <string>
#include <fstream>
#include <cmath>
#include <vector>
#include <algorithm>
#include <aclibrary/cgal_config.h>
#include <aclibrary/constants.h>
#include <aclibrary/polygon_utilities.h>

#include <CGAL/Polygon_2_algorithms.h>
#include <aclibrary/pgn_print.h>
#include <aclibrary/compute_altitude.h>

namespace aclibrary {

	typedef std::pair <Segment_2, Segment_2> SegmentPair;

	struct SplitPolygon {
		bool is_split_ = false;
		double msa_ = kDoubleMax;
		std::vector <Polygon_with_holes_2> split_polygons_;
		SplitPolygon() : is_split_(false), msa_(kDoubleMax) {
			split_polygons_.clear();
		}
		SplitPolygon(bool is_split, double msa, std::vector <Polygon_with_holes_2> split_polygons) : is_split_(is_split), msa_(msa) {
			split_polygons_ = split_polygons;
		}
	};

	struct PWHwithDir {
		Polygon_with_holes_2 pwh;
		Line_2 dir;
		double alt;
	};

	class Env {
		private:
			Polygon_with_holes_2 env_;
			Polygon_2 outer_polygon_, outer_polygon_input_, outer_polygon_off_;
			std::vector <Polygon_2> holes_, holes_input_, holes_off_;
			Polygon_2 robot_, robot_image_;
			std::vector <size_t> points_id_;
			Arrangement_2 arr_;
			std::vector <Polygon_2> arr_decomposition_;
			std::vector <Polygon_2> init_decomposition_;
			std::vector <Polygon_2> final_decomposition_;
			std::list <PWHwithDir> final_decomposition_pwh_dir_;
			std::vector <Line_2> cell_longitudinal_dirs_;
			std::vector <std::vector <Segment_2> > sweep_segments_decompositions_;
			std::vector <std::vector <Point_2> > tracks_vertices_;
			std::vector <std::vector <std::pair<int, int> > > parallel_tracks_edges_;
			double final_msa_, bcd_msa_;
			size_t bcd_num_polygons_;
			Kernel::FT offset_ = 0;

		public:
			Env() {}

			Env(const std::string &outer_polygon_filename) {
				ReadOuterPolygon(outer_polygon_filename);
				env_ = Polygon_with_holes_2(outer_polygon_);
				outer_polygon_input_ = outer_polygon_;
				holes_input_ = holes_;
			}

			Env(const std::string &outer_polygon_filename, const std::string &holes_filename) {
				ReadOuterPolygon(outer_polygon_filename);
				ReadHoles(holes_filename);
				if(holes_.empty()) {
					env_ = Polygon_with_holes_2(outer_polygon_);
				}
				else {
					env_ = Polygon_with_holes_2(outer_polygon_, holes_.begin(), holes_.end());
				}
				outer_polygon_input_ = outer_polygon_;
				holes_input_ = holes_;
			}

			Env(const std::string &outer_polygon_filename, const std::string &holes_filename, const std::string &robot_filename, const double offset) {
				offset_ = FT(offset);
				ReadOuterPolygon(outer_polygon_filename);
				ReadHoles(holes_filename);
				ReadRobot(robot_filename);
				outer_polygon_input_ = outer_polygon_;
				holes_input_ = holes_;

				ComputeFreeSpace();
				if(holes_.empty()) {
					env_ = Polygon_with_holes_2(outer_polygon_);
				}
				else {
					env_ = Polygon_with_holes_2(outer_polygon_, holes_.begin(), holes_.end());
				}

			}

			void ComputeFreeSpace();
			inline bool HasRobot() const{
				if(robot_.size() > 0) {
					return true;
				}
				return false;
			}

			void GetVertices(std::vector<Point_2> &vertex_vec) {
				for(auto it = outer_polygon_.vertices_begin(); it != outer_polygon_.vertices_end(); ++it) {
					vertex_vec.push_back(*it);
				}
				for(const auto hole:holes_) {
					for(auto it = hole.vertices_begin(); it != hole.vertices_end(); ++it) {
						vertex_vec.push_back(*it);
					}
				}
			}

			void GetEdgesOffset(std::list<Segment_2> &edge_list) {
				for(auto it = outer_polygon_off_.edges_begin(); it != outer_polygon_off_.edges_end(); ++it) {
					edge_list.push_back(*it);
				}
				for(const auto hole:holes_off_) {
					for(auto it = hole.edges_begin(); it != hole.edges_end(); ++it) {
						edge_list.push_back(*it);
					}
				}
			}

			void GetEdges(std::list<Segment_2> &edge_list) {
				for(auto it = outer_polygon_.edges_begin(); it != outer_polygon_.edges_end(); ++it) {
					edge_list.push_back(*it);
				}
				for(const auto hole:holes_) {
					for(auto it = hole.edges_begin(); it != hole.edges_end(); ++it) {
						edge_list.push_back(*it);
					}
				}
			}

			/* int GetPointType(double x, double xprev, double xnext) { */
			/* 	if(x == xprev and x == xnext) */
			/* 		return 0; */
			/* 	if(x < xprev and x < xnext) */
			/* 		return 1; */
			/* 	if(x > xprev and x > xnext) */
			/* 		return -1; */
			/* 	return 0; */
			/* } */
			void CheckDecomposition() {

			}
			void Test() {
				std::cout << "---------- Test begin----------\n";


				std::cout << "---------- Test end  ----------\n";
			}

			void PrintArea() {
				auto area = outer_polygon_input_.area();
				std::cout << "Outer Area: " << area << std::endl;
				for(const auto &h:holes_input_) {
					area += h.area();
					std::cout << "Area: " << area << std::endl;
				}
			}
			void ComputeArrangement(Arrangement_2 &arr) const {
				/* CGAL::insert_non_intersecting_curves(arr, outer_polygon_.edges_begin(), outer_polygon_.edges_end()); */
				CGAL::insert (arr, outer_polygon_.edges_begin(), outer_polygon_.edges_end());
				for(auto h_it = holes_.begin(); h_it != holes_.end(); ++h_it) {
					CGAL::insert (arr, h_it->edges_begin(), h_it->edges_end());
					/* CGAL::insert_non_intersecting_curves(arr, h_it->edges_begin(), h_it->edges_end()); */
				}
			}

			void ComputeOuterArrangement(Arrangement_2 &arr) const {
				CGAL::insert (arr, outer_polygon_.edges_begin(), outer_polygon_.edges_end());
			}

			void ComputeOffsetArrangement(Arrangement_2 &arr) const {
				/* CGAL::insert_non_intersecting_curves(arr, outer_polygon_off_.edges_begin(), outer_polygon_off_.edges_end()); */
				CGAL::insert (arr, outer_polygon_off_.edges_begin(), outer_polygon_off_.edges_end());
				for(auto h_it = holes_off_.begin(); h_it != holes_off_.end(); ++h_it) {
					CGAL::insert (arr, h_it->edges_begin(), h_it->edges_end());
					/* CGAL::insert_non_intersecting_curves(arr, h_it->edges_begin(), h_it->edges_end()); */
				}
			}

			void ComputeOuterOffsetArrangement(Arrangement_2 &arr) const {
				CGAL::insert (arr, outer_polygon_off_.edges_begin(), outer_polygon_off_.edges_end());
			}

			void GetTracks(std::vector<Point_2> &vertices, std::vector<std::pair<int , int>> &edges) {
				for(const auto &v_list:tracks_vertices_) {
					for(const auto &v:v_list)
						vertices.push_back(v);
				}
				for(const auto &p:outer_polygon_) {
					vertices.push_back(p);
				}
				for(const auto &h:holes_) {
					for(const auto &p:h) {
						vertices.push_back(p);
					}
				}
				size_t node_count = 0;
				for(size_t i = 0; i < parallel_tracks_edges_.size(); ++i) {
					if(i > 0)
						node_count += tracks_vertices_[i - 1].size();
					auto edge_list = parallel_tracks_edges_[i];
					for(const auto &e:edge_list) {
						edges.push_back(std::pair<int, int> (node_count + std::get<0>(e), node_count + std::get<1>(e)));
					}
				}
			}

			void GetSweepTracks(std::vector<Point_2> &vertices, std::vector<std::pair<int , int>> &edges) {
				int vertex_count = 0;
				for(const auto &sweep_segments:sweep_segments_decompositions_) {
					for (const auto &segment:sweep_segments) {
						vertices.push_back(segment.source());
						++vertex_count;
						vertices.push_back(segment.target());
						++vertex_count;
						edges.push_back(std::pair<int, int> (vertex_count - 2, vertex_count - 1));
					}
				}
			}

			PWHwithDir GetOptimalPWHwithDir(const Polygon_with_holes_2 &pwh) {
				auto new_pwh = pwh;
				RemoveCollinearVerticesFromPolygon(new_pwh);
				double alt; Vector_2 dir; Line_2 dir_line;
				ComputeBestAltitude(new_pwh, dir, alt);
				dir_line = Line_2(new_pwh.outer_boundary()[0], dir);
				PWHwithDir pwh_dir;
				pwh_dir.pwh = new_pwh;
				pwh_dir.dir = dir_line;
				pwh_dir.alt = alt;
				return pwh_dir;
			}
			PWHwithDir GetOptimalPWHwithDir(const Polygon_2 &p) {
				return GetOptimalPWHwithDir(Polygon_with_holes_2(p)); }

			void GetServiceTracks(std::string filename, std::vector<Point_2> &vertices, std::vector<std::pair<int , int>> &edges) {
				int vertex_count = 0;
				std::ifstream infile(filename);
				double x1, y1, x2, y2;
				while(infile >> x1 >> y1 >> x2 >> y2) {
					vertices.push_back(Point_2(x1, y1));
					++vertex_count;
					vertices.push_back(Point_2(x2, y2));
					++vertex_count;
					edges.push_back(std::pair<int, int> (vertex_count - 2, vertex_count - 1));
				}
				infile.close();
			}

			void SolveBCDmerge() {
				Clear();
				ComputeAltitudeBCD(init_decomposition_);
				std::cout << "BCD init done\n";
				Polygon_set_2 res_set;
				for(const auto &p:init_decomposition_) {
					res_set.join(p);
				}
				res_set.complement();
				res_set.intersection(env_);
				std::list<Polygon_with_holes_2> res;
				res_set.polygons_with_holes (std::back_inserter (res));
				for(size_t i = 0; i < init_decomposition_.size(); ++i) {
					auto pwh_dir = GetOptimalPWHwithDir(init_decomposition_[i]);
					cell_longitudinal_dirs_[i] = pwh_dir.dir;
					final_decomposition_pwh_dir_.push_back(pwh_dir);
				}
				final_decomposition_ = init_decomposition_;
				for(auto &pwh:res) {
					auto pwh_dir = GetOptimalPWHwithDir(pwh);
					final_decomposition_pwh_dir_.push_back(pwh_dir);
					cell_longitudinal_dirs_.push_back(pwh_dir.dir);
					final_decomposition_.push_back(pwh.outer_boundary());
				}
				/* std::cout << "SolveBCDMerge: initialized with size: " << final_decomposition_pwh_dir_.size() << std::endl;; */
				/* std::cout << "Initial decomposition\n"; */
				bcd_msa_ = 0;
				for(auto &pwh_dir:final_decomposition_pwh_dir_) {
					/* std::cout << pwh_dir.pwh << std::endl; */
					/* std::cout << "Alt: " << pwh_dir.alt << std::endl; */
					bcd_msa_ += pwh_dir.alt;
				}
				bcd_num_polygons_ = final_decomposition_pwh_dir_.size();

				std::cout << "Cell decomposition done\n";

				SolveSplit();
				std::cout << "Split decomposition done\n";
				MergeFinalDecomposition();
				std::cout << "Merge decomposition done\n";

				/* std::cout << "Final decomposition\n"; */
				final_msa_ = 0;
				for(auto &pwh_dir:final_decomposition_pwh_dir_) {
					/* std::cout << pwh_dir.pwh << std::endl; */
					/* std::cout << "Alt: " << pwh_dir.alt << std::endl; */
					final_msa_ += pwh_dir.alt;
				}
			}

			void MergeFinalDecomposition() {
				/* std::cout << "Inside merge\n"; */
				auto it_main = final_decomposition_pwh_dir_.begin();
				while(it_main != final_decomposition_pwh_dir_.end()) {
					bool merge = false;
					auto it = std::next(it_main);
					while(it != final_decomposition_pwh_dir_.end()) {
						bool valid_merge = true;
						if(CGAL::parallel(it_main->dir, it->dir) == false) {
							/* std::advance(it, 1); */
							/* continue; */
							valid_merge = false;
							double rad1deg = 1. * M_PI/180.;
							auto outer_main = it_main->pwh.outer_boundary();
							auto outer_sec = it->pwh.outer_boundary();
							if(it->dir.b() == 0 and it_main->dir.b() == 0) {
								valid_merge = true;
							}
							else {
								if(it_main->dir.b() == 0){
									auto ang1 = M_PI/2.;
									auto ang2 = atan(CGAL::to_double(- it->dir.a()/it->dir.b()));
									if(std::abs(ang1 - ang2) < rad1deg or std::abs(ang1 + ang2) < rad1deg) {
										valid_merge = true;
									}
								}
								else if(it->dir.b() == 0){
									auto ang1 = M_PI/2.;
									auto ang2 = atan(CGAL::to_double(- it_main->dir.a()/it_main->dir.b()));
									if(std::abs(ang1 - ang2) < rad1deg or std::abs(ang1 + ang2) < rad1deg) {
										valid_merge = true;
									}
								}
								else {
									auto m1 = - it_main->dir.a()/it_main->dir.b();
									auto m2 = - it->dir.a()/it->dir.b();
									auto rslope = (m2 - m1)/(1. + m1 * m2);
									auto ang1 = atan(CGAL::to_double(rslope));
									auto ang2 = atan(CGAL::to_double(-rslope));
									if(std::abs(ang1) < rad1deg or std::abs(ang2) < rad1deg) {
										valid_merge = true;
									}
								}
							}
						}
						if(valid_merge == false) {
							std::advance(it, 1);
							continue;
						}

						Polygon_with_holes_2 res;
						if(CGAL::join(it_main->pwh, it->pwh, res) == false) {
							std::advance(it, 1);
							continue;
						}

						typename CGAL::Gps_default_traits<Polygon_with_holes_2>::Traits  tr;
						if(res.outer_boundary().is_simple() == false) {
							std::advance(it, 1);
							continue;
						}
						RemoveCollinearVerticesFromPolygon(res);
						if(not CGAL::are_holes_and_boundary_pairwise_disjoint(res, tr)) {
							std::advance(it, 1);
							continue;
						}

						final_decomposition_pwh_dir_.erase(it);
						it_main = final_decomposition_pwh_dir_.erase(it_main);

						auto new_pwh_dir = GetOptimalPWHwithDir(res);
						final_decomposition_pwh_dir_.push_back(new_pwh_dir);
						merge = true;
						break;
					}
					if(merge == false) {
						std::advance(it_main, 1);
					}
				}
				/* std::cout << "SolveBCDMerge: final_decomposition_pwh_dir_ computed with size: " << final_decomposition_pwh_dir_.size() << std::endl; */
				final_decomposition_.clear();
				cell_longitudinal_dirs_.clear();
				for(auto &pwh_dir:final_decomposition_pwh_dir_) {
					final_decomposition_.push_back(pwh_dir.pwh.outer_boundary());
					cell_longitudinal_dirs_.push_back(pwh_dir.dir);
				}
				/* std::cout << "Final decomposition computed\n"; */
				/* std::cout << "BCD size: " << final_decomposition_.size() << std::endl; */
				/* std::cout << "End merge\n"; */
			}

			void GetPWHwithDirList(std::list <PWHwithDir> &list) {
				list = final_decomposition_pwh_dir_;
			}

			void SolveBCD() {
				Clear();
				ComputeAltitudeBCD(final_decomposition_);
				std::cout << "BCD size: " << final_decomposition_.size() << std::endl;
			}

			void Clear() {
				arr_decomposition_.clear();
				init_decomposition_.clear();
				final_decomposition_.clear();
				cell_longitudinal_dirs_.clear();
				final_decomposition_pwh_dir_.clear();
			}

			bool SplitPWH(const PWHwithDir &, const Point_2 &ll, const Point_2 &rl, const Point_2 &ru, const Point_2 &lu, const Line_2 &ls, std::vector<PWHwithDir> &);
			/* bool PerformSplit(const Polygon_with_holes_2 &poly, const double &, std::vector <Polygon_with_holes_2> &poly_split_list); */
			bool PerformSplit_bbox(const PWHwithDir &poly, std::vector <PWHwithDir> &poly_split_list);
			/* bool PerformSplitPar(const Polygon_with_holes_2 &poly, std::vector <Polygon_with_holes_2> &poly_split_list); */
			/* bool PerformSplitAux(const Polygon_with_holes_2 &poly, const Line_2 &ls, const Arrangement_2 &arr_split, const double minimum_msa, std::shared_ptr<SplitPolygon> &split); */

			void SolveSplit() {
				/* Clear(); */
				/* std::cout << "Solve split start\n"; */
				std::vector <PWHwithDir> poly_split_list;
				for(const auto &pwh_dir:final_decomposition_pwh_dir_) {
					if(PerformSplit_bbox(pwh_dir, poly_split_list) == false) {
						poly_split_list.push_back(pwh_dir);
					}
				}

				final_decomposition_.clear();
				cell_longitudinal_dirs_.clear();
				final_decomposition_pwh_dir_.clear();
				for(auto &p:poly_split_list) {
					auto new_pwh_dir = GetOptimalPWHwithDir(p.pwh);
					final_decomposition_.push_back(new_pwh_dir.pwh.outer_boundary());
					cell_longitudinal_dirs_.push_back(new_pwh_dir.dir);
					final_decomposition_pwh_dir_.push_back(new_pwh_dir);
				}
				/* MergeFinalDecomposition(); */
				/* std::cout << "Final size: " << final_decomposition_.size() << std::endl; */
				/* std::cout << "Solve split end\n"; */
			}

			/** IO functions **/
			void ReadOuterPolygon(const std::string &);
			void ReadHoles(const std::string &);
			void ReadRobot(const std::string &);

			void WriteVerticesEdges(
					const std::string &vertices_filename,
					const std::string &edges_filename) const;

			void WriteEnv(const std::string &filename) const;
			void WriteEnvInput(const std::string &filename) const;
			void WriteOuterPolygonGnuplot(const std::string &) const;
			void WriteHolesGnuplot(const std::string &) const;

			/** Basic functionalities **/
			inline bool HasHoles() const {
				if(holes_.size() > 0)
					return true;
				else
					return false;
			}

			inline size_t GetNumHoles() const {
				return holes_.size();
			}

			bool CheckPointInHoles (const Point_2 &) const;
			bool CheckPointOnHoles (const Point_2 &p) const;
			bool CheckPointOnHole (const Polygon_2 &h, const Point_2 &p) const;
			bool CheckPointOnHole (size_t i, const Point_2 &p) const;
			bool CheckIntersectsHoleEdge(const Point_2 &u, const Point_2 &v) const;

			Point_2 GetCentroid() {
				Point_2 c2 = CGAL::centroid(outer_polygon_.begin(), outer_polygon_.end(),CGAL::Dimension_tag<0>());
				return c2;
			}

			Point_2 GetVertexClosesetToCentroid() {
				Point_2 c2 = GetCentroid();
				Point_2 closest;
				K::FT min_dist_sqr = kDoubleMax;
				for(const auto &v:outer_polygon_) {
					auto dist = CGAL::squared_distance(c2, v);
					if(min_dist_sqr > dist){
						min_dist_sqr = dist;
						closest = v;
					}
				}
				for(const auto &h:holes_) {
					for(const auto &v:h) {
						auto dist = CGAL::squared_distance(c2, v);
						if(min_dist_sqr > dist){
							min_dist_sqr = dist;
							closest = v;
						}
					}
				}
				return closest;
			}
			/** Decomposition **/

			void WriteDecomposition(const std::vector <Polygon_2> &decomposition, const std::string &filename) const {
				std::ofstream outfile (filename);
				outfile << std::setprecision(std::numeric_limits<long double>::digits10 + 1);
				for(const auto &p:decomposition) {
					if(p.size() == 0)
						continue;
					PrintPolygon(p, outfile);
				}
				outfile.close();
			}

			void WriteCoverage(const Kernel::FT &offset, const std::string &filename) {
				std::vector <Polygon_2> poly_list;
				TracksToPWH(tracks_vertices_, parallel_tracks_edges_, offset, poly_list);
				WriteDecomposition(poly_list, filename);
			}

			void WriteArrangementDecomposition(const std::string &filename) const {
				WriteDecomposition(arr_decomposition_, filename);
			}

			void WriteInitDecomposition(const std::string &filename) const {
				WriteDecomposition(init_decomposition_, filename);
			}

			void WriteFinalDecomposition(const std::string &filename) const {
				WriteDecomposition(final_decomposition_, filename);
			}

			bool CheckEdgeDirs(const std::vector<Direction_2> &, const Direction_2 &) const;
			bool CheckParallelLines(const std::vector<Line_2> &, const Line_2 &) const;
			void GetNonConvexVertices(const Polygon_2 &, std::vector <Point_2> &) const;
			void GetNonConvexVertices(const Polygon_2 &, std::vector <std::array<Point_2,3>> &) const;
			void AddBCDLines (const Polygon_with_holes_2 &, std::vector <Line_2> &) const;
			/* void AddBCDLines1 (const Polygon_with_holes_2 &, std::vector <Line_2> &) const; */
			void AddBCDLines (std::vector <Line_2> &cv) const;
			void AddBCDLines (std::vector <Segment_2> &cv) const;
			void AddNonConvexLines(const Polygon_2 &poly, std::vector <Line_2> &cv) const;
			void AddNonConvexLines(const Polygon_2 &poly, std::vector <Segment_2> &, const bool uni = false) const;
			void AddNonConvexLinesHoles(const Polygon_2 &, std::list <SegmentPair> &);
			void AddLinesFromEdges(const Polygon_2 &poly, std::vector <Line_2> &cv, std::vector <Segment_2> &cv1) const;
			/* void ComputeArrangement(); */
			void FindIntersectingSegment(const Segment_2 &ls, Segment_2 &extension) const;
			void FindIntersectingSegmentUni(const Segment_2 &ls, Segment_2 &extension, const bool opposite=false) const;

			void ComputeFinalDecomposition();

			double ComputeAltitudeBCD(const Polygon_with_holes_2 &poly, std::vector<Polygon_2> &decomposition, std::vector<Line_2> &dirs);

			double ComputeAltitudeBCD(std::vector<Polygon_2> &decomposition);

			double ComputeAltitudeBCD(const Polygon_with_holes_2 &poly);

			size_t GetNumNonConvexVertices(const Polygon_2 &poly) const {
				auto v_it = poly.vertices_circulator();
				auto v_it_begin = v_it;
				size_t num_ncvx = 0;
				do {
					auto p1 = *(std::prev(v_it));
					auto p2 = *v_it;
					auto p3 = *(std::next(v_it));
					Vector_2 v1(p2, p1);
					Vector_2 v2(p2, p3);
					auto cross = v2[0] * v1[1] - v2[1] * v1[0];
					if(cross <= 0) {
						++num_ncvx;
					}
					std::advance(v_it, 1);
				} while(v_it != v_it_begin);
				return num_ncvx;
			}

			void WriteResults(const std::string &filename) const {
				size_t num_non_convex_vertices = 0;
				num_non_convex_vertices += GetNumNonConvexVertices(outer_polygon_);
				for(const auto &h:holes_) {
					num_non_convex_vertices += GetNumNonConvexVertices(h);
				}
				std::ofstream outfile (filename, std::ofstream::app);
				size_t num_vertices = 0;
				num_vertices = outer_polygon_.size();
				auto area = polygon_area_2(outer_polygon_input_.vertices_begin(), outer_polygon_input_.vertices_end(), K());
				for(const auto &h:holes_input_) {
					num_vertices += h.size();
					area += polygon_area_2(h.begin(), h.end(), K());
				}
				outfile << num_vertices << " " << CGAL::to_double(area) << " " << num_non_convex_vertices<< " " << bcd_msa_ << " " << bcd_num_polygons_ << " " << final_msa_ << " " << final_decomposition_.size() << " ";
				outfile.close();
			}

	};
}

#endif /* ACLIBRARY_ENV_H_ */
