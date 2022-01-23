/**
 * This file is part of the AreaCoverage-library.
 * The file contains greedy improvement for cell decomposition
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

	bool Env::SplitPWH(const PWHwithDir &pda, const Point_2 &ll, const Point_2 &rl, const Point_2 &ru, const Point_2 &lu, const Line_2 &ls, std::vector<PWHwithDir> &pda_list) {
		Point_2 pt1, pt2, pt3, pt4, pta, ptb;
		auto flag1 = GetLineEdgeIntersection(ls, Segment_2(ll, rl), pt1);
		auto flag2 = GetLineEdgeIntersection(ls, Segment_2(rl, ru), pt2);
		auto flag3 = GetLineEdgeIntersection(ls, Segment_2(ru, lu), pt3);
		auto flag4 = GetLineEdgeIntersection(ls, Segment_2(lu, ll), pt4);
		bool flag_pta = false, flag_ptb = false;
		std::vector<Point_2> pnt_list;
		if(flag1 == 1 or flag2 == 1 or flag3 == 1 or flag4 == 1)
			return 1;
		if(flag1 + flag2 + flag3 + flag4 > 4)
			return 1;

		pnt_list.push_back(ll);
		if(flag1 == 0 and pt1 != ll and pt1 != rl) {
			pnt_list.push_back(pt1);
		}
		if(flag1 == 0) {
			pta = pt1; flag_pta = true;
		}
		pnt_list.push_back(rl);

		if(flag2 == 0 and pt2 != rl and pt2 != ru) {
			pnt_list.push_back(pt2);
		}
		if(flag2 == 0) {
			if(flag_pta == false) {
				pta = pt2; flag_pta = true;
			} else if(pta != pt2){
				ptb = pt2;
				flag_ptb = true;
			}
		}
		pnt_list.push_back(ru);

		if(flag3 == 0 and pt3 != ru and pt3 != lu) {
			pnt_list.push_back(pt3);
		}
		if(flag3 == 0) {
			if(flag_pta == false) {
				pta = pt3; flag_pta = true;
			} else if(pta != pt3){
				ptb = pt3;
				flag_ptb = true;
			}
		}
		pnt_list.push_back(lu);

		if(flag4 == 0 and pt4 != lu and pt4 != ll) {
			pnt_list.push_back(pt4);
		}
		if(flag4 == 0) {
			if(flag_pta == false) {
				pta = pt4; flag_pta = true;
			} else if(pta != pt4){
				ptb = pt4;
				flag_ptb = true;
			}
		}
		if(flag_pta == false or flag_ptb == false) {
			return 1;
		}

		auto n_begin = pnt_list.begin();
		for(auto it = pnt_list.begin(); it != pnt_list.end(); ++it) {
			if(*it == pta) {
				n_begin = it;
				break;
			}
		}
		std::rotate(pnt_list.begin(), n_begin, pnt_list.end());
		/* for(const auto p:pnt_list) { */
		/* 	std::cout << p << std::endl; */
		/* } */
		Polygon_2 bb1, bb2;
		for(auto &pt:pnt_list) {
			bb1.push_back(pt);
			if(pt == ptb) {
				break;
			}
		}
		bool flag = false;
		for(auto &pt:pnt_list) {
			if(pt != ptb and flag == false) {
				continue;
			}
			flag = true;
			bb2.push_back(pt);
		}
		bb2.push_back(pnt_list[0]);

		std::list <Polygon_with_holes_2> poly_list;
		CGAL::intersection (pda.pwh, bb1, std::back_inserter(poly_list));
		CGAL::intersection (pda.pwh, bb2, std::back_inserter(poly_list));
		for(auto &pp:poly_list) {
			/* if(not pp.outer_boundary().is_simple()) { */
			/* 	std::cout << "not simple: rejected\n"; */
			/* 	std::cout << pp << std::endl; */
			/* 	return 1; */
			/* } */
			if(pp.holes_begin() == pp.holes_end() and !pp.outer_boundary().is_simple()) {
				Arrangement_2 arr_split;
				CGAL::insert (arr_split, pp.outer_boundary().edges_begin(), pp.outer_boundary().edges_end());
				std::vector<Polygon_2> arr_list;
				GeneratePolygons(arr_split, arr_list);
				for(auto &arrp:arr_list) {
					poly_list.push_back(Polygon_with_holes_2(arrp));
				}
				continue;
			}
			pda_list.push_back(GetOptimalPWHwithDir(pp));
		}
		return 0;
	}

	bool Env::PerformSplit_bbox(const PWHwithDir &pwh_dir, std::vector <PWHwithDir> &poly_split_list) {
		auto poly = pwh_dir.pwh;
		Polygon_2 outer_polygon = poly.outer_boundary();
		std::vector <Line_2> cv;
		AddBCDLines(poly, cv);
		/* std::cout << "No. of BCD lines: " << cv.size() << std::endl; */
		AddNonConvexLines(outer_polygon, cv);
		for(auto h_it = poly.holes_begin(); h_it != poly.holes_end(); ++h_it) {
			AddNonConvexLines(*h_it, cv);
		}
		/* std::cout << "No. of total lines: " << cv.size() << std::endl; */
		double minimum_msa = pwh_dir.alt;
		bool is_improved = false;
		std::vector <PWHwithDir> best_poly_list;
		/* std::cout << "No. of non-convex edges: " << cv.size() << std::endl; */
		auto bbox = poly.bbox();
		Point_2 ll = Point_2(bbox.xmin(), bbox.ymin());
		Point_2 rl = Point_2(bbox.xmax(), bbox.ymin());
		Point_2 ru = Point_2(bbox.xmax(), bbox.ymax());
		Point_2 lu = Point_2(bbox.xmin(), bbox.ymax());
		/* std::cout << poly << std::endl; */

		for(const auto &ls:cv) {
			/* std::cout << ls << std::endl; */

			size_t intersection_count = 0;
			for(auto e_it = outer_polygon.edges_begin(); e_it != outer_polygon.edges_end(); ++e_it ) {
				if(CGAL::do_intersect(*e_it, ls)) {
					if(ls.has_on(e_it->source()) or ls.has_on(e_it->target())) {
						continue;
					}
					++intersection_count;
				}
				if(intersection_count > 2) {
					break;
				}
			}
			if(intersection_count > 2) {
				/* std::cout << "Rejected" << std::endl; */
				continue;
			}
			intersection_count = 0;
			for(auto hit = poly.holes_begin(); hit != poly.holes_end(); ++hit) {
				for(auto e_it = hit->edges_begin(); e_it != hit->edges_end(); ++e_it ) {
					if(CGAL::do_intersect(*e_it, ls)) {
						if(ls.has_on(e_it->source()) or ls.has_on(e_it->target())) {
							continue;
						}
						++intersection_count;
					}
					if(intersection_count > 2) {
						break;
					}
				}
				if(intersection_count > 2) {
					break;
				}
			}
			if(intersection_count > 2) {
				/* std::cout << "Rejected" << std::endl; */
				continue;
			}
			std::vector <PWHwithDir> pda_list;
			/* std::cout << "Count: " << ++c << std::endl; */
			/* std::cout << ls << std::endl; */
			if(SplitPWH(pwh_dir, ll, rl, ru, lu, ls, pda_list)) {
				continue;
			}

			bool flag = false;
			for(const auto &pda:pda_list) {
				auto p = pda.pwh;
				if(not p.outer_boundary().is_simple()) {
					flag = true;
					break;
				}
				for(auto hit = p.holes_begin(); hit != p.holes_end(); ++hit) {
					if(not hit->is_simple()) {
						flag = true;
						break;
					}
				}
				auto p_outer = p.outer_boundary();
				for(auto hit1 = p.holes_begin(); hit1 != p.holes_end() and flag == false; ++hit1) {
					auto h1 = *hit1; h1.reverse_orientation();
					if(CGAL::do_intersect(h1, p_outer)) {
						flag = true;
						break;
					}
					for(auto hit2 = p.holes_begin(); hit2 != p.holes_end() and flag == false; ++hit2) {
						if(hit1 == hit2)
							continue;
						auto h2 = *hit2; h2.reverse_orientation();
						if(CGAL::do_intersect(h1, h2)) {
							flag = true;
							break;
						}
					}
				}

				if(flag == true)
					break;
			}
			if(flag == true)
				continue;

			double minimum_msa_sum = 0;
			for(auto &pda:pda_list) {
				minimum_msa_sum += pda.alt;
			}
			if(minimum_msa_sum - minimum_msa <= 0) {
				minimum_msa = minimum_msa_sum;
				best_poly_list = pda_list;
				is_improved = true;
			}
		}
		if(is_improved) {
			for(const auto &p:best_poly_list) {
				std::vector <PWHwithDir> split;
				if(PerformSplit_bbox(p, split) == false) {
					poly_split_list.push_back(p);
				}
				else {
					for(const auto &p1:split) {
						poly_split_list.push_back(p1);
					}
				}
			}
		}
		return is_improved;

	}
}
