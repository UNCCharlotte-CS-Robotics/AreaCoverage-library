/**
 * This file is part of the AreaCoverage-library.
 * The file contains definitions for IO functions in the Env class
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

	void Env::ReadOuterPolygon(const std::string &filename) {
		std::ifstream infile (filename);
		size_t id = 0;
		Kernel::FT x, y;
		while(infile >> x >> y) {
			points_id_.push_back(id++);
			outer_polygon_.push_back(Point_2(x, y));
		}
		infile.close();
		if(CGAL::orientation_2(outer_polygon_.begin(), outer_polygon_.end()) == CGAL::CLOCKWISE)
			outer_polygon_.reverse_orientation();
	}

	void Env::ReadHoles(const std::string &filename) {
		std::ifstream infile (filename);
		if(infile.is_open() == false)
			return;
		Polygon_2 hole;
		std::string s;
		bool flag = 0;
		while (std::getline(infile, s)) {
			if (s.empty() && flag == 1) {
				if(CGAL::orientation_2(hole.begin(), hole.end()) == CGAL::COUNTERCLOCKWISE)
					hole.reverse_orientation();
				if(not hole.is_simple()) {
					std::cerr << "Hole is not simple: ";
					print_polygon(hole);
				}
				holes_.push_back(hole);
				hole.clear();
			}
			else {
				std::istringstream tmp(s);
				size_t id = outer_polygon_.size();
				Kernel::FT x, y;
				tmp >> x >> y;
				points_id_.push_back(id++);
				hole.push_back(Point_2(x, y));
				flag = 1;
			}
		}
		if(CGAL::orientation_2(hole.begin(), hole.end()) == CGAL::COUNTERCLOCKWISE)
			hole.reverse_orientation();
		holes_.push_back(hole);
		infile.close();

	}

	void Env::ReadRobot(const std::string &filename) {
		std::ifstream infile (filename);
		size_t id = 0;
		Kernel::FT x, y;
		while(infile >> x >> y) {
			points_id_.push_back(id++);
			robot_.push_back(Point_2(x, y));
			robot_image_.push_back(Point_2(-x, -y));
		}
		infile.close();
		if(CGAL::orientation_2(robot_.begin(), robot_.end()) == CGAL::CLOCKWISE)
			robot_.reverse_orientation();
		if(CGAL::orientation_2(robot_image_.begin(), robot_image_.end()) == CGAL::CLOCKWISE)
			robot_image_.reverse_orientation();
	}

	void Env::WriteVerticesEdges(const std::string &vertices_filename, const std::string &edges_filename) const {
		std::ofstream outfile (vertices_filename);
		size_t vertex_id = 0;
		for(const auto &vertex_set:tracks_vertices_) {
			for(const auto &p:vertex_set) {
				outfile << vertex_id << " " << p.x() << " " << p.y() << std::endl;
				++vertex_id;
			}
		}
		for(const auto &p:outer_polygon_) {
			outfile << vertex_id << " " << p.x() << " " << p.y() << std::endl;
			++vertex_id;
		}
		for(const auto &h:holes_) {
			for(const auto &p:h) {
				outfile << vertex_id << " " << p.x() << " " << p.y() << std::endl;
				++vertex_id;
			}
		}

		outfile.close();
		outfile = std::ofstream (edges_filename);
		size_t node_count = 0;
		for(size_t i = 0; i < parallel_tracks_edges_.size(); ++i) {
			if(i > 0)
				node_count += tracks_vertices_[i - 1].size();
			auto edge_list = parallel_tracks_edges_[i];
			for(const auto &e:edge_list) {
				outfile << node_count + std::get<0>(e) << " " << node_count + std::get<1>(e) << std::endl;
			}
		}
		outfile.close();
	}

	void Env::WriteEnv(const std::string &filename) const {
		std::ofstream outfile (filename);
		PrintPolygon(outer_polygon_, outfile);
		for(const auto &h:holes_) {
			PrintPolygon(h, outfile);
		}
		outfile.close();
	}

	void Env::WriteEnvInput(const std::string &filename) const {
		std::ofstream outfile (filename);
		PrintPolygon(outer_polygon_input_, outfile);
		for(const auto &h:holes_input_) {
			PrintPolygon(h, outfile);
		}
		outfile.close();
	}

	void Env::WriteOuterPolygonGnuplot(const std::string & filename) const {
		std::ofstream outfile (filename);
		auto poly = outer_polygon_;
		if(HasRobot()) {
			poly = outer_polygon_input_;
		}
		for(const auto &p:poly) {
			outfile << p[0] << " " << p[1] << std::endl;
		}
		outfile.close();
	}

	void Env::WriteHolesGnuplot(const std::string &filename) const {
		auto holes = holes_;
		if(HasRobot()) {
			holes = holes_input_;
		}
		size_t hole_count = 0;
		for(const auto h:holes) {
			std::ofstream outfile (filename + std::to_string(hole_count));
			for(const auto &p:h) {
				outfile << p[0] << " " << p[1] << std::endl;
			}
			outfile.close();
			++hole_count;
		}
	}
}
