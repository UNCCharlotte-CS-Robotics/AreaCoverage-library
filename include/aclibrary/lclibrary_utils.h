/**
 * This file is part of the AreaCoverage-library.
 * The file contains function to create graph for line coverage
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

#ifndef ACLIBRARY_LCLIBRARY_UTILS_H_
#define ACLIBRARY_LCLIBRARY_UTILS_H_

#include <memory>
#include <aclibrary/service_tracks.h>
#include <aclibrary/polygon_utilities.h>
#include <lclibrary/core/core.h>
#include <lclibrary/core/transform_lla_xy.h>
#include <CGAL/centroid.h>
namespace aclibrary {
	// Define the used visibility class

	/* bool ComputeVisibilityRegion(const Arrangement_2 &arr, const Point_2 &q, Polygon_2 &visibility_polygon); */
	inline void FilterEdge(lclibrary::Graph *G, size_t i) {
		G->SetCosts(i, kDoubleMax, kIsNotRequired);
	}

	void FilterNonRequiredEdges(const Env *env, lclibrary::Graph *G) {
		if(env->HasHoles() == false)
			return;

		for(size_t i = 0; i < G->GetMnr(); ++i) {
			lclibrary::Vec2d u, v;
			G->GetVertexCoordinateofEdge(i, u, v, kIsNotRequired);
			Point_2 up(u.x, u.y);
			Point_2 vp(v.x, v.y);
			if(env->CheckPointInHoles(up) == true or env->CheckPointInHoles(vp)) {
				FilterEdge(G, i);
				continue;
			}
			if(env->CheckIntersectsHoleEdge(up, vp)) {
				FilterEdge(G, i);
				continue;
			}
		}

	}

	inline std::shared_ptr <lclibrary::Graph> GenerateLCGraph(Env &env, std::string filename, ServiceTracks *service_tracks, bool add_all_nreq, Point_2 depot, bool is_depot_defined = false, std::shared_ptr <lclibrary::LLAtoXY> lla_to_xy = nullptr) {
		std::vector <Point_2> vertices;
		/* vertices.push_back(Point_2(0,0)); */
		if(is_depot_defined == false) {
			vertices.push_back(env.GetVertexClosesetToCentroid());
			std::cout << "Depot: " << env.GetVertexClosesetToCentroid() << std::endl;
		} else {
			vertices.push_back(depot);
		}
		/* std::cout << "Note: depot set to (0,0). It may be outside the polygon."; */
		std::vector <std::pair<int, int>> edges;
		if(service_tracks != nullptr) {
			service_tracks->GetTracks(vertices, edges);
		}
		else {
			env.GetTracks(vertices, edges);
		}
		/* env.GetSweepTracks(vertices, edges); */
		/* env.GetServiceTracks(filename, vertices, edges); */
		std::cout << "Num of vertices, edges: " << vertices.size() << " " << edges.size() << std::endl;
		std::vector <lclibrary::Vertex> lc_vertices;
		for(size_t i = 0; i < vertices.size(); ++i) {
			lclibrary::Vertex lc_v(i);
			lc_v.SetXY(CGAL::to_double(vertices[i].x()), CGAL::to_double(vertices[i].y()));
			if(lla_to_xy != nullptr) {
				double lat, lon, alt;
				lla_to_xy->flatToLLA(CGAL::to_double(vertices[i].x()), CGAL::to_double(vertices[i].y()), 0.0, lat, lon, alt);
				lc_v.SetLLA(lat, lon, alt);
			}
			lc_vertices.push_back(lc_v);
		}
		std::vector <lclibrary::Edge> lc_edges;
		for(const auto &e:edges) {
			lclibrary::Edge lc_e(std::get<0>(e) + 1, std::get<1>(e) + 1, lclibrary::kIsRequired);
			lc_edges.push_back(lc_e);
		}
		size_t n = lc_vertices.size();
		size_t m = lc_edges.size();
		size_t m_nr = 0;

		Arrangement_2 arr;
		if(add_all_nreq == true) {
			if(env.HasRobot()) {
				env.ComputeOuterOffsetArrangement(arr);
			} else {
				env.ComputeOuterArrangement(arr);
			}
		}
		else {
			if(env.HasRobot()) {
				env.ComputeOffsetArrangement(arr);
			}
			env.ComputeArrangement(arr);
		}
		for(size_t i = 0; i < n; ++i) {
			auto q = vertices[i];
			Polygon_2 visibility_polygon;
			if(ComputeVisibilityRegion(arr, q, visibility_polygon) == true) {
				continue;
			}
			for (size_t j = i + 1; j < n; ++j) {
				auto p = vertices[j];
				if(CGAL::bounded_side_2(visibility_polygon.begin(), visibility_polygon.end(), p) ==  CGAL::ON_UNBOUNDED_SIDE) {
					continue;
				}
				lclibrary::Edge new_edge(lc_vertices[i].GetID(), lc_vertices[j].GetID(), kIsNotRequired);
				lc_edges.push_back(new_edge);
				++m_nr;
			}
		}
		auto g = std::make_shared <lclibrary::Graph> (lc_vertices, lc_edges, m, m_nr);
		g->SetDepot(0);
		double depot_x, depot_y;
		g->GetDepotXY(depot_x, depot_y);
		std::cout << depot_x << " depot " << depot_y << std::endl;
		return g;
	}

	inline void AddCompleteNonRequiredEdgesVisibility(const Env *env, lclibrary::Graph *G) {
		Arrangement_2 arr;
		env->ComputeArrangement(arr);
		auto n = G->GetN();
		size_t m_nr = n * (n - 1)/2;
		std::vector <lclibrary::Edge> edge_list;
		edge_list.reserve(m_nr);
		std::cout << "Number of vertices: " << n << std::endl;
		for(size_t i = 0; i < n; ++i) {
			lclibrary::Vec2d xy;
			G->GetVertexXY(i, xy);
			Point_2 q(xy.x, xy.y);
			Polygon_2 visibility_polygon;
			if(ComputeVisibilityRegion(arr, q, visibility_polygon) == true) {
				continue;
			}
			for (size_t j = i + 1; j < n; ++j) {
				G->GetVertexXY(j, xy);
				Point_2 p(xy.x, xy.y);
				if(CGAL::bounded_side_2(visibility_polygon.begin(), visibility_polygon.end(), p) ==  CGAL::ON_UNBOUNDED_SIDE) {
					continue;
				}
				lclibrary::Edge new_edge(G->GetVertexID(i), G->GetVertexID(j), kIsNotRequired);
				edge_list.push_back(new_edge);
			}
		}
		G->AddEdge(edge_list, size_t(0), edge_list.size());
	}

	inline void AddCompleteNonRequiredEdges(const Env *env, lclibrary::Graph *G) {
		auto n = G->GetN();
		size_t m_nr = n * (n - 1)/2;
		std::vector <lclibrary::Edge> edge_list;
		edge_list.reserve(m_nr);
		for(size_t i = 0; i < n; ++i) {
			for (size_t j = i + 1; j < n; ++j) {
				if(i == 159 and j == 168) {
					std::cout << "Found\n";
					lclibrary::Vec2d u, v;
					G->GetVertexXY(i, u);
					G->GetVertexXY(j, v);
					Point_2 up(u.x, u.y);
					Point_2 vp(v.x, v.y);
					std::cout << up << std::endl;
					std::cout << vp << std::endl;
					if(env->CheckIntersectsHoleEdge(up, vp)) {
						std::cout << "intersecting\n";
					}
					std::cout << "PoH: " << env->CheckPointOnHole(4, up) << std::endl;
					std::cout << "PoH: " << env->CheckPointOnHole(4, vp) << std::endl;
					std::cout << G->GetVertexID(i) << std::endl;
				}

				lclibrary::Vec2d u, v;
				G->GetVertexXY(i, u);
				G->GetVertexXY(j, v);
				Point_2 up(u.x, u.y);
				Point_2 vp(v.x, v.y);
				/* if(env->CheckPointInHoles(up) == true or env->CheckPointInHoles(vp)) { */
				/* 	continue; */
				/* } */
				if(env->CheckIntersectsHoleEdge(up, vp)) {
					continue;
				}
				lclibrary::Edge new_edge(G->GetVertexID(i), G->GetVertexID(j), kIsNotRequired);
				edge_list.push_back(new_edge);
			}
		}
		G->AddEdge(edge_list, size_t(0), edge_list.size());
	}

	std::shared_ptr <lclibrary::LLAtoXY> ConfigureGeoData(Config &config, Point_2 &depot) {

		struct Node {
			size_t id;
			double lat, lon, alt;
			double x, y, z;
		};

		std::string out_filename = config.database.dir + "nodes_data";
		auto lla_to_xy_ptr = std::make_shared <lclibrary::LLAtoXY> (config.database.dir + config.nodes_ll, out_filename);

		std::vector <Node> node_list;
		std::ifstream node_file(out_filename);
		Node new_node;
		while(node_file >> new_node.id >> new_node.x >> new_node.y >> new_node.lat >> new_node.lon >> new_node.alt) {
			node_list.push_back(new_node);
		}
		node_file.close();

		std::unordered_map <size_t, size_t> node_map; /*! Stores a map of the index of vertices to actual ID of the node <ID, index>*/
		for(size_t i = 0; i < node_list.size(); ++i) {
			node_map[node_list[i].id] = i;
		}

		std::ofstream outer_polygon_file(config.database.dir + config.filenames.outer_polygon);
		std::ifstream outer_polygon_nodefile(config.database.dir + config.outer_nodes);
		size_t node_id;
		outer_polygon_file.precision(16);
		while(outer_polygon_nodefile >> node_id) {
			auto node = node_map[node_id];
			outer_polygon_file << node_list[node].x << " " << node_list[node].y << std::endl;
		}
		outer_polygon_file.close();
		outer_polygon_nodefile.close();

		std::ofstream holes_file(config.database.dir + config.filenames.holes);
		std::ifstream holes_nodefile(config.database.dir + config.holes_nodes);
		holes_file.precision(16);
		while(holes_nodefile >> node_id) {
			auto node = node_map[node_id];
			holes_file << node_list[node].x << " " << node_list[node].y << std::endl;
		}
		holes_file.close();
		holes_nodefile.close();

		depot = Point_2(node_list[0].x, node_list[0].y);
		return lla_to_xy_ptr;

	}


}

#endif /* ACLIBRARY_LCLIBRARY_UTILS_H_ */
