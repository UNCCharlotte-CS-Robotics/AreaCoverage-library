/**
 * This file is part of the AreaCoverage-library.
 * The file provides the main function for custom robot type
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

#include <aclibrary_config.h>
#include <aclibrary/config.h>
#include <aclibrary/cgal_config.h>

#include <aclibrary/env.h>
#include <aclibrary/plot_graph.h>
#include <aclibrary/lclibrary_utils.h>
#include <aclibrary/service_tracks.h>

#include <lclibrary/mlc/mlc.h>
#include <lclibrary/core/graph_io.h>

#include <string>
#include <chrono>
#include <memory>
#include <iostream>

int main (int argc, char **argv) {

	if(argc < 2) {
		std::cerr << "Usage: " << argv[0] << " <config_file.yaml" << std::endl;
		return 1;
	}

	std::string config_file = argv[1];
	aclibrary::Config config(config_file);

	if(argc == 3) {
		std::string data_dir = argv[2];
		config = aclibrary::Config(config_file, data_dir);
	}

	if(argc == 4) {
		std::string data_dir = argv[2];
		double capacity = std::stod(argv[3]);
		config = aclibrary::Config(config_file, data_dir, capacity);
	}

	if (config.ParseConfig()) {
		std::cerr << "Config parsing failed with config file: " << config_file << std::endl;
	}

	if(config.robot_model != "custom") {
		std::cerr << "The robot model should be set to custom\n";
		return 1;
	}
	std::chrono::time_point<std::chrono::system_clock> time_start_decom;
	std::chrono::time_point<std::chrono::system_clock> time_end_decom;

	time_start_decom = std::chrono::system_clock::now();

	auto filenames = config.filenames;

	Point_2 depot(0, 0);
	bool use_depot = false;
	std::shared_ptr <lclibrary::LLAtoXY> lla_to_xy_ptr = nullptr;
	if(config.env_with_geo == true) {
		lla_to_xy_ptr = ConfigureGeoData(config, depot);
		use_depot = true;
	}


	aclibrary::Env env(config.database.dir + filenames.outer_polygon, config.database.dir + filenames.holes, config.database.dir + filenames.robot_polygon, config.field_of_view);

	env.WriteEnv(config.database.dir + filenames.env_data);
	env.WriteEnvInput(config.database.dir + filenames.env_data + "_input");

	env.PrintArea();

	env.SolveBCDmerge();
	time_end_decom = std::chrono::system_clock::now();

	auto time_start_service = std::chrono::system_clock::now();

	aclibrary::ServiceTracks service = aclibrary::ServiceTracks(env, config.field_of_view, true);

	std::cout << "Service tracks length: " << service.ComputeServiceTrackLength() << std::endl;
	auto time_end_service = std::chrono::system_clock::now();

	auto time_start_routing = std::chrono::system_clock::now();
	std::shared_ptr <lclibrary::Graph> graph;
	graph = GenerateLCGraph(env, config.database.dir + config.filenames.tracks, &service, config.travel_over_holes, depot, use_depot, lla_to_xy_ptr);

	if(config.cost_function == "ramp") {
		graph->SetRampEdgeCosts(config.ramp.acceleration, config.ramp.speed);
	}
	else if(config.cost_function == "travel_time") {
		auto params = config.travel_time;
		lclibrary::EdgeCost_TravelTime edge_cost_fn (params.service_speed, params.deadhead_speed, params.wind_speed, params.wind_dir);
		lclibrary::ComputeAllEdgeCosts(graph, edge_cost_fn);
	} else if(config.cost_function == "euclidean") {
		graph->SetDefaultEdgeCosts();
	} else {
		std::cerr << "Unknown cost model\n";
		return 1;
	}

	graph->SetCapacity(config.capacity);
	graph->SetDemandsToCosts();
	/* lclibrary::WriteNodes(graph, config.sol_dir + "node_data"); */
	/* lclibrary::WriteNonRequiredEdges(graph, config.sol_dir + "non_required_edge_list"); */

	auto mlc_solver = std::make_unique <lclibrary::MLC_MEM> (graph);
	mlc_solver->Use2Opt(false);

	if(mlc_solver->Solve()) {
		return 1;
	}

	auto time_end_routing = std::chrono::system_clock::now();

	std::chrono::duration<double> duration_decom = time_end_decom - time_start_decom;
	std::chrono::duration<double> duration_service = time_end_service - time_start_service;
	std::chrono::duration<double> duration_routing = time_end_routing - time_start_routing;

	double depot_x, depot_y;
	graph->GetDepotXY(depot_x, depot_y);
	graph->PrintNM();

	env.WriteInitDecomposition(config.sol_dir + config.filenames.init_decomp);
	env.WriteFinalDecomposition(config.sol_dir + config.filenames.final_decomp);

	service.WriteServiceTracks(config.sol_dir + config.filenames.tracks);
	service.WriteCoverage(config.sol_dir + config.filenames.coverage);


	size_t num_holes = env.GetNumHoles();

	std::string plot_temp_dir = config.database.dir + "plot/";
	std::string plot_data_filename = plot_temp_dir + "plot_data";
	std::string gnuplot_filename = plot_temp_dir + "plot.gp";
	std::string output_filename = config.database.dir + config.plot_service_tracks.name;

	std::filesystem::create_directory(plot_temp_dir);

	env.WriteHolesGnuplot(config.database.dir + "/plot/hole");
	aclibrary::GnuplotMap(graph, config.database.dir + config.filenames.outer_polygon, config.database.dir + "/plot/hole", plot_data_filename, gnuplot_filename, output_filename, config.plot_service_tracks.plot_nreq_edges, num_holes);

	auto gnuplot_status = std::system(("gnuplot " + gnuplot_filename).c_str());
	std::filesystem::remove_all(plot_temp_dir);
	if(gnuplot_status != 0) {
		std::cerr << "gnuplot failed\n";
	}

	plot_temp_dir = config.sol_dir + "plot/";
	plot_data_filename = plot_temp_dir + "plot_data";
	gnuplot_filename = plot_temp_dir + "plot.gp";

	output_filename = config.sol_dir + "routes";

	std::vector <std::shared_ptr <const lclibrary::Graph>> sol_digraph_list;
	mlc_solver->GetSolDigraphList(sol_digraph_list);

	std::filesystem::create_directories(plot_temp_dir);
	env.WriteHolesGnuplot(plot_temp_dir + "hole");
	aclibrary::GnuplotMap(sol_digraph_list, config.database.dir + config.filenames.outer_polygon, plot_temp_dir + "hole", plot_data_filename, gnuplot_filename, output_filename, true, depot_x, depot_y, num_holes);
	gnuplot_status = std::system(("gnuplot " + gnuplot_filename).c_str());
	if(gnuplot_status != 0) {
		std::cerr << "gnuplot failed\n";
	}
	std::filesystem::remove_all(plot_temp_dir);

	std::cout << std::boolalpha;
	std::cout << "soldigraphlist size: " << sol_digraph_list.size() << std::endl;
	std::cout << "Solution check: " << mlc_solver->CheckSolution() << std::endl;
	std::cout << "Number of routes: " << mlc_solver->GetNumOfRoutes() << std::endl;
	std::cout << "Total cost: " << mlc_solver->GetRouteCost() << std::endl;

	std::string route_data_filename = config.sol_dir + config.filenames.route_data;
	mlc_solver->WriteWaypointsRoutes(route_data_filename);

	std::string vertex_filename = config.database.dir + "/node_data";
	std::string required_edge_filename = config.database.dir + "/req_edge_list";
	std::string non_required_edge_filename = config.database.dir + "/non_req_edge_list";
	lclibrary::WriteNodes(graph, vertex_filename);
	lclibrary::WriteRequiredEdges(graph, required_edge_filename);
	lclibrary::WriteNonRequiredEdges(graph, non_required_edge_filename);


	std::string results_filename = config.sol_dir + config.filenames.results;
	std::ofstream outfile (results_filename, std::ofstream::app);
	outfile << config.database.data_dir << " ";
	outfile.close();
	env.WriteResults(results_filename);
	outfile.open(results_filename, std::ofstream::app);
	outfile << mlc_solver->GetRouteCost() << " " << mlc_solver->GetTotalNumTurns() << " " << mlc_solver->GetNumOfRoutes() << " ";
	outfile << service.ComputeServiceTrackLength() << " ";
	outfile << duration_decom.count() << " ";
	outfile << duration_service.count() << " ";
	outfile << duration_routing.count() << std::endl;
	outfile.close();

	return 0;
}
