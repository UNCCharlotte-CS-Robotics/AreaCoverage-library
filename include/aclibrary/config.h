/**
 * This file is part of the AreaCoverage-library.
 * The file contains class for parsing and storing configuration. Uses yaml-cpp.
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

#ifndef ACLIBRARY_CORE_CONFIG_H_
#define ACLIBRARY_CORE_CONFIG_H_

#include <aclibrary/constants.h>
#include <yaml-cpp/yaml.h>

#include <string>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <cmath>

namespace aclibrary {

	class Config {

		public:
			std::string config_file_;

			struct Database {
				std::string path;
				std::string data_dir;
				std::string dir;
				bool arg = false;
			} database;

			std::string sol_dir;

			double field_of_view = 10;
			bool travel_over_holes = false;
			bool add_boundary_edges = false;

			std::string robot_model;
			bool offset_outer_boundary = false;
			bool minkowski_sum = false;

			bool env_with_geo = false;

			std::string nodes_ll, outer_nodes, holes_nodes;

			struct Filenames {
				std::string outer_polygon;
				std::string holes;
				std::string robot_polygon;
				std::string env_data;
				std::string tracks;
				std::string init_decomp;
				std::string final_decomp;
				std::string coverage;
				std::string results;
				std::string route_data;
			} filenames;

			enum DepotsMode {cluster_auto, cluster, user } depots_mode;
			size_t depot_ID = 0;
			std::vector <size_t> depot_IDs;
			size_t num_depots;
			size_t num_runs;
			bool use_seed = false;
			double seed = 0;

			bool use_2opt;

			struct PlotServiceTracks {
				bool plot = true;
				bool plot_nreq_edges = false;
				std::string name;
			} plot_service_tracks;

			struct WriteGeoJSON {
				bool write = false;
				bool non_req_edges = false;
				std::string filename;
				std::string var_name;
			} writeGeoJSON;

			double capacity;
			double discretize_len;
			bool cap_arg = false;

			std::string cost_function;
			struct TravelTime {
				double service_speed;
				double deadhead_speed;
				double wind_speed;
				double wind_dir;
			} travel_time;

			struct Ramp {
				double speed;
				double acceleration;
			} ramp;

			struct Output {
				bool env_data = false;
				bool video = false;
				bool kml = false;
				bool data = false;
				bool edge_data = false;
				bool geojson = false;
				bool agg_results = false;
				bool append = false;
				bool clear_dir = false;
			} output;

			Config () {}

			Config (const std::string &config_file) {
				config_file_ = config_file;
			}

			Config (const std::string &config_file, const std::string &data_dir):Config(config_file) {
				database.data_dir = data_dir;
				database.arg = true;
			}

			Config (const std::string &config_file, const std::string &data_dir, const double cap):Config(config_file, data_dir) {
				capacity = cap;
				cap_arg = true;
			}

			void CopyConfig(const std::string &filename) const {
				std::filesystem::copy(config_file_, filename, std::filesystem::copy_options::overwrite_existing);
			}

			int WriteConfig(const std::string &filename) const {
				YAML::Node yaml_config_ = YAML::LoadFile(config_file_);
				if(database.arg == true) {
					yaml_config_["database"]["data_dir"] = database.data_dir;
				}

				if(cap_arg == true) {
					yaml_config_["capacity"] = capacity;
				}

				std::ofstream fout(filename);
				fout << yaml_config_;
				fout.close();
				return kSuccess;
			}

			int ParseConfig () {
				std::cout << "Using config file: " << config_file_ << std::endl;
				if(not std::filesystem::exists(config_file_)) {
					std::cerr << "Could not find config file " << config_file_ << std::endl;
					return kFail;
				}
				YAML::Node yaml_config_ = YAML::LoadFile(config_file_);

				if(database.arg == true) {
					yaml_config_["database"]["data_dir"] = database.data_dir;
				}

				if(cap_arg == true) {
					yaml_config_["capacity"] = capacity;
				}

				auto database_yaml = yaml_config_["database"];
				database.path = database_yaml["path"].as<std::string>();
				database.data_dir = database_yaml["data_dir"].as<std::string>();
				database.dir = database.path + "/" + database.data_dir + "/";

				sol_dir = database.dir + "mem/";

				if(not std::filesystem::exists(database.dir)) {
					std::cerr << "Database does not exist\n";
					std::cerr << database.dir << std::endl;
					return kFail;
				}

				field_of_view = yaml_config_["field_of_view"].as<double>();
				travel_over_holes = yaml_config_["travel_over_holes"].as<bool>();
				add_boundary_edges = yaml_config_["add_boundary_edges"].as<bool>();

				robot_model = yaml_config_["robot_model"].as<std::string>();
				offset_outer_boundary = yaml_config_["offset_outer_boundary"].as<bool>();
				minkowski_sum = yaml_config_["minkowski_sum"].as<bool>();

				env_with_geo = yaml_config_["env_with_geo"].as<bool>();
				if(env_with_geo == true) {
					nodes_ll = yaml_config_["geo_filenames"]["nodes_ll"].as<std::string>();
					outer_nodes = yaml_config_["geo_filenames"]["outer_nodes"].as<std::string>();
					holes_nodes = yaml_config_["geo_filenames"]["holes_nodes"].as<std::string>();
				}

				auto filenames_yaml = yaml_config_["filenames"];
				filenames.outer_polygon = filenames_yaml["outer_polygon"].as<std::string>();
				filenames.holes = filenames_yaml["holes"].as<std::string>();
				filenames.robot_polygon = filenames_yaml["robot_polygon"].as<std::string>();
				filenames.env_data = filenames_yaml["env_data"].as<std::string>();
				filenames.tracks = filenames_yaml["tracks"].as<std::string>();
				filenames.init_decomp = filenames_yaml["init_decomp"].as<std::string>();
				filenames.final_decomp = filenames_yaml["final_decomp"].as<std::string>();
				filenames.coverage = filenames_yaml["coverage"].as<std::string>();
				filenames.results = filenames_yaml["results"].as<std::string>();
				filenames.route_data = filenames_yaml["route_data"].as<std::string>();

				std::string depots_config = yaml_config_["depots"]["mode"].as<std::string>();
				if(depots_config == "cluster_auto") {
					depots_mode = cluster_auto;
				} else if (depots_config == "cluster"){
					depots_mode = cluster;
					num_depots = yaml_config_["depots"]["num_depots"].as<size_t>();
				} else if (depots_config == "user") {
					depots_mode = user;
					auto yaml_depots = yaml_config_["depots"]["IDs"];
					for(YAML::const_iterator it = yaml_depots.begin(); it != yaml_depots.end(); ++it) {
						depot_IDs.push_back(it->as<size_t>());
					}
				}
				use_seed = yaml_config_["depots"]["use_seed"].as<bool>();
				if(use_seed) {
					seed = yaml_config_["depots"]["seed"].as<double>();
				}
				num_runs = yaml_config_["depots"]["num_runs"].as<size_t>();

				plot_service_tracks.plot = yaml_config_["plot_service_tracks"]["plot"].as<bool>();
				plot_service_tracks.plot_nreq_edges = yaml_config_["plot_service_tracks"]["plot_nreq_edges"].as<bool>();
				plot_service_tracks.name = yaml_config_["plot_service_tracks"]["name"].as<std::string>();

				auto writeGeoJSON_yaml = yaml_config_["writeGeoJSON"];
				writeGeoJSON.write = writeGeoJSON_yaml["write"].as<bool>();
				writeGeoJSON.non_req_edges = writeGeoJSON_yaml["non_req_edges"].as<bool>();
				writeGeoJSON.filename = writeGeoJSON_yaml["filename"].as<std::string>();
				writeGeoJSON.var_name = writeGeoJSON_yaml["var_name"].as<std::string>();

				capacity = yaml_config_["capacity"].as<double>();

				use_2opt = yaml_config_["use_2opt"].as<bool>();
				discretize_len = yaml_config_["discretize_len"].as<double>();

				cost_function = yaml_config_["cost_function"].as<std::string>();
				if(cost_function == "travel_time") {
					auto travel_time_yaml = yaml_config_["travel_time_config"];
					travel_time.service_speed = travel_time_yaml["service_speed"].as<double>();
					travel_time.deadhead_speed = travel_time_yaml["deadhead_speed"].as<double>();
					travel_time.wind_speed = travel_time_yaml["wind_speed"].as<double>();
					travel_time.wind_dir = travel_time_yaml["wind_dir"].as<double>() * M_PI/180.;
				}

				if(cost_function == "ramp") {
					auto ramp_yaml = yaml_config_["ramp_config"];
					ramp.speed = ramp_yaml["speed"].as<double>();
					ramp.acceleration = ramp_yaml["acceleration"].as<double>();
				}

				auto output_yaml = yaml_config_["output"];
				output.env_data = output_yaml["env_data"].as<bool>();
				output.kml = output_yaml["kml"].as<bool>();
				output.video = output_yaml["video"].as<bool>();
				output.data = output_yaml["data"].as<bool>();
				output.edge_data = output_yaml["edge_data"].as<bool>();
				output.geojson = output_yaml["geojson"].as<bool>();
				output.agg_results = output_yaml["agg_results"].as<bool>();
				output.append = output_yaml["append"].as<bool>();
				output.clear_dir = output_yaml["clear_dir"].as<bool>();
				if(std::filesystem::exists(sol_dir)) {
					if(output.clear_dir) {
						std::filesystem::remove_all(sol_dir);
					}
				}

				return kSuccess;
			}

	};

} // namespace lclibrary

#endif /* ACLIBRARY_CORE_CONFIG_H_ */
