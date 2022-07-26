/**
 * This file is part of the AreaCoverage-library.
 * The file contains functins for plotting using Gnuplot
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

#include <cmath>
#include <cstring>
#include <lclibrary/core/core.h>
#include <lclibrary/utils/utils.h>
#include <lclibrary/algorithms/algorithms.h>
#include <lclibrary/mlc/mlc.h>

namespace aclibrary {

	/*! Write required edges into a file in format suitable for gnuplot */
	inline void PlotDataRequiredEdges(
			std::shared_ptr <const lclibrary::Graph> G,
			const std::string filename,
			std::string color = "0"){

		std::ofstream out_file (filename);
		out_file.precision(16);
		size_t m = G->GetM();
		for(size_t i = 0; i < m; ++i) {
			lclibrary::Vec2d tail_xy, head_xy;
			G->GetVertexCoordinateofEdge(i, tail_xy, head_xy, true);
			out_file << tail_xy.x << " " << tail_xy.y << " " << color << "\n";
			out_file << head_xy.x << " " << head_xy.y << " " << color << "\n";
			out_file <<"\n";
		}
		out_file.close();
	}

	inline bool PlotDataNonRequiredEdges(
			std::shared_ptr <const lclibrary::Graph> G,
			const std::string filename,
			std::string color = "0"){

		std::ofstream out_file (filename);
		out_file.precision(16);
		size_t m_nr = G->GetMnr();
		bool exists_flag = false;
		for(size_t i = 0; i < m_nr; ++i) {
			exists_flag = true;
			lclibrary::Vec2d tail_xy, head_xy;
			G->GetVertexCoordinateofEdge(i, tail_xy, head_xy, false);
			out_file << tail_xy.x << " " << tail_xy.y << " " << color << "\n";
			out_file << head_xy.x << " " << head_xy.y << " " << color << "\n";
			out_file <<"\n";
		}
		out_file.close();
		return exists_flag;
	}

	inline void GnuplotMap(
			std::shared_ptr <const lclibrary::Graph> G,
			const std::string poly_file_name,
			const std::string holes_file_name,
			const std::string data_file_name,
			const std::string gnuplot_file_name,
			const std::string output_plot_file_name,
			bool plot_non_required,
			size_t num_holes = 0) {

		aclibrary::PlotDataRequiredEdges(G, data_file_name);
		if(plot_non_required) {
			plot_non_required = aclibrary::PlotDataNonRequiredEdges(G, data_file_name + "_nr");
		}
		double minX, maxX, minY, maxY;
		G->GetLimits(minX, maxX, minY, maxY);
		std::ofstream out_file (gnuplot_file_name);
		out_file.precision(16);
		double ratioXY;
		if(maxX - minX < 1e-10)
			ratioXY = 1;
		else
			ratioXY = (maxY - minY)/(maxX - minX);
		double size_x = 12;
		double size_y = ratioXY * size_x;
		double size_x_PNG = 900;
		double size_y_PNG = ratioXY * size_x_PNG;
		int scale_order = std::max((int(log10 (maxX-minX))), (int(log10 (maxY-minY))));
		if(scale_order < 1)
			scale_order = 1;
		double scale = std::pow(10, scale_order);
		out_file << "set terminal pdfcairo enhanced font 'Times,14' size " << size_x <<"cm, " << size_y << "cm crop\n";
		out_file << "set o \"" << output_plot_file_name << ".pdf\"\n";
		out_file << "unset key\nunset colorbox\nunset grid\n";
		out_file << "set cbrange [0:19]\n";
		out_file << "set xrange [" << minX/scale - 0.05 << ":" << maxX/scale + 0.05 << "]\n";
		out_file << "set yrange [" << minY/scale - 0.05 << ":" << maxY/scale + 0.05 << "]\n";
		/* out_file << "set palette defined (0 \"#d62728\", 1 \"#1f77b4\", 2 \"#2ca02c\", 3 \"#9467bd\", 4 \"#000000\", 5 \"#8c564b\", 6 \"#e377c2\", 7 \"#ff7f0e\", 8 \"#bcbd22\", 9 \"#17becf\", 10 \"#d69f9f\", 11 \"#89b2cf\", 12 \"#91ca91\", 13 \"#beabcf\", 14 \"#dbdbdb\", 15 \"#bfa6a1\", 16 \"#d5b6cc\", 17 \"#eeb98b\", 18 \"#d5d5a2\", 19 \"#a3ccd1\")\n"; */
		out_file << "set palette defined (0 '#2980b9', 1 '#cd6155', 2 \"#d62728\", 3 \"#9467bd\", 4 \"#000000\", 5 \"#8c564b\", 6 \"#e377c2\", 7 \"#ff7f0e\", 8 \"#bcbd22\", 9 \"#17becf\", 10 \"#d69f9f\", 11 \"#89b2cf\", 12 \"#91ca91\", 13 \"#beabcf\", 14 \"#dbdbdb\", 15 \"#bfa6a1\", 16 \"#d5b6cc\", 17 \"#eeb98b\", 18 \"#d5d5a2\", 19 \"#a3ccd1\")\n";
		out_file << "set xlabel \'X-axis (x 10^"<< scale_order<<" m)\' offset 0,1.0 \n";
		out_file << "set ylabel 'Y-axis (x 10^"<< scale_order <<" m)' offset 2.0,0\n";
		out_file << "set size ratio -1\n";
		out_file << "set xtics -10,0.5,10\n";
		out_file << "set ytics -10,0.5,10\n";
		out_file << "set style textbox noborder\n";
		out_file << "set xtics border offset -0.0,0.3\n";
		if(num_holes > 0) {
			out_file << "plot \"" << poly_file_name << "\" u ($1/"<<scale<<"):($2/"<<scale <<") with filledcurves closed lw 2 lc rgbcolor \"gray40\" fill transparent solid 0,";
			for(size_t ii = 0; ii < num_holes; ++ii) {
				out_file << "\"" << holes_file_name + std::to_string(ii) << "\" u ($1/"<<scale<<"):($2/"<<scale <<") w filledcurves closed lw 2 lc rgbcolor \"gray40\" fill transparent solid 0.4,";
			}
		}
		else {
			out_file << "plot \"" << poly_file_name << "\" u ($1/"<<scale<<"):($2/"<<scale <<") w filledcurves closed lw 2 lc rgbcolor \"gray40\" fill transparent solid 0,";
		}
		if(plot_non_required) {
			out_file << "\"" << data_file_name << "\" u ($1/" << scale << "):($2/" << scale <<"):3  w lines lw 1 palette, ";
			out_file << "\"" << data_file_name + "_nr" << "\" u ($1/"<<scale<<"):($2/"<<scale <<"):3  w lines lw 1 dashtype 2 palette\n";
		}
		else {
			out_file << "\"" << data_file_name << "\" u ($1/"<<scale<<"):($2/"<<scale<<"):3  w lines lw 1 palette\n";
		}

		out_file << "set terminal pngcairo enhanced font 'Times,14' size " << size_x_PNG << ", " << size_y_PNG << std::endl;
		out_file << "set o \"" << output_plot_file_name << ".png\"" << std::endl;
		out_file << "replot";
		out_file.close();
	}


	void GnuplotMap(
			const std::vector <std::shared_ptr <const lclibrary::Graph>> &graph_list,
			const std::string poly_file_name,
			const std::string holes_file_name,
			const std::string data_file_name,
			const std::string gnuplot_file_name,
			const std::string output_plot_file_name,
			bool plot_non_required,
			double depot_x,
			double depot_y,
			size_t num_holes = 0) {

		if(graph_list.empty()) {
			return;
		}
		auto G = graph_list[0];
		std::cout << depot_x << " depot " << depot_y << std::endl;

		double minX, maxX, minY, maxY;
		G->GetLimits(minX, maxX, minY, maxY);
		std::ofstream out_file (gnuplot_file_name);
		out_file.precision(16);
		double ratioXY;
		if(maxX - minX < 1e-10)
			ratioXY = 1;
		else
			ratioXY = (maxY - minY)/(maxX - minX);
		double size_x = 12;
		double size_y = ratioXY * size_x;
		double size_x_PNG = 900;
		double size_y_PNG = ratioXY * size_x_PNG;
		int scale_order = std::max((int(log10 (maxX-minX))), (int(log10 (maxY-minY))));
		if(scale_order < 1)
			scale_order = 1;
		double scale = std::pow(10, scale_order);
		scale = 1;
		scale_order = 1;
		out_file << "set terminal pdfcairo enhanced font 'Times,14' size " << size_x <<"cm, " << size_y << "cm crop\n";
		out_file << "set o \"" << output_plot_file_name << ".pdf\"\n";
		out_file << "unset key\nunset colorbox\nunset grid\n";
		out_file << "set cbrange [0:19]\n";
		out_file << "set xrange [" << minX/scale - 5 << ":" << maxX/scale + 5 << "]\n";
		out_file << "set yrange [" << minY/scale - 5 << ":" << maxY/scale + 5 << "]\n";
		/* out_file << "set palette defined (0 \"#d62728\", 1 \"#1f77b4\", 2 \"#2ca02c\", 3 \"#9467bd\", 4 \"#000000\", 5 \"#8c564b\", 6 \"#e377c2\", 7 \"#ff7f0e\", 8 \"#bcbd22\", 9 \"#17becf\", 10 \"#d69f9f\", 11 \"#89b2cf\", 12 \"#91ca91\", 13 \"#beabcf\", 14 \"#dbdbdb\", 15 \"#bfa6a1\", 16 \"#d5b6cc\", 17 \"#eeb98b\", 18 \"#d5d5a2\", 19 \"#a3ccd1\")\n"; */
		/* out_file << "set palette defined (0 \"#1f77b4\", 1 \"#d62728\", 2 \"#2ca02c\", 3 \"#9467bd\", 4 \"#000000\", 5 \"#8c564b\", 6 \"#e377c2\", 7 \"#ff7f0e\", 8 \"#bcbd22\", 9 \"#17becf\", 10 \"#d69f9f\", 11 \"#89b2cf\", 12 \"#91ca91\", 13 \"#beabcf\", 14 \"#dbdbdb\", 15 \"#bfa6a1\", 16 \"#d5b6cc\", 17 \"#eeb98b\", 18 \"#d5d5a2\", 19 \"#a3ccd1\")\n"; */
		out_file << "set palette defined (0 '#1b4f72', 1 '#d98880', 2 '#bcbd22', 3 '#9467bd', 4 '#89b2cf', 5 '#8c564b', 6 '#e377c2', 7 '#ff7f0e', 8 '#d62728', 9 '#17becf', 10 '#d69f9f', 11 '#000000', 12 \"#91ca91\", 13 \"#beabcf\", 14 \"#dbdbdb\", 15 \"#bfa6a1\", 16 \"#d5b6cc\", 17 \"#eeb98b\", 18 \"#d5d5a2\", 19 \"#a3ccd1\")\n";
		/* out_file << "set xlabel \'X-axis (x 10^"<< scale_order<<" m)\' offset 0,1.0 \n"; */
		/* out_file << "set ylabel 'Y-axis (x 10^"<< scale_order <<" m)' offset 2.0,0\n"; */
		out_file << "set xlabel 'X-axis' \n";
		out_file << "set ylabel 'Y-axis' \n";
		out_file << "set size ratio -1\n";
		/* out_file << "set xtics -10,0.5,10\n"; */
		/* out_file << "set ytics -10,0.5,10\n"; */
		out_file << "set style textbox noborder\n";
		out_file << "set xtics border offset -0.0,0.3\n";
		if(num_holes > 0) {
			out_file << "plot \"" << poly_file_name << "\" u ($1/"<<scale<<"):($2/"<<scale <<") with filledcurves closed lw 1 lc rgbcolor \"#7F7F7F\" fill transparent solid 0,";
			for(size_t ii = 0; ii < num_holes; ++ii) {
				if(ii < num_holes - 1) {
					out_file << "\"" << holes_file_name + std::to_string(ii) << "\" u ($1/"<<scale<<"):($2/"<<scale <<") w filledcurves closed lw 0 lc rgbcolor \"#7F7F7F\" fill transparent solid 1 fillcolor '#7F7F7F', ";
				} else {
					out_file << "\"" << holes_file_name + std::to_string(ii) << "\" u ($1/"<<scale<<"):($2/"<<scale <<") w filledcurves closed lw 0 lc rgbcolor \"#7F7F7F\" fill transparent solid 1 fillcolor '#7F7F7F'";
				}
			}
		}
		else {
			out_file << "plot \"" << poly_file_name << "\" u ($1/"<<scale<<"):($2/"<<scale <<") w filledcurves closed lw 2 lc rgbcolor \"gray40\" fill transparent solid 0";
		}
		for(size_t i = 0; i < graph_list.size(); ++i) {
			bool this_plot_non_required = false;
			std::string str_i = std::to_string(i);
			aclibrary::PlotDataRequiredEdges(graph_list[i], data_file_name + str_i, str_i);
			out_file << ", ";
			if(plot_non_required) {
				this_plot_non_required = aclibrary::PlotDataNonRequiredEdges(graph_list[i], data_file_name + "_nr" + str_i, str_i);
			}
			if(this_plot_non_required) {
				out_file << "\"" << data_file_name + str_i << "\" u ($1/" << scale << "):($2/" << scale <<"):3  w lines lw 0.5 palette, ";
				out_file << "\"" << data_file_name + "_nr" + str_i << "\" u ($1/"<<scale<<"):($2/"<<scale <<"):3  w lines lw 0.5 dashtype 8 palette";
			}
			else {
				out_file << "\"" << data_file_name + str_i << "\" u ($1/"<<scale<<"):($2/"<<scale<<"):3  w lines lw 0.5 palette";
			}

		}
		out_file << ", \"< echo \'" << std::to_string(depot_x/scale) << " " << std::to_string(depot_y/scale) << "\' \"  w points pt 5 ps 1 lc rgb \"0x000000 \" ";
		out_file << "\n";


		out_file << "set terminal pngcairo enhanced font 'Times,14' size " << size_x_PNG << ", " << size_y_PNG << std::endl;
		out_file << "set o \"" << output_plot_file_name << ".png\"" << std::endl;
		out_file << "replot";
		out_file.close();
	}

	void GnuplotMap(
			const std::vector <std::shared_ptr <const lclibrary::Graph>> &graph_list,
			const std::string poly_file_name,
			const std::string holes_file_name,
			const std::string data_file_name,
			const std::string gnuplot_file_name,
			const std::string output_plot_file_name,
			bool plot_non_required,
			size_t num_holes = 0) {

		if(graph_list.empty()) {
			return;
		}

		auto G = graph_list[0];
		aclibrary::PlotDataRequiredEdges(G, data_file_name, "0");
		bool this_plot_non_required = false;
		if(plot_non_required) {
			this_plot_non_required = aclibrary::PlotDataNonRequiredEdges(G, data_file_name + "_nr", "0");
		}
		double minX, maxX, minY, maxY;
		G->GetLimits(minX, maxX, minY, maxY);
		std::ofstream out_file (gnuplot_file_name);
		out_file.precision(16);
		double ratioXY;
		if(maxX - minX < 1e-10)
			ratioXY = 1;
		else
			ratioXY = (maxY - minY)/(maxX - minX);
		double size_x = 12;
		double size_y = ratioXY * size_x;
		double size_x_PNG = 900;
		double size_y_PNG = ratioXY * size_x_PNG;
		int scale_order = std::max((int(log10 (maxX-minX))), (int(log10 (maxY-minY))));
		if(scale_order < 1)
			scale_order = 1;
		double scale = std::pow(10, scale_order);
		out_file << "set terminal pdfcairo enhanced font 'Times,14' size " << size_x <<"cm, " << size_y << "cm crop\n";
		out_file << "set o \"" << output_plot_file_name << ".pdf\"\n";
		out_file << "unset key\nunset colorbox\nunset grid\n";
		out_file << "set cbrange [0:19]\n";
		out_file << "set xrange [" << minX/scale - 0.05 << ":" << maxX/scale + 0.05 << "]\n";
		out_file << "set yrange [" << minY/scale - 0.05 << ":" << maxY/scale + 0.05 << "]\n";
		out_file << "set palette defined ("
			"0 '#1b4f72',"
			"1 '#d98880',"
			"2 '#bcbd22',"
			"3 '#9467bd',"
			"4 '#89b2cf',"
			"5 '#8c564b',"
			"6 '#e377c2',"
			"7 '#ff7f0e',"
			"8 '#d62728',"
			"9 '#17becf',"
			"10 '#d69f9f',"
			"11 '#000000',"
			"12 '#91ca91',"
			"13 '#beabcf',"
			"14 '#dbdbdb',"
			"15 '#bfa6a1',"
			"16 '#d5b6cc',"
			"17 '#eeb98b',"
			"18 '#d5d5a2',"
			"19 '#a3ccd1'"
			")\n";
		out_file << "set xlabel 'X-axis (x 10^"<< scale_order<<" m)'\n";
		out_file << "set ylabel 'Y-axis (x 10^"<< scale_order <<" m)' \n";
		out_file << "set size ratio -1\n";
		out_file << "set xtics -10,0.5,10\n";
		out_file << "set ytics -10,0.5,10\n";
		out_file << "set style textbox noborder\n";
		/* out_file << "set xtics border offset -0.0,0.3\n"; */
		double depot_x, depot_y;
		out_file << "plot \"" << poly_file_name << "\" u ($1/"<<scale<<"):($2/"<<scale <<") with filledcurves closed lw 1 lc rgbcolor \"#7F7F7F\" fill transparent solid 0";
		for(size_t ii = 0; ii < num_holes; ++ii) {
			out_file << ",\"" << holes_file_name + std::to_string(ii) << "\" u ($1/"<<scale<<"):($2/"<<scale <<") w filledcurves closed lw 0 lc rgbcolor \"#7F7F7F\" fill transparent solid 1 fillcolor '#7F7F7F' ";
		}

		if(this_plot_non_required) {
			out_file << ",\"" << data_file_name << "\" u ($1/" << scale << "):($2/" << scale <<"):3  w lines lw 1.5 palette";
			out_file << ",\"" << data_file_name + "_nr" << "\" u ($1/"<<scale<<"):($2/"<<scale <<"):3  w lines lw 1.5 dashtype 2 palette";
		}
		else {
			out_file << ",\"" << data_file_name << "\" u ($1/"<<scale<<"):($2/"<<scale<<"):3  w lines lw 1.5 palette";
		}
		graph_list[0]->GetDepotXY(depot_x, depot_y);
		out_file << ", \"< echo \'" << std::to_string(depot_x/scale) << " " << std::to_string(depot_y/scale) << "\' \"  w points pt 5 ps 1 lc rgb \"0x000000 \" ";
		for(size_t i = 1; i < graph_list.size(); ++i) {
			auto str_i = std::to_string(i);
			aclibrary::PlotDataRequiredEdges(graph_list[i], data_file_name + str_i, str_i);
			if(plot_non_required) {
				this_plot_non_required = aclibrary::PlotDataNonRequiredEdges(graph_list[i], data_file_name + "_nr" + str_i, str_i);
			}
			if(this_plot_non_required) {
				out_file << ", \"" << data_file_name + str_i << "\" u ($1/" << scale << "):($2/" << scale <<"):3  w lines lw 1.5 palette";
				out_file << ", \"" << data_file_name + "_nr" + str_i << "\" u ($1/"<<scale<<"):($2/"<<scale <<"):3  w lines lw 1.5 dashtype 2 palette";
			}
			else {
				out_file << ", \"" << data_file_name + str_i << "\" u ($1/"<<scale<<"):($2/"<<scale<<"):3  w lines lw 1.5 palette";
			}
			graph_list[i]->GetDepotXY(depot_x, depot_y);
			out_file << ", \"< echo \'" << std::to_string(depot_x/scale) << " " << std::to_string(depot_y/scale) << "\' \"  w points pt 5 ps 1 lc rgb \"0x000000 \" ";
		}

		out_file << "\n";
		out_file << "\n";

		out_file << "set terminal pngcairo enhanced font 'Times,14' size " << size_x_PNG << ", " << size_y_PNG << std::endl;
		out_file << "set o \"" << output_plot_file_name << ".png\"" << std::endl;
		out_file << "replot";
		out_file.close();
	}

} /* lclibrary */
