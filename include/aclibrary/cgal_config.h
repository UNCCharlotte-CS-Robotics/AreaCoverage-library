/**
 * This file is part of the AreaCoverage-library.
 * The file contains CGAL includes and typedefs
 *
 * TODO: clean up unnecessary includes and typedefs
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

#ifndef ACLIBRARY_CGAL_CONFIG_H_
#define ACLIBRARY_CGAL_CONFIG_H_


#include <CGAL/Cartesian.h>
#include <CGAL/MP_Float.h>
#include <CGAL/Quotient.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arrangement_2.h>
#include <CGAL/Arrangement_on_surface_2.h>
#include <CGAL/Arr_linear_traits_2.h>
#include <CGAL/centroid.h>
#include <CGAL/Vector_2.h>
#include <CGAL/min_quadrilateral_2.h>
#include <CGAL/squared_distance_2.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/General_polygon_with_holes_2.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
/* #include <CGAL/Aff_transformation_2.h> */
#include <CGAL/partition_2.h>
#include <CGAL/minkowski_sum_2.h>
/* #include <CGAL/Gps_circle_segment_traits_2.h> */
#include <CGAL/approximated_offset_2.h>
/* #include <CGAL/Aff_transformation_2.h> */
#include <CGAL/Polygon_set_2.h>
#include <CGAL/create_offset_polygons_2.h>
#include <boost/shared_ptr.hpp>
typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
typedef CGAL::Exact_predicates_exact_constructions_kernel K;
/* typedef CGAL::Quotient<CGAL::MP_Float> Number_type; */
/* typedef CGAL::Cartesian<Number_type> Kernel; */
/* typedef CGAL::Cartesian<Number_type> K; */
/* typedef CGAL::Gps_circle_segment_traits_2<Kernel> CircleSegmentTraits; */
/* typedef CGAL::Aff_transformation_2<K> Transformation; */
typedef K::FT FT;
typedef K::Point_2 Point_2;
typedef K::Point_3 Point_3;
typedef K::Vector_2 Vector_2;
typedef K::Direction_2 Direction_2;
typedef K::Line_2 Line_2;
typedef K::Ray_2 Ray_2;
typedef K::Intersect_2 Intersect_2;
typedef K::Plane_3 Plane_3;
typedef K::Segment_2 Segment_2;
typedef K::Triangle_2 Triangle_2;
typedef CGAL::Polygon_2<K> Polygon_2;
typedef CGAL::Polygon_set_2<K> Polygon_set_2;
typedef Polygon_2::Vertex_const_iterator VertexConstIterator;
typedef Polygon_2::Vertex_const_circulator VertexConstCirculator;
typedef Polygon_2::Vertex_iterator VertexIterator;
typedef Polygon_2::Vertex_circulator VertexCirculator;
typedef Polygon_2::Edge_const_iterator EdgeConstIterator;
typedef Polygon_2::Edge_const_circulator EdgeConstCirculator;
typedef CGAL::Polygon_with_holes_2<K> PolygonWithHoles;
/* typedef CGAL::Exact_predicates_inexact_constructions_kernel InexactKernel; */
typedef CGAL::Straight_skeleton_2<K> Ss;

/* typedef CGAL::Partition_traits_2<K> PartitionTraits; */
/* typedef CGAL::Quotient<CGAL::MP_Float> Number_type; */
/* typedef CGAL::Cartesian<Number_type> Kernel; */
/* typedef CGAL::Arr_segment_traits_2<Kernel> Traits_2; */
typedef CGAL::Arr_linear_traits_2<Kernel> Traits_2;
/* typedef Traits_2::Point_2 Point_2; */
/* typedef Traits_2::Vector_2 Vector_2; */
/* typedef Traits_2::X_monotone_curve_2 Segment_2; */
/* typedef Traits_2::Line_2 Line_2; */
typedef CGAL::Arrangement_2<Traits_2> Arrangement_2;
typedef CGAL::Polygon_with_holes_2<Kernel> Polygon_with_holes_2;
/* typedef CGAL::Polygon_2<Kernel> Polygon_2; */
/* typedef CGAL::Aff_transformation_2<Traits_2> Transform2; */
typedef CGAL::Simple_cartesian<double>  K_double;
typedef K_double::Point_2 Point_2_double;

#endif /* ACLIBRARY_CGAL_CONFIG_H_ */
