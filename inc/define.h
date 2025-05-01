#ifndef _DEFINE_H_
#define _DEFINE_H_

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/property_map.h>
#include <CGAL/Search_traits_3.h>
#include <CGAL/Orthogonal_k_neighbor_search.h>
#include <CGAL/linear_least_squares_fitting_3.h>

#include <CGAL/Point_set_3.h>
#include <CGAL/Point_set_3/IO.h>
#include <CGAL/Octree.h>

#include <CGAL/Polyhedron_3.h>
#include <CGAL/poisson_surface_reconstruction.h>

#include <CGAL/Bbox_3.h>

#include <CGAL/IO/read_ply_points.h>
#include <CGAL/IO/write_xyz_points.h>
#include <CGAL/IO/polygon_soup_io.h>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/SparseCore>
#include <Eigen/SparseLU>
#include <Eigen/SparseQR>
#include <Eigen/SparseCholesky>
#include <Eigen/IterativeLinearSolvers>

#include <boost/filesystem.hpp>
#include <vector>
#include <memory>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <sstream>
#include <thread>
#include <random>
#include <limits>
#include <chrono>
#include <list>
#include <unordered_set>
#include <algorithm>
#include <functional>
#include <any>
#include <bitset>
#include <set>
#include <unordered_set>
#include <cassert>
#include <fstream>

#include <omp.h>

#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"

#include "MyUtility.h"

using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using Point = Kernel::Point_3;
using Point_2 = Kernel::Point_2;
using Vector = Kernel::Vector_3;
using Segment = Kernel::Segment_3;
using Bbox_3 = CGAL::Bbox_3;
using Plane = Kernel::Plane_3;
using PWN = std::pair<Point, Vector>;
using Vector_PWN = std::vector<PWN>;
using Point_map = CGAL::First_of_pair_property_map<PWN>;
using Normal_map = CGAL::Second_of_pair_property_map<PWN>;

using SearchTraits_3 = CGAL::Search_traits_3<Kernel>;
using Neighbor_search = CGAL::Orthogonal_k_neighbor_search<SearchTraits_3>;
using Tree = Neighbor_search::Tree;

using Point_set = CGAL::Point_set_3<Point>;
using Point_map_Octree = Point_set::Point_map;
using Octree = CGAL::Octree<Kernel, Point_set, Point_map_Octree>;
using NodeIndex = Octree::Node_index;
using OctreeBbox3 = Octree::Bbox;
using LeavesTraversal = CGAL::Orthtrees::Leaves_traversal<Octree>;
using PreorderTraversal = CGAL::Orthtrees::Preorder_traversal<Octree>;
using LevelTraversal = CGAL::Orthtrees::Level_traversal<Octree>;
using PostorderTraversal = CGAL::Orthtrees::Postorder_traversal<Octree>;

using Pwn = std::pair<Point, Vector>;
using Polyhedron = CGAL::Polyhedron_3<Kernel>;

#endif