#ifndef _POISSON_RECONSTRUCTION_H_
#define _POISSON_RECONSTRUCTION_H_

#include "algorithm.h"

class PoissonReconstruction : public Algorithm
{
    public:
        PoissonReconstruction();
        ~PoissonReconstruction()=default;

        void Execute();

        void SetParameters(const std::vector<std::any>& parameters);
        
        void OutputOctree(std::string filename);

    private:
        std::shared_ptr<spdlog::logger>                                 p_logger;

        Vector_PWN                                                      m_PWNs;
        std::string                                                     m_filename;

        std::set<Point>                                                 m_virtual_points;
        Point_set                                                       m_points_set;
        std::shared_ptr<Octree>                                         p_octree;
        double                                                          m_co_bbox;
        bool                                                            m_normal_reversed;

        std::unordered_map<NodeIndex, Eigen::Vector3d>                  m_alphaOS;
        std::unordered_map<NodeIndex, std::vector<NodeIndex>>           m_overlap_map;
        std::unordered_map<NodeIndex, std::unordered_set<NodeIndex>>    m_overlap_set;
        std::unordered_map<NodeIndex, 
                           std::vector<std::pair<NodeIndex,
                                                 Bbox_3>>>              m_overlap;
        std::unordered_map<NodeIndex, 
                           std::unordered_map<NodeIndex,
                                              Bbox_3>>                  m_overlap_map_map;
        std::unordered_map<NodeIndex, int>                                          
                                                                        m_F02Node;
        std::vector<NodeIndex>                                          m_Node2F0;
        std::vector<std::unordered_map<NodeIndex,
                                        Bbox_3>>                        m_overlap_vec;
        double                                                          m_num_of_nodes;
        Eigen::MatrixXd                                                 m_L;
        Eigen::VectorXd                                                 m_V;
        Eigen::VectorXd                                                 m_X;
        Eigen::SparseMatrix<double>                                     sm_L;
        double                                                          m_scale_coefficient;
        std::vector<Point>                                              m_polygon_points;
        std::vector<std::vector<int>>                                   m_polygon_soup;
        std::array<std::array<int,16>, 256>                             m_triTable;
        std::array<std::array<int,2>, 12>                               m_edgeTable;
        std::map<NodeIndex,std::array<double, 8>>                       m_node_indicators;
        std::map<NodeIndex,std::bitset<8>>                              m_indices;
        std::unordered_map<int, bool>                                   m_checked;        

        bool loadPointsWithNormal();
        bool reverseNormal();
        bool buildOctree();
        bool buildVectorField();
        bool buildVectorFieldD();
        bool computeV();
        bool computeVD();
        bool computeV2();
        bool computeL();
        bool computeLD();
        bool computeL2();
        bool computeX();
        bool computeScaleC();
        bool marchingCubes();
        void validation();

        std::pair<NodeIndex, Point> findAdjacentNodeCenter(NodeIndex const& node, int dir1, int dir2, int dir3, int depth);
        NodeIndex splitAndGetNode(NodeIndex const& node, int dir, int depth);
        Point calBboxCen(NodeIndex const& node);
        void trilinearInterpolation(std::vector<std::pair<NodeIndex,Point>> const& bboxcenter, Point const& sp, Vector const& sn);
        void calOverlapRecursive();
        void calOverlap();
        void calOverlapD();
        double calBboxFilterFunctionConvolve3(double t);
        double calBboxFilterFunctionConvolve3Differ(double t);
        double calBboxFilterFunctionConvolve3Differ2(double t);
        double calv0(NodeIndex node1, NodeIndex node2, Bbox_3& overlap);
        double calBboxFilterFunctionConvolve3(double to, NodeIndex node, int dir);
        double calBboxFilterFunctionConvolve3Differ(double to, NodeIndex node, int dir);
        double calBboxFilterFunctionConvolve3Differ2(double to, NodeIndex node, int dir);
        double call0(NodeIndex node1, NodeIndex node2, Bbox_3& overlap);
        double calIndicator(Point const& point);

        double calF0(Point const& q, NodeIndex oNode);
        double calF0PX(Point const& q, NodeIndex oNode);
        double calF0PY(Point const& q, NodeIndex oNode);
        double calF0PZ(Point const& q, NodeIndex oNode);
        double calF0PX2(Point const& q, NodeIndex oNode);
        double calF0PY2(Point const& q, NodeIndex oNode);
        double calF0PZ2(Point const& q, NodeIndex oNode);
        void outputBbox(NodeIndex node);
        void outputBboxF0();
        void outputBboxD(int depth);
        void outputLeavesBbox();
        void output3Bbox(NodeIndex iNode, NodeIndex iNode2);
        void outputTriInter(Point& point, std::vector<std::pair<NodeIndex,Point>>& neighborCenter);
        void outputMarchingCubesResuilt();
        bool checkSymmetric();

        void CGALPSR();

        template <typename OutputIterator>
        OutputIterator OverlapLeavesNodes(const NodeIndex& query, OutputIterator output) const {
            return OverlapLeavesNodesRecursive(query, p_octree->root(), output);
        }

        template <typename Node_output_iterator>
        Node_output_iterator OverlapLeavesNodesRecursive(const NodeIndex& query, NodeIndex node, 
                                                    Node_output_iterator output) const{
            // Check if the current node intersects with the query
            Bbox_3 queryBbox3 = p_octree->bbox(query).bbox(), nodeBbox3 = p_octree->bbox(node).bbox();
            queryBbox3.scale(3.0);
            nodeBbox3.scale(3.0);
            if (CGAL::do_intersect(queryBbox3, nodeBbox3)) {
                
                // if this node is a leaf, then it's considered an intersecting node
                if (p_octree->is_leaf(node)) {
                    *output++ = node;
                    return output;
                }

                // Otherwise, each of the children need to be checked
                for (int i = 0; i < 8; ++i) {
                    OverlapLeavesNodesRecursive(query, p_octree->child(node, i), output);
                }
            }
            return output;
        }

        void calNodeVerIndicatorAndIndex(NodeIndex node, std::vector<Point>& nodeVertices);
        std::unordered_set<Segment> finestEdges(Segment& edge, NodeIndex node);
        std::variant<Point,int> finestZeroCrossing(Segment& edge, NodeIndex node);
        std::optional<Point> interPolatePoint(Segment const& edge);
        bool preSplitRecursively(NodeIndex node);
        Point InterPolatePoint(Point const& p1, Point const& p2, double indicator1, double indicator2);
        bool checkSplit(NodeIndex node, std::array<int, 16>& allEdgeIDs, std::vector<Point>& nodeVertices); 
        void splitRecursively(NodeIndex node);
};


#endif