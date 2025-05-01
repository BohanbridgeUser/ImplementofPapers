#ifndef _MAINFRAME_H_
#define _MAINFRAME_H_
#include "define.h"
#include "my_thread_pool.h"

#include "smoothness_constraint_segmentation.h"

class MainFrame
{
    public:
        /// @name Type Define
        /// @{
            using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
            using Point = Kernel::Point_3;
            using Point_2 = Kernel::Point_2;
            using Vector = Kernel::Vector_3;
            using Segment = Kernel::Segment_3;
            using Plane = Kernel::Plane_3;
            using PWN = std::pair<Point, Vector>;
            using Vector_PWN = std::vector<PWN>;
            using Point_map = CGAL::First_of_pair_property_map<PWN>;
            using Normal_map = CGAL::Second_of_pair_property_map<PWN>;

            using SearchTraits_3 = CGAL::Search_traits_3<Kernel>;
            using Neighbor_search = CGAL::Orthogonal_k_neighbor_search<SearchTraits_3>;
            using Tree = Neighbor_search::Tree;

        /// @}
        /// @name Life Circle
        /// @{
            MainFrame(int num_of_threads = 24);
            ~MainFrame();
 
        /// @}
        /// @name Operators
        /// @{
        /// @}
        /// @name Operations
        /// @{
            int load_points(const std::string& filename);
            void set_parameters(double normal_threshold, double rth=0.3, int knn = 40);
            
            template<typename A, typename... Args>
            void set_algorithm(A&& algorithm, Args&& ... args)
            {
                m_algorithm=std::function<void()>(std::bind(std::forward<A>(algorithm), std::forward<Args>(args)...));
            }

            void run();

            void smoothness_constraint_segmentation();
            
        /// @}
        /// @name Access
        /// @{
        /// @}
        /// @name Inquiry
        /// @{
        /// @}
        /// @name Input and Output
        /// @{
            void output_point_segmentation(const std::string& filename);
        /// @}
    protected:
        /// @name Protected Static Member Variables
        /// @{
        /// @}
        /// @name Protected Member Variables
        /// @{
        /// @}
        /// @name Protected Operatiors
        /// @{
        /// @}
        /// @name Protected Operations
        /// @{
        /// @}
        /// @name Protected Access
        /// @{
        /// @}
        /// @name Protected Inquiry
        /// @{
        /// @}
    private:
        /// @name Private Static Member Variables
        /// @{
        /// @}
        /// @name Private Member Variables
        /// @{
            std::shared_ptr<spdlog::logger>                     p_logger;
            double                                              m_normal_threshold;

            /**
             * @param m_if_oriented_normal: if the normal of the point is oriented
             * @param m_PWNs: the all input points with normals
             */
            bool                                                m_if_oriented_normal;
            Vector_PWN                                          m_PWNs;

            /**
             * @param m_thread_pool:            the thread pool for multi-threading
             */
            MyThreadPool                                        m_threads_pool;

            std::function<void()>                               m_algorithm;


            /**
             * @param m_rth threshold of smoothness constraint segmentation
             * @param m_knn num of nearest neighobrs
             * @param m_regions segmentations
             */
            double                                              m_rth;
            size_t                                              m_knn;
            std::vector<std::vector<int>>                       m_regions;


        /// @}
        /// @name Private Operatiors
        /// @{
        /// @}
        /// @name Private Operations
        /// @{
            int load_ply(const std::string &filename);
            void union_set(const int p1, const int p2, std::vector<int>& union_set);
            int find_set(int p1, std::vector<int>& union_set);
        /// @}
        /// @name Private Access
        /// @{
        /// @}
        /// @name Private Inquiry
        /// @{
        /// @}
        /// @name Private Input and Output
        /// @{
        void output_ply_without_normal(const std::string& filename, std::vector<std::vector<Point>>& Points, 
                                       int nb_pts, int nb_faces,
                                       std::vector<CGAL::Color>& colors);
        /// @}
};

#endif