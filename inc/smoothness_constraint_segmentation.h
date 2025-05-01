#ifndef _SMOOTHNESS_CONSTRAINT_SEGMENTATION_H_
#define _SMOOTHNESS_CONSTRAINT_SEGMENTATION_H_


#include "algorithm.h"

class SmoothnessConstraintSegmentation : public Algorithm
{
    public:
        SmoothnessConstraintSegmentation();
        ~SmoothnessConstraintSegmentation();

        void Execute();

        void SetParameters(const std::vector<std::any>& parameters);

        void output_point_segmentation(const std::string& filename);
        
    private:
        std::shared_ptr<spdlog::logger>                     p_logger;
        double                                              m_normal_threshold;
        bool                                                m_if_oriented_normal;
        Vector_PWN                                          m_PWNs;
        double                                              m_rth;
        size_t                                              m_knn;
        std::vector<std::vector<int>>                       m_regions;
        std::string                                         m_filename;


        int load_ply(const std::string &filename);
        void smoothness_constraint_segmentation();
        void output_ply_without_normal(const std::string& filename, std::vector<std::vector<Point>>& Points, 
                                       int nb_pts, int nb_faces,
                                       std::vector<CGAL::Color>& colors);
};

#endif