#include "mainframe.h"




/// @brief public:
/// @name Type Define
/// @{
/// @}
/// @name Life Circle
/// @{
MainFrame::MainFrame(int num_of_threads):m_threads_pool(num_of_threads)
{
    if(spdlog::get("CAD_Reconstruction")){
        p_logger = spdlog::get("CAD_Reconstruction");
    }
    else{
        p_logger = spdlog::stdout_color_mt("CAD_Reconstruction");
    }

    spdlog::set_level(spdlog::level::info);
    spdlog::set_pattern("[%H:%M:%S] [%n] [%l] %v");

    m_if_oriented_normal = true;
    m_normal_threshold = 0.95; // 0.;70

    m_knn = 20;
    m_rth = 0.3;
}

MainFrame::~MainFrame()
{

}

/// @}
/// @name Operators
/// @{
/// @}
/// @name Operations
/// @{
int MainFrame::load_points(const std::string& filename)
{
    std::string path_point_cloud = filename;
    std::string path_point_cloud_extension = boost::filesystem::extension(path_point_cloud);

    if(!boost::filesystem::exists(path_point_cloud)){
        p_logger->error("File {} does not exist",path_point_cloud);
        return 1;
    }

    p_logger->info("Load points from {}", filename);

    m_PWNs.clear();

    if (path_point_cloud_extension == ".ply")
        load_ply(filename);
    else
    {
        p_logger->error("{} is not a valid points file",path_point_cloud_extension);
        return 1;
    }
        

    p_logger->info("Loaded {} points",m_PWNs.size());
    
    return 0;
}

void MainFrame::set_parameters(double normal_threshold, double rth, int knn) 
{
    m_normal_threshold = normal_threshold;
    m_rth = rth;
    m_knn = knn;
};

void MainFrame::smoothness_constraint_segmentation()
{
    p_logger->info("Normal threshold : {}", m_normal_threshold);
    p_logger->info("Rth    threshold : {}", m_rth);
    std::vector<std::vector<int>>& regions = m_regions;
    std::unordered_map<Point,int>  map_id_points;
    std::list<Point>               list_points;
    std::vector<int>               union_label;
    std::vector<int>               available;
    for(size_t i = 0; i < m_PWNs.size(); ++i)
    {
        list_points.push_back(m_PWNs[i].first);
        map_id_points[m_PWNs[i].first] = i;
        union_label.push_back(-1);
        available.push_back(-1);
    }
    Tree tree(list_points.begin(), list_points.end());    
    std::vector<double>             r_all;
    std::vector<Neighbor_search>    neighbor_searches;
    std::vector<Plane>              tangent_planes;
    for(int i=0;i<m_PWNs.size();++i)
    {
        double r = 0.0;
        Neighbor_search search(tree, m_PWNs[i].first, m_knn + 1);
        neighbor_searches.push_back(search);
        std::vector<Point> neighbors;
        for(auto it = search.begin(); it != search.end(); ++it)
                neighbors.push_back(it->first);
        Plane plane;
        linear_least_squares_fitting_3(neighbors.begin(), neighbors.end(), plane, CGAL::Dimension_tag<0>());
        tangent_planes.push_back(plane);
        for(int j=0;j<neighbors.size();++j)
            r+= std::sqrt(CGAL::squared_distance(neighbors[j], plane));
        r_all.push_back(r);
    }
    std::sort(r_all.begin(),r_all.end());
    double rth = r_all[r_all.size() * 80 /100 ];

    for(int i=0;i<m_PWNs.size();++i)
    {
        if(available[i] != -1)
            continue;
        std::vector<int> region;
        std::vector<int> seeds;
        region.push_back(i);
        seeds.push_back(i);
        available[i] = 0;
        for(int j=0;j<seeds.size();++j)
        {
            Point seed = m_PWNs[seeds[j]].first;
            Vector seed_normal = m_PWNs[seeds[j]].second;
            // Vector seed_normal = tangent_planes[seeds[j]].orthogonal_vector();
            // seed_normal /= std::sqrt(seed_normal.squared_length());
            Neighbor_search& search = neighbor_searches[seeds[j]];
            for(Neighbor_search::iterator it = search.begin(); it != search.end(); ++it)
            {
                if(map_id_points.find(it->first) != map_id_points.end() && map_id_points[it->first] != i && available[map_id_points[it->first]] == -1)
                {
                    size_t id = map_id_points[it->first];
                    // Vector normal = tangent_planes[id].orthogonal_vector();
                    // normal /= std::sqrt(normal.squared_length());
                    Vector& normal = m_PWNs[id].second;
                    if(std::fabs(normal * seed_normal) > m_normal_threshold )
                    {
                        region.push_back(id);
                        available[id] = 0;
                        if(r_all[id] < rth)
                            seeds.push_back(id);
                    }
                }
            }
        }
        regions.push_back(region);
    }
    p_logger->info("done");
    p_logger->info("Number of regions : {}", regions.size());
    for(size_t i = 0; i < regions.size(); ++i)
    {
        if(regions[i].size() > 100)
            p_logger->info("{} region has {} points", i, regions[i].size());
    }
        
        
    /**
     * @brief Output the segmentation
     */
    std::string filename = "../output/con_seg.ply";
    std::ofstream stream(filename);
    if(!stream)
    {
        std::cerr << "Error: cannot open file" << std::endl;
        return;
    }
    stream << std::fixed << std::setprecision(10);
    int nb_pts = 0;
    std::vector<CGAL::Color> colors;
    std::vector<std::vector<Point>> points;
    std::default_random_engine generator;
    std::uniform_int_distribution<int> uniform_distribution(0, 225);
    for(int i=0;i<regions.size();++i)
    {
        std::vector<int>& pi = regions[i];
        float r = 0.0, g = 0.0, b = 0.0;
        CGAL::Color color;
        r = (float)uniform_distribution(generator);
        g = (float)uniform_distribution(generator);
        b = (float)uniform_distribution(generator);
        color = CGAL::Color(r,g,b);
        points.push_back(std::vector<Point>());
        colors.push_back(color);
        for(int j=0;j<pi.size();++j)
        {
            points.back().push_back(m_PWNs[pi[j]].first);
            nb_pts++;
        }
    }
    stream << "ply" << std::endl;
    stream << "format ascii 1.0" << std::endl;
    stream << "element vertex " << nb_pts << std::endl;
    stream << "property float x" << std::endl;
    stream << "property float y" << std::endl;
    stream << "property float z" << std::endl;
    stream << "property uchar red" << std::endl;
    stream << "property uchar green" << std::endl;
    stream << "property uchar blue" << std::endl;
    stream << "element face " << 0 << std::endl;
    stream << "property list uchar int vertex_index" << std::endl;
    stream << "end_header" << std::endl;
    // Vertices
    for (size_t i = 0; i < points.size(); ++i) {
        for (size_t j = 0; j < points[i].size(); ++j) {
            stream << points[i][j].x() << " " << points[i][j].y() << " " << points[i][j].z()
                << " " << (int)colors[i].r() << " " << (int)colors[i].g() << " " << (int)colors[i].b() << std::endl;
        }
    }
    std::cout << "Outputing ply file end" << std::endl;
    stream << std::endl;
    stream.close();
}


/// @}
/// @name Access
/// @{

/// @}
/// @name Inquiry
/// @{
/// @}
/// @name Input and Output
/// @{
/// @}


/// @brief protected:
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

/// @brief private:
/// @name Private Static Member Variables
/// @{
/// @}
/// @name Private Member Variables
/// @{
/// @}
/// @name Private Operatiors
/// @{
/// @}
/// @name Private Operations
/// @{
int MainFrame::load_ply(const std::string &filename)
{
    std::ifstream streamb(filename);
    int line = 0;
    std::string s;
    while (streamb && line < 2) {
        if (!getline(streamb, s)) break;
        line++;
    }
    streamb.close();

    if (s == "format ascii 1.0") {
        std::ifstream stream(filename);
        if (!stream || !CGAL::IO::read_PLY_with_properties(stream, std::back_inserter(m_PWNs),
                                                                   CGAL::IO::make_ply_point_reader(Point_map()),
                                                                   CGAL::IO::make_ply_normal_reader(Normal_map()))) {
            return 1;
        }
    }
    else {
        std::ifstream stream(filename, std::ios_base::binary);
        if (!stream || !CGAL::IO::read_PLY_with_properties(stream, std::back_inserter(m_PWNs),
                                                           CGAL::IO::make_ply_point_reader(Point_map()),
                                                           CGAL::IO::make_ply_normal_reader(Normal_map()))) {
            return 1;
        }
    }
    
    if(m_PWNs[0].second == Vector(0,0,0))
    {
        p_logger->warn("Normal is not oriented, orienting normal");
        m_if_oriented_normal= false;
    }
    else
        m_if_oriented_normal= true;
    return 0;
}

void MainFrame::union_set(const int p1, const int p2, std::vector<int>& union_set)
{
    int root1 = p1, root2 = p2;
    while(union_set[root1] != -1)
        root1 = union_set[root1];
    if(root1 != p1)
        union_set[p1] = root1;
    while(union_set[root2] != -1)
        root2 = union_set[root2];
    if(p2 != root2)
        union_set[p2] = root2;
        
    if(root1 != root2)
        union_set[root1] = root2;
}

int MainFrame::find_set(const int p1, std::vector<int>& union_set)
{
    int root1 = p1;
    while(union_set[root1] != -1)
        root1 = union_set[root1];
    return root1;
}

/// @}
/// @name Private Access
/// @{
/// @}
/// @name Private Inquiry
/// @{
/// @}










