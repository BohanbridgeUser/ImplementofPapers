#include "mainframe.h"

#include <chrono>

int main(int argc, char* argv[])
{
    std::string filename;
    double normal_threshold = 0.95;
    double rth = 0.8;
    int knn = 40;
    
    if(argc > 1)
    {
        filename = std::string(argv[1]);
        normal_threshold = std::stod(std::string(argv[2]));
        rth = std::stod(std::string(argv[3]));
        knn = std::stod(std::string(argv[4]));
    }   
    else
        filename = "../data/anchor.ply";
    
    MainFrame mainframe;
    mainframe.load_points(filename);
    mainframe.set_parameters(normal_threshold, rth, knn);
    mainframe.smoothness_constraint_segmentation();
    return 0;
}