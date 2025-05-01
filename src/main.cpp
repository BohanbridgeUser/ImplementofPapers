#include "mainframe.h"
#include "algorithm_factory.h"
#include "PoissonReconstruction.h"

#include <chrono>

int main(int argc, char* argv[])
{
    // /* Smoothness Constraint Segmentation*/
    // std::string filename;
    // double normal_threshold = 0.95;
    // double rth = 0.8;
    // int knn = 40;
    // if(argc > 1)
    // {
    //     filename = std::string(argv[1]);
    //     normal_threshold = std::stod(std::string(argv[2]));
    //     rth = std::stod(std::string(argv[3]));
    //     knn = std::stod(std::string(argv[4]));
    // }   
    // else
    // {
    //     filename = "../data/anchor.ply";
    // }
    // AlgorithmFactory factory;
    // std::shared_ptr<Algorithm> p_algorithm = factory.Create_Algorithm("smooth");
    // std::vector<std::any> parameters;
    // parameters.push_back(filename);
    // parameters.push_back(normal_threshold);
    // parameters.push_back(rth);
    // parameters.push_back(knn);
    // p_algorithm->SetParameters(parameters);
    // p_algorithm->Execute();

    /* Poisson Reconstruction */
    std::string filename;
    double cbbox = 3.0;
    if(argc > 1)
    {
        filename = std::string(argv[1]);
        cbbox = std::stod(argv[2]);
    }   
    else
    {
        filename = "../data/bunny_pc.ply";
    }
    AlgorithmFactory factory;
    std::shared_ptr<Algorithm> p_algorithm = factory.Create_Algorithm("Poisson");
    std::vector<std::any> parameters;
    parameters.push_back(filename);
    parameters.push_back(cbbox);
    p_algorithm->SetParameters(parameters);
    p_algorithm->Execute();
    return 0;
}