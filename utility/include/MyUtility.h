#ifndef _MY_UTILITY_H_
#define _MY_UTILITY_H_

#include <vector>
#include <array>
#include <functional>

class MyUtility
{
    public:
        static double Gauss_Legendre(double a, double b, std::function<double(double)> fun);
        static double Gauss_Legendre3(double xmin, double ymin, double zmin,
                                      double xmax, double ymax, double zmax, 
                                      std::function<double(double, double ,double)> fun);
        static double Gauss_Legendre3_debug(double xmin, double ymin, double zmin,
                                  double xmax, double ymax, double zmax, int node1, int node2,
                                  std::function<double(double, double ,double)> fun);
};




#endif