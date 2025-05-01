#include "MyUtility.h"
#include <iostream>
#include <fstream>
double MyUtility::Gauss_Legendre(double a, double b, std::function<double(double)> fun)
{
    std::vector<double>  x({-0.9061793, -0.5384693, 0, 0.9061793,  0.5384693});
    std::array<double,5> coefficients({0.2369269, 0.4786287, 0.5688889,0.2369269, 0.4786287});
    double ans = 0.0;
    for(int i=0;i<5;++i)
    {
        double funv = fun(0.5 * (b - a) * x[i] + 0.5 * (a + b));
        ans += 0.5 * (b - a) * coefficients[i] * funv;
        // std::cout << "fun : " << funv << std::endl;
        // std::cout << "ans : " << ans << std::endl;
    }
    return ans;
}


double MyUtility::Gauss_Legendre3(double xmin, double ymin, double zmin,
                                  double xmax, double ymax, double zmax, 
                                  std::function<double(double, double ,double)> fun)
{
    // std::array<double,5> x({-0.9061793, -0.5384693, 0, 0.5384693, 0.9061793});
    // std::array<double,5> c({0.2369269, 0.4786287, 0.5688889, 0.4786287, 0.2369269});
    std::array<double,6> x({-0.93246951, -0.66120939, -0.23861919, 0.23861919, 0.66120939, 0.93246951});
    std::array<double,6> c({ 0.17132449,  0.36076157,  0.46791393, 0.46791393, 0.36076157, 0.17132449});
    double ans = 0.0;

    const double dx = 0.5 * (xmax - xmin);
    const double dy = 0.5 * (ymax - ymin);
    const double dz = 0.5 * (zmax - zmin);
    const double volume_factor = dx * dy * dz;  // 合并缩放系数

    for(int i=0;i<x.size();++i)
    {
        for(int j=0;j<x.size();++j)
        {
            for(int k=0;k<x.size();++k)
            {
                const double x_trans = dx * x[i] + 0.5 * (xmax + xmin);  // 坐标变换
                const double y_trans = dy * x[j] + 0.5 * (ymax + ymin);
                const double z_trans = dz * x[k] + 0.5 * (zmax + zmin);
                const double weight_product = c[i] * c[j] * c[k];
                ans += weight_product * fun(x_trans, y_trans, z_trans);
                // double x_ = x[i], y_ = x[j], z_ = x[k];
                // double c_ = c[i] * c[j] * c[k];
                // ans += c_ * (xmax - xmin) * 0.5 * (ymax - ymin) * 0.5 * (zmax - zmin) * 0.5 *
                //             fun((xmax - xmin) * 0.5 * x_ + (xmax + xmin) * 0.5,
                //                 (ymax - ymin) * 0.5 * y_ + (ymax + ymin) * 0.5,
                //                 (zmax - zmin) * 0.5 * z_ + (zmax + zmin) * 0.5);
            }
        }
    }
    return ans * volume_factor;  // 集中处理缩放系数
    // return ans;
}


double MyUtility::Gauss_Legendre3_debug(double xmin, double ymin, double zmin,
                                  double xmax, double ymax, double zmax, int node1, int node2,
                                  std::function<double(double, double ,double)> fun)
{
    // std::array<double,5> x({-0.9061793, -0.5384693, 0, 0.5384693, 0.9061793});
    // std::array<double,5> c({0.2369269, 0.4786287, 0.5688889, 0.4786287, 0.2369269});
    std::array<double,6> x({-0.93246951, -0.66120939, -0.23861919, 0.23861919, 0.66120939, 0.93246951});
    std::array<double,6> c({ 0.17132449,  0.36076157,  0.46791393, 0.46791393, 0.36076157, 0.17132449});
    static int cnt = 0;
    std::string filename ="../output/detail_loi/debug_gauss_legendre_" + std::to_string(node1) + " " + std::to_string(node2) + " " + std::to_string(cnt) + ".txt";
    std::ofstream stream(filename);
    double ans = 0.0;
    for(int i=0;i<x.size();++i)
    {
        for(int j=0;j<x.size();++j)
        {
            for(int k=0;k<x.size();++k)
            {
                double x_ = x[i], y_ = x[j], z_ = x[k];
                double c_ = c[i] * c[j] * c[k];
                double temp = c_ * (xmax - xmin) * 0.5 * (ymax - ymin) * 0.5 * (zmax - zmin) * 0.5 *
                       fun((xmax - xmin) * 0.5 * x_ + (xmax + xmin) * 0.5,
                           (ymax - ymin) * 0.5 * y_ + (ymax + ymin) * 0.5,
                           (zmax - zmin) * 0.5 * z_ + (zmax + zmin) * 0.5);
                ans += temp;
                double funtemp = fun((xmax - xmin) * 0.5 * x_ + (xmax + xmin) * 0.5,
                                     (ymax - ymin) * 0.5 * y_ + (ymax + ymin) * 0.5,
                                     (zmax - zmin) * 0.5 * z_ + (zmax + zmin) * 0.5);
                stream << temp << " " << funtemp <<  " ";
            }
            stream << std::endl;
        }
        stream << std::endl;
    }
    stream << "ans : " << ans << std::endl;
    stream << "end \n";
    cnt++;
    return ans;
}