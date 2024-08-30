#include <vector>
#include <iostream>
#include <math.h>

#include <Eigen/Dense> 

#include "CoordinateConversion.hpp"

int main()
{
    std::vector<Eigen::Vector3d> p1 = 
    {
        Eigen::Vector3d(-4637.10, 6521.83, 1191.98), 
        Eigen::Vector3d(-7090.79, -4915.28, 445.28), 
        Eigen::Vector3d(-4418.70, 1921.64, 904.06),
        Eigen::Vector3d(-5145.94, -3165.80, 1213.44)
    };

    std::vector<Eigen::Vector3d> p2 = 
    {
        Eigen::Vector3d(1344.15, 342.17, 6.42),
        Eigen::Vector3d(13041.84, -399.23, 2.12),
        Eigen::Vector3d(5795.92, 56.90, -1173.72),
        Eigen::Vector3d(10922.47, 368.79, -1531.35)
    };

    SK::CoordinateConversion conversion;

    try
    {
        if (conversion.computeConversion(p1, p2))
        {
            Eigen::Matrix3d rotationMatrix = conversion.getRotationMatrix();
            Eigen::Vector3d translationVector = conversion.getTranslationVector();

            std::cout << "Rotation Matrix:\n" << rotationMatrix << std::endl;  
            std::cout << "Translation Vector:\n" << translationVector << std::endl;  

            std::cout << "Calculate Deviation:" << std::endl;

            for (size_t i = 0; i < p1.size(); i++)
            {
                std::cout << "point "<< i << "\n" << p2[i] - (rotationMatrix * p1[i] + translationVector) << "\n-------------------" << std::endl;
            }

            std::cout << "Angle (degrees): Rx,Ry,Rz \n" << conversion.getEulerAnglesZYX ()* (180.0 / acos(-1)) << std::endl; 

        }
        else
        {
            std::cout << "compute error" << std::endl;
        }
    }
    catch (std::exception &e)
    {
        std::cout << "exception happened: " << e.what() << std::endl;
    }

}
