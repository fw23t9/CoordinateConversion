#include <Eigen/Dense> 

namespace SK
{

class CoordinateConversion
{
public:
    CoordinateConversion()
        : m_isValid(false)
    {

    }

    ~CoordinateConversion() {}

public:
    // converse from p1 to p2
    bool computeConversion(const std::vector<Eigen::Vector3d>& p1, const std::vector<Eigen::Vector3d>& p2)
    {
        computeInternal(p1, p2);
        m_isValid = true;
        
        return m_isValid;
    }

    bool isValid() { return m_isValid; }
    Eigen::Matrix3d getRotationMatrix() { return m_rotationMatrix; }
    Eigen::Vector3d getTranslationVector() { return m_translationVector; }

    Eigen::Vector3f getEulerAnglesZYX() 
    {
        if (!m_isValid) return Eigen::Vector3f(0, 0, 0);

        float roll = atan2f(m_rotationMatrix(2, 1), m_rotationMatrix(2, 2));
        float pitch = asinf(-m_rotationMatrix(2, 0));
        float yaw = atan2f(m_rotationMatrix(1, 0), m_rotationMatrix(0, 0));

        // rad angle here
        return Eigen::Vector3f(roll, pitch, yaw);
    }

private:
    bool m_isValid;

    Eigen::Matrix3d m_rotationMatrix;
    Eigen::Vector3d m_translationVector;

private:
    Eigen::Matrix3d computeInternal(const std::vector<Eigen::Vector3d>& p1, const std::vector<Eigen::Vector3d>& p2) 
    {  
        int n = (int)p1.size();  
        Eigen::Vector3d center1 = Eigen::Vector3d::Zero();
        Eigen::Vector3d center2 = Eigen::Vector3d::Zero();  
        for (int i = 0; i < n; ++i) 
        {  
            center1 += p1[i];  
            center2 += p2[i];  
        }  
        center1 /= n;  
        center2 /= n;  
    
        Eigen::MatrixXd centered_p1(3, n);
        Eigen::MatrixXd centered_p2(3, n);
        for (int i = 0; i < n; ++i) 
        {  
            centered_p1.col(i) = p1[i] - center1;
            centered_p2.col(i) = p2[i] - center2;
        }  

        Eigen::MatrixXd H = centered_p1 * centered_p2.transpose();

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);  
        Eigen::Matrix3d U = svd.matrixU();  
        Eigen::Matrix3d V = svd.matrixV().transpose();  
    
        if (U.determinant() * V.determinant() < 0) {  
            V.col(2) *= -1;
        }

        m_rotationMatrix = V.transpose() * U.transpose();

        m_translationVector = center2 - m_rotationMatrix * center1;

        return m_rotationMatrix;
    }  

};


} // namespace SK
