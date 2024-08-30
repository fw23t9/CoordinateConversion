using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

namespace conversion
{
    class CoordinateConversion
    {
        private Vector<double> m_translationVector = new DenseVector(3);
        private Matrix<double> m_rotationMatrix = new DenseMatrix(3);

        public Vector<double> TranslationVector { get { return m_translationVector; } }
        public Matrix<double> RotationMatrix { get { return m_rotationMatrix; } }

        private bool m_isValid = false;
        public bool isValid() { return m_isValid; }

        // from p1 to p2
        public bool computeConversion(List<Vector<double>> p1, List<Vector<double>> p2)
        {
            computeInternal(p1, p2);
            m_isValid = true;

            return m_isValid;
        }

        public Vector<double> getEulerAnglesZYXInDegree()
        {
            double beta = Math.Atan2(-m_rotationMatrix[2, 0], Math.Sqrt(m_rotationMatrix[2, 1] * m_rotationMatrix[2, 1] + m_rotationMatrix[2, 2] * m_rotationMatrix[2, 2]));
            double alpha = Math.Atan2(m_rotationMatrix[2, 1] / Math.Cos(beta), m_rotationMatrix[2, 2] / Math.Cos(beta));
            double theta = Math.Atan2(m_rotationMatrix[1, 0] / Math.Cos(beta), m_rotationMatrix[0, 0] / Math.Cos(beta));

            beta = beta / Math.Acos(-1) * 180;
            alpha = alpha / Math.Acos(-1) * 180;
            theta = theta / Math.Acos(-1) * 180;

            return DenseVector.OfArray(new double[] { alpha, beta, theta });
        }

        // from p1 to p2
        private Matrix<double> computeInternal(List<Vector<double>> p1, List<Vector<double>> p2)
        {
            int n = (int)p1.Count;

            Vector<double> center1 = new DenseVector(3) { 0 ,0 ,0 };
            Vector<double> center2 = new DenseVector(3) { 0 ,0 ,0 };

            for (int i = 0; i < n; ++i)
            {
                center1 += p1[i];
                center2 += p2[i];
            }
            center1 /= n;
            center2 /= n;

            Matrix<double> centered_p1 = new DenseMatrix(3, n);
            Matrix<double> centered_p2 = new DenseMatrix(3, n);
            for (int i = 0; i < centered_p1.ColumnCount; ++i)
            {
                for (int j = 0; j < centered_p1.RowCount; ++j)
                {
                    centered_p1[j, i] = p1[i][j] - center1[j];
                    centered_p2[j, i] = p2[i][j] - center2[j];
                }
            }

            Matrix<double> H = centered_p1 * centered_p2.Transpose();

            var svd = H.Svd(true);

            Matrix<double> U = svd.U;
            Vector<double> S = svd.S;
            Matrix<double> Vt = svd.VT;

            if (U.Determinant()  * Vt.Determinant() < 0) {
                for (int i = 0; i < Vt.RowCount; i++)  
                {  
                    Vt[i, 1] = -Vt[i, 1];  
                }  
            }

            m_rotationMatrix = Vt.Transpose() * U.Transpose();

            m_translationVector = center2 - m_rotationMatrix * center1;
            
            return m_rotationMatrix;
        }
    }
}
