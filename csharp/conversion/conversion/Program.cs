using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

namespace conversion
{
    class Program
    {
        static void Main(string[] args)
        {
            List<Vector<double>> p1 = new List<Vector<double>>()
            {
                DenseVector.OfArray(new double []{ -4637.10, 6521.83, 1191.98 }),
                DenseVector.OfArray(new double []{ -7090.79, -4915.28, 445.28 }), 
                DenseVector.OfArray(new double []{ -4418.70, 1921.64, 904.06 }),
                DenseVector.OfArray(new double []{ -5145.94, -3165.80, 1213.44 })
            };

            List<Vector<double>> p2 = new List<Vector<double>>()
            {
                DenseVector.OfArray(new double []{ 1344.15, 342.17, 6.42 }),
                DenseVector.OfArray(new double []{ 13041.84, -399.23, 2.12 }),
                DenseVector.OfArray(new double []{ 5795.92, 56.90, -1173.72 }),
                DenseVector.OfArray(new double []{ 10922.47, 368.79, -1531.35 })
            };

            CoordinateConversion conversion = new CoordinateConversion();

            try
            {
                if (conversion.computeConversion(p1, p2))
                {
                    Matrix<double> rotationMatrix = conversion.RotationMatrix;
                    Vector<double> translationVector = conversion.TranslationVector;

                    Console.WriteLine("Rotation Matrix: ");
                    Console.WriteLine(rotationMatrix.ToString("#,##0.000000\t"));

                    Console.WriteLine("Translation Vector:");
                    Console.WriteLine(translationVector.ToString("#,##0.000000\t"));

                    Console.WriteLine("Calculate Deviation:");

                    for(int i = 0; i < p1.Count; i++)
                    {
                        Console.WriteLine($"point {i}");

                        Vector<double> calcRes = p2[i] - (rotationMatrix * p1[i] + translationVector);
                        Console.WriteLine($"{calcRes.ToString("#,##0.000000\t")}");
                        Console.WriteLine("--------------------------------");
                    }

                    Console.WriteLine(conversion.getEulerAnglesZYXInDegree().ToString("#,##0.000000\t"));
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"exception happened: {ex.Message}");
            }

            Console.ReadKey();
        }
    }
}
