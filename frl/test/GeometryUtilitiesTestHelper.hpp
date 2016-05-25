#ifndef __GEOMETRY_UTILITIES_TEST_HELPER_HPP__
#define __GEOMETRY_UTILITIES_TEST_HELPER_HPP__

#include <random>
#include <memory>
#include <math.h>
#include <eigen3/Eigen/Eigen>
#include <unistd.h>

namespace frl
{

	namespace geometry
	{

		class GeometryUtilitiesTestHelper
		{
		public:
			static double getRandomDouble()
			{
				// Generates random double between -1000 & 1000
				return (2000.0 * rand() / RAND_MAX - 1000.0);
			}

			template<typename T>
			static T getRandomNumber()
			{
				double val = (2000.0 * rand() / RAND_MAX - 1000.0);
				return static_cast<T>(val);
			}

			static float getRandomFloat()
			{
				// Generates random double between -100 & 100
				return (float)(200.0 * rand() / RAND_MAX - 100.0);
			}

			template<typename T>
			static T getRandomNumberBetween1AndMinus1()
			{
				T value = 2.0 * rand() / RAND_MAX - 1.0;
				return static_cast<T>(value);
			}

			template<typename T>
			static T getRandomAngle()
			{
				double angle = 2 * (M_PI - 0.01) * rand() / RAND_MAX - (M_PI - 0.01);
				return static_cast<T>(angle);
			}

			static std::vector<double> getRandom3dVector()
			{
				std::vector<double> vector(3);
				vector[0] = getRandomDouble();
				vector[1] = getRandomDouble();
				vector[2] = getRandomDouble();

				return vector;
			}

			template<typename T>
			static Point3<T> getRandomPoint3()
			{
				Point3<T> point;
				point.setX(getRandomDouble());
				point.setY(getRandomDouble());
				point.setZ(getRandomDouble());

				return point;
			}

			template<typename T>
			static Eigen::Matrix<T,3,3> createRandomRotationMatrix()
			{
				Eigen::Matrix<T,3,3> rotX = createRandomRotationMatrixX<T>();
				Eigen::Matrix<T,3,3> rotY = createRandomRotationMatrixY<T>();
				Eigen::Matrix<T,3,3> rotZ = createRandomRotationMatrixZ<T>();

				return rotX * rotY * rotZ;
			}

			template<typename T>
			static Eigen::Matrix<T,3,3> createRandomRotationMatrixX()
			{
				Eigen::Matrix<T,3,3> rotX;
				rotX(0, 0) = 1;
				rotX(0, 1) = 0;
				rotX(0, 2) = 0;
				rotX(1, 0) = 0;
				rotX(2, 0) = 0;

				T angle = getRandomAngle<T>();
				rotX(1, 1) = cos(angle);
				rotX(2, 2) = cos(angle);
				rotX(1, 2) = -sin(angle);
				rotX(2, 1) = sin(angle);

				return rotX;
			}

			template<typename T>
			static Eigen::Matrix<T,3,3> createRandomRotationMatrixY()
			{
				Eigen::Matrix<T,3,3> rotY;
				rotY(2, 1) = 0;
				rotY(0, 1) = 0;
				rotY(1, 0) = 0;
				rotY(1, 1) = 1;
				rotY(1, 2) = 0;

				T angle = getRandomAngle<T>();
				rotY(0, 0) = cos(angle);
				rotY(2, 2) = cos(angle);
				rotY(2, 0) = -sin(angle);
				rotY(0, 2) = sin(angle);

				return rotY;
			}

			template<typename T>
			static Eigen::Matrix<T,3,3> createRandomRotationMatrixZ()
			{
				Eigen::Matrix<T,3,3> rotZ;
				rotZ(0, 2) = 0;
				rotZ(1, 2) = 0;
				rotZ(2, 0) = 0;
				rotZ(2, 1) = 0;
				rotZ(2, 2) = 1;

				T angle = getRandomAngle<T>();
				rotZ(0, 0) = cos(angle);
				rotZ(1, 1) = cos(angle);
				rotZ(0, 1) = -sin(angle);
				rotZ(1, 0) = sin(angle);

				return rotZ;
			}

			template<typename T>
			static Eigen::Matrix<T,4,4> createRandomTransformationMatrix()
			{
				Eigen::Matrix<T,3,3> rotationMatrix = createRandomRotationMatrix<T>();

				Eigen::Matrix<T,4,4> transform;

				transform(0, 0) = rotationMatrix(0, 0);
				transform(0, 1) = rotationMatrix(0, 1);
				transform(0, 2) = rotationMatrix(0, 2);
				transform(1, 0) = rotationMatrix(1, 0);
				transform(1, 1) = rotationMatrix(1, 1);
				transform(1, 2) = rotationMatrix(1, 2);
				transform(2, 0) = rotationMatrix(2, 0);
				transform(2, 1) = rotationMatrix(2, 1);
				transform(2, 2) = rotationMatrix(2, 2);

				transform(0, 3) = getRandomNumber<T>();
				transform(1, 3) = getRandomNumber<T>();
				transform(2, 3) = getRandomNumber<T>();

				transform(3, 0) = 0.0;
				transform(3, 1) = 0.0;
				transform(3, 2) = 0.0;
				transform(3, 3) = 1.0;

				return transform;
			}

			template<typename T>
			static Eigen::Matrix<T,4,4> createRandomMatrix4()
			{
				Eigen::Matrix<T,4,4> matrix;

				matrix(0, 0) = getRandomNumberBetween1AndMinus1<T>();
				matrix(0, 1) = getRandomNumberBetween1AndMinus1<T>();
				matrix(0, 2) = getRandomNumberBetween1AndMinus1<T>();
				matrix(0, 3) = getRandomNumberBetween1AndMinus1<T>();
				matrix(1, 0) = getRandomNumberBetween1AndMinus1<T>();
				matrix(1, 1) = getRandomNumberBetween1AndMinus1<T>();
				matrix(1, 2) = getRandomNumberBetween1AndMinus1<T>();
				matrix(1, 3) = getRandomNumberBetween1AndMinus1<T>();
				matrix(2, 0) = getRandomNumberBetween1AndMinus1<T>();
				matrix(2, 1) = getRandomNumberBetween1AndMinus1<T>();
				matrix(2, 2) = getRandomNumberBetween1AndMinus1<T>();
				matrix(2, 3) = getRandomNumberBetween1AndMinus1<T>();
				matrix(3, 0) = 0;
				matrix(3, 1) = 0;
				matrix(3, 2) = 0;
				matrix(3, 3) = 1;

				return matrix;
			}

			template<typename T>
			static bool checkOrthogonality(Eigen::Matrix<T,4,4> matrix,T thresh=1e-8)
			{
				bool xMag = (1.0 - (sqrt(pow(matrix(0, 0), 2) + pow(matrix(1, 0), 2) + pow(matrix(2, 0), 2))) < thresh);
				bool yMag = (1.0 - (sqrt(pow(matrix(0, 1), 2) + pow(matrix(1, 1), 2) + pow(matrix(2, 1), 2))) < thresh);
				bool zMag = (1.0 - (sqrt(pow(matrix(0, 2), 2) + pow(matrix(1, 2), 2) + pow(matrix(2, 2), 2))) < thresh);

				return (xMag && yMag && zMag);
			}

			template<typename T>
			static Eigen::AngleAxis<T> createRandomAxisAngle()
			{
				T x, y, z;
				x = getRandomNumber<T>();
				y = getRandomNumber<T>();
				z = getRandomNumber<T>();

				T mag = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));

				x *= 1 / mag;
				y *= 1 / mag;
				z *= 1 / mag;

				T angle = getRandomAngle<T>();

				Eigen::AngleAxis<T> ret;
				ret.axis()(0) = x;
				ret.axis()(1) = y;
				ret.axis()(2) = z;
				ret.angle() = angle;
				return ret;
			}

			template<typename T>
			static Eigen::Quaternion<T> createRandomQuaternion()
			{
				Eigen::Quaternion<T> quaternion(getRandomNumber<T>(), getRandomNumber<T>(), getRandomNumber<T>(), getRandomNumber<T>());

				quaternion.normalize();

				return quaternion;
			}

			template<typename T>
			static bool areAxisAngleEpsilonEqual(const Eigen::AngleAxis<T> &a1, const Eigen::AngleAxis<T> &a2, const double &eps)
			{
				if ((fabs(a1.axis()[0] - a2.axis()[0]) < eps && fabs(a1.axis()[1] - a2.axis()[1]) < eps && fabs(a1.axis()[2] - a2.axis()[2]) < eps && fabs(a1.angle() - a2.angle()) < eps) ||
					(fabs(-a1.axis()[0] - a2.axis()[0]) < eps && fabs(-a1.axis()[1] - a2.axis()[1]) < eps && fabs(-a1.axis()[2] - a2.axis()[2]) < eps && fabs(-a1.angle() - a2.angle()) < eps))
				{
					return true;
				}

				if ((fabs(a1.axis()[0] - a2.axis()[0]) < eps && fabs(a1.axis()[1] - a2.axis()[1]) < eps && fabs(a1.axis()[2] - a2.axis()[2]) < eps))
				{
					if (M_PI - fabs(a1.angle()) < 1e-4 && M_PI - fabs(a2.angle()) < 1e-4)
					{
						return true;
					}
					else
					{
						return false;
					}
				}
			}

			template<typename T>
			static bool areQuaternionsEpsilonEqual(const Eigen::Quaternion<T> &q1, const Eigen::Quaternion<T> &q2, const double &eps)
			{
				if ((fabs(q1.x() - q2.x()) < eps && fabs(q1.y() - q2.y()) < eps && fabs(q1.z() - q2.z()) < eps && fabs(q1.w() - q2.w()) < eps) ||
					(fabs(-q1.x() - q2.x()) < eps && fabs(-q1.y() - q2.y()) < eps && fabs(-q1.z() - q2.z()) < eps && fabs(-q1.w() - q2.w()) < eps))
				{
					return true;
				}

				return false;
			}

            template<typename T>
			static bool areVector3sEpsilonEqual(const Eigen::Matrix<T,3,1> &v1, const Eigen::Matrix<T,3,1> &v2, const double &eps)
			{
				return (fabs(v1(0) - v2(0)) < eps && fabs(v1(1) - v2(1)) < eps && fabs(v1(2) - v2(2)) < eps);
			}

            template<typename T>
			static bool areVector4sEpsilonEqual(const Eigen::Matrix<T,4,1> &v1, const Eigen::Matrix<T,4,1> &v2, const double &eps)
			{
				return (fabs(v1(0) - v2(0)) < eps && fabs(v1(1) - v2(1)) < eps && fabs(v1(2) - v2(2)) < eps && fabs(v1(3) - v2(3)) < eps);
			}

			template<typename T>
			static Eigen::Matrix<T,3,1> createRandomVector3()
			{
				Eigen::Matrix<T,3,1> vector;

				vector(0) = getRandomNumber<T>();
				vector(1) = getRandomNumber<T>();
				vector(2) = getRandomNumber<T>();

				return vector;
			}

            template<typename T>
			static bool areMatrix3EpsilonEqual(const Eigen::Matrix<T,3,3> &m1, const Eigen::Matrix<T,3,3> &m2, double epsilon)
			{
				for (int i = 0; i < 3; i++)
				{
					for (int j = 0; j < 3; j++)
					{
						bool tmp = fabs(m1(i, j) - m2(i, j)) < epsilon;
						if (!tmp)
						{
							return false;
						}
					}
				}

				return true;
			}

            template<typename T>
			static bool areMatrix4EpsilonEqual(const Eigen::Matrix<T,4,4> &m1, const Eigen::Matrix<T,4,4> &m2, double epsilon)
			{
				for (int i = 0; i < 4; i++)
				{
					for (int j = 0; j < 4; j++)
					{
						bool tmp = fabs(m1(i, j) - m2(i, j)) < epsilon;
						if (!tmp)
						{
							return false;
						}
					}
				}

				return true;
			}
		};

	}
}
#endif