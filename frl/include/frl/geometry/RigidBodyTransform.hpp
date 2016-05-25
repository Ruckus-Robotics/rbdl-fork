#ifndef RIGID_BODY_TRANSFORM_HPP
#define RIGID_BODY_TRANSFORM_HPP

#include <eigen3/Eigen/Eigen>
#include "frl/geometry/Point3.hpp"

namespace frl
{
	namespace geometry
	{
		template<typename T>
		class RigidBodyTransform
        {
        public:
            RigidBodyTransform()
            {
                setIdentity();
            }

            template<typename TYPE>
            RigidBodyTransform(const RigidBodyTransform<TYPE> &transform)
            {
                set(transform);
            }

            template<typename TYPE>
            RigidBodyTransform(const Eigen::Matrix<TYPE, 4, 4> &matrix)
            {
                set(matrix);
            }

            template<typename T1, typename T2>
            RigidBodyTransform(const Eigen::Matrix<T1, 3, 3> &matrix, const Eigen::Matrix<T2, 3, 1> &vector)
            {
                set(matrix, vector);
            };

            template<typename T1, typename T2>
            RigidBodyTransform(const Eigen::AngleAxis<T1> &axisAngle, const Eigen::Matrix<T2, 3, 1> &vector)
            {
                set(axisAngle, vector);
            }

            template<typename T1, typename T2>
            RigidBodyTransform(const Eigen::Quaternion<T1> &quaternion, const Eigen::Matrix<T2, 3, 1> &vector)
            {
                set(quaternion, vector);
            };

            template<typename TYPE>
            RigidBodyTransform(const Eigen::Matrix<TYPE, 3, 3> &matrix)
            {
                setRotation(matrix);
                setTranslation(0.0, 0.0, 0.0);
            }

            template<typename TYPE>
            RigidBodyTransform(const Eigen::Quaternion<TYPE> &quat)
            {
                setRotation(quat.w(),quat.x(),quat.y(),quat.z());
                setTranslation(0.0, 0.0, 0.0);
            }

            template<typename TYPE>
            RigidBodyTransform(const Eigen::AngleAxis<TYPE> &axisAngle)
            {
                setRotation(axisAngle);
                setTranslation(0.0, 0.0, 0.0);
            }

            ~RigidBodyTransform()
            { };

            /**
            * Set transformation matrix to Identity, meaning no rotation or
            * translation.
            */
            void setIdentity()
            {

                qw = 1.0;
                qx = 0.0;
                qy = 0.0;
                qz = 0.0;

                x = 0.0;
                y = 0.0;
                z = 0.0;
            }

            T norm()
            {
                return sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
            }

            template<typename TYPE>
            void set(const RigidBodyTransform<TYPE> &transform)
            {
                qw = transform.qw;
                qx = transform.qx;
                qy = transform.qy;
                qz = transform.qz;

                x = transform.x;
                y = transform.y;
                z = transform.z;
            }

            /**
             * Set elements of transform
             *
             * @param matrix
             */
            template<typename TYPE>
            void set(const Eigen::Matrix<TYPE, 4, 4> &matrix)
            {
                setRotationWithRotationMatrix(matrix(0,0),matrix(1,0),matrix(2,0),matrix(0,1),matrix(1,1),matrix(2,1),matrix(0,2),matrix(1,2),matrix(2,2));
                setTranslation(matrix(0,3),matrix(1,3),matrix(2,3));
            }

            /**
            * Set this transform to have translation described in vector
            * and a rotation equal to matrix.
            *
            * @param Eigen::Matrix matrix
            * @param Eigen::Matrix vector
            */
            template<typename T1, typename T2>
            void set(const Eigen::Matrix<T1, 3, 3> &matrix, const Eigen::Matrix<T2, 3, 1> &vector)
            {
                setRotation(matrix);
                setTranslation(vector);
            }

            /**
            * Set this transform to have translation described in vector
            * and a rotation equal to axisAngles.
            *
            * @param Eigen::AxisAngle axisAngle
            * @param Eigen::Matrix vector
            */
            template<typename T1, typename T2>
            void set(const Eigen::AngleAxis<T1> &axisAngle, const Eigen::Matrix<T2, 3, 1> &vector)
            {
                setRotation(axisAngle);
                setTranslation(vector);
            }

            /**
            * Set this transform to have translation described in vector
            * and a rotation equal to quat.
            *
            * @param Eigen::Quaternion quat
            * @param Eigen::Matrix vector
            */
            template<typename T1, typename T2>
            void set(const Eigen::Quaternion<T1> &quat, const Eigen::Matrix<T2, 3, 1> &vector)
            {
                setRotation(quat);
                setTranslation(vector);
            }


            /**
             * Set translational portion of the transformation matrix
             *
             * @param x The x-component of the translation
             * @param y The y-component of the translation
             * @param z The z-component of the translation
             */
            template<typename TYPE>
            void setTranslation(const TYPE x, const TYPE y, const TYPE z)
            {
                this->x = x;
                this->y = y;
                this->z = z;
            }

            /**
             * Set translational portion of the transformation matrix
             *
             * @param vector
             */
            template<typename TYPE>
            void setTranslation(const Eigen::Matrix<TYPE, 3, 1> &vector)
            {
                this->x = vector(0);
                this->y = vector(1);
                this->z = vector(2);
            }

            void zeroTranslation()
            {
                x = 0.0;
                y = 0.0;
                z = 0.0;
            }

            template<typename TYPE>
            void setRotation(Eigen::Matrix<TYPE, 3, 3> matrix)
            {
                setRotationWithRotationMatrix(matrix(0,0),matrix(1,0),matrix(2,0),matrix(0,1),matrix(1,1),matrix(2,1),matrix(0,2),matrix(1,2),matrix(2,2));
            }

            void setRotation(const T qw, const T qx, const T qy, const T qz)
            {
                this->qx = qx;
                this->qy = qy;
                this->qz = qz;
                this->qw = qw;
            }

            template<typename TYPE>
            void setRotation(const Eigen::Quaternion<TYPE> &quat)
            {
                this->qx = quat.x();
                this->qy = quat.y();
                this->qz = quat.z();
                this->qw = quat.w();
            }

            template<typename TYPE>
			void setRotation(const Eigen::AngleAxis<TYPE> &axisAngle)
            {
                setRotationWithAxisAngle(axisAngle.axis()[0], axisAngle.axis()[1], axisAngle.axis()[2], axisAngle.angle());
            }

			template<typename TYPE>
			void setRotationWithAxisAngle(const TYPE axisAngleX, const TYPE axisAngleY, const TYPE &axisAngleZ, const TYPE axisAngleTheta)
            {
                qx = axisAngleX;
                qy = axisAngleY;
                qz = axisAngleZ;
                T n = sqrt(qx*qx + qy*qy + qz*qz);
                // Division by zero might happen here. Meh
                T s = sin(0.5*axisAngleTheta)/n;
                qx *= s;
                qy *= s;
                qz *= s;
                qw = cos(0.5*axisAngleTheta);
            }

            template<typename TYPE>
			void setRotationAndZeroTranslation(const Eigen::Matrix<TYPE,3,3> &matrix)
            {
                setRotation(matrix);
                setTranslation(0.0,0.0,0.0);
            }

            template<typename TYPE>
			void setRotationAndZeroTranslation(const Eigen::Quaternion<TYPE> &quat)
            {
                setRotation(quat);
                setTranslation(0.0,0.0,0.0);
            }

            template<typename TYPE>
			void setRotationAndZeroTranslation(const Eigen::AngleAxis<TYPE> &axisAngle)
            {
                setRotation(axisAngle);
                setTranslation(0.0,0.0,0.0);
            }

            /**
            * Set this transform to have an identity rotation and a translation given
            * by the Eigen::Vector3d vector.
            *
            * @param vector
            */
            template<typename TYPE>
			void setTranslationAndIdentityRotation(const Eigen::Matrix<TYPE,3,1> &vector)
            {
                setTranslation(vector(0), vector(1), vector(2));
                setRotationToIdentity();
            }

            /**
            * Sets rotation to the identity, does not effect the translational component of the Transform
            *
            */
			void setRotationToIdentity()
            {
                qw = 1.0;
                qx = 0.0;
                qy = 0.0;
                qz = 0.0;
            }

            /**
            * Set the rotational component of the transform to the rotation matrix
            * created given an X-Y-Z rotation described by the angles in vector which
            * describe angles of rotation about the X, Y, and Z axis, respectively. The
            * orientation of each rotation is not effected by any of the other
            * rotations. This method sets the translational component of this
            * transform3d to zeros.
            *
            * @param vector
            */
            template<typename TYPE>
			void setEulerXYZ(const Eigen::Matrix<TYPE,3,1> &vector)
            {
                setEulerXYZ(vector(0), vector(1), vector(2));
            }

            /**
             * Set the rotational component of the transform to the rotation matrix
             * created given an X-Y-Z rotation described by the angles in vector which
             * describe angles of rotation about the X, Y, and Z axis, respectively. The
             * orientation of each rotation is not effected by any of the other
             * rotations. This method sets the translational component of this
             * transform3d to zeros.
             *
             * @param rotX
             * @param rotY
             * @param rotZ
             */
            template<typename TYPE>
			void setEulerXYZ(const TYPE roll, const TYPE pitch, const TYPE yaw)
            {
                Eigen::Matrix<T,3,3> m;

                T sina = sin(roll);
                T sinb = sin(pitch);
                T sinc = sin(yaw);
                T cosa = cos(roll);
                T cosb = cos(pitch);
                T cosc = cos(yaw);

                m(0,0) = cosb * cosc;
                m(0,1) = -(cosa * sinc) + (sina * sinb * cosc);
                m(0,2) = (sina * sinc) + (cosa * sinb * cosc);
                m(1,0) = cosb * sinc;
                m(1,1) = (cosa * cosc) + (sina * sinb * sinc);
                m(1,2) = -(sina * cosc) + (cosa * sinb * sinc);
                m(2,0) = -sinb;
                m(2,1) = sina * cosb;
                m(2,2) = cosa * cosb;

                setRotation(m);

                x = 0.0;
                y = 0.0;
                z = 0.0;
            }

            /**
            * Computes the RPY angles from the rotation matrix for rotations about the
            * X, Y, and Z axes respectively. Note that this method is here for the
            * purpose of unit testing the method setEuler. This particular solution is
            * only valid for -pi/2.0 < vector.y < pi/2.0 and for vector.y != 0.
            *
            * @param vector
            */
            template<typename TYPE>
			void getEulerXYZ(Eigen::Matrix<TYPE,3,1> &vector) const
            {
                Eigen::Matrix<T,3,3> m;
                getRotation(m);

                vector(0) = atan2(m(2,1), m(2,2));
                vector(1) = atan2(-m(2,0), sqrt(m(2,1) * m(2,1) + m(2,2) * m(2,2)));
                vector(2) = atan2(m(1,0), m(0,0));
            }

            /**
            * Return rotation in quaternion form.
            *
            * @param Eigen::Quaternion quat
            */
            template<typename TYPE>
            void getQuaternion(Eigen::Quaternion<TYPE> &q) const
            {
                q.x() = qx;
                q.y() = qy;
                q.z() = qz;
                q.w() = qw;
            }

            /**
            * Return rotation matrix
            *
            * @param matrix
            */
            template<typename TYPE>
			void getRotation(Eigen::Matrix<TYPE,3,3> &matrix) const
            {
                T sqw = qw*qw;
                T sqx = qx*qx;
                T sqy = qy*qy;
                T sqz = qz*qz;

                T invs = 1 / (sqx + sqy + sqz + sqw);
                matrix(0,0) = ( sqx - sqy - sqz + sqw)*invs ;
                matrix(1,1) = (-sqx + sqy - sqz + sqw)*invs ;
                matrix(2,2)= (-sqx - sqy + sqz + sqw)*invs ;

                T tmp1 = qx*qy;
                T tmp2 = qz*qw;
                matrix(1,0) = 2.0 * (tmp1 + tmp2)*invs;
                matrix(0,1) = 2.0 * (tmp1 - tmp2)*invs;

                tmp1 = qx*qz;
                tmp2 = qy*qw;
                matrix(2,0) = 2.0 * (tmp1 - tmp2)*invs ;
                matrix(0,2) = 2.0 * (tmp1 + tmp2)*invs ;
                tmp1 = qy*qz;
                tmp2 = qx*qw;
                matrix(2,1) = 2.0 * (tmp1 + tmp2)*invs ;
                matrix(1,2) = 2.0 * (tmp1 - tmp2)*invs ;
            }

            template<typename TYPE>
            void getRotation(Eigen::Quaternion<TYPE> &quat) const
            {
                quat.x() = qx;
                quat.y() = qy;
                quat.z() = qz;
                quat.w() = qw;
            }

            /**
            * Return rotation in AxisAngle form.
            *
            * @param axisAngle
            */
            template<typename TYPE>
			void getRotation(Eigen::AngleAxis<TYPE> &axisAngle) const
            {
                T sin_a2 = sqrt(qx*qx + qy*qy + qz*qz);
                axisAngle.angle() = 2.0*atan2(sin_a2, qw);

                axisAngle.axis().x() = qx/sqrt(1-qw*qw);
                axisAngle.axis().y() = qy/sqrt(1-qw*qw);
                axisAngle.axis().z() = qz/sqrt(1-qw*qw);
            }

            /**
            * Return translational part
            *
            * @param vector
            */
            template<typename TYPE>
			void getTranslation(Eigen::Matrix<TYPE,3,1> &vector) const
            {
                vector(0) = x;
                vector(1) = y;
                vector(2) = z;
            }

            /**
            * Return translational part
            *
            * @param point
            */
            template<typename TYPE>
			void getTranslation(Point3<TYPE> &point) const
            {
                point.x = x;
                point.y = y;
                point.z = z;
            }

            /**
            * Pack transform into matrix
            *
            * @param matrix
            */
            template<typename TYPE>
			void get(Eigen::Matrix<TYPE,4,4> &matrix) const
            {
                Eigen::Matrix<T,3,3> rotationMatrix;
                getRotation(rotationMatrix);

                matrix(0,0) = rotationMatrix(0,0);
                matrix(0,1) = rotationMatrix(0,1);
                matrix(0,2) = rotationMatrix(0,2);
                matrix(1,0) = rotationMatrix(1,0);
                matrix(1,1) = rotationMatrix(1,1);
                matrix(1,2) = rotationMatrix(1,2);
                matrix(2,0) = rotationMatrix(2,0);
                matrix(2,1) = rotationMatrix(2,1);
                matrix(2,2) = rotationMatrix(2,2);

                matrix(0,3) = x;
                matrix(1,3) = y;
                matrix(2,3) = z;

                matrix(3,0) = 0.0;
                matrix(3,1) = 0.0;
                matrix(3,2) = 0.0;
                matrix(3,3) = 1.0;
            }

            template<typename TYPE>
			void get(Eigen::Matrix<TYPE,3,3> &matrix, Eigen::Matrix<TYPE,3,1> &vector) const
            {
                getRotation(matrix);
                getTranslation(vector);
            }

            template<typename TYPE>
			void get(Eigen::Quaternion<TYPE> &quat, Eigen::Matrix<TYPE,3,1> &vector) const
            {
                getRotation(quat);
                getTranslation(vector);
            }

            template<typename TYPE>
			void get(Eigen::Quaternion<TYPE> &quat, Point3<TYPE> &point) const
            {
                getRotation(quat);
                getTranslation(point);
            }

            /**
            * Transform the Point3 point by this transform and place result back in
            * point.
            *
            * @param point
            */
            template<typename TYPE>
			void transform(Point3<TYPE> &point)
            {
                TYPE tmpX = (qw*qw + qx*qx - qy*qy - qz*qz)*point.x + 2.0*(qx*qy - qw*qz)*point.y + 2.0*(qw*qy + qx*qz)*point.z + x;
                TYPE tmpY = 2.0*(qx*qy + qw*qz)*point.x + (qw*qw - qx*qx + qy*qy - qz*qz)*point.y + 2.0*(-qw*qx + qy*qz)*point.z + y;
                point.z = (-2.0*qw*qy + 2.0*qx*qz)*point.x + 2.0*(qw*qx + qy*qz)*point.y + (qw*qw - qx*qx - qy*qy + qz*qz)*point.z + z;

                point.x = tmpX;
                point.y = tmpY;
            }

            /**
            * Transform vector by multiplying it by this transform and put result back
            * into vector.
            *
            * @param vector
            */
            template<typename TYPE>
			void transform(Eigen::Matrix<TYPE,4,1> &vector)
            {
                T qwS = qw*qw;
                T qxS = qx*qx;
                T qyS = qy*qy;
                T qzS = qz*qz;

                TYPE tmpX = (qwS + qxS - qyS - qzS)*vector(0) + 2.0*(qx*qy - qw*qz)*vector(1) + 2.0*(qw*qy + qx*qz)*vector(2) + x;
                TYPE tmpY = 2.0*(qx*qy + qw*qz)*vector(0) + (qwS - qxS + qyS - qzS)*vector(1) + 2.0*(-qw*qx + qy*qz)*vector(2) + y;
                vector(2) = (-2.0*qw*qy + 2.0*qx*qz)*vector(0) + 2.0*(qw*qx + qy*qz)*vector(1) + (qwS - qxS - qyS + qzS)*vector(2) + z;
                
                vector(0) = tmpX;
                vector(1) = tmpY;
                vector(3) = 1.0;
            }

            /**
             * Transform vector by multiplying it by this transform and put result back
             * into vector.
             *
             * @param vector
             */
            template<typename TYPE>
			void transform(Eigen::Matrix<TYPE,3,1> &vector)
            {
                T qwS = qw*qw;
                T qxS = qx*qx;
                T qyS = qy*qy;
                T qzS = qz*qz;
                
                T tmpX = (qwS + qxS - qyS - qzS)*vector(0) + 2.0*(qx*qy - qw*qz)*vector(1) + 2.0*(qw*qy + qx*qz)*vector(2);
                T tmpY = 2.0*(qx*qy + qw*qz)*vector(0) + (qwS - qxS + qyS - qzS)*vector(1) + 2.0*(-qw*qx + qy*qz)*vector(2);
                vector(2) = (-2.0*qw*qy + 2.0*qx*qz)*vector(0) + 2.0*(qw*qx + qy*qz)*vector(1) + (qwS - qxS - qyS + qzS)*vector(2);

                vector(0) = tmpX;
                vector(1) = tmpY;
            }

            /**
            * Transform vector by multiplying it by this transform and put result back
            * into vector.
            *
            * @param vector
            */
            template<typename TYPE>
			void transform(const Eigen::Matrix<TYPE,3,1> &vectorIn, Eigen::Matrix<TYPE,3,1> &vectorOut)
            {
                T qwS = qwS;
                T qxS = qxS;
                T qyS = qyS;
                T qzS = qzS;

                vectorOut(0) = (qwS + qxS - qyS - qzS)*vectorIn(0) + 2.0*(qx*qy - qw*qz)*vectorIn(1) + 2.0*(qw*qy + qx*qz)*vectorIn(2);
                vectorOut(1) = 2.0*(qx*qy + qw*qz)*vectorIn(0) + (qwS - qxS + qyS - qzS)*vectorIn(1) + 2.0*(-qw*qx + qy*qz)*vectorIn(2);
                vectorOut(2) = (-2.0*qw*qy + 2.0*qx*qz)*vectorIn(0) + 2.0*(qw*qx + qy*qz)*vectorIn(1) + (qwS - qxS - qyS + qzS)*vectorIn(2);
            }

            /**
            * Transform vectorIn using this transform and store result in vectorOut.
            *
            * @param vectorIn
            * @param vectorOut
            */
            template<typename TYPE>
			void transform(const Eigen::Matrix<TYPE,4,1> &vectorIn, Eigen::Matrix<TYPE,4,1> &vectorOut)
            {
                vectorOut(0) = (qw*qw + qx*qx - qy*qy - qz*qz)*vectorIn(0) + 2.0*(qx*qy - qw*qz)*vectorIn(1) + 2.0*(qw*qy + qx*qz)*vectorIn(2) + x;
                vectorOut(1) = 2.0*(qx*qy + qw*qz)*vectorIn(0) + (qw*qw - qx*qx + qy*qy - qz*qz)*vectorIn(1) + 2.0*(-qw*qx + qy*qz)*vectorIn(2) + y;
                vectorOut(2) = (-2.0*qw*qy + 2.0*qx*qz)*vectorIn(0) + 2.0*(qw*qx + qy*qz)*vectorIn(1) + (qw*qw - qx*qx - qy*qy + qz*qz)*vectorIn(2) + z;
                vectorOut(3) = 1.0;
            }

            /**
            * Transform the Point3d pointIn by this transform and place result in
            * pointOut.
            *
            * @param point
            */
            template<typename TYPE>
			void transform(const Point3<TYPE> &pointIn, Point3<TYPE> &pointOut)
            {
                pointOut.x = (qw*qw + qx*qx - qy*qy - qz*qz)*pointIn.x + 2.0*(qx*qy - qw*qz)*pointIn.y + 2.0*(qw*qy + qx*qz)*pointIn.z + x;
                pointOut.y = 2.0*(qx*qy + qw*qz)*pointIn.x + (qw*qw - qx*qx + qy*qy - qz*qz)*pointIn.y + 2.0*(-qw*qx + qy*qz)*pointIn.z + y;
                pointOut.z = (-2.0*qw*qy + 2.0*qx*qz)*pointIn.x + 2.0*(qw*qx + qy*qz)*pointIn.y + (qw*qw - qx*qx - qy*qy + qz*qz)*pointIn.z + z;
            }

            /**
            *  Apply a x-axis rotation to the current transform.
            */
			template<typename TYPE>
			void applyRotationX(const TYPE angle)
            {
                RigidBodyTransform<T> temp;
                temp.rotX(angle);
                multiply(temp);
            }

            /**
            *  Apply a y-axis rotation to the current transform.
            */
			template<typename TYPE>
			void applyRotationY(const TYPE angle)
            {
                RigidBodyTransform<T> temp;
                temp.rotY(angle);
                multiply(temp);
            }

            /**
            *  Apply a z-axis rotation to the current transform.
            */
			template<typename TYPE>
			void applyRotationZ(const TYPE angle)
            {
                RigidBodyTransform<T> temp;
                temp.rotZ(angle);
                multiply(temp);
            }

			template<typename TYPE>
			void rotX(const TYPE angle)
            {
                x = 0.0;
                y = 0.0;
                z = 0.0;

                qx = sin(angle/2.0);
                qy = 0.0;
                qz = 0.0;
                qw = cos(angle/2.0);
            }

            /**
            * Create RigidBodyTransform with zero translation and the rotation matrix being a
            * rotation about the y-axis by angle.
            *
            * @param angle
            */
			template<typename TYPE>
			void rotY(const TYPE angle)
            {
                x = 0.0;
                y = 0.0;
                z = 0.0;

                qx = 0.0;
                qy = sin(angle/2.0);
                qz = 0.0;
                qw = cos(angle/2.0);
            }

            /**
             * Create RigidBodyTransform with zero translation and the rotation matrix being a
             * rotation about the z-axis by angle.
             *
             * @param angle
             */
			template<typename TYPE>
			void rotZ(const TYPE angle)
            {
                x = 0.0;
                y = 0.0;
                z = 0.0;

                qx = 0.0;
                qy = 0.0;
                qz = sin(angle/2.0);
                qw = cos(angle/2.0);
            }

            /**
            * Multiplies this RigidBodyTransform by transform and stores the result in this,
            * i.e. this = this*transform
            *
            * @param transform
            */
            template<typename TYPE>
			void multiply(const RigidBodyTransform<TYPE> &transform)
            {
                T tmpQw = qw*transform.qw - qx*transform.qx - qy*transform.qy - qz*transform.qz;
                T tmpQx = qx*transform.qw + qw*transform.qx - qz*transform.qy + qy*transform.qz;
                T tmpQy = qy*transform.qw + qz*transform.qx + qw*transform.qy - qx*transform.qz;
                T tmpQz = qz*transform.qw - qy*transform.qx + qx*transform.qy + qw*transform.qz;

                T tmpX = (qw*qw + qx*qx - qy*qy - qz*qz)*transform.x +
                         2.0*(qx*qy - qw*qz)*transform.y + 2.0*(qw*qy + qx*qz)*transform.z + x;

                T tmpY = 2.0*(qx*qy + qw*qz)*transform.x + (qw*qw - qx*qx + qy*qy - qz*qz)*transform.y +
                        2.0*(-qw*qx + qy*qz)*transform.z + y;

                T tmpZ = (-2.0*qw*qy + 2.0*qx*qz)*transform.x + 2.0*(qw*qx + qy*qz)*transform.y +
                        (qw*qw - qx*qx - qy*qy + qz*qz)*transform.z + z;

                qw = tmpQw;
                qx = tmpQx;
                qy = tmpQy;
                qz = tmpQz;

                x = tmpX;
                y = tmpY;
                z = tmpZ;
            }

            /**
            * Multiplies transform1 and transform and puts result into this. this =
            * transform1*transform
            *
            * @param transform1
            * @param transform
            */
            template<typename TYPE>
			void multiply(const RigidBodyTransform<TYPE> &transform1, const RigidBodyTransform<TYPE> &transform2)
            {
                T tmpQw = transform1.qw*transform2.qw - transform1.qx*transform2.qx - transform1.qy*transform2.qy - transform1.qz*transform2.qz;
                T tmpQx = transform1.qx*transform2.qw + transform1.qw*transform2.qx - transform1.qz*transform2.qy + transform1.qy*transform2.qz;
                T tmpQy = transform1.qy*transform2.qw + transform1.qz*transform2.qx + transform1.qw*transform2.qy - transform1.qx*transform2.qz;
                T tmpQz = transform1.qz*transform2.qw - transform1.qy*transform2.qx + transform1.qx*transform2.qy + transform1.qw*transform2.qz;

                T tmpX = (transform1.qw*transform1.qw + transform1.qx*transform1.qx - transform1.qy*transform1.qy - transform1.qz*transform1.qz)*transform2.x +
                         2.0*(transform1.qx*transform1.qy - transform1.qw*transform1.qz)*transform2.y + 2.0*(transform1.qw*transform1.qy + transform1.qx*transform1.qz)*transform2.z + transform1.x;

                T tmpY = 2.0*(transform1.qx*transform1.qy + transform1.qw*transform1.qz)*transform2.x + (transform1.qw*transform1.qw - transform1.qx*transform1.qx + transform1.qy*transform1.qy - transform1.qz*transform1.qz)*transform2.y +
                         2.0*(-transform1.qw*transform1.qx + transform1.qy*transform1.qz)*transform2.z + transform1.y;

                T tmpZ = (-2.0*transform1.qw*transform1.qy + 2.0*transform1.qx*transform1.qz)*transform2.x + 2.0*(transform1.qw*transform1.qx + transform1.qy*transform1.qz)*transform2.y +
                         (transform1.qw*transform1.qw - transform1.qx*transform1.qx - transform1.qy*transform1.qy + transform1.qz*transform1.qz)*transform2.z + transform1.z;

                qw = tmpQw;
                qx = tmpQx;
                qy = tmpQy;
                qz = tmpQz;

                x = tmpX;
                y = tmpY;
                z = tmpZ;
            }

			bool isRotationMatrixEpsilonIdentity(const double epsilon) const
            {
                return fabs(qx) < epsilon && fabs(qy) < epsilon && fabs(qz) < epsilon && fabs(1-qw) < epsilon;
            }

            /**
            * Compute the inverse of the RigidBodyTransform passed in as an
            * argument exploiting the orthogonality of the rotation matrix
            * and store the result in this.
            * @param transform
            */
            template<typename TYPE>
			void invert(const RigidBodyTransform<TYPE> &transform)
            {
                set(transform);
                invert();
            }

			void invert()
            {
                T n = norm();
                qx/=-n;
                qy/=-n;
                qz/=-n;
                qw/=n;

                T tempX = -(qw*qw*x + qx*qx*x - (qy*qy + qz*qz)*x + qw*(-2.0*qz*y + 2.0*qy*z) +2.0*qx*(qy*y + qz*z));
                T tempY = -(2.0*qx*qy*x + 2.0*qw*qz*x + qw*qw*y - qx*qx*y + qy*qy*y - qz*qz*y - 2.0*qw*qx*z + 2.0*qy*qz*z);
                z = -(qw*(-2.0*qy*x + 2.0*qx*y) + 2.0*qz*(qx*x + qy*y) + qw*qw*z - (qx*qx + qy*qy - qz*qz)*z);

                x = tempX;
                y = tempY;
            }

			void invertRotationButKeepTranslation()
            {
                T n = norm();
                qx/=-n;
                qy/=-n;
                qz/=-n;
                qw/=n;
            }

            /**
            * Check if the elements of this are within epsilon of the elements of
            * transform.
            *
            * @param transform
            * @param epsilon
            * @return
            */
            template<typename TYPE>
			bool epsilonEquals(const RigidBodyTransform<TYPE> &transform, const double &epsilon) const
            {
                if (!fabs(qx - transform.qx) < epsilon)
                {
                    return false;
                }

                if (!fabs(qy - transform.qy) < epsilon)
                {
                    return false;
                }

                if (!fabs(qz - transform.qz) < epsilon)
                {
                    return false;
                }

                if (!fabs(qw - transform.qw) < epsilon)
                {
                    return false;
                }

                if (!fabs(x - transform.x) < epsilon)
                {
                    return false;
                }

                if (!fabs(y - transform.y) < epsilon)
                {
                    return false;
                }

                if (!fabs(z - transform.z) < epsilon)
                {
                    return false;
                }

                return true;
            }

            /**
            * Orthonormalization of the rotation matrix using Gram-Schmidt method.
            */
			void normalize()
            {
                T mag = norm();

                qx/=mag;
                qy/=mag;
                qz/=mag;
                qw/=mag;
            }

            T normSquared()
            {
                return (qx*qx + qy*qy + qz*qz + qw*qw);
            }

            template<typename TYPE>
			static Eigen::Matrix<TYPE,3,1> getTranslationDifference(const RigidBodyTransform<TYPE> &transform1, const RigidBodyTransform<TYPE> &transform2)
            {
                Eigen::Matrix<TYPE,3,1> pos1;
                Eigen::Matrix<TYPE,3,1> pos2;
                transform1.getTranslation(pos1);
                transform2.getTranslation(pos2);

                return (pos2 - pos1);
            }

            template<typename TYPE>
            RigidBodyTransform<T>& operator*=(const RigidBodyTransform<TYPE> &transform)
            {
                this->multiply(transform);
                return *this;
            }

            template<typename TYPE>
            RigidBodyTransform<T>& operator=(RigidBodyTransform<TYPE> rhs)
            {
                qw = rhs.qw;
                qx = rhs.qx;
                qy = rhs.qy;
                qz = rhs.qz;

                x = rhs.x;
                y = rhs.y;
                z = rhs.z;

                return *this;
            }

            T x,y,z;
            T qx,qy,qz,qw;

        private:

            void setRotationWithRotationMatrix(const T xx, const T xy, const T xz, const T yx, const T yy, const T yz, const T zx, const T zy, const T zz)
            {
                T t = xx + yy + zz;

                if (t >= 0)
                {
                    T s = sqrt(t + 1);
                    qw = 0.5 * s;
                    s = 0.5 / s;
                    qx = (yz - zy) * s;
                    qy = (zx - xz) * s;
                    qz = (xy - yx) * s;
                }
                else if ((xx > yy) && (xx > zz))
                {
                    T s = sqrt(1.0 + xx - yy - zz);
                    qx = s * 0.5;
                    s = 0.5 / s;
                    qy = (xy + yx) * s;
                    qz = (zx + xz) * s;
                    qw = (yz - zy) * s;
                }
                else if (yy > zz)
                {
                    T s = sqrt(1.0 + yy - xx - zz);
                    qy = s * 0.5;
                    s = 0.5 / s;
                    qx = (xy + yx) * s;
                    qz = (yz + zy) * s;
                    qw = (zx - xz) * s;
                }
                else
                {
                    T s = sqrt(1.0 + zz - xx - yy);
                    qz = s * 0.5;
                    s = 0.5 / s;
                    qx = (zx + xz) * s;
                    qy = (yz + zy) * s;
                    qw = (xy - yx) * s;
                }
            }
		};

        template<typename TYPE>
        inline RigidBodyTransform<TYPE> operator*(RigidBodyTransform<TYPE> transform1, const RigidBodyTransform<TYPE> &transform2)
        {
            transform1*=transform2;
            return transform1;
        }

        template<typename TYPE>
        inline bool operator==(const RigidBodyTransform<TYPE> &lhs, const RigidBodyTransform<TYPE> &rhs)
        {
            return lhs.epsilonEquals(rhs,1e-10);
        }

        typedef RigidBodyTransform<double> RigidBodyTransform3d;
        typedef RigidBodyTransform<float> RigidBodyTransform3f;
	}
}

#endif
