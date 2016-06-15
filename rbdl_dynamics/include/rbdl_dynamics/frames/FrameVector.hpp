//#ifndef __FRAME_VECTOR_HPP__
//#define __FRAME_VECTOR_HPP__
//
///**
// * This class and its implementation is an adaptation of FrameVector.java by Jerry Pratt and the IHMC Robotics Group.
// * All credit goes to them.
// */
//
//#include <eigen3/Eigen/Eigen>
//#include "frl/frames/ReferenceFrame.hpp"
//#include "frl/frames/ReferenceFrameHolder.hpp"
//
//namespace frl
//{
//    namespace frames
//    {
//        template<typename T>
//        class FrameVector : public ReferenceFrameHolder
//        {
//        public:
//            template<typename TYPE>
//            FrameVector(const std::string &name, ReferenceFrame *referenceFrame, const TYPE x, const TYPE y, const TYPE z) : vector(x,y,z)
//            {
//                if (!referenceFrame)
//                {
//                    throw std::runtime_error("Reference frame cannot be nullptr!");
//                }
//
//                this->name = name;
//                this->referenceFrame = referenceFrame;
//            }
//
//            template<typename TYPE>
//            FrameVector(const std::string &name, ReferenceFrame *referenceFrame, const Eigen::Matrix<TYPE,3,1> &vector) : vector(vector)
//            {
//                if (!referenceFrame)
//                {
//                    throw std::runtime_error("Reference frame cannot be nullptr!");
//                }
//
//                this->name = name;
//                this->referenceFrame = referenceFrame;
//            }
//
//            ~FrameVector()
//            { };
//
//            template<typename TYPE>
//            void setIncludingFrame(const TYPE x, const TYPE y, const TYPE z, ReferenceFrame *referenceFrame)
//            {
//                vector(0) = x;
//                vector(1) = y;
//                vector(2) = z;
//
//                this->referenceFrame = referenceFrame;
//            }
//
//            template<typename TYPE>
//            void setIncludingFrame(const Eigen::Matrix<TYPE,3,1> &vector, ReferenceFrame *referenceFrame)
//            {
//                this->vector(0) = vector(0);
//                this->vector(1) = vector(1);
//                this->vector(2) = vector(2);
//
//                this->referenceFrame = referenceFrame;
//            }
//
//            template<typename TYPE>
//            void setAndKeepFrame(const TYPE x, const TYPE y, const TYPE z)
//            {
//                vector(0) = x;
//                vector(1) = y;
//                vector(2) = z;
//            }
//
//            template<typename TYPE>
//            void setAndKeepFrame(const Eigen::Matrix<TYPE,3,1> vector)
//            {
//                this->vector(0) = vector(0);
//                this->vector(1) = vector(1);
//                this->vector(2) = vector(2);
//            }
//
//            template<typename TYPE>
//            T dot(const FrameVector<TYPE> &frameVector) const
//            {
//                checkReferenceFramesMatch(&frameVector);
//                return this->vector.dot(frameVector.getVector());
//            }
//
//            template<typename TYPE>
//            void cross(const FrameVector<TYPE> &frameVector, FrameVector<TYPE> &frameVectorToPack) const
//            {
//                checkReferenceFramesMatch(&frameVector);
//                checkReferenceFramesMatch(&frameVectorToPack);
//                Eigen::Matrix<T,3,1> tmpVector3 = this->vector.cross(frameVector.getVector());
//                frameVectorToPack.setAndKeepFrame(tmpVector3(0), tmpVector3(1), tmpVector3(2));
//            }
//
//            template<typename TYPE>
//            Eigen::Matrix<TYPE,3,1> cross(const FrameVector<TYPE> &frameVector) const
//            {
//                checkReferenceFramesMatch(&frameVector);
//                return this->vector.cross(frameVector.getVector());
//            };
//
//            void changeFrame(ReferenceFrame *desiredFrame)
//            {
//                if (desiredFrame != this->referenceFrame)
//                {
//                    this->referenceFrame->verifyFramesHaveSameRoot(desiredFrame);
//
//                    geometry::RigidBodyTransform<T> thisFramesTransformToRoot, desiredFramesInverseTransformToRoot;
//                    thisFramesTransformToRoot = this->referenceFrame->template getTransformToRoot<T>();
//                    desiredFramesInverseTransformToRoot = desiredFrame->template getInverseTransformToRoot<T>();
//
//                    if (this->referenceFrame && desiredFrame)
//                    {
//                        thisFramesTransformToRoot.transform(vector);
//                        desiredFramesInverseTransformToRoot.transform(vector);
//                        this->referenceFrame = desiredFrame;
//                    }
//                    else
//                    {
//                        throw std::runtime_error("Cannot change the frame of a FrameVector if either this's reference frame or the desired frame is nullptr!");
//                    }
//                }
//            }
//
//            T length() const
//            {
//                return this->vector.norm();
//            }
//
//            std::string getName() const
//            {
//                return name;
//            }
//
//            T getX() const
//            {
//                return vector(0);
//            }
//
//            T getY() const
//            {
//                return vector(1);
//            }
//
//            T getZ() const
//            {
//                return vector(2);
//            }
//
//            template<typename TYPE>
//            T getAngleBetweenVectors(const FrameVector<TYPE> &frameVector) const
//            {
//                checkReferenceFramesMatch(&frameVector);
//
//                T vDot = this->vector.dot(frameVector.getVector()) / (this->vector.norm() * frameVector.getVector().norm());
//
//                if (vDot < -1.0)
//                {
//                    vDot = -1.0;
//                }
//                else if (vDot > 1.0)
//                {
//                    vDot = 1.0;
//                }
//
//                return acos(vDot);
//            }
//
//            ReferenceFrame *getReferenceFrame() const
//            {
//                return this->referenceFrame;
//            }
//
//            Eigen::Matrix<T,3,1> getVector() const
//            {
//                return this->vector;
//            }
//
//            template<typename TYPE>
//            FrameVector<T>& operator=(FrameVector<TYPE> rhs)
//            {
//                vector = rhs.vector;
//                name = rhs.name;
//                referenceFrame = rhs.referenceFrame;
//
//                return *this;
//            }
//
//        private:
//            Eigen::Matrix<T,3,1> vector;
//            ReferenceFrame *referenceFrame;
//            std::string name;
//        };
//    }
//}
//#endif