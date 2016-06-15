//#ifndef FRAME_POINT_HPP
//#define FRAME_POINT_HPP
//
///**
// * This class and its implementation is an adaptation of FramePoint.java by Jerry Pratt and the IHMC Robotics Group.
// * All credit goes to them.
// */
//
//#include "frl/frames/ReferenceFrame.hpp"
//#include "frl/frames/ReferenceFrameHolder.hpp"
//#include "frl/geometry/Point3.hpp"
//
//namespace frl
//{
//    namespace frames
//    {
//        template<typename T>
//        class FramePoint : public ReferenceFrameHolder, public geometry::Point3<T>
//        {
//        public:
//            FramePoint(const std::string &name, ReferenceFrame *referenceFrame, const T x, const T y, const T z) : geometry::Point3<T>(x,y,z)
//            {
//                this->name = name;
//                this->referenceFrame = referenceFrame;
//            }
//
//            FramePoint(const std::string &name, ReferenceFrame *referenceFrame, T array[3]) : geometry::Point3<T>(array)
//            {
//                this->name = name;
//                this->referenceFrame = referenceFrame;
//            }
//
//            FramePoint(const std::string &name, ReferenceFrame *referenceFrame, std::vector<T> vector)
//            {
//                this->name = name;
//                this->referenceFrame = referenceFrame;
//            }
//
//            template<typename TYPE>
//            FramePoint(const std::string &name, ReferenceFrame *referenceFrame, const geometry::Point3<TYPE> &point) : geometry::Point3<T>(point)
//            {
//                this->name = name;
//                this->referenceFrame = referenceFrame;
//            }
//
//            FramePoint(const FramePoint &framePoint) : geometry::Point3<T>(framePoint.x,framePoint.y,framePoint.z)
//            {
//                this->referenceFrame = framePoint.referenceFrame;
//            }
//
//            FramePoint(const std::string &name, ReferenceFrame *referenceFrame) : geometry::Point3<T>()
//            {
//                this->name = name;
//                this->referenceFrame = referenceFrame;
//            }
//
//            ~FramePoint()
//            { };
//
//            template<typename TYPE>
//            void setIncludingFrame(const TYPE &x, const TYPE &y, const TYPE &z, ReferenceFrame *referenceFrame)
//            {
//                if (!referenceFrame)
//                {
//                    throw std::runtime_error("Reference frame cannot be nullptr!");
//                }
//
//                set(x,y,z);
//                this->referenceFrame = referenceFrame;
//            }
//
//            template<typename TYPE>
//            void setIncludingFrame(const geometry::Point3<TYPE> &point, ReferenceFrame *referenceFrame)
//            {
//                if (!referenceFrame)
//                {
//                    throw std::runtime_error("Reference frame cannot be nullptr!");
//                }
//
//                set(point.x,point.y,point.z);
//                this->referenceFrame = referenceFrame;
//            }
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
//                        thisFramesTransformToRoot.transform(*this);
//                        desiredFramesInverseTransformToRoot.transform(*this);
//                        this->referenceFrame = desiredFrame;
//                    }
//                    else
//                    {
//                        throw std::runtime_error("Cannot change the frame of a FramePoint if either this's reference frame or the desired frame is nullptr!");
//                    }
//                }
//            }
//
//            T distanceSquared(const FramePoint &point) const
//            {
//                checkReferenceFramesMatch(&point);
//
//                return frl::utils::computeDistanceBetweenPointsSquared(this->x,this->y,this->z,point.x,point.y,point.z);
//            }
//
//            T distance(const FramePoint &point) const
//            {
//                checkReferenceFramesMatch(&point);
//
//                return frl::utils::computeDistanceBetweenPoints(this->x,this->y,this->z,point.x,point.y,point.z);
//            }
//
//            T distanceL1(const FramePoint &point) const
//            {
//                checkReferenceFramesMatch(&point);
//
//                return frl::utils::distanceL1(this->x,this->y,this->z,point.x,point.y,point.z);
//            }
//
//            T distanceLinf(const FramePoint &point) const
//            {
//                checkReferenceFramesMatch(&point);
//
//                return frl::utils::distanceLinf(this->x,this->y,this->z,point.x,point.y,point.z);
//            }
//
//            ReferenceFrame *getReferenceFrame() const
//            {
//                return this->referenceFrame;
//            }
//
//            ReferenceFrame *referenceFrame;
//            std::string name;
//        };
//
//    }
//}
//#endif