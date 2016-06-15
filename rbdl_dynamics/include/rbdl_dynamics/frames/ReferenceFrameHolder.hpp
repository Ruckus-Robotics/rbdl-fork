//#ifndef REFERENCE_FRAME_HOLDER_HPP
//#define REFERENCE_FRAME_HOLDER_HPP
//
//#include <frl/frames/ReferenceFrame.hpp>
//
///** This class and its implementation are an adaptation
//**  of the ReferenceFrameHolder.java by Jerry Pratt and the IHMC robotics group.
//**  All credit goes to them.
//**/
//
//namespace frl
//{
//	namespace frames
//	{
//
//		class ReferenceFrameHolder
//		{
//		public:
//			virtual ReferenceFrame *getReferenceFrame() const = 0;
//
//			void checkReferenceFramesMatch(ReferenceFrame *referenceFrame) const
//			{
//				getReferenceFrame()->checkReferenceFramesMatch(referenceFrame);
//			}
//
//			void checkReferenceFramesMatch(const ReferenceFrame *referenceFrame) const
//			{
//				getReferenceFrame()->checkReferenceFramesMatch(referenceFrame);
//			}
//
//			void checkReferenceFramesMatch(const ReferenceFrameHolder *referenceFrameHolder) const
//			{
//				getReferenceFrame()->checkReferenceFramesMatch(referenceFrameHolder->getReferenceFrame());
//			}
//
//			void checkReferenceFramesMatch(ReferenceFrameHolder *referenceFrameHolder) const
//			{
//				getReferenceFrame()->checkReferenceFramesMatch(referenceFrameHolder->getReferenceFrame());
//			}
//		};
//
//	}
//
//}
//
//#endif
