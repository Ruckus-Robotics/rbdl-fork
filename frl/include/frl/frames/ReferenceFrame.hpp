#ifndef REFERENCE_FRAME_HPP
#define REFERENCE_FRAME_HPP

/** This class and its implementation are an adaptation
**  of the ReferenceFrame.java by Jerry Pratt and the IHMC robotics group.
**  All credit goes to them.
**/

#include <memory>
#include "frl/geometry/RigidBodyTransform.hpp"
#include <string>
#include <vector>
#include <climits>

namespace frl
{
	namespace frames
	{
		class ReferenceFrame
		{
		public:
			ReferenceFrame(const ReferenceFrame &referenceFrameToCopy)
			{
				parentFrame = referenceFrameToCopy.parentFrame;
				frameName = referenceFrameToCopy.frameName;
				framesStartingWithRootEndingWithThis = referenceFrameToCopy.framesStartingWithRootEndingWithThis;
				transformToParent = referenceFrameToCopy.transformToParent;
				transformToRoot = referenceFrameToCopy.transformToRoot;
				isWorldFrame = referenceFrameToCopy.isWorldFrame;
				isBodyCenteredFrame = referenceFrameToCopy.isBodyCenteredFrame;
			}

			ReferenceFrame(const std::string &frameName, ReferenceFrame *parentFrame, bool isWorldFrame, bool isBodyCenteredFrame)
			{
				this->transformToParent.setIdentity();
				this->transformToRoot.setIdentity();
				this->inverseTransformToRoot.setIdentity();

				this->frameName = frameName;
				this->parentFrame = parentFrame;
				this->isWorldFrame = isWorldFrame;
				this->isBodyCenteredFrame = isBodyCenteredFrame;
				this->framesStartingWithRootEndingWithThis = constructVectorOfFramesStartingWithRootEndingWithThis(this);
			}

			ReferenceFrame(const std::string &frameName, ReferenceFrame* parentFrame, const geometry::RigidBodyTransform<double> &transformToParent, bool isWorldFrame, bool isBodyCenteredFrame)
			{
				this->frameName = frameName;
				this->parentFrame = parentFrame;
				this->transformToParent = transformToParent;
				this->transformToRoot.setIdentity();
				this->inverseTransformToRoot.setIdentity();
				this->isWorldFrame = isWorldFrame;
				this->isBodyCenteredFrame = isBodyCenteredFrame;
				this->framesStartingWithRootEndingWithThis = constructVectorOfFramesStartingWithRootEndingWithThis(this);
			}

			ReferenceFrame(const std::string &frameName, bool isWorldFrame, bool isBodyCenteredFrame)
			{
				this->frameName = frameName;
				this->isWorldFrame = isWorldFrame;
				this->isBodyCenteredFrame = isBodyCenteredFrame;
				this->parentFrame = nullptr;
				this->transformToRoot.setIdentity();
				this->inverseTransformToRoot.setIdentity();
				this->transformToRootID = 0;

				this->transformToParent.setIdentity();
				std::vector<ReferenceFrame *> vector;
				vector.push_back(this);
				this->framesStartingWithRootEndingWithThis = vector;
			}

			ReferenceFrame(const std::string &frameName, ReferenceFrame *parentFrame, const geometry::RigidBodyTransform<double> &transfomToParent, bool isBodyCenteredFrame)
			{
				this->frameName = frameName;
				this->parentFrame = parentFrame;
				this->transformToParent = transformToParent;
				this->isBodyCenteredFrame = isBodyCenteredFrame;
				this->transformToRoot.setIdentity();
				this->inverseTransformToRoot.setIdentity();
				this->isWorldFrame = false;
				this->framesStartingWithRootEndingWithThis = constructVectorOfFramesStartingWithRootEndingWithThis(this);
			}

			ReferenceFrame()
			{ }

			~ReferenceFrame()
			{

			}

			void update()
			{
				// NOTE: IF YOU OVERRIDE UPDATE, YOU MUST RESET this->transformToRootID TO LLONG_MIN IN THE UPDATE METHOD!!

				updateTransformToParent(this->transformToParent);

				this->transformToRootID = LLONG_MIN;
			}

            template<typename T=double>
			void getTransformToDesiredFrame(geometry::RigidBodyTransform<T> &transformToPack, ReferenceFrame *desiredFrame)
			{
				verifyFramesHaveSameRoot(desiredFrame);

				this->computeTransform();
				desiredFrame->computeTransform();

				geometry::RigidBodyTransform<T> tmpTransform = desiredFrame->inverseTransformToRoot;
				geometry::RigidBodyTransform<T> tmpTransform2 = this->transformToRoot;

				tmpTransform *= tmpTransform2;

				transformToPack = tmpTransform;
			}

            template<typename T=double>
			geometry::RigidBodyTransform<T> getTransformToDesiredFrame(ReferenceFrame *desiredFrame)
			{
				geometry::RigidBodyTransform<T> transform;
				getTransformToDesiredFrame(transform, desiredFrame);
				return transform;
			}

			void verifyFramesHaveSameRoot(ReferenceFrame *frame)
			{
				if (!(frame->getRootFrame() == this->getRootFrame()))
				{
					throw std::runtime_error("Frames do not have the same root!");
				}
			}

			void setTransformToParent(const geometry::RigidBodyTransform<double> &transformToParent)
			{
				this->transformToParent = transformToParent;
			}

			void checkReferenceFramesMatch(ReferenceFrame *referenceFrame) const
			{
				if (referenceFrame == nullptr)
				{
					throw std::runtime_error("referenceFrame is NULL!");
				}

				if (referenceFrame != this)
				{
					throw std::runtime_error("Frame mismatch!");
				}
			}

			void checkReferenceFramesMatch(const ReferenceFrame *referenceFrame) const
			{
				if (referenceFrame == nullptr)
				{
					throw std::runtime_error("referenceFrame is NULL!");
				}

				if (referenceFrame != this)
				{
					throw std::runtime_error("Frame mismatch!");
				}
			}

            template<typename T=double>
			geometry::RigidBodyTransform<T> getTransformToRoot()
			{
				computeTransform();

                return this->transformToRoot;
			}

            template<typename T=double>
			geometry::RigidBodyTransform<T> getInverseTransformToRoot()
			{
                return this->inverseTransformToRoot;
			}

			ReferenceFrame *getRootFrame()
			{
				return this->framesStartingWithRootEndingWithThis[0];
			}

			ReferenceFrame *getParentFrame()
			{
				return this->parentFrame;
			}

			std::string getName()
			{
				return this->frameName;
			}

			const std::vector<ReferenceFrame *> getFramesStartingWithRootEndingWithThis()
			{
				return this->framesStartingWithRootEndingWithThis;
			}

			static std::unique_ptr<ReferenceFrame> createAWorldFrame(const std::string &frameName)
			{
				std::unique_ptr<ReferenceFrame> worldFrame(new ReferenceFrame(frameName, true, false));
				return worldFrame;
			}

			static std::unique_ptr<ReferenceFrame> createARootFrame(const std::string &frameName)
			{
				std::unique_ptr<ReferenceFrame> rootFrame(new ReferenceFrame(frameName, false, false));
				return rootFrame;
			}

			static ReferenceFrame* getWorldFrame()
			{
				return worldFrame.get();
			}

			virtual void updateTransformToParent(geometry::RigidBodyTransform<double> &transformToParent)
			{ };

			inline geometry::RigidBodyTransform<double> getTransformToParent()
			{
				return this->transformToParent;
			}

		protected:

		private:
			static std::vector<ReferenceFrame *> constructVectorOfFramesStartingWithRootEndingWithThis(ReferenceFrame *thisFrame)
			{
				ReferenceFrame* parentFrame = thisFrame->getParentFrame();

				if (parentFrame == nullptr)
				{
					// referenceFrame is the root frame.
					std::vector<ReferenceFrame *> vector(1);
					vector[0] = thisFrame;

					return vector;
				}

				// Need to add refereceFrame to the chain.
				int nElements = parentFrame->framesStartingWithRootEndingWithThis.size() + 1;
				std::vector<ReferenceFrame*> vector(nElements);

				for (int i = 0; i < (nElements - 1); i++)
				{
					vector[i] = parentFrame->framesStartingWithRootEndingWithThis[i];
				}

				vector[nElements - 1] = thisFrame;

				return vector;
			}

			void computeTransform()
			{
				int chainLength = this->framesStartingWithRootEndingWithThis.size();

				bool updateFromHereOnOut = false;
				long previousUpdateID = 0;

				for (int i = 0; i < chainLength; i++)
				{
					ReferenceFrame *frame = this->framesStartingWithRootEndingWithThis[i];

					if (!updateFromHereOnOut)
					{
						if (frame->transformToRootID < previousUpdateID)
						{
							updateFromHereOnOut = true;
							nextTransformToRootID++;
						}
					}

					if (updateFromHereOnOut)
					{
						if (frame->getParentFrame() != nullptr)
						{
							geometry::RigidBodyTransform<double> parentsTransformToRoot = frame->getParentFrame()->transformToRoot;

							frame->transformToRoot = parentsTransformToRoot;

							frame->transformToRoot *= frame->transformToParent;

							geometry::RigidBodyTransform<double> transformToRoot = frame->transformToRoot;
							frame->inverseTransformToRoot = transformToRoot;
							frame->inverseTransformToRoot.invert();

							frame->transformToRootID = nextTransformToRootID;
						}
					}

					previousUpdateID = frame->transformToRootID;
				}
			}

			static long nextTransformToRootID;
			long transformToRootID = LLONG_MIN;
			std::vector<ReferenceFrame *> framesStartingWithRootEndingWithThis;
			std::string frameName;
			ReferenceFrame *parentFrame;
			static std::unique_ptr<ReferenceFrame> worldFrame;
			geometry::RigidBodyTransform<double> transformToParent;
			geometry::RigidBodyTransform<double> transformToRoot;
			geometry::RigidBodyTransform<double> inverseTransformToRoot;
			bool isWorldFrame;
			bool isBodyCenteredFrame;
		};

		std::unique_ptr<ReferenceFrame> ReferenceFrame::worldFrame = ReferenceFrame::createAWorldFrame("World");
		long ReferenceFrame::nextTransformToRootID = 1;
	}

}

#endif
