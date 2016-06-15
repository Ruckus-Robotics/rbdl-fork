#ifndef __REFERENCE_FRAME_HPP__
#define __REFERENCE_FRAME_HPP__

/** This class and its implementation are an adaptation
**  of the ReferenceFrame.java by Jerry Pratt and the IHMC robotics group.
**  All credit goes to them.
**/

#include <memory>
#include "rbdl_dynamics/rbdl_math.h"
#include <string>
#include <vector>
#include <climits>

namespace frl
{
	namespace frames
	{
		class RBDL_DLLAPI ReferenceFrame
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

            /**
             * @constructor
             *
             *
             */
			ReferenceFrame(const std::string &frameName, ReferenceFrame *parentFrame, bool isWorldFrame, bool isBodyCenteredFrame)
			{
                // transformToRoot = identity
                // inverseTransformToRoot = identity
                // transformToParent = identity

				this->frameName = frameName;
				this->parentFrame = parentFrame;
				this->isWorldFrame = isWorldFrame;
				this->isBodyCenteredFrame = isBodyCenteredFrame;
				this->framesStartingWithRootEndingWithThis = constructVectorOfFramesStartingWithRootEndingWithThis(this);
			}

			ReferenceFrame(const std::string &frameName, ReferenceFrame* parentFrame, const RigidBodyDynamics::Math::SpatialTransform &transformToParent, bool isWorldFrame, bool isBodyCenteredFrame)
			{
                // transformToRoot = identity
                // inverseTransformToRoot = identity
				this->frameName = frameName;
				this->parentFrame = parentFrame;
				this->transformToParent = transformToParent;
				this->isWorldFrame = isWorldFrame;
				this->isBodyCenteredFrame = isBodyCenteredFrame;
				this->framesStartingWithRootEndingWithThis = constructVectorOfFramesStartingWithRootEndingWithThis(this);
			}

			ReferenceFrame(const std::string &frameName, bool isWorldFrame, bool isBodyCenteredFrame)
			{
                // transformToRoot = identity
                // inverseTransformToRoot = identity
                // transformToParent = identity

				this->frameName = frameName;
				this->isWorldFrame = isWorldFrame;
				this->isBodyCenteredFrame = isBodyCenteredFrame;
				this->parentFrame = nullptr;
				this->transformToRootID = 0;

				std::vector<ReferenceFrame *> vector;
				vector.push_back(this);
				this->framesStartingWithRootEndingWithThis = vector;
			}

			ReferenceFrame(const std::string &frameName, ReferenceFrame *parentFrame, const RigidBodyDynamics::Math::SpatialTransform &transfomToParent, bool isBodyCenteredFrame)
			{
                // transformToRoot = identity
                // inverseTransformToRoot = identity

				this->frameName = frameName;
				this->parentFrame = parentFrame;
				this->transformToParent = transformToParent;
				this->isBodyCenteredFrame = isBodyCenteredFrame;
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

			void getTransformToDesiredFrame(RigidBodyDynamics::Math::SpatialTransform &transformToPack, ReferenceFrame *desiredFrame)
			{
				verifyFramesHaveSameRoot(desiredFrame);

				this->computeTransform();
				desiredFrame->computeTransform();

				RigidBodyDynamics::Math::SpatialTransform tmpTransform = desiredFrame->inverseTransformToRoot;
				RigidBodyDynamics::Math::SpatialTransform tmpTransform2 = this->transformToRoot;

				tmpTransform *= tmpTransform2;

				transformToPack = tmpTransform;
			}

			RigidBodyDynamics::Math::SpatialTransform getTransformToDesiredFrame(ReferenceFrame *desiredFrame)
			{
				RigidBodyDynamics::Math::SpatialTransform transform;
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

			void setTransformToParent(const RigidBodyDynamics::Math::SpatialTransform &transformToParent)
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

			RigidBodyDynamics::Math::SpatialTransform getTransformToRoot()
			{
				computeTransform();

                return this->transformToRoot;
			}

			RigidBodyDynamics::Math::SpatialTransform getInverseTransformToRoot()
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

			virtual void updateTransformToParent(RigidBodyDynamics::Math::SpatialTransform &transformToParent)
			{ };

			inline RigidBodyDynamics::Math::SpatialTransform getTransformToParent()
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
							RigidBodyDynamics::Math::SpatialTransform parentsTransformToRoot = frame->getParentFrame()->transformToRoot;

							frame->transformToRoot = parentsTransformToRoot;

							frame->transformToRoot *= frame->transformToParent;

							RigidBodyDynamics::Math::SpatialTransform transformToRoot = frame->transformToRoot;
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
			RigidBodyDynamics::Math::SpatialTransform transformToParent;
			RigidBodyDynamics::Math::SpatialTransform transformToRoot;
			RigidBodyDynamics::Math::SpatialTransform inverseTransformToRoot;

			bool isWorldFrame;
			bool isBodyCenteredFrame;
		};
	}

}

#endif
