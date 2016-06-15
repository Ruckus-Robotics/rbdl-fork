
#include "rbdl_dynamics/frames/ReferenceFrame.hpp"

namespace frl
{
	namespace frames
	{
		std::unique_ptr<ReferenceFrame> ReferenceFrame::worldFrame = ReferenceFrame::createAWorldFrame("World");
		long ReferenceFrame::nextTransformToRootID = 1;
	}
}