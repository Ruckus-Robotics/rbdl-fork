/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2016 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#include "rbdl_dynamics/Logging.h"

RBDL_DLLAPI std::ostringstream LogOutput;

RBDL_DLLAPI void ClearLogOutput() {
  LogOutput.str("");
}
