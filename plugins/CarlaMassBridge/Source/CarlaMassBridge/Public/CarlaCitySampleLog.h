// Copyright (c) 2026 City Sample -> CARLA.
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "CoreMinimal.h"
#include "Logging/LogMacros.h"

/// Own category rather than LogCarla: LogCarla is declared in Carla.h but its
/// definition is not exported from the Carla module, so referencing it from
/// another module is LNK2001. A dedicated category is also easier to grep for
/// in a server log that is mostly engine chatter.
DECLARE_LOG_CATEGORY_EXTERN(LogCarlaCitySample, Log, All);
