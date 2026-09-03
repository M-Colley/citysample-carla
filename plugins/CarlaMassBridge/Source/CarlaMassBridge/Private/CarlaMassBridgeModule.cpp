// Copyright (c) 2026 City Sample -> CARLA.
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Modules/ModuleManager.h"
#include "CarlaCitySampleLog.h"

DEFINE_LOG_CATEGORY(LogCarlaCitySample);

// Every UE module needs this. Without it the DLL loads and then the module
// manager reports "could not be initialized successfully after it was loaded"
// and the process aborts during plugin startup - the module has no
// IModuleInterface to hand back.
IMPLEMENT_MODULE(FDefaultModuleImpl, CarlaMassBridge)
