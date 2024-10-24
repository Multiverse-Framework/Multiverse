#include "pxr/pxr.h"
#include "pxr/base/tf/registryManager.h"
#include "pxr/base/tf/scriptModuleLoader.h"
#include "pxr/base/tf/token.h"

#include <vector>

PXR_NAMESPACE_OPEN_SCOPE

TF_REGISTRY_FUNCTION(TfScriptModuleLoader) {
    // List of direct dependencies for this library.
    const std::vector<TfToken> reqs = {
        TfToken("usd"),
        TfToken("tf")
    };
    TfScriptModuleLoader::GetInstance().
        RegisterLibrary(TfToken("usdMujoco"), TfToken("pxr.UsdMujoco"), reqs);
}

PXR_NAMESPACE_CLOSE_SCOPE


