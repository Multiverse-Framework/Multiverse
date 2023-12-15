#include "pxr/base/tf/pySafePython.h"
#include "pxr/pxr.h"
#include "pxr/base/tf/pyModule.h"

PXR_NAMESPACE_USING_DIRECTIVE

TF_WRAP_MODULE
{
    TF_WRAP(UsdUrdfUrdf);

    TF_WRAP(UsdUrdfUrdfLinkAPI);
    TF_WRAP(UsdUrdfUrdfJointAPI);
}
