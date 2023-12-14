#include "pxr/base/tf/pySafePython.h"
#include "pxr/pxr.h"
#include "pxr/base/tf/pyModule.h"

PXR_NAMESPACE_USING_DIRECTIVE

TF_WRAP_MODULE
{
    TF_WRAP(UsdMujocoMujoco);
    TF_WRAP(UsdMujocoMujocoOptionAPI);
    TF_WRAP(UsdMujocoMujocoBodyAPI);
    TF_WRAP(UsdMujocoMujocoJointAPI);
    TF_WRAP(UsdMujocoMujocoGeomAPI);
}
