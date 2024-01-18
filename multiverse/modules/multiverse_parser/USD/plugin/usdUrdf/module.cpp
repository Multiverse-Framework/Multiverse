#include "pxr/base/tf/pySafePython.h"
#include "pxr/pxr.h"
#include "pxr/base/tf/pyModule.h"

PXR_NAMESPACE_USING_DIRECTIVE

TF_WRAP_MODULE
{
    TF_WRAP(UsdUrdfUrdf);
    TF_WRAP(UsdUrdfUrdfRobot);
    TF_WRAP(UsdUrdfUrdfMaterial);

    TF_WRAP(UsdUrdfUrdfLinkAPI);
    TF_WRAP(UsdUrdfUrdfLinkInertialAPI);
    TF_WRAP(UsdUrdfUrdfLinkVisualAPI);
    TF_WRAP(UsdUrdfUrdfLinkCollisionAPI);
    TF_WRAP(UsdUrdfUrdfGeometryBoxAPI);
    TF_WRAP(UsdUrdfUrdfGeometryCylinderAPI);
    TF_WRAP(UsdUrdfUrdfGeometrySphereAPI);
    TF_WRAP(UsdUrdfUrdfGeometryMeshAPI);

    TF_WRAP(UsdUrdfUrdfJointAPI);
}
