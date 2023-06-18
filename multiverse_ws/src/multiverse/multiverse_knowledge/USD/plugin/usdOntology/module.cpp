#include "pxr/base/tf/pySafePython.h"
#include "pxr/pxr.h"
#include "pxr/base/tf/pyModule.h"

PXR_NAMESPACE_USING_DIRECTIVE

TF_WRAP_MODULE
{
    TF_WRAP(UsdOntologyOntology);
    TF_WRAP(UsdOntologyRdfAPI);
    TF_WRAP(UsdOntologySemanticTagAPI);
}
