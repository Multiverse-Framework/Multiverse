set(PXR_PACKAGE usdMujoco)

pxr_plugin(${PXR_PACKAGE}
    LIBRARIES
        usd
        tf

    INCLUDE_DIRS
        ${Boost_INCLUDE_DIRS}
        ${PYTHON_INCLUDE_DIRS}

    PUBLIC_HEADERS
        api.h

    PUBLIC_CLASSES
        mujoco
        mujocoOptionAPI
        mujocoAsset
        mujocoMesh
        mujocoBodyAPI
        mujocoJointAPI
        mujocoGeomAPI
        mujocoEquality
        mujocoEqualityJoint
        tokens

    PYTHON_CPPFILES
        moduleDeps.cpp    

    PYMODULE_FILES
        __init__.py

    PYMODULE_CPPFILES
        module.cpp
        wrapMujoco.cpp 
        wrapMujocoOptionAPI.cpp
        wrapMujocoAsset.cpp
        wrapMujocoMesh.cpp
        wrapMujocoBodyAPI.cpp
        wrapMujocoJointAPI.cpp
        wrapMujocoGeomAPI.cpp
        wrapMujocoEquality.cpp
        wrapMujocoEqualityJoint.cpp

    RESOURCE_FILES
        generatedSchema.usda
        plugInfo.json
        schema.usda:usdMujoco/schema.usda
)
