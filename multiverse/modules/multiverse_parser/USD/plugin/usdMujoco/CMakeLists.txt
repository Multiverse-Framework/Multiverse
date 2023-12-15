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
        mujocoBodyAPI
        mujocoJointAPI
        mujocoGeomAPI
        mujocoEquality
        mujocoEqualityJointAPI
        tokens

    PYTHON_CPPFILES
        moduleDeps.cpp    

    PYMODULE_FILES
        __init__.py

    PYMODULE_CPPFILES
        module.cpp
        wrapMujoco.cpp 
        wrapMujocoOptionAPI.cpp   
        wrapMujocoBodyAPI.cpp
        wrapMujocoJointAPI.cpp
        wrapMujocoGeomAPI.cpp
        wrapMujocoEquality.cpp
        wrapMujocoEqualityJointAPI.cpp

    RESOURCE_FILES
        generatedSchema.usda
        plugInfo.json
        schema.usda:usdMujoco/schema.usda
)
