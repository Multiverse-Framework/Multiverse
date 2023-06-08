#!/usr/bin/env python3

import sys
import os
from pxr import Usd, UsdOntology, UsdGeom, UsdPhysics, Gf, Vt
from owlready2 import onto_path, get_ontology, declare_datatype
from numpy import float32, float64
import rospkg

onto_map = dict()


def float_parser(string: str):
    return float32(string)


def float_unparser(x: float32):
    return str(x)


declare_datatype(float32, 'http://www.w3.org/2001/XMLSchema#float',
                 float_parser, float_unparser)


def double_parser(string: str):
    return float64(string)


def double_unparser(x: float64):
    return str(x)


declare_datatype(float64, 'http://www.w3.org/2001/XMLSchema#double',
                 double_parser, double_unparser)


def point3f_parser(string: str):
    return Gf.Vec3f(eval(string))


def point3f_unparser(vec3f: Gf.Vec3f):
    return '[' + ', '.join(str(x) for x in vec3f) + ']'


declare_datatype(Gf.Vec3f, 'https://ease-crc.org/ont/USD.owl#point3f',
                 point3f_parser, point3f_unparser)


def token_array_parser(string: str):
    return Vt.TokenArray(eval(string))


def token_array_unparser(token_array: Vt.TokenArray):
    return '[' + ', '.join([str(x) for x in token_array]) + ']'


declare_datatype(Vt.TokenArray, 'https://ease-crc.org/ont/USD.owl#token_array',
                 token_array_parser, token_array_unparser)


def matrix4d_parser(string: str):
    return Gf.Matrix4d(eval(string))


def matrix4d_unparser(matrix4d: Gf.Matrix4d):
    return '[' + ', '.join(str(row) for row in [list(row) for row in matrix4d]) + ']'


declare_datatype(Gf.Matrix4d, 'https://ease-crc.org/ont/USD.owl#matrix4d',
                 matrix4d_parser, matrix4d_unparser)


def color3f_array_parser(string: str):
    return Vt.Vec3fArray(eval(string))


def color3f_array_unparser(vec3f_array: Vt.Vec3fArray):
    return '[' + ', '.join(str(row) for row in [list(row) for row in vec3f_array]) + ']'


declare_datatype(Vt.Vec3fArray, 'https://ease-crc.org/ont/USD.owl#color3f_array',
                 color3f_array_parser, color3f_array_unparser)


def float_array_parser(string: str):
    return Vt.FloatArray(eval(string))


def float_array_unparser(float_array: Vt.FloatArray):
    return '[' + ', '.join([str(x) for x in float_array]) + ']'


declare_datatype(Vt.FloatArray, 'https://ease-crc.org/ont/USD.owl#float_array',
                 float_array_parser, float_array_unparser)


def quatf_parser(string: str):
    return Gf.Quatf(eval(string))


def quatf_unparser(quatf: Gf.Quatf):
    return '[' + str(quatf.GetReal()) + ', '.join([str(x) for x in quatf.GetImaginary()]) + ']'


declare_datatype(Gf.Quatf, 'https://ease-crc.org/ont/USD.owl#quatf',
                 quatf_parser, quatf_unparser)


def import_ontos(onto) -> None:
    for imported_onto in onto.imported_ontologies:
        imported_onto.load()
        onto_map[imported_onto.base_iri] = imported_onto
        import_ontos(imported_onto)

    return None


def usd_to_owl(usd_file : str, onto_file : str) -> None:
    rospack = rospkg.RosPack()

    onto_path.append(rospack.get_path('mujoco_sim') + '/model/owl/')

    save_path = os.path.splitext(os.path.basename(usd_file))[0]
    save_path = os.path.join(os.path.dirname(usd_file), save_path)
    save_path += '.owl'
    ABox_onto = get_ontology('file://' + save_path)

    dul_onto = get_ontology('http://www.ontologydesignpatterns.org/ont/dul/DUL.owl')
    dul_onto.load()
    onto_map[dul_onto.base_iri] = dul_onto

    usd_onto = get_ontology('https://ease-crc.org/ont/USD.owl')
    onto_map[usd_onto.base_iri] = usd_onto

    TBox_onto = get_ontology('file://' + onto_file)
    TBox_onto.load()
    onto_map[TBox_onto.base_iri] = TBox_onto

    import_ontos(TBox_onto)

    ABox_onto.imported_ontologies.append(TBox_onto)

    prim_dict = dict()

    stage = Usd.Stage.Open(usd_file)
    with ABox_onto:
        for prim in stage.Traverse():
            prim_inst = usd_onto.Prim(prim.GetName(), namespace=usd_onto)
            prim_dict[prim] = prim_inst

            if prim.HasAPI(UsdOntology.SemanticTagAPI):
                semanticTagAPI = UsdOntology.SemanticTagAPI.Apply(prim)
                for prim_path in semanticTagAPI.GetSemanticLabelRel().GetTargets():
                    label_prim = stage.GetPrimAtPath(prim_path)
                    if label_prim.IsValid():
                        rdfAPI = UsdOntology.RdfAPI.Apply(label_prim)
                        onto_ns = rdfAPI.GetRdfNamespaceAttr().Get()
                        onto = onto_map.get(onto_ns)
                        if onto is None:
                            print(f'{onto_ns} not found in onto_map')
                            continue
                        prim_inst.is_a.append(
                            onto[rdfAPI.GetRdfConceptNameAttr().Get()])

            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                hasAPI_prop = usd_onto.hasAPI
                prim_inst.is_a.append(
                    hasAPI_prop.some(usd_onto.RigidBodyAPI))

                rigidBodyEnabled_inst = dul_onto.Quality(
                    prim.GetName() + '_rigidBodyEnabled', namespace=dul_onto)
                prim_inst.hasQuality.append(rigidBodyEnabled_inst)

                rigidBodyAPI = UsdPhysics.RigidBodyAPI.Apply(prim)
                rigidBodyEnabled_inst.physics_rigidBodyEnabled = [
                    rigidBodyAPI.GetRigidBodyEnabledAttr().Get()]

            if prim.HasAPI(UsdPhysics.CollisionAPI):
                hasAPI_prop = usd_onto.hasAPI
                prim_inst.is_a.append(
                    hasAPI_prop.some(usd_onto.CollisionAPI))

                rigidCollisionEnabled_inst = dul_onto.Quality(
                    prim.GetName() + '_rigidCollisionEnabled', namespace=dul_onto)
                prim_inst.hasQuality.append(rigidCollisionEnabled_inst)

                collisionAPI = UsdPhysics.CollisionAPI.Apply(prim)
                rigidCollisionEnabled_inst.physics_collisionEnabled = [
                    collisionAPI.GetCollisionEnabledAttr().Get()]

            if prim.HasAPI(UsdPhysics.MassAPI):
                hasAPI_prop = usd_onto.hasAPI
                prim_inst.is_a.append(
                    hasAPI_prop.some(usd_onto.MassAPI))

                mass_inst = dul_onto.Quality(
                    prim.GetName() + '_mass', namespace=dul_onto)
                prim_inst.hasQuality.append(mass_inst)
                com_inst = dul_onto.Quality(
                    prim.GetName() + '_centerOfMass', namespace=dul_onto)
                prim_inst.hasQuality.append(com_inst)

                massAPI = UsdPhysics.MassAPI.Apply(prim)
                mass_inst.physics_mass = [
                    float32(massAPI.GetMassAttr().Get())]
                com_inst.physics_centerOfMass = [
                    massAPI.GetCenterOfMassAttr().Get()]

            if prim.IsA(UsdGeom.Xformable):
                xformable = UsdGeom.Xformable(prim)
                xformOpOrderAttr = xformable.GetXformOpOrderAttr().Get()
                if xformOpOrderAttr is None:
                    continue

                xformOpOrder_inst = dul_onto.Quality(
                    prim.GetName() + '_xformOpOrder', namespace=dul_onto)
                prim_inst.hasQuality.append(xformOpOrder_inst)

                xformOpOrder_inst.xformOpOrder = [xformOpOrderAttr]

                for xformOp in xformOpOrderAttr:
                    if not prim.HasAttribute(xformOp):
                        continue
                    if xformOp == 'xformOp:translate':
                        xformOpTranslate_inst = dul_onto.Quality(
                            prim.GetName() + '_xformOp_translate', namespace=dul_onto)
                        prim_inst.hasQuality.append(
                            xformOpTranslate_inst)

                        xformOpTransform_inst.xformOp_translate = [
                            prim.GetAttribute(xformOp).Get()]

                    if xformOp == 'xformOp:rotate':
                        xformOpRotate_inst = dul_onto.Quality(
                            prim.GetName() + '_xformOp_rotate', namespace=dul_onto)
                        prim_inst.hasQuality.append(
                            xformOpRotate_inst)

                        xformOpRotate_inst.xformOp_rotate = [
                            prim.GetAttribute(xformOp).Get()]

                    if xformOp == 'xformOp:transform':
                        xformOpTransform_inst = dul_onto.Quality(
                            prim.GetName() + '_xformOp_transform', namespace=dul_onto)
                        prim_inst.hasQuality.append(
                            xformOpTransform_inst)

                        xformOpTransform_inst.xformOp_transform = [
                            prim.GetAttribute(xformOp).Get()]

                    if xformOp == 'xformOp:transform':
                        xformOpTransform_inst = dul_onto.Quality(
                            prim.GetName() + '_xformOp_transform', namespace=dul_onto)
                        prim_inst.hasQuality.append(
                            xformOpTransform_inst)

                        xformOpTransform_inst.xformOp_transform = [
                            prim.GetAttribute(xformOp).Get()]

            if prim.IsA(UsdGeom.Gprim):
                displayColor_inst = dul_onto.Quality(
                    prim.GetName() + '_dislayColor', namespace=dul_onto)
                prim_inst.hasQuality.append(displayColor_inst)
                displayOpacity_inst = dul_onto.Quality(
                    prim.GetName() + '_dislayOpacity', namespace=dul_onto)
                prim_inst.hasQuality.append(displayOpacity_inst)

                gprim = UsdGeom.Gprim(prim)
                displayColor_inst.primvars_displayColor = [
                    gprim.GetDisplayColorPrimvar().Get()]
                displayOpacity_inst.primvars_displayOpacity = [
                    gprim.GetDisplayOpacityPrimvar().Get()]

            if prim.IsA(UsdGeom.Xform):
                hasXformSchema_prop = usd_onto.hasTypedSchema
                prim_inst.is_a.append(
                    hasXformSchema_prop.some(usd_onto.XformSchema))

            elif prim.IsA(UsdGeom.Cube):
                hasCubeSchema_prop = usd_onto.hasTypedSchema
                prim_inst.is_a.append(
                    hasCubeSchema_prop.some(usd_onto.CubeSchema))

                size_inst = dul_onto.Quality(
                    prim.GetName() + '_size', namespace=dul_onto)
                prim_inst.hasQuality.append(size_inst)

                cube = UsdGeom.Cube(prim)
                size_inst.size = [float64(cube.GetSizeAttr().Get())]

            elif prim.IsA(UsdGeom.Sphere):
                hasSphereSchema_prop = usd_onto.hasTypedSchema
                prim_inst.is_a.append(
                    hasSphereSchema_prop.some(usd_onto.SphereSchema))

                radius_inst = dul_onto.Quality(
                    prim.GetName() + '_radius', namespace=dul_onto)
                prim_inst.hasQuality.append(radius_inst)

                sphere = UsdGeom.Sphere(prim)
                radius_inst.radius = [
                    float64(sphere.GetRadiusAttr().Get())]

            elif prim.IsA(UsdGeom.Cylinder):
                hasCylinderSchema_prop = usd_onto.hasTypedSchema
                prim_inst.is_a.append(
                    hasCylinderSchema_prop.some(usd_onto.CylinderSchema))

                radius_inst = dul_onto.Quality(
                    prim.GetName() + '_radius', namespace=dul_onto)
                prim_inst.hasQuality.append(radius_inst)
                height_inst = dul_onto.Quality(
                    prim.GetName() + '_height', namespace=dul_onto)
                prim_inst.hasQuality.append(height_inst)

                cylinder = UsdGeom.Cylinder(prim)
                radius_inst.radius = [
                    float64(cylinder.GetRadiusAttr().Get())]
                height_inst.height = [
                    float64(cylinder.GetHeightAttr().Get())]

        for prim in stage.Traverse():
            prim_inst = prim_dict.get(prim)
            if prim_inst is None:
                continue
            for prim_child in prim.GetChildren():
                prim_child_inst = prim_dict.get(prim_child)
                if prim_child_inst is not None:
                    prim_inst.hasPart.append(prim_child_inst)

            if prim.IsA(UsdPhysics.RevoluteJoint):
                prim_inst = prim_dict.get(prim)
                hasRevoluteJointSchema_prop = usd_onto.hasTypedSchema
                prim_inst.is_a.append(hasRevoluteJointSchema_prop.some(
                    usd_onto.PhysicsRevoluteJointSchema))

                collisionEnabled_inst = dul_onto.Quality(
                    prim.GetName() + '_collisionEnabled', namespace=dul_onto)
                prim_inst.hasQuality.append(collisionEnabled_inst)
                pos0_inst = dul_onto.Quality(
                    prim.GetName() + '_pos0', namespace=dul_onto)
                prim_inst.hasQuality.append(pos0_inst)
                pos1_inst = dul_onto.Quality(
                    prim.GetName() + '_pos1', namespace=dul_onto)
                prim_inst.hasQuality.append(pos1_inst)
                rot0_inst = dul_onto.Quality(
                    prim.GetName() + '_rot0', namespace=dul_onto)
                prim_inst.hasQuality.append(rot0_inst)
                rot1_inst = dul_onto.Quality(
                    prim.GetName() + '_rot1', namespace=dul_onto)
                prim_inst.hasQuality.append(rot1_inst)

                revoluteJoint = UsdPhysics.RevoluteJoint(prim)
                body0 = stage.GetPrimAtPath(
                    revoluteJoint.GetBody0Rel().GetTargets()[0])
                prim_inst.physics_body0 = prim_dict.get(body0)
                body1 = stage.GetPrimAtPath(
                    revoluteJoint.GetBody1Rel().GetTargets()[0])
                prim_inst.physics_body1 = prim_dict.get(body1)
                collisionEnabled_inst.physics_collisionEnabled = [
                    revoluteJoint.GetCollisionEnabledAttr().Get()]
                pos0_inst.physics_localPos0 = [
                    revoluteJoint.GetLocalPos0Attr().Get()]
                pos1_inst.physics_localPos1 = [
                    revoluteJoint.GetLocalPos1Attr().Get()]
                rot0_inst.physics_localRot0 = [
                    revoluteJoint.GetLocalRot0Attr().Get()]
                rot1_inst.physics_localRot1 = [
                    revoluteJoint.GetLocalRot1Attr().Get()]

                jointValue_inst = dul_onto.Quality(
                    prim.GetName() + '_jointValue', namespace=usd_onto)
                prim_inst.hasQuality.append(jointValue_inst)
                jointValue_inst.hasJointValue = [float64(0.5)]

    print(f'Save usd stage that represents {onto_file} to {save_path}')
    ABox_onto.save(save_path)

    return None


if __name__ == '__main__':
    if len(sys.argv) >= 3:
        (usd_file, onto_file) = (sys.argv[1], sys.argv[2])
    else:
        print('Usage: in_usd.usda in_onto.owl')
        sys.exit(1)
    usd_to_owl(usd_file, onto_file)
