#!/usr/bin/env python3

clean_up_meshes_script = """
if len(bpy.data.objects) == 0:
    raise ValueError("No object in the scene.")
    
for selected_object in bpy.data.objects:    
    # Check if the active object is a mesh
    if selected_object.type != 'MESH':
        continue
    
    bpy.context.view_layer.objects.active = selected_object

    # Switch to Edit mode
    bpy.ops.object.mode_set(mode='EDIT')

    bpy.ops.mesh.quads_convert_to_tris(quad_method="BEAUTY", ngon_method="BEAUTY")
    if selected_object.scale[0] * selected_object.scale[1] * selected_object.scale[2] < 0:
        bpy.ops.mesh.flip_normals()

    # Switch back to Object mode
    bpy.ops.object.mode_set(mode='OBJECT')
"""


def export_usd(out_usd: str) -> str:
    return f"""
import os.path

{clean_up_meshes_script}
bpy.ops.wm.usd_export(filepath='{out_usd}', selected_objects_only=True, overwrite_textures=True)
"""


def export_dae(out_dae: str) -> str:
    return f"""
import os.path
import re
import shutil

{clean_up_meshes_script}
bpy.ops.wm.collada_export(filepath='{out_dae}', 
                          use_texture_copies=True, 
                          export_global_forward_selection="Y", 
                          export_global_up_selection="Z")
out_dae_dir = os.path.dirname('{out_dae}')
os.makedirs(name=os.path.join(out_dae_dir, "..", "..", "textures"), exist_ok=True)

with open('{out_dae}', encoding="utf-8") as file:
    file_contents = file.read()

pattern = r'<init_from>([^<]*\.png)</init_from>'
matches = re.findall(pattern, file_contents)
for match in matches:
    new_value = "../../textures/" + match
    file_contents = file_contents.replace("<init_from>" + match + "</init_from>", f"<init_from>" + new_value + "</init_from>")
    
    texture_abspath = match
    if not os.path.isabs(texture_abspath):
        texture_abspath = os.path.join(out_dae_dir, texture_abspath)
    new_texture_path = new_value
    new_texture_abspath = os.path.join(out_dae_dir, new_texture_path)
    if not os.path.exists(new_texture_abspath):
        shutil.copy(texture_path, new_texture_abspath)
    os.remove(texture_abspath)
    
with open('{out_dae}', "w", encoding="utf-8") as output:
    output.write(file_contents)
"""


def export_obj(out_obj: str) -> str:
    return f"""
import os.path
import shutil
from PIL import Image

{clean_up_meshes_script}
out_obj_dir = os.path.dirname('{out_obj}')
os.makedirs(name=os.path.join(out_obj_dir, "..", "..", "textures"), exist_ok=True)
    
bpy.ops.wm.obj_export(filepath='{out_obj}', 
                      export_selected_objects=False, 
                      forward_axis="Y", 
                      up_axis="Z", 
                      path_mode="RELATIVE")
out_mtl = '{out_obj}'.replace(".obj", ".mtl")
with open(out_mtl, "r") as file:
    lines = file.readlines()
    
for i, line in enumerate(lines):
    if line.startswith("map_Kd"):
        texture_path = os.path.join(out_obj_dir, line.split("map_Kd")[1].strip())
        texture_file_name = os.path.basename(texture_path)
        new_texture_path = os.path.join("..", "..", "textures", texture_file_name)
        new_texture_abspath = os.path.join(out_obj_dir, new_texture_path)
        if not os.path.exists(new_texture_abspath):
            shutil.copy(texture_path, new_texture_abspath)
        lines[i] = "map_Kd " + new_texture_path + "\\n"
        img = Image.open(texture_path)
        png_file_name = texture_file_name.replace(".jpg", ".png").replace(".JPG", ".png")
        img.save(os.path.join(out_obj_dir, new_texture_path).replace(".jpg", ".png").replace(".JPG", ".png"), "PNG")

with open(out_mtl, "w") as file:
    file.writelines(lines)
"""


def export_stl(out_stl: str) -> str:
    return f"""
import os.path

{clean_up_meshes_script}
os.makedirs(name=os.path.dirname('{out_stl}'), exist_ok=True)
selected_object = bpy.context.object
if len([vertex for obj in bpy.data.objects for vertex in obj.data.vertices]) > 1000:
    selected_object.modifiers.new("Weld", "WELD")
    bpy.ops.object.modifier_apply(modifier="Weld")
bpy.ops.export_mesh.stl(filepath='{out_stl}', 
                        use_selection=False, 
                        axis_forward="Y", 
                        axis_up="Z")
"""


def export_fbx(out_fbx: str) -> str:
    return f"""
{clean_up_meshes_script}
bpy.ops.export_scene.fbx(filepath='{out_fbx}', 
                         use_selection=False, 
                         axis_forward="Y", 
                         axis_up="Z",
                         mesh_smooth_type="FACE",
                         use_triangles=True,
                         object_types={{"MESH"}})
"""
