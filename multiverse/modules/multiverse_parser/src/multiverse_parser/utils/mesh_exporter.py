#!/usr/bin/env python3

clean_up_meshes_script = """
def clean_up_meshes(bpy, file_path: str) -> None:
    for selected_object in bpy.context.selected_objects:
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
    
        bpy.ops.object.transform_apply(location=True, rotation=True, scale=True, isolate_users=True)
"""


def export_usd(out_usd: str) -> str:
    return f"""
import os.path

{clean_up_meshes_script}
clean_up_meshes(bpy, '{out_usd}')
bpy.ops.wm.usd_export(filepath='{out_usd}', selected_objects_only=True, overwrite_textures=True)
"""


def export_dae(out_dae: str) -> str:
    return f"""
import os.path

{clean_up_meshes_script}
clean_up_meshes(bpy, '{out_dae}')
out_dae_dir = os.path.dirname('{out_dae}')
os.makedirs(name=os.path.join(out_dae_dir, "..", "..", "textures"), exist_ok=True)
bpy.ops.wm.collada_export(filepath='{out_dae}', use_texture_copies=True)
"""


def export_obj(out_obj: str) -> str:
    return f"""
import os.path
import shutil
from PIL import Image

{clean_up_meshes_script}
clean_up_meshes(bpy, '{out_obj}')
out_obj_dir = os.path.dirname('{out_obj}')
os.makedirs(name=os.path.join(out_obj_dir, "..", "..", "textures"), exist_ok=True)
bpy.ops.wm.obj_export(filepath='{out_obj}', export_selected_objects=True, forward_axis="Y", up_axis="Z",
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
clean_up_meshes(bpy, '{out_stl}')
os.makedirs(name=os.path.dirname('{out_stl}'), exist_ok=True)
selected_object = bpy.context.object
selected_object.modifiers.new("Weld", "WELD")
bpy.ops.object.modifier_apply(modifier="Weld")
bpy.ops.export_mesh.stl(filepath='{out_stl}', use_selection=True, axis_forward="Y", axis_up="Z")
"""
