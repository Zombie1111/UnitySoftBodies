bl_info = {
    "name": "Distance-Based Bone Weights",
    "author": "Your Name",
    "version": (1, 0),
    "blender": (4, 3, 2),
    "location": "View3D > Sidebar > Tools",
    "description": "Sets vertex weights based on distance to bones within radius",
    "category": "Object",
}

import bpy
from mathutils import Vector

class OBJECT_OT_SetBoneWeightsByDistance(bpy.types.Operator):
    """Set vertex weights based on distance to bones within radius"""
    bl_idname = "object.set_bone_weights_by_distance"
    bl_label = "Set Bone Weights by Distance"
    bl_options = {'REGISTER', 'UNDO'}

    radius: bpy.props.FloatProperty(
        name="Radius",
        default=3.0,
        min=0.01,
        max=1000.0,
        description="Influence radius for bones"
    )

    @classmethod
    def poll(cls, context):
        return context.selected_objects is not None

    def execute(self, context):
        for obj in context.selected_objects:
            if obj.type != 'MESH':
                continue

            # Find armature modifier
            armature_mod = next((m for m in obj.modifiers if m.type == 'ARMATURE' and m.object), None)
            if not armature_mod:
                self.report({'WARNING'}, f"No armature modifier found on {obj.name}")
                continue

            armature_obj = armature_mod.object
            if armature_obj.type != 'ARMATURE':
                self.report({'WARNING'}, f"{armature_obj.name} is not an armature")
                continue

            bones = armature_obj.data.bones
            bone_names = [b.name for b in bones]

            # Manage vertex groups
            existing_groups = [vg.name for vg in obj.vertex_groups]
            for bone_name in bone_names:
                if bone_name in existing_groups:
                    obj.vertex_groups.remove(obj.vertex_groups[bone_name])
            for bone in bones:
                obj.vertex_groups.new(name=bone.name)

            # Precompute bone positions
            bone_positions = {
                b.name: armature_obj.matrix_world @ b.head_local
                for b in bones
            }

            mesh_matrix = obj.matrix_world
            mesh = obj.data

            for v in mesh.vertices:
                v_world = mesh_matrix @ v.co
                weights = []
                total = 0.0

                for bone in bones:
                    bpos = bone_positions[bone.name]
                    distance = (v_world - bpos).length
                    
                    if distance <= self.radius:
                        weight = 1.0 - (distance / self.radius)
                        weights.append((bone.name, weight))
                        total += weight

                if total > 0:
                    for bone_name, weight in weights:
                        vg = obj.vertex_groups[bone_name]
                        vg.add([v.index], weight / total, 'REPLACE')

        return {'FINISHED'}

class VIEW3D_PT_BoneWeightsPanel(bpy.types.Panel):
    """UI Panel for Bone Weights Tool"""
    bl_label = "Bone Weights by Distance"
    bl_idname = "VIEW3D_PT_bone_weights_panel"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Tool'

    def draw(self, context):
        layout = self.layout
        props = layout.operator(OBJECT_OT_SetBoneWeightsByDistance.bl_idname)
        layout.prop(props, "radius")

def register():
    bpy.utils.register_class(OBJECT_OT_SetBoneWeightsByDistance)
    bpy.utils.register_class(VIEW3D_PT_BoneWeightsPanel)

def unregister():
    bpy.utils.unregister_class(OBJECT_OT_SetBoneWeightsByDistance)
    bpy.utils.unregister_class(VIEW3D_PT_BoneWeightsPanel)

if __name__ == "__main__":
    register()