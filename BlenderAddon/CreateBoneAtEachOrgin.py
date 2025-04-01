bl_info = {
    "name": "Add Bones at Object Origins",
    "author": "Your Name",
    "version": (1, 0),
    "blender": (4, 3, 2),
    "location": "View3D > Sidebar > Tools",
    "description": "Creates bones at the origin of selected mesh objects",
    "category": "Object",
}

import bpy
from mathutils import Vector

class OBJECT_OT_AddBonesAtOrigins(bpy.types.Operator):
    """Create bones at origins of selected mesh objects"""
    bl_idname = "object.add_bones_at_origins"
    bl_label = "Add Bones at Origins"
    bl_options = {'REGISTER', 'UNDO'}

    bone_length: bpy.props.FloatProperty(
        name="Bone Length",
        default=0.5,
        min=0.01,
        max=100.0
    )

    def execute(self, context):
        # Get all selected mesh objects
        selected_meshes = [obj for obj in context.selected_objects if obj.type == 'MESH']
        
        if not selected_meshes:
            self.report({'WARNING'}, "No mesh objects selected")
            return {'CANCELLED'}

        # Ensure we're in object mode
        if context.object and context.object.mode != 'OBJECT':
            bpy.ops.object.mode_set(mode='OBJECT')

        # Create new armature
        armature = bpy.data.armatures.new("OriginBones")
        armature_obj = bpy.data.objects.new("OriginBones_Armature", armature)
        
        # Link armature to scene and make active
        context.collection.objects.link(armature_obj)
        context.view_layer.objects.active = armature_obj
        armature_obj.select_set(True)

        # Create bones in edit mode
        bpy.ops.object.editmode_toggle()
        for mesh in selected_meshes:
            bone = armature.edit_bones.new(mesh.name + "_Bone")
            bone.head = mesh.location
            bone.tail = mesh.location + Vector((0, 0, self.bone_length))
        bpy.ops.object.editmode_toggle()

        return {'FINISHED'}

class VIEW3D_PT_BonesAtOriginsPanel(bpy.types.Panel):
    """Creates a Panel in the 3D Viewport"""
    bl_label = "Bones at Origins"
    bl_idname = "VIEW3D_PT_bones_at_origins"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Tool'

    def draw(self, context):
        layout = self.layout
        props = layout.operator(OBJECT_OT_AddBonesAtOrigins.bl_idname)
        layout.prop(props, "bone_length")

def register():
    bpy.utils.register_class(OBJECT_OT_AddBonesAtOrigins)
    bpy.utils.register_class(VIEW3D_PT_BonesAtOriginsPanel)

def unregister():
    bpy.utils.unregister_class(OBJECT_OT_AddBonesAtOrigins)
    bpy.utils.unregister_class(VIEW3D_PT_BonesAtOriginsPanel)

if __name__ == "__main__":
    register()