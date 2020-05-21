#  ***** GPL LICENSE BLOCK *****
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see <http://www.gnu.org/licenses/>.
#  All rights reserved.
#
#  ***** GPL LICENSE BLOCK *****

import os, bpy, importlib, bmesh
from bpy_extras import view3d_utils
from mathutils import Vector
if 'internal' in locals():
    importlib.reload(internal)
from . import internal

bl_info = {
    'name': 'Straight Skeleton',
    'author': 'Alexander Meißner',
    'version': (1, 0, 0),
    'blender': (2, 80, 0),
    'category': 'Add Mesh',
    'wiki_url': 'https://github.com/lichtso/straight_skeleton/',
    'tracker_url': 'https://github.com/lichtso/straight_skeleton/issues'
}

class StraightSkeleton(bpy.types.Operator):
    bl_idname = 'mesh.straight_skeleton'
    bl_description = bl_label = 'Straight Skeleton'
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        return bpy.context.object != None and (bpy.context.object.type == 'MESH' or bpy.context.object.type == 'CURVE')

    def execute(self, context):
        src_obj = bpy.context.object
        polygons = internal.selectedPolygons(src_obj)
        if len(polygons) != 1:
            self.report({'WARNING'}, 'Invalid selection')
            return {'CANCELLED'}
        dst_obj = internal.addObject('MESH', 'Straight Skeleton')
        plane_matrix = internal.straightSkeletonOfPolygon(polygons[0], dst_obj.data)
        if isinstance(plane_matrix, str):
            self.report({'WARNING'}, result)
            return {'CANCELLED'}
        dst_obj.matrix_world = src_obj.matrix_world@plane_matrix
        return {'FINISHED'}

class SliceMesh(bpy.types.Operator):
    bl_idname = 'mesh.slice'
    bl_description = bl_label = 'Slice Mesh'
    bl_options = {'REGISTER', 'UNDO'}

    pitch: bpy.props.FloatProperty(name='Pitch', unit='LENGTH', description='Distance between two slices', default=0.1)
    offset: bpy.props.FloatProperty(name='Offset', unit='LENGTH', description='Position of first slice along the axis', default=0.0)
    slice_count: bpy.props.IntProperty(name='Count', description='Number of slices', min=1, default=3)
    output_type: bpy.props.EnumProperty(name='Output', items=[
        ('MESH', 'Mesh', 'Outputs a mesh object with edge loops'),
        ('CURVE', 'Curve', 'Outputs a curve object with poly splines')
    ], default='MESH')

    @classmethod
    def poll(cls, context):
        return bpy.context.object != None and bpy.context.object.mode == 'OBJECT'

    def perform(self):
        internal.sliceMesh(self.mesh, self.dst_obj, [(i*self.pitch+self.offset) for i in range(0, self.slice_count)], Vector((0.0, 0.0, 1.0)))

    def execute(self, context):
        depsgraph = context.evaluated_depsgraph_get()
        self.src_obj = bpy.context.object
        self.dst_obj = internal.addObject(self.output_type, 'Slices')
        self.dst_obj.matrix_world = bpy.context.scene.cursor.matrix
        self.mesh = bmesh.new()
        self.mesh.from_object(self.src_obj, depsgraph, deform=True, cage=False, face_normals=True)
        self.mesh.transform(self.dst_obj.matrix_world.inverted()@self.src_obj.matrix_world)
        self.perform()
        return {'FINISHED'}

    def invoke(self, context, event):
        if bpy.context.object.type != 'MESH':
            self.report({'WARNING'}, 'Active object must be a mesh')
            return {'CANCELLED'}
        self.pitch = 0.1
        self.offset = 0.0
        self.slice_count = 3
        self.mode = 'PITCH'
        self.execute(context)
        context.window_manager.modal_handler_add(self)
        return {'RUNNING_MODAL'}

    def modal(self, context, event):
        if event.type == 'MOUSEMOVE':
            mouse = (event.mouse_region_x, event.mouse_region_y)
            input_value = internal.nearestPointOfLines(
                bpy.context.scene.cursor.location,
                bpy.context.scene.cursor.matrix.col[2].xyz,
                view3d_utils.region_2d_to_origin_3d(context.region, context.region_data, mouse),
                view3d_utils.region_2d_to_vector_3d(context.region, context.region_data, mouse)
            )[1]
            if self.mode == 'PITCH':
                self.pitch = input_value/(self.slice_count-1) if self.slice_count > 2 else input_value
            elif self.mode == 'OFFSET':
                self.offset = input_value-self.pitch*0.5*((self.slice_count-1) if self.slice_count > 2 else 1.0)
        elif event.type == 'WHEELUPMOUSE':
            if self.slice_count > 2:
                self.pitch *= (self.slice_count-1)
            self.slice_count += 1
            if self.slice_count > 2:
                self.pitch /= (self.slice_count-1)
        elif event.type == 'WHEELDOWNMOUSE':
            if self.slice_count > 2:
                self.pitch *= (self.slice_count-1)
            if self.slice_count > 1:
                self.slice_count -= 1
            if self.slice_count > 2:
                self.pitch /= (self.slice_count-1)
        elif event.type == 'LEFTMOUSE' and event.value == 'RELEASE':
            if self.mode == 'PITCH':
                self.mode = 'OFFSET'
                return {'RUNNING_MODAL'}
            elif self.mode == 'OFFSET':
                self.mesh.free()
                return {'FINISHED'}
        elif event.type in {'RIGHTMOUSE', 'ESC'}:
            self.mesh.free()
            if self.dst_obj.type == 'MESH':
                bpy.data.meshes.remove(self.dst_obj.data)
            else:
                bpy.data.curves.remove(self.dst_obj.data)
            bpy.context.view_layer.objects.active = self.src_obj
            return {'CANCELLED'}
        else:
            return {'PASS_THROUGH'}
        self.perform()
        return {'RUNNING_MODAL'}

classes = [StraightSkeleton, SliceMesh]

def menu_mesh_add(self, context):
    self.layout.separator()
    self.layout.operator(StraightSkeleton.bl_idname)

def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    bpy.types.VIEW3D_MT_mesh_add.append(menu_mesh_add)

def unregister():
    for cls in classes:
        bpy.utils.unregister_class(cls)
    bpy.types.VIEW3D_MT_mesh_add.remove(menu_mesh_add)
