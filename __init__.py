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

import os, bpy, importlib
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
        polygons = []
        src_obj = bpy.context.object
        # TODO: Allow selecting multiple polygons for holes
        if src_obj.type == 'CURVE':
            if src_obj.mode == 'EDIT':
                splines = internal.selectedSplines(False, True)
            else:
                splines = src_obj.data.splines
            for spline in splines:
                polygons.append(list(point.co.xyz for point in spline.points))
        else:
            loops = []
            for face in src_obj.data.polygons:
                if src_obj.mode == 'EDIT' and not face.select:
                    continue
                polygon = []
                for vertex_index in face.vertices:
                    polygon.append(src_obj.data.vertices[vertex_index].co)
                polygons.append(polygon)
        if len(polygons) != 1:
            self.report({'WARNING'}, 'Invalid selection')
            return {'CANCELLED'}
        dst_obj = internal.addObject('MESH', 'Straight Skeleton')
        result = internal.straightSkeletonOfPolygon(polygons[0], dst_obj.data)
        if result != True:
            self.report({'WARNING'}, result)
            return {'CANCELLED'}
        src_obj.select_set(False)
        dst_obj.matrix_basis = src_obj.matrix_basis
        return {'FINISHED'}

classes = [StraightSkeleton]

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
