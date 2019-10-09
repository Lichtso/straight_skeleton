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

class StraightSkeletonOperator(bpy.types.Operator):
    bl_idname = 'mesh.straight_skeleton'
    bl_description = bl_label = 'Straight Skeleton'
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        return bpy.context.object != None and bpy.context.object.type == 'CURVE'

    def execute(self, context):
        # TODO: Allow edge loops in meshes to be used as source polygons too
        # TODO: Allow selecting multiple polygons for holes

        if bpy.context.object.mode == 'EDIT':
            splines = internal.selectedSplines(False, True)
        else:
            splines = bpy.context.object.data.splines

        if len(splines) == 0:
            self.report({'WARNING'}, 'Nothing selected')
            return {'CANCELLED'}

        for spline in splines:
            roofModelObj = internal.addObject('MESH', 'Straight Skeleton')
            polygon_vertices = list(point.co.xyz for point in spline.points)
            result = internal.straightSkeletonOfPolygon(polygon_vertices, roofModelObj.data)
            if result != True:
                self.report({'WARNING'}, result)
            return {'CANCELLED'}

        return {'FINISHED'}

classes = [StraightSkeletonOperator]

def menu_mesh_add(self, context):
    self.layout.separator()
    self.layout.operator(StraightSkeletonOperator.bl_idname)

def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    bpy.types.VIEW3D_MT_mesh_add.append(menu_mesh_add)

def unregister():
    for cls in classes:
        bpy.utils.unregister_class(cls)
    bpy.types.VIEW3D_MT_mesh_add.remove(menu_mesh_add)
