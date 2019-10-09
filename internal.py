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

import bpy, math
from mathutils import Vector
from collections import namedtuple

Plane = namedtuple('Plane', 'normal distance')

def normalOfPolygon(vertices):
    normal = Vector((0.0, 0.0, 0.0))
    for index, current in enumerate(vertices):
        prev = vertices[index-1]
        normal += (prev-vertices[0]).cross(current-vertices[0])
    return normal

def areaOfPolygon(vertices):
    return normalOfPolygon(vertices).length*0.5

def linePlaneIntersection(origin, dir, plane):
    # return mathutils.geometry.intersect_line_plane(origin, origin+dir, plane.normal*plane.distance, plane.normal)
    det = dir@plane.normal
    return float('nan') if det == 0 else (plane.distance-origin@plane.normal)/det

def planePlaneIntersection(planeA, planeB, tollerance=0.0001):
    # return mathutils.geometry.intersect_plane_plane(planeA.normal*planeA.distance, planeA.normal, planeB.normal*planeB.distance, planeB.normal)
    if 1.0-abs(planeA.normal@planeB.normal) < tollerance:
        return (None, None)
    dir = planeA.normal.cross(planeB.normal).normalized()
    ray_origin = planeA.normal*planeA.distance
    ray_dir = planeA.normal.cross(dir)
    point = ray_origin+ray_dir*linePlaneIntersection(ray_origin, ray_dir, plane)
    return (point, dir)

def linePointDistance(begin, dir, point):
    return (point-begin).cross(dir.normalized()).length

def nearestPointOfLines(originA, dirA, originB, dirB, param_tollerance=0.0, dist_tollerance=0.0001):
    # https://en.wikipedia.org/wiki/Skew_lines#Nearest_Points
    normal = dirA.cross(dirB)
    normalA = dirA.cross(normal)
    normalB = dirB.cross(normal)
    divisorA = dirA@normalB
    divisorB = dirB@normalA
    originAB = originB-originA
    if abs(divisorA) <= param_tollerance or abs(divisorB) <= param_tollerance:
        if linePointDistance(originA, dirA, originB) > dist_tollerance:
            return ('Parallel', float('nan'), float('nan'))
        paramA =  originAB@dirA
        paramB = -originAB@dirB
        return ('Coaxial', paramA, paramB)
    else:
        paramA =  originAB@normalB/divisorA
        paramB = -originAB@normalA/divisorB
        nearestPointA = originA+dirA*paramA
        nearestPointB = originB+dirB*paramB
        return ('Crossing' if (nearestPointA-nearestPointB).length < dist_tollerance else 'Skew', paramA, paramB)

def lineSegmentLineSegmentIntersection(lineAVertexA, lineAVertexB, lineBVertexA, lineBVertexB):
    dirA = lineAVertexB-lineAVertexA
    dirB = lineBVertexB-lineBVertexA
    type, paramA, paramB = nearestPointOfLines(lineAVertexA, dirA, lineBVertexA, dirB)
    if type == 'Parallel' or type == 'Skew':
        return (float('nan'), float('nan'))
    if type == 'Coaxial':
        if paramA < 0.0 and paramB < 0.0: # Facing away from one another
            return (float('nan'), float('nan'))
        if paramA > 0.0 and paramB > 0.0: # Facing towards each other
            if paramA > 1.0 and (lineBVertexB-lineAVertexA)@dirA > 1.0: # End of B is not in A
                return (float('nan'), float('nan'))
        elif paramA > 1.0 or paramB > 1.0: # One is chasing the other but out of reach
            return (float('nan'), float('nan'))
        paramA = max(0.0, (lineBVertexB-lineAVertexA)@dirA)
        paramB = max(0.0, (lineAVertexB-lineBVertexA)@dirB)
        return (paramA, paramB)
    if paramA < 0.0 or paramA > 1.0 or paramB < 0.0 or paramB > 1.0: # Intersection is outside the line segments
        return (float('nan'), float('nan'))
    return (paramA, paramB)

def rayLineSegmentIntersection(originA, dirA, lineVertexA, lineVertexB):
    dirB = lineVertexB-lineVertexA
    type, paramA, paramB = nearestPointOfLines(originA, dirA, lineVertexA, dirB)
    if type == 'Parallel' or type == 'Skew':
        return float('nan')
    if type == 'Coaxial':
        if paramA > 0.0:
            return paramA if (paramB < 0.0) else max(0.0, (lineVertexB-originA)@dirA)
        else:
            return float('nan') if (paramB < 0.0 or paramB > 1.0) else 0.0
    if paramA < 0.0 or paramB < 0.0 or paramB > 1.0: # Intersection is behind the rays origin or outside of the line segment
        return float('nan')
    return paramA

def rayRayIntersection(originA, dirA, originB, dirB):
    type, paramA, paramB = nearestPointOfLines(originA, dirA, originB, dirB)
    if type == 'Parallel' or type == 'Skew':
        return (float('nan'), float('nan'))
    if type == 'Coaxial':
        if paramA < 0.0 and paramB < 0.0: # Facing away from one another
            return (float('nan'), float('nan'))
        if paramA > 0.0 and paramB > 0.0: # Facing towards each other
            return (paramA, paramB)
        return (paramA, 0.0) if paramA > 0.0 else (0.0, paramB) # One is chasing the other
    if paramA < 0.0 or paramB < 0.0: # Intersection is behind the rays origins
        return (float('nan'), float('nan'))
    return (paramA, paramB)

def insort_right(sorted_list, keyfunc, entry, lo=0, hi=None):
    if hi == None:
        hi = len(sorted_list)
    while lo < hi:
        mid = (lo+hi)//2
        if keyfunc(entry) < keyfunc(sorted_list[mid]):
            hi = mid
        else:
            lo = mid+1
    sorted_list.insert(lo, entry)



def selectedSplines(include_bezier, include_polygon, allow_partial_selection=False):
    result = []
    for spline in bpy.context.object.data.splines:
        selected = not allow_partial_selection
        if spline.type == 'BEZIER':
            if not include_bezier:
                continue
            for index, point in enumerate(spline.bezier_points):
                if point.select_left_handle == allow_partial_selection or \
                   point.select_control_point == allow_partial_selection or \
                   point.select_right_handle == allow_partial_selection:
                    selected = allow_partial_selection
                    break
        elif spline.type == 'POLY':
            if not include_polygon:
                continue
            for index, point in enumerate(spline.points):
                if point.select == allow_partial_selection:
                    selected = allow_partial_selection
                    break
        else:
            continue
        if selected:
            result.append(spline)
    return result

def addObject(type, name):
    bpy.ops.object.select_all(action='DESELECT')
    if type == 'CURVE':
        data = bpy.data.curves.new(name=name, type='CURVE')
        data.dimensions = '3D'
    elif type == 'MESH':
        data = bpy.data.meshes.new(name=name)
    obj = bpy.data.objects.new(name, data)
    obj.location = bpy.context.scene.cursor.location
    bpy.context.scene.collection.objects.link(obj)
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    return obj



class Slab:
    __slots__ = ['edge', 'slope', 'plane', 'prev_lightcycles', 'next_lightcycles', 'prev_vertices', 'next_vertices']
    def __init__(self, polygon_normal, prev_polygon_vertex, next_polygon_vertex):
        self.edge = (next_polygon_vertex-prev_polygon_vertex).normalized()
        edge_orthogonal = self.edge.cross(polygon_normal).normalized()
        normal = (polygon_normal+edge_orthogonal).normalized()
        self.slope = (polygon_normal-edge_orthogonal).normalized()
        self.plane = Plane(normal=normal, distance=next_polygon_vertex@normal)
        self.prev_lightcycles = []
        self.next_lightcycles = []
        self.prev_vertices = [prev_polygon_vertex]
        self.next_vertices = [next_polygon_vertex]

    def isOuter(self, in_dir, out_dir, polygon_normal):
        normal = in_dir.cross(polygon_normal)
        return (normal@self.plane.normal > 0.0) == (normal@out_dir > 0.0)

    def calculateVerticesFromLightcycles(self):
        def handleSide(vertices, lightcycles):
            for lightcycle in lightcycles:
                vertices.append(lightcycle.origin+lightcycle.ground_velocity*(lightcycle.collision.looser_time-lightcycle.start_time)+lightcycle.normal*lightcycle.collision.looser_time)
        handleSide(self.prev_vertices, self.prev_lightcycles)
        handleSide(self.next_vertices, self.next_lightcycles)

class Collision:
    __slots__ = ['winner_time', 'looser_time', 'winner', 'loosers', 'child']
    def __init__(self, winner_time, looser_time, winner, loosers):
        self.winner_time = winner_time
        self.looser_time = looser_time
        self.winner = winner
        self.loosers = loosers
        self.child = None

    def checkCandidate(self):
        if self.winner != None and self.winner.collision != None and self.winner.collision.looser_time < self.winner_time:
            return False
        for looser in self.loosers:
            if looser.collision != None:
                return False
        return True

    def collide(self, lightcycles, collision_candidates, polygon_vertices, polygon_normal):
        for looser in self.loosers:
            looser.collision = self
        if len(self.loosers) > 2:
            return 'Manyfold collision' # TODO
        if len(self.loosers) == 2:
            position = self.loosers[0].origin+self.loosers[0].ground_velocity*self.looser_time
            dirA = self.loosers[0].ground_velocity.normalized()
            dirB = self.loosers[1].ground_velocity.normalized()
            if dirA@dirB == 0.0:
                return 'Headon / parallel collision' # TODO
            index = 1 if self.loosers[0].prev_slab.isOuter(dirA, dirA+dirB, polygon_normal) else 0
            if dirA.cross(dirB)@polygon_normal > 0.0:
                index = 1-index
            self.child = Lightcycle(
                lightcycles, collision_candidates, polygon_vertices, polygon_normal, None,
                self.looser_time,
                self.loosers[index].prev_slab, self.loosers[1-index].next_slab,
                position, dirB, dirA
            )
        return True

class Lightcycle:
    __slots__ = ['start_time', 'prev_slab', 'next_slab', 'origin', 'ground_velocity', 'normal', 'collision']
    def __init__(self, lightcycles, collision_candidates, polygon_vertices, polygon_normal, immunity_index, start_time, prev_slab, next_slab, position, prev_dir, next_dir):
        exterior_angle = math.pi-math.acos(prev_slab.edge@-next_slab.edge)
        # pitch_angle = math.atan(math.cos(exterior_angle*0.5))
        ground_speed = 1.0/math.cos(exterior_angle*0.5)
        self.start_time = start_time
        self.prev_slab = prev_slab
        self.next_slab = next_slab
        self.origin = position
        self.ground_velocity = (prev_dir+next_dir).normalized()*ground_speed
        self.normal = prev_dir.cross(next_dir).normalized()
        self.collision = None
        if self.normal@polygon_normal > 0.0:
            self.prev_slab.next_lightcycles.append(self)
            self.next_slab.prev_lightcycles.append(self)
        self.collideWithLightcycles(lightcycles, collision_candidates)
        self.collideWithPolygon(collision_candidates, polygon_vertices, immunity_index)
        lightcycles.append(self)

    def collideWithLightcycles(self, lightcycles, collision_candidates, arrival_tollerance=0.001):
        for i in range(0, len(lightcycles)):
            timeA, timeB = rayRayIntersection(self.origin, self.ground_velocity, lightcycles[i].origin, lightcycles[i].ground_velocity)
            if math.isnan(timeA) or math.isnan(timeB):
                continue
            timeA += self.start_time
            timeB += lightcycles[i].start_time
            no_winner = abs(timeA-timeB) < arrival_tollerance
            insort_right(collision_candidates, lambda collision: collision.looser_time, Collision(
                winner_time=min(timeA, timeB),
                looser_time=max(timeA, timeB),
                winner=(None if no_winner else self if timeA < timeB else lightcycles[i]),
                loosers=([self, lightcycles[i]] if no_winner else [self if timeA > timeB else lightcycles[i]])
            ))

    def collideWithPolygon(self, collision_candidates, polygon_vertices, immunity_index):
        min_time = float('inf')
        for index in range(0, len(polygon_vertices)):
            if immunity_index != None and (index == immunity_index or index == (immunity_index+1)%len(polygon_vertices)):
                continue
            time = rayLineSegmentIntersection(self.origin, self.ground_velocity, polygon_vertices[index-1], polygon_vertices[index])
            if not math.isnan(time):
                min_time = min(time+self.start_time, min_time)
        if min_time < float('inf'):
            insort_right(collision_candidates, lambda collision: collision.looser_time, Collision(
                winner_time=None,
                looser_time=min_time,
                winner=None,
                loosers=[self]
            ))

def straightSkeletonOfPolygon(polygon_vertices, mesh_data, height=1.5):
    polygon_normal = normalOfPolygon(polygon_vertices).normalized()
    # polygon_plane = Plane(normal=polygon_normal, distance=polygon_vertices[0]@polygon_normal)
    slabs = []
    lightcycles = []
    collision_candidates = []

    for index, next_polygon_vertex in enumerate(polygon_vertices):
        prev_polygon_vertex = polygon_vertices[index-1]
        slabs.append(Slab(polygon_normal, prev_polygon_vertex, next_polygon_vertex))

    for index, prev_slab in enumerate(slabs):
        next_slab = slabs[(index+1)%len(polygon_vertices)]
        Lightcycle(lightcycles, collision_candidates, polygon_vertices, polygon_normal, index, 0.0, prev_slab, next_slab, polygon_vertices[index], prev_slab.edge, -next_slab.edge)

    i = 0
    while i < len(collision_candidates):
        collision = collision_candidates[i]
        if collision.checkCandidate():
            result = collision.collide(lightcycles, collision_candidates, polygon_vertices, polygon_normal)
            if result != True:
                return result
        i += 1

    verts = []
    edges = []
    faces = []

    for index, lightcycle in enumerate(lightcycles):
        duration = height-lightcycle.start_time if lightcycle.collision == None else lightcycle.collision.looser_time-lightcycle.start_time
        verts.append(lightcycle.origin+lightcycle.normal*lightcycle.start_time)
        verts.append(lightcycle.origin+lightcycle.normal*(lightcycle.start_time+duration)+lightcycle.ground_velocity*duration)
        edges.append((len(verts)-2, len(verts)-1))

    for index, slab in enumerate(slabs):
        vert_index = len(verts)
        slab.calculateVerticesFromLightcycles()
        slope_vec = slab.slope*height/(slab.slope@polygon_normal)
        def fillSlopeToHeight(position):
            verts.append(position+slope_vec*(1.0-(position@polygon_normal)/(slope_vec@polygon_normal)))
        fillSlopeToHeight(slab.prev_vertices[-1])
        verts += reversed(slab.prev_vertices)
        verts += slab.next_vertices
        fillSlopeToHeight(slab.next_vertices[-1])
        face = []
        for i in range(vert_index, len(verts)):
            # edges.append((i-1 if i > vert_index else len(verts)-1, i))
            face.append(i)
        faces.append(face)

    mesh_data.from_pydata(verts, edges, faces)
    mesh_data.update(calc_edges=True)
    return True
