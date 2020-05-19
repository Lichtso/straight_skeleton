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
    origin = ray_origin+ray_dir*linePlaneIntersection(ray_origin, ray_dir, planeB)
    return (origin, dir)

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
        if dirA@dirA == 0.0 or dirB@dirB == 0.0 or linePointDistance(originA, dirA, originB) > dist_tollerance:
            return ('Parallel', float('nan'), float('nan'))
        paramA =  originAB@dirA/(dirA@dirA)
        paramB = -originAB@dirB/(dirB@dirB)
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
        paramA = max(0.0, (lineBVertexB-lineAVertexA)@dirA/(dirA@dirA))
        paramB = max(0.0, (lineAVertexB-lineBVertexA)@dirB/(dirB@dirB))
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
            return paramA if (paramB < 0.0) else max(0.0, (lineVertexB-originA)@dirA/(dirA@dirA))
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
            paramSum = paramA+paramB
            paramA = paramA*paramA/paramSum
            paramB = paramB*paramB/paramSum
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



class SlabIntersection:
    __slots__ = ['prev_slab', 'next_slab', 'origin', 'dir', 'begin_param', 'end_param']
    def __init__(self, prev_slab, next_slab, origin, dir, begin_param, end_param):
        self.prev_slab = prev_slab
        self.next_slab = next_slab
        self.origin = origin
        self.dir = dir
        self.begin_param = begin_param
        self.end_param = end_param

    def reverse(self):
        self.dir *= -1.0
        [self.begin_param, self.end_param] = [-self.end_param, -self.begin_param]

    def otherSlab(self, slab):
        return self.prev_slab if slab == self.next_slab else self.next_slab

class Slab:
    __slots__ = ['edge', 'slope', 'plane', 'prev_slab', 'next_slab', 'prev_polygon_vertex', 'next_polygon_vertex', 'prev_lightcycles', 'next_lightcycles', 'vertices', 'slab_intersections']
    def __init__(self, polygon_normal, prev_polygon_vertex, next_polygon_vertex):
        self.edge = (next_polygon_vertex-prev_polygon_vertex).normalized()
        edge_orthogonal = self.edge.cross(polygon_normal).normalized()
        normal = (polygon_normal+edge_orthogonal).normalized()
        self.slope = (polygon_normal-edge_orthogonal).normalized()
        self.plane = Plane(normal=normal, distance=next_polygon_vertex@normal)
        self.prev_lightcycles = []
        self.next_lightcycles = []
        self.prev_polygon_vertex = prev_polygon_vertex
        self.next_polygon_vertex = next_polygon_vertex
        self.vertices = [self.prev_polygon_vertex, self.next_polygon_vertex]
        self.slab_intersections = []

    def isOuterOfCollision(self, in_dir, out_dir, polygon_normal):
        normal = in_dir.cross(polygon_normal)
        return (normal@self.plane.normal > 0.0) == (normal@out_dir > 0.0)

    def calculateVerticesFromLightcycles(self):
        def handleSide(lightcycles, prepend):
            for lightcycle in lightcycles:
                vertex = lightcycle.slab_intersection.origin+lightcycle.slab_intersection.dir*lightcycle.slab_intersection.end_param
                if prepend:
                    self.vertices.insert(0, vertex)
                else:
                    self.vertices.append(vertex)
        handleSide(self.prev_lightcycles, True)
        handleSide(self.next_lightcycles, False)

    def lineIntersection(self, origin, dir, tollerance=0.0001):
        intersections = []
        for i in range(0, len(self.vertices)+1):
            is_last = (i == 0 or i == len(self.vertices))
            type, paramA, paramB = nearestPointOfLines(origin, dir, self.vertices[0 if i == 0 else i-1], self.slope if is_last else self.vertices[i]-self.vertices[i-1])
            if type == 'Crossing':
                if paramB > -tollerance and (is_last or paramB < 1.0+tollerance):
                    intersections.append((i, paramA))
            elif type == 'Coaxial':
                assert(not is_last)
                intersections.append((i-1, paramA))
                intersections.append((i, (self.vertices[i]-origin)@dir/(dir@dir)))
        intersections.sort(key=lambda entry: entry[1])
        i = 1
        while i < len(intersections):
            if intersections[i][1]-intersections[i-1][1] < tollerance:
                del intersections[i]
            else:
                i += 1
        return intersections

    def intersect(self, other_slab):
        origin, dir = planePlaneIntersection(self.plane, other_slab.plane)
        if origin == None:
            return None
        intersectionsA = self.lineIntersection(origin, dir)
        intersectionsB = other_slab.lineIntersection(origin, dir)
        if len(intersectionsA) == 2 and len(intersectionsB) == 2:
            begin_param = max(intersectionsA[0][1], intersectionsB[0][1])
            end_param = min(intersectionsA[1][1], intersectionsB[1][1])
            if begin_param < end_param:
                slab_intersection = SlabIntersection(self, other_slab, origin+begin_param*dir, dir, 0.0, end_param-begin_param)
                self.slab_intersections.append(slab_intersection)
                other_slab.slab_intersections.append(slab_intersection)

    def calculateVerticesFromIntersections(self, tollerance=0.001):
        # def findLineOfOtherSlab(slab):
        #     for candidate in self.slab_intersections:
        #         if candidate.otherSlab(self) == slab:
        #             return candidate
        #     return None
        pivot = self.prev_polygon_vertex
        current_line = None
        for candidate in self.slab_intersections:
            if candidate.prev_slab == self.prev_slab or candidate.next_slab == self.prev_slab:
                current_line = candidate
                break
        if abs((current_line.origin+current_line.dir*current_line.begin_param-pivot)@current_line.dir) > abs((current_line.origin+current_line.dir*current_line.end_param-pivot)@current_line.dir):
            current_line.reverse()
        self.vertices = [self.prev_polygon_vertex, self.next_polygon_vertex]
        print('start', pivot)
        while current_line.prev_slab != self.next_slab and current_line.next_slab != self.next_slab:
            self.slab_intersections.remove(current_line)
            pivot_param = (pivot-current_line.origin)@current_line.dir
            print('current_line', len(self.slab_intersections), pivot, pivot_param, current_line.origin+current_line.dir*current_line.begin_param, current_line.origin+current_line.dir*current_line.end_param)
            best_candidate = None
            # other_slab = getOtherSlab(current_line)
            # lightcycles = []
            # if len(other_slab.prev_lightcycles) > 0:
            #     lightcycles.append(other_slab.prev_lightcycles[-1])
            # if len(other_slab.next_lightcycles) > 0:
            #     lightcycles.append(other_slab.next_lightcycles[-1])
            # for lightcycle in lightcycles:
            #     third_slab = lightcycle.next_slab if lightcycle.prev_slab == other_slab else lightcycle.prev_slab
            #     if third_slab != self:
            #         for candidate in self.slab_intersections:
            #             if getOtherSlab(candidate) == third_slab:
            #                 best_candidate = candidate
            # print('best_candidate', best_candidate)
            # if best_candidate != None:
            #     pivot = current_line[2]+current_line[3]*current_line[5]
            #     if abs((best_candidate[2]+best_candidate[3]*best_candidate[4]-pivot)@best_candidate[3]) > abs((best_candidate[2]+best_candidate[3]*best_candidate[5]-pivot)@best_candidate[3]):
            #         flipLine(best_candidate)
            # else:
            best_param = float('nan')
            for candidate in self.slab_intersections:
                type, paramA, paramB = nearestPointOfLines(current_line.origin, current_line.dir, candidate.origin, candidate.dir)
                print(type, paramA, paramB, pivot_param <= paramA and current_line.begin_param-tollerance <= paramA, paramA <= current_line.end_param+tollerance, candidate.begin_param-tollerance <= paramB, paramB <= candidate.end_param+tollerance)
                print(pivot_param, paramA)
                print(current_line.begin_param-tollerance, paramA, current_line.end_param+tollerance)
                print(candidate.begin_param-tollerance, paramB, candidate.end_param+tollerance)
                if type == 'Crossing':
                    if pivot_param <= paramA and current_line.begin_param-tollerance <= paramA and paramA <= current_line.end_param+tollerance and candidate.begin_param-tollerance <= paramB and paramB <= candidate.end_param+tollerance:
                        # print('vertex', paramA, candidate, current_line.origin+current_line.dir*current_line.begin_param)
                        if best_candidate == None or best_param > paramA:
                            best_candidate = candidate
                            best_param = paramA
                elif type == 'Coaxial':
                    pivot = current_line.origin+current_line.dir*current_line.end_param
                    if abs((candidate.origin+candidate.dir*candidate.begin_param-pivot)@current_line.dir) <= tollerance or abs((candidate.origin+candidate.dir*candidate.end_param-pivot)@current_line.dir) <= tollerance:
                        best_candidate = candidate
                        best_param = paramA
            if best_candidate == None:
                print('error')
                break
            pivot = current_line.origin+current_line.dir*best_param
            begin_param = abs((best_candidate.origin+best_candidate.dir*best_candidate.begin_param-pivot)@best_candidate.dir)
            end_param = abs((best_candidate.origin+best_candidate.dir*best_candidate.end_param-pivot)@best_candidate.dir)
            if begin_param <= tollerance or end_param <= tollerance:
                if begin_param > end_param:
                    best_candidate.reverse()
            else:
                normal = self.plane.normal.cross(current_line.dir)
                if (best_candidate.origin+best_candidate.dir*best_candidate.begin_param-pivot)@normal < (best_candidate.origin+best_candidate.dir*best_candidate.end_param-pivot)@normal:
                    best_candidate.reverse()
            current_line = best_candidate
            self.vertices.insert(0, pivot)
        self.slab_intersections = None

class Collision:
    __slots__ = ['winner_time', 'looser_time', 'winner', 'loosers', 'children']
    def __init__(self, winner_time, looser_time, winner, loosers):
        self.winner_time = winner_time
        self.looser_time = looser_time
        self.winner = winner
        self.loosers = loosers
        self.children = []

    def checkCandidate(self):
        if self.winner != None and self.winner.collision != None and self.winner.collision.looser_time < self.winner_time:
            return False
        for looser in self.loosers:
            if looser.collision != None:
                return False
        return True

    def collide(self, lightcycles, collision_candidates, polygon_vertices, polygon_normal, tollerance=0.0001):
        for looser in self.loosers:
            looser.collision = self
        if len(self.loosers) == 2:
            assert(self.loosers[0].normal@self.loosers[1].normal > 0.0)
            position = self.loosers[0].ground_origin+self.loosers[0].ground_velocity*self.looser_time
            dirA = self.loosers[0].ground_velocity.normalized()
            dirB = self.loosers[1].ground_velocity.normalized()
            ground_dir = dirA+dirB
            if ground_dir.length > tollerance:
                index = 1 if self.loosers[0].slab_intersection.prev_slab.isOuterOfCollision(dirA, ground_dir, polygon_normal) else 0
                if dirA.cross(dirB)@polygon_normal > 0.0:
                    index = 1-index
                self.children = [Lightcycle(
                    lightcycles, collision_candidates, polygon_vertices, polygon_normal, False,
                    self.looser_time, self.loosers[index].slab_intersection.prev_slab, self.loosers[1-index].slab_intersection.next_slab,
                    position, ground_dir.normalized(), self.loosers[0].normal
                )]
            else:
                ground_dir = dirA.cross(self.loosers[0].normal)
                index = 1 if self.loosers[0].slab_intersection.prev_slab.isOuterOfCollision(dirA, ground_dir, polygon_normal) else 0
                self.children = [Lightcycle(
                    lightcycles, collision_candidates, polygon_vertices, polygon_normal, False,
                    self.looser_time, self.loosers[index].slab_intersection.prev_slab, self.loosers[1-index].slab_intersection.next_slab,
                    position, ground_dir, self.loosers[0].normal
                ), Lightcycle(
                    lightcycles, collision_candidates, polygon_vertices, polygon_normal, True,
                    self.looser_time, self.loosers[1-index].slab_intersection.prev_slab, self.loosers[index].slab_intersection.next_slab,
                    position, -ground_dir, self.loosers[0].normal
                )]

class Lightcycle:
    __slots__ = ['start_time', 'ground_origin', 'ground_velocity', 'normal', 'collision', 'slab_intersection']
    def __init__(self, lightcycles, collision_candidates, polygon_vertices, polygon_normal, immunity, start_time, prev_slab, next_slab, position, ground_dir, normal):
        exterior_angle = math.pi-math.acos(prev_slab.edge@-next_slab.edge)
        # pitch_angle = math.atan(math.cos(exterior_angle*0.5))
        ground_speed = 1.0/math.cos(exterior_angle*0.5)
        self.start_time = start_time
        self.ground_origin = position
        self.ground_velocity = ground_dir*ground_speed
        self.normal = normal
        self.collision = None
        self.slab_intersection = SlabIntersection(prev_slab, next_slab, None, None, 0.0, 0.0)
        if self.normal@polygon_normal > 0.0:
            prev_slab.next_lightcycles.append(self)
            next_slab.prev_lightcycles.append(self)
        self.collideWithLightcycles(lightcycles, collision_candidates, immunity)
        self.collideWithPolygon(collision_candidates, polygon_vertices, immunity)
        lightcycles.append(self)

    def collideWithLightcycles(self, lightcycles, collision_candidates, immunity, arrival_tollerance=0.001):
        for i in range(0, len(lightcycles)-1 if immunity == True else len(lightcycles)):
            timeA, timeB = rayRayIntersection(self.ground_origin, self.ground_velocity, lightcycles[i].ground_origin, lightcycles[i].ground_velocity)
            if math.isnan(timeA) or math.isnan(timeB):
                continue
            timeA += self.start_time
            timeB += lightcycles[i].start_time
            winner = None if abs(timeA-timeB) < arrival_tollerance else self if timeA < timeB else lightcycles[i]
            # TODO: Insert in manyfold collision
            insort_right(collision_candidates, lambda collision: collision.looser_time, Collision(
                winner_time=min(timeA, timeB),
                looser_time=max(timeA, timeB),
                winner=winner,
                loosers=([self, lightcycles[i]] if winner == None else [self if timeA > timeB else lightcycles[i]])
            ))

    def collideWithPolygon(self, collision_candidates, polygon_vertices, immunity):
        min_time = float('inf')
        for index in range(0, len(polygon_vertices)):
            if type(immunity) is int and (index == immunity or index == (immunity+1)%len(polygon_vertices)):
                continue
            time = rayLineSegmentIntersection(self.ground_origin, self.ground_velocity, polygon_vertices[index-1], polygon_vertices[index])
            if not math.isnan(time):
                min_time = min(time+self.start_time, min_time)
        if min_time < float('inf'):
            insort_right(collision_candidates, lambda collision: collision.looser_time, Collision(
                winner_time=0.0,
                looser_time=min_time,
                winner=None,
                loosers=[self]
            ))

    def calculateLineSegement(self, tollerance=0.0001):
        if self.collision == None:
            return
        self.slab_intersection.origin = self.ground_origin+self.normal*self.start_time
        dir = self.ground_origin+self.ground_velocity*(self.collision.looser_time-self.start_time)+self.normal*self.collision.looser_time-self.slab_intersection.origin
        self.slab_intersection.dir = dir.normalized()
        self.slab_intersection.end_param = dir@self.slab_intersection.dir
        if self.slab_intersection.prev_slab.plane.normal@self.slab_intersection.next_slab.plane.normal >= 1.0-tollerance:
            self.slab_intersection.prev_slab.slab_intersections.append(self.slab_intersection)
            self.slab_intersection.next_slab.slab_intersections.append(self.slab_intersection)

def straightSkeletonOfPolygon(polygon_vertices, mesh_data, height=1.5, tollerance=0.0001):
    polygon_normal = normalOfPolygon(polygon_vertices).normalized()
    polygon_plane = Plane(normal=polygon_normal, distance=polygon_vertices[0]@polygon_normal)
    for polygon_vertex in polygon_vertices:
        if abs(polygon_vertex@polygon_plane.normal-polygon_plane.distance) > tollerance:
            return 'Polygon is not planar / level'

    slabs = []
    lightcycles = []
    collision_candidates = []

    for index, next_polygon_vertex in enumerate(polygon_vertices):
        prev_polygon_vertex = polygon_vertices[index-1]
        slabs.append(Slab(polygon_normal, prev_polygon_vertex, next_polygon_vertex))

    for index, prev_slab in enumerate(slabs):
        next_slab = slabs[(index+1)%len(polygon_vertices)]
        next_slab.prev_slab = prev_slab
        prev_slab.next_slab = next_slab
        Lightcycle(
            lightcycles, collision_candidates, polygon_vertices, polygon_normal, index,
            0.0, prev_slab, next_slab, polygon_vertices[index],
            (prev_slab.edge-next_slab.edge).normalized(), prev_slab.edge.cross(-next_slab.edge).normalized()
        )

    i = 0
    while i < len(collision_candidates):
        collision = collision_candidates[i]
        if collision.checkCandidate():
            collision.collide(lightcycles, collision_candidates, polygon_vertices, polygon_normal)
            if len(collision.loosers) > 2:
                return 'Manyfold collision' # TODO
        i += 1

    for lightcycle in lightcycles:
        lightcycle.calculateLineSegement()

    for j, slabA in enumerate(slabs):
        slabA.calculateVerticesFromLightcycles()
        for i, slabB in enumerate(slabs):
            if i >= j:
                continue
            slabA.intersect(slabB)

    for slab in slabs:
        slab.calculateVerticesFromIntersections()

    verts = []
    edges = []
    faces = []

    # slab = slabs[9]
    # for slab_intersection in slab.slab_intersections:
    #     edges.append((len(verts), len(verts)+1))
    #     verts.append(slab_intersection[2]+slab_intersection[3]*slab_intersection[4])
    #     verts.append(slab_intersection[2]+slab_intersection[3]*slab_intersection[5])
    # slab.calculateVerticesFromLineSegments()

    # for index, lightcycle in enumerate(lightcycles):
    #     duration = height-lightcycle.start_time if lightcycle.collision == None else lightcycle.collision.looser_time-lightcycle.start_time
    #     verts.append(lightcycle.ground_origin+lightcycle.normal*lightcycle.start_time)
    #     verts.append(lightcycle.ground_origin+lightcycle.normal*(lightcycle.start_time+duration)+lightcycle.ground_velocity*duration)
    #     edges.append((len(verts)-2, len(verts)-1))

    for index, slab in enumerate(slabs):
        vert_index = len(verts)
        # slope_vec = slab.slope*height/(slab.slope@polygon_normal)
        # def fillSlopeToHeight(position):
        #     verts.append(position+slope_vec*(1.0-(position@polygon_normal)/(slope_vec@polygon_normal)))
        # fillSlopeToHeight(slab.vertices[0])
        verts += slab.vertices
        # fillSlopeToHeight(slab.vertices[-1])
        face = []
        for i in range(vert_index, len(verts)):
            # edges.append((i-1 if i > vert_index else len(verts)-1, i))
            face.append(i)
        faces.append(face)

    mesh_data.from_pydata(verts, edges, faces)
    # mesh_data.update(calc_edges=True)
    return True
