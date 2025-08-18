# -*- coding: utf-8 -*-

import math
import sys
# import string
import copy
import numpy as np
# import trimesh
# from trimesh.transformations import translation_matrix
# from trimesh.geometry import align_vectors
import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D

#printer specific constants, should be suplied as args
bedWidth = 150.0#mm
delta=0.002# What precision I want to reach when reading nr: tolerance of the technology
# supportInfill = .5
# hatch_distance=0.2

class Point:
    def __init__(self, x_, y_, z_):
        self.x = x_
        self.y = y_
        self.z = z_

    def dotProduct(self, p):
        return self.x*p.x + self.y*p.y + self.z*p.z

    def normalize(self):
        return math.sqrt(self.x**2 + self.y**2 + self.z**2)

    def toString(self):
        return "Point("+str(self.x)+","+str(self.y)+","+str(self.z)+")"
    def equals(self, p2):
        if close(self.x,p2.x) and close(self.y,p2.y) and close(self.z,p2.z):
            return True
        else:
            return False

def pointInLine(p, line):
    if close(p.x,line.p0.x) and close(p.y,line.p0.y) and close(p.z,line.p0.z):
        return True
    elif close(p.x,line.p1.x) and close(p.y,line.p1.y) and close(p.z,line.p1.z):
        return True
    else:
        return False

class Line:
    def __init__(self, p0_, p1_):
        self.p0 = p0_
        self.p1 = p1_

    def toString(self):
        return "Line("+self.p0.toString()+","+self.p1.toString()+")"
    def reverse(self):
        x_ = copy.copy(self.p0.x)
        y_ = copy.copy(self.p0.y)
        z_ = copy.copy(self.p0.z)
        self.p0.x = copy.copy(self.p1.x)
        self.p0.y = copy.copy(self.p1.y)
        self.p0.z = copy.copy(self.p1.z)
        self.p1.x = x_
        self.p1.y = y_
        self.p1.z = z_
        return self

#for floating point comparison
def close(f1,f2):
    comp = (max(f1,f2) - min(f1,f2))
    return (comp > -delta) and (comp < delta)

def lineEqual(L1,L2):
    if ((close(L1.p0.x, L2.p0.x) and close(L1.p0.y, L2.p0.y)
     and close(L1.p1.x, L2.p1.x) and close(L1.p1.y, L2.p1.y)) 
    or (close(L1.p0.x, L2.p1.x) and close(L1.p0.y, L2.p1.y) 
     and close(L1.p1.x, L2.p0.x) and close(L1.p1.y, L2.p0.y))):
        return True
    else:
        return False

class Triangle:
    def __init__(self, p0_, p1_, p2_, norm_):
        self.p0 = p0_
        self.p1 = p1_
        self.p2 = p2_
        self.norm = norm_
    def toString(self):
        return "Triangle("+self.p0.toString()+","+self.p1.toString()+","+self.p2.toString()+")"

def triangleEqual(T1,T2):
    if ((T1.p0.equals(T2.p0) and T1.p1.equals(T2.p1) and T1.p2.equals(T2.p2))
        or (T1.p0.equals(T2.p0) and T1.p1.equals(T2.p2) and T1.p2.equals(T2.p1))
        or (T1.p0.equals(T2.p1) and T1.p1.equals(T2.p0) and T1.p2.equals(T2.p2))
        or (T1.p0.equals(T2.p1) and T1.p1.equals(T2.p2) and T1.p2.equals(T2.p0))
        or (T1.p0.equals(T2.p2) and T1.p1.equals(T2.p0) and T1.p2.equals(T2.p1))
        or (T1.p0.equals(T2.p2) and T1.p1.equals(T2.p1) and T1.p2.equals(T2.p0))):
        return True
    else:
        return False

class Slice:
    def __init__(self, zValue_, perimeter_, isSurface_):
        self.zValue = zValue_
        self.perimeter = perimeter_
        self.isSurface = isSurface_
        self.support = list()
        self.infill = list()

# given an stl file of standard format,
# returns a list of the triangles
def fileToTriangles(filename):
    with open(filename, 'r') as f:
        next(f)  # Skip the first line
        triangles = []
        points = []
        counter = 0
        for line in f:
            l = line.strip().split()
            if not l:
                continue

            if counter == 6:
                counter = 0  # Reset counter for next triangle
                continue

            if counter == 0 and l[0] == 'endsolid':
                break  # Stop parsing at end of file

            if counter in {2, 3, 4}:
                points.append(Point(float(l[1]), float(l[2]), float(l[3])))

            if counter == 0:  # Normal vector line
                normal = Point(float(l[2]), float(l[3]), float(l[4]))

            counter += 1

            if counter == 5:  # After reading 3 points, form a triangle
                triangles.append(Triangle(points[0], points[1], points[2], normal))
                points.clear()  # Reset points list for the next triangle

    return triangles

# given a line segment and a plane,
# returns the point at which those two
# planes intersect. Will return the first point
# in the line if the whole line is in the plane
def intersectSlice(line, plane):
    if line.p0.z == line.p1.z and line.p1.z == plane:
        return line.p0
    elif line.p0.z == line.p1.z:
        return None
    else:
        slope = Point(x_=line.p1.x-line.p0.x, y_=line.p1.y-line.p0.y, z_=line.p1.z-line.p0.z)
        t = float(plane-line.p0.z)/float(slope.z)

        if t >= 0 and t <= 1:
            testZ = line.p0.z+t*slope.z
            if testZ <= max(line.p0.z, line.p1.z) and testZ >= min(line.p0.z, line.p1.z):
                testP = Point(x_=line.p0.x+t*slope.x, y_=line.p0.y+t*slope.y, z_=line.p0.z+t*slope.z)
                return Point(x_=line.p0.x+t*slope.x, y_=line.p0.y+t*slope.y, z_=line.p0.z+t*slope.z)

            else: 
                return None
        else:
            return None


#helper for aboveTriangle
def sign(p1, p2, p3):
    return (p1.x-p3.x)*(p2.y-p3.y) - (p2.x-p3.x)*(p1.y-p3.y)

# given a point and a triangle, 
# returns True if that point is directly above that triangle,
# False otherwise
def aboveTriangle(point,triangle):
    if  (point.z > (triangle.p0.z-delta) and
        point.z > (triangle.p1.z-delta) and
        point.z > (triangle.p2.z-delta)):

        b1 = (sign(point, triangle.p0, triangle.p1) < 0.0)
        b2 = (sign(point, triangle.p1, triangle.p2) < 0.0)
        b3 = (sign(point, triangle.p2, triangle.p0) < 0.0)
        ret = ((b1 == b2) and (b2 == b3))
        return ret

    else:
        return False
        
# given a list of triangles in 3D space,
# returns a tuple of the highest and lowest Z values
def findBoundaries(triangles):
    bottomZ = 500
    topZ = -500

    for triangle in triangles:
        maximum = max(triangle.p0.z, triangle.p1.z, triangle.p2.z)
        minimum = min(triangle.p0.z, triangle.p1.z, triangle.p2.z)

        if maximum > topZ:
            topZ = maximum
        if minimum < bottomZ:
            bottomZ = minimum

    return (bottomZ, topZ)

# given a list of triangles and a thickness per layer,
# computes the total number of layers and checks which
# line segments need to be drawn in each layer,
# returns the list of slices with the list of line segments
# to draw per slice, as a tuple with a bool for if the slice 
# is a bottom or top
def separateSlices(triangles, layerThickness):
    bounds = findBoundaries(triangles)
    numSlices = int((bounds[1]-bounds[0])/layerThickness)
    slices = [bounds[0]+z*layerThickness for z in range(0, numSlices+1)]
    segments = list()
    for s in slices:
        currentSegment = list()
        currentSegmentSurface = False
        for triangle in triangles:
            point1 = intersectSlice(Line(p0_=triangle.p0, p1_=triangle.p1), s)
            point2 = intersectSlice(Line(p0_=triangle.p1, p1_=triangle.p2), s)
            point3 = intersectSlice(Line(p0_=triangle.p2, p1_=triangle.p0), s)

            points_ = list(set([point1, point2, point3]))
            points = list()

            for point in points_:
                if point == None:
                    points_.remove(None)
                    break
            
            for i in range(0,len(points_)):
                j = i+1
                unique = True
                while j < len(points_):
                    if points_[i].equals(points_[j]):
                        unique = False
                    j+=1
                if unique:
                    points.insert(0,copy.deepcopy(points_[i]))

            if s <= (bounds[0]+layerThickness) or s >= (bounds[1]-layerThickness):
                currentSegmentSurface = True

            #if len(points) == 1:
                #currentSegment.append(Line(points[0], points[0]))
            if len(points) == 2:
                currentSegment.append(Line(points[0], points[1]))
            elif len(points) == 3:
                segment1 = Line(points[0], points[1])
                segment2 = Line(points[1], points[2])
                segment3 = Line(points[2], points[0])
                currentSegmentSurface = True

                currentSegment.append(segment1)
                currentSegment.append(segment2)
                currentSegment.append(segment3)
         
        segments.append(Slice(zValue_=s, perimeter_=copy.deepcopy(currentSegment),isSurface_=currentSegmentSurface))
        '''
        for line in currentSegment:
            print("appended "+line.toString())
        '''
    return segments

# given two lines on the same z plane, 
# returns the point at which they intersect,
# or None if there is no intersection
def intersection(L1,L2):

    #make sure all lines are on the same z plane
    #assert (math.isclose(L1.p0.z, L1.p1.z, abs_tol=0.0001))
    #assert (L2.p0.z == L2.p1.z)
    #assert (L1.p0.z == L2.p0.z)

    x1 = L1.p0.x
    y1 = L1.p0.y
    x2 = L1.p1.x
    y2 = L1.p1.y
    x3 = L2.p0.x
    y3 = L2.p0.y
    x4 = L2.p1.x
    y4 = L2.p1.y

    xnum = (x1*y2-y1*x2)*(x3-x4) - (x1-x2)*(x3*y4-y3*x4)
    xden = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4)
    ynum = (x1*y2-y1*x2)*(y3-y4) - (y1-y2)*(x3*y4-y3*x4)
    yden = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4)

    try:
        intersect = Point(xnum/xden,ynum/yden,L1.p0.z) 

        if ((intersect.x >= min(x1,x2)-delta) and (intersect.x <= max(x1,x2)+delta) and
            (intersect.y >= min(y1,y2)-delta) and (intersect.y <= max(y1,y2)+delta) and
            (intersect.x >= min(x3,x4)-delta) and (intersect.x <= max(x3,x4)+delta) and
            (intersect.y >= min(y3,y4)-delta) and (intersect.y <= max(y3,y4)+delta)):
            return intersect
        else:
            return None
        # return intersect
    except:
        return None

#given a list of lines that make a manifold perimeter on a slice,
#and a percentage of space that should be infill,
#returns a list of infill lines (grid pattern) for that slice
#assumes print bed area is a square
def infill(perimeter, hatch_distance):
    assert hatch_distance > 0

    if len(perimeter) == 0:
        return []

    Z = perimeter[0].p0.z  # should be the same across all lines
    infill = []

    # Find bounding box of the STL layer
    min_x = min(line.p0.x for line in perimeter)
    max_x = max(line.p0.x for line in perimeter)

    x_pos = min_x  # Start from the leftmost perimeter point

    # Generate lines until we reach the rightmost boundary of the STL layer
    while x_pos <= max_x:
        fullLine = Line(Point(x_pos, -bedWidth / 2, Z), Point(x_pos, bedWidth / 2, Z))
        inters = []

        # Find intersections without repeats
        for line in perimeter:
            sect = intersection(line, fullLine)
            if sect is not None:
                new = True
                for i in inters:
                    if close(i.y, sect.y):
                        new = False
                if new:
                    inters.append(copy.deepcopy(sect))

        # Sort by y to get matching pairs for internal lines
        inters.sort(key=lambda point: point.y)

        if len(inters) % 2 == 0:  # Ensure even count for valid pairs
            for i in range(len(inters)):
                if i % 2 != 0:
                    overlap = False
                    newLine = Line(inters[i - 1], inters[i])
                    for l in perimeter:
                        if lineEqual(l, newLine):
                            overlap = True
                    if not overlap:
                        infill.append(newLine)

        x_pos += hatch_distance  # Move to the next line position

    return infill



# given a list of line segments and a starting point,
# returns the location of the next line that connects to the points,
# or None if no point follows
def findNextPoint(point, lines):
    for i in range(0, len(lines)):
        line = lines[i]
        if pointInLine(point, line):
            return i
    return None

# given a slice with a list of line segments,
# returns a new slice free of duplicate or interior line segments
# and in order for optimized drawing
def cleanPerimeter(s):
    #for line in s:
        #if L is a duplicate and if every triangle containing L is on the slice, remove all L in base
    setPerimeter = copy.deepcopy(s.perimeter)
    
    i = 0
    while i < len(setPerimeter):
        j = i+1
        while j < len(setPerimeter):
            if lineEqual(setPerimeter[i],setPerimeter[j]):
                setPerimeter.remove(setPerimeter[j])
            else:
                j+=1
        i+=1

    finalPerimeter = setPerimeter
    #need to order perimeter such that it is manifold
    return Slice(zValue_=s.zValue, perimeter_=finalPerimeter, isSurface_=s.isSurface)



# given a list of triangles,
# returns a list of any downward-facing triangles
def downward(triangles):
    trianglesDown = list()
    for triangle in triangles:
        if triangle.norm.z < 0:
            trianglesDown.insert(0, copy.deepcopy(triangle))
    return trianglesDown

# given a downward-facing triangle and a list of all triangles,
# returns True if no triangles are in the way of the downward triangle
# and False if a triangle blocks the path directly downward
def supportNeeded(triangle, triangles, bottomZ):
    if (close(triangle.p0.z, bottomZ)
        and close(triangle.p1.z, bottomZ)
        and close(triangle.p2.z, bottomZ)):
        return False

    for tri in triangles:
        if (aboveTriangle(triangle.p0, tri) 
            or aboveTriangle(triangle.p1, tri) 
            or aboveTriangle(triangle.p2, tri)):
            return False

    return True


# given a triangle that requires support and a minimum Z value,
# returns the list of triangles required to form the support shape under that triangle
def generateSupportShape(triangle, bottomZ):
    triangleTop = copy.deepcopy(triangle)
    triangleBottom = Triangle(Point(triangleTop.p0.x, triangleTop.p0.y, bottomZ), 
                                Point(triangleTop.p1.x, triangleTop.p1.y, bottomZ), 
                                Point(triangleTop.p2.x, triangleTop.p2.y, bottomZ), None)
    newShape = [triangleTop]
    newShape.insert(0, triangleBottom)

    newShape.insert(0, Triangle(triangleTop.p0, triangleTop.p1, triangleBottom.p0, None))
    newShape.insert(0, Triangle(triangleTop.p1, triangleTop.p2, triangleBottom.p1, None))
    newShape.insert(0, Triangle(triangleTop.p2, triangleTop.p0, triangleBottom.p2, None))
    newShape.insert(0, Triangle(triangleBottom.p0, triangleBottom.p1, triangleTop.p1, None))
    newShape.insert(0, Triangle(triangleBottom.p1, triangleBottom.p2, triangleTop.p2, None))
    newShape.insert(0, Triangle(triangleBottom.p2, triangleBottom.p0, triangleTop.p0, None))

    i = 0
    while i < len(newShape):
        j = i+1
        while j < len(newShape):
            if triangleEqual(newShape[i],newShape[j]):
                newShape.remove(newShape[j])
            else:
                j+=1
        i+=1

    return newShape

# given a list of slices with the list of line segments
# to draw per slice, as a tuple with if the slice is a
# bottom or top, and a filename,
# write the G code to the given file 
def transform_point(p, T):
    vec = np.array([p.x, p.y, p.z, 1])
    result = T @ vec
    return result[:3] 

# def compute_alignment_transform(mesh, which_axis):
#     """
#     Returns a 4×4 matrix that:
#       1) centers the mesh CoM at the origin
#       2) rotates so that one principal axis aligns with global +Z
#          - which_axis='smallest' ⇒ plane normal is smallest axis ⇒ longest plane in XY
#          - which_axis='largest'  ⇒ longest axis goes to Z
#       3) translates so the lowest Z sits at Z=0
#     """
#     # 1. Center CoM
#     com = mesh.center_mass
#     T_center = translation_matrix(-com)
    
#     # 2. PCA of vertices
#     verts = np.asarray(mesh.vertices)
#     cov  = np.cov(verts.T)
#     eigvals, eigvecs = np.linalg.eigh(cov)
    
#     fig = plt.figure()
#     ax = fig.add_subplot(111, projection='3d')

#    # Downsample vertices
#     max_points = 1000
#     if len(verts) > max_points:
#         sampled_indices = np.random.choice(len(verts), max_points, replace=False)
#         sampled_verts = verts[sampled_indices]
#     else:
#         sampled_verts = verts

#     ax.scatter(*sampled_verts.T, color='gray', s=10, alpha=0.4)

#     # Plot origin
#     ax.scatter(*com, color='black', s=20, label='Center of Mass')

#     # Plot PCA axes
#     colors = ['r', 'g', 'b']
#     labels = ['Smallest', 'Middle', 'Largest']
#     sorted_indices = np.argsort(eigvals)

#     for i, idx in enumerate(sorted_indices):
#         vec = eigvecs[:, idx]
#         ax.quiver(*com, *vec, length=5, color=colors[i], label=labels[i])

#     ax.set_title("Principal Axes + Bounding Box")
#     ax.legend()
#     plt.show()
    
#     # 3. Pick which eigenvector - ask user 
#     choice = input("Select your preferred axis (smallest, middle, largest): ").strip().lower()
#     choice_map = {'smallest': sorted_indices[0], 'middle': sorted_indices[1], 'largest': sorted_indices[2]}
#     idx = choice_map[choice]
#     principal_axis = eigvecs[:, idx]
    
#     # if which_axis == 'smallest':
#     #     idx = np.argmin(eigvals)
#     # elif which_axis == 'largest':
#     #     idx = np.argmax(eigvals)
#     # else:
#     #     raise ValueError("which_axis must be 'smallest' or 'largest'")
#     # principal = eigvecs[:, idx]
#     # # ensure it points upward
#     # if principal[2] < 0:
#     #     principal = -principal
    
#     # 4. Rotation to align `principal` → Z
#     R= align_vectors(principal_axis, [0,0,1])
    
#     # 5. Compute combined center+rotate
#     T_align = R @ T_center
    
#     # 6. Apply to a copy to measure its new min Z
#     tmp = mesh.copy()
#     tmp.apply_transform(T_align)
#     min_z = tmp.bounds[0][2]
    
#     # 7. Flat translation
#     T_flat = translation_matrix([0, 0, -min_z])
    
#     # 8. Total transform
#     T_total = T_flat @ T_align
#     return T_total


def writeGcode(slices,filename,mode): #,T_restore
    # extrudeRate = 0.05
    f = open(filename[:-3] + "gcode",'w')

    #preamble
    f.write(";Start GCode\n")
    f.write("M109 S210.000000\n")
    f.write("G28 X0 Y0 Z0\n")
    # f.write("G92 E0\n")
    f.write("G29\n")

    o = 0 #origin
    layer = 0; #current layer/slice
    E = 0; #extrusion accumulator
    # T_inverse=np.linalg.inv(T_restore)
    for s in slices:

        f.write(";Layer "+str(layer)+" of "+str(len(slices))+"\n")


        if len(s.infill) > 0:
            f.write(";infill\n")
            if mode=='mono': 
                for l in s.infill:
                    #move to start of line
                    f.write("G0 F2700 X"+str(o+l.p0.x)+" Y"+str(o+l.p0.y)+" Z"+str(l.p0.z)+"\n")
                    #move to end while extruding
                    # dist = math.sqrt(pow(l.p1.x-l.p0.x,2) + pow(l.p1.y-l.p0.y,2))
                    # E += dist*extrudeRate
                    f.write("G1 F900 X"+str(o+l.p1.x)+" Y"+str(o+l.p1.y)+"\n") #+" E"+str(E)
                    # x0, y0, z0 = transform_point(l.p0, T_inverse)
                    # x1, y1, z1 = transform_point(l.p1, T_inverse)

                    # f.write(f"G0 F2700 X{x0:.4f} Y{y0:.4f} Z{z0:.4f}\n")
                    # f.write(f"G1 F900 X{x1:.4f} Y{y1:.4f} Z{z1:.4f}\n")
            if mode=='bidi':
                for i, l in enumerate(s.infill):
                    if i % 2 == 0:
                        # Even lines: go from p0 to p1
                        f.write("G0 F2700 X"+str(o+l.p0.x)+" Y"+str(o+l.p0.y)+" Z"+str(l.p0.z)+"\n")
                        f.write("G1 F900 X"+str(o+l.p1.x)+" Y"+str(o+l.p1.y)+"\n")
                    else:
                        # Odd lines: go from p1 to p0 (reverse direction)
                        f.write("G0 F2700 X"+str(o+l.p1.x)+" Y"+str(o+l.p1.y)+" Z"+str(l.p1.z)+"\n")
                        f.write("G1 F900 X"+str(o+l.p0.x)+" Y"+str(o+l.p0.y)+"\n")
        
        layer+=1

    #postamble
    f.write(";End GCode\n")
    f.write("M104 S0\n")
    f.write("M140 S0\n")
    f.write("G91\n")
    f.write("G1 F300\n") #E-1
    f.write("G1 Z+0.5 X-20 Y-20 F2v700\n") #E-5
    f.write("G28 X0 Y0\n")
    f.write("M84\n")
    f.write("G90\n")

def main():
    filename = sys.argv[1]
    layerThickness = float(sys.argv[2])
    hatch_distance = float(sys.argv[3])
    
    # mesh=trimesh.load("prova_stl.stl")
    # T_restore=compute_alignment_transform(mesh,'smallest')
    # # Convert vertices to homogeneous coordinates (add a column of 1s)
    # vertices_hom = np.hstack((mesh.vertices, np.ones((mesh.vertices.shape[0], 1))))

    # # Apply transformation
    # transformed_vertices = (T_restore @ vertices_hom.T).T[:, :3]  # back to (n, 3)

    # # Update mesh with transformed vertices
    # mesh.vertices = transformed_vertices
    
    triangles = fileToTriangles(filename)

    slices_ = separateSlices(triangles, layerThickness)

    slices = list()

    for s in slices_:
        slices += [cleanPerimeter(s)]

    for s in slices:
            s.infill = infill(s.perimeter, hatch_distance)

    mode = input("Monodirectional or Bidirectional Strategy? (mono, bidi): ").strip().lower()

    writeGcode(slices,filename,mode)

if __name__ == "__main__":
    main()
