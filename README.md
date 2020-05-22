# Straight Skeleton
Based on the algorithms of Stefan Huber and Martin Held this add-on can calculate the straight skeleton of a polygon, slice meshes for iso lines and insets.
It works for polygons described by the faces of a mesh or the poly splines of a curve.
Currently it supports only one polygon at a time.

### Straight Skeleton
Calculates the roof model of the [straight skeleton](https://en.wikipedia.org/wiki/Straight_skeleton) form the selected polygon.

### Slice Mesh
Calculates the [isolines](https://en.wikipedia.org/wiki/Contour_line) of a mesh
with respect to the 3D cursor (which acts as pivot and defines the orientation of the slice planes).
It works similar to the loop cut tool:
- Select exactly one face or poly spline while in edit mode
- First enter the pitch with the mouse and click
- Use the mouse wheel to add more slice layers
- Then enter the offset with the mouse and click again
- Right click or pressing escape cancels the operation
- After the operation each face will be separate. Use "Merge Vertices > By Distance" if you want to connect them

### Inset Polygon
Insets the selected polygon by executing "Straight Skeleton", "Slice Mesh" and a projection.
Usage:
- Select exactly one face or poly spline
- Search for the name of the operator while in edit mode
- Enter the offset with the mouse
- Use the mouse wheel to divide it further
- Click to finish
- Right click or pressing escape cancels the operation
