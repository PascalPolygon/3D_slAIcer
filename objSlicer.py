import os
from mpl_toolkits import mplot3d
import ply
import meshcut
import numpy as np
import matplotlib.pyplot as plt
import itertools
import openmesh as om
from obj2verts import *


class ObjSlicer:

    def singleSlice(self, objFile, plane_norm, planeOrig='firstVert'):
        o2v = Obj2MeshVerts()
        geometry = o2v.readFile(objFile)
        verts = geometry.v
        faces = geometry.f

        mesh = meshcut.TriangleMesh(verts, faces)

        if planeOrig == 'firstVert':
            # Plane orig 0 means the orgin of the cutting plane is the first vertex
            firstVert = verts[0]

        # Place plane origin at the firt vertex
        plane_orig = (firstVert[0], firstVert[1], firstVert[2])
        # plane_norm = (0, 0, 1)

        plane = meshcut.Plane(plane_orig, plane_norm)
        return meshcut.cross_section_mesh(mesh, plane)
