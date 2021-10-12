import numpy as np
from tqdm import tqdm


class ObjGeometry:

    def __init__(self):
        self.v = []
        self.f = []
        self.e = []

    def getFaceVerts(self, face):
        faceVerts = np.zeros((3, 3))
        faceVerts[0] = self.v[face[0]-1]  # -1 because start index is 0
        faceVerts[1] = self.v[face[1]-1]
        faceVerts[2] = self.v[face[2]-1]
        return faceVerts

    def findMatchingVerts(self, f1Id, f2Id):
        # Common values between 2 arrays
        return np.intersect1d(self.f[f1Id], self.f[f2Id])

    def findEdges(self):
        face1Id = 0
        for i in tqdm(range(len(self.f)), desc='Finding Edges...'):
            face1 = self.f[i]
            # face1Verts = self.getFaceVerts(face1)
            face2Id = 0
            for face2 in self.f:
                if face1Id != face2Id:
                    matches = self.findMatchingVerts(face1Id, face2Id)
                    if matches.size == 2:
                        edge = {
                            # Edge between these 2 vertex ids
                            'v': matches,
                            # Edge between these 2 face ids
                            'f': np.array([face1Id+1, face2Id+1])
                        }
                        self.e.append(edge)
                face2Id += 1
            face1Id += 1
        # print(f'Done! {len(self.e)} edges found')
        return self.e

    def findAngleVertex(self, triangle, edge):
        edgeVert1Id = edge[0]
        edgeVert2Id = edge[1]
        # get triangle vertex that isn't on the edge
        mask = np.logical_and(triangle != edgeVert1Id, triangle != edgeVert2Id)
        # print(f'Angle vertex: {triangle[mask]}')
        return triangle[mask][0]

    def calculateAdjacentVecs(self, angleVertId, edge):
        edgeVert1Id = edge[0]
        edgeVert2Id = edge[1]
        vec1 = self.v[angleVertId-1] - self.v[edgeVert1Id-1]
        vec2 = self.v[angleVertId-1] - self.v[edgeVert2Id-1]
        # print(
        # f'Vector between verts {angleVertId} ({self.v[angleVertId-1]}) and {edgeVert1Id} ({self.v[edgeVert1Id-1]}): {vec1}')
        # print(
        # f'Vector between verts {angleVertId} ({self.v[angleVertId-1]}) and {edgeVert2Id} ({self.v[edgeVert2Id-1]}): {vec2}')
        # return vec1, vec2
        return np.array([vec1, vec2])

    def calculateInnerAngles(self, thisEdge, angleVertIds, neighborTriangles, vecs):
        innerAngles = np.zeros((2,))

        for i, t in enumerate(neighborTriangles):

            # angleVertId = self.findAngleVertex(t, thisEdge)
            angleVertId = angleVertIds[i]
            # vec1, vec2 = self.calculateAdjacentVecs(angleVertId, thisEdge)
            vec1 = vecs[i][0]
            vec2 = vecs[i][1]

            theta = np.degrees(
                np.arccos(vec1.dot(vec2)/(np.linalg.norm(vec1) * np.linalg.norm(vec2))))
            # print(f'theta = {theta}')
            innerAngles[i] = theta
            # print('---------------')
        return innerAngles

    def calculateEdgeLenRatio(self, edge, vecs):
        edgeVert1Id = edge[0]
        edgeVert2Id = edge[1]
        # print('Calculating edge ratios')
        # print(f'edge: {edge}')
        # print(f'Ange vert Ids: {angleVertIds}')
        edgeLen = np.linalg.norm(self.v[edgeVert1Id-1] - self.v[edgeVert2Id-1])
        edgeMidVert = np.zeros((3,))

        edgeMidVert = np.array(
            [(self.v[edgeVert1Id-1][0] + self.v[edgeVert2Id-1][0])/2,
             (self.v[edgeVert1Id-1][1] + self.v[edgeVert2Id-1][1])/2,
             (self.v[edgeVert1Id-1][2] + self.v[edgeVert2Id-1][2])/2])

        # print(f'EdgeLen : {edgeLen}')
        # print(f'EdgeMidVert: {edgeMidVert}')
        # Only need one vec (hypthenuse)
        a = np.linalg.norm(vecs[0][0])  # side 1 for 1st triangle
        c = np.linalg.norm(vecs[0][1])  # Hypotenuse 2 for 1st triangle

        b = edgeLen
        # print(f'a: {a}, b: {b}, c:{c}')
        s = (a+b+c)/2
        A = np.sqrt(s*(s-a)*(s-b)*(s-c))  # Heron's formula
        h1 = A*2/b
        # print(f'h1 : {h1}')

        a = np.linalg.norm(vecs[1][0])  # side 1 for 2nd triangle
        c = np.linalg.norm(vecs[1][1])  # side 2 for 2nd triangle
        # b = edgeLen
        s = (a+b+c)/2
        A = np.sqrt(s*(s-a)*(s-b)*(s-c))  # Heron's formula
        h2 = A*2/b
        # print(f'a: {a}, b: {b}, c:{c}')
        # print(f'h2 : {h2}')

        # h1 = np.linalg.norm()
        # h1 = np.linalg.norm(edgeMidVert-self.v[angleVertIds[0]-1])
        # h2 = np.linalg.norm(edgeMidVert-self.v[angleVertIds[1]-1])

        # print(f'h1: {h1}, h2: {h2}')

        return np.array([edgeLen/h1, edgeLen/h2])

        # edgeMidVert[0] = (self.v[edgeVert1Id][0] + self.v[edgeVert2Id][0])/2
        # edgeMidVert[1] = (self.v[edgeVert1Id][1] + self.v[edgeVert2Id][1])/2
        # edgeMidVert[2] = (self.v[edgeVert1Id][2] + self.v[edgeVert2Id][2])/2
        # calculate edge len
        # calculate triangle height
    def calculateDihedral(self, edge, angleVertIds):
        edgeVert1Id = edge[0]
        edgeVert2Id = edge[1]
        N = []
        # angleVert1
        # Calcuate normal for each triangle
        for i, angleVertId in enumerate(angleVertIds):
            # angleVertId = angleVertIds[i]
            # print(f'edgeVert1Id: {edgeVert1Id}, angleVert{i+1}Id: {angleVertId}')
            A = self.v[edgeVert1Id-1] - self.v[angleVertId-1]
            # print(f'A: {A}')
            B = self.v[edgeVert2Id-1] - self.v[angleVertId-1]
            # print(f'edgeVert2Id: {edgeVert2Id}, angleVert{i+1}Id: {angleVertId}')
            # print(f'B: {B}')
            N.append(np.cross(A, B))  # Calculate normal vector

        vec1 = N[0]
        vec2 = N[1]

        # print(f'N0: {vec1}')
        # print(f'N1: {vec2}')

        thetaInv = vec1.dot(vec2)/(np.linalg.norm(vec1) * np.linalg.norm(vec2))

        # Limit to 4 point precision, so number stays in domain of arccos
        thetaInv = float("{:.4f}".format(thetaInv))
        # print(f'theta inv: {thetaInv}')
        # Using dot product formula
        # Angle between normals is same as angle between faces
        return np.degrees(np.arccos(thetaInv))

    def calculateFeatures(self):
        # Caculate dihedral angle between 2 faces
        edges = self.findEdges()
        # Calculate inner angle of triangles
        # for edge in edges:
        nEdges = len(edges)
        features = np.zeros((nEdges, 5))
        for i in tqdm(range(nEdges), desc='Calculating Features...'):
            # Find dihedral of first triangle
            edge = edges[i]
            # print(f'edge: {edge}')
            t1Id = edge['f'][0]
            t2Id = edge['f'][1]

            t1 = self.f[t1Id-1]  # Index 0 for python list
            t2 = self.f[t2Id-1]

            # print(f'Triangle {t1Id}: {t1}')
            # print(f'Triangle {t2Id}: {t2}')

            neighborTriangles = [t1, t2]
            thisEdge = [edge['v'][0], edge['v'][1]]
            angleVertIds = []
            angleVertIds.append(self.findAngleVertex(t1, thisEdge))
            angleVertIds.append(self.findAngleVertex(t2, thisEdge))

            # vecs = np.zeros((2, 2,3))
            vecs = []
            vecs.append(self.calculateAdjacentVecs(
                angleVertIds[0], thisEdge))  # For 1st triangle
            vecs.append(self.calculateAdjacentVecs(
                angleVertIds[1], thisEdge))  # For 2nd triangle
            # vecs[0] = self.calculateAdjacentVecs(angleVertIds[0], thisEdge)
            # vecs[1] = self.calculateAdjacentVecs(angleVertIds[1], thisEdge)

            # vec1, vec2 = self.calculateAdjacentVecs(angleVertId, thisEdge)
            ia = self.calculateInnerAngles(
                thisEdge, angleVertIds, neighborTriangles, vecs)  # Inner angles
            elr = self.calculateEdgeLenRatio(thisEdge, vecs)
            dihedral = self.calculateDihedral(thisEdge, angleVertIds)

            # Calculate dihedral angle
            ia = np.sort(ia)
            elr = np.sort(elr)

            features[i] = np.array([ia[0], ia[1], elr[0], elr[1],  dihedral])

            # print(f'Inner angles: {ia}')
            # print(f'Edge length ratios: {elr}')
            # print(f'Dihedral : {dihedral}')
            # print(f'Edge featues: {np.array([ia[0], ia[1], elr[0], elr[1],  dihedral])}')
            # print(f'---------------')
        return features

        # calculate edge ratios


class Obj2verts:
    # def __init__(self):
    def readFile(self, filePath):
        geom = ObjGeometry()
        lines = []
        verts = []
        faces = []
        with open(filePath) as f:
            lines = f.readlines()

        for line in lines:
            line = line.strip().split(' ')
            if line[0] == 'v':
                geom.v.append(
                    np.array([line[1], line[2], line[3]], dtype=np.float))
                # verts.append(np.array([line[1], line[2], line[3]], dtype=np.float))
            elif line[0] == 'f':
                geom.f.append(
                    np.array([line[1], line[2], line[3]], dtype=np.int))
                # faces.append(np.array([line[1], line[2], line[3]], dtype=np.int))
        return geom


class Obj2MeshVerts:
    # def __init__(self):
    def readFile(self, filePath):
        geom = ObjGeometry()
        lines = []
        verts = []
        faces = []
        with open(filePath) as f:
            lines = f.readlines()

        for line in lines:
            line = line.strip().split(' ')
            if line[0] == 'v':
                geom.v.append(
                    np.array([line[1], line[2], line[3]], dtype=np.float))
            elif line[0] == 'f':
                l = []
                l.append(int(line[1])-1)
                l.append(int(line[2])-1)
                l.append(int(line[3])-1)
                geom.f.append(l)
        return geom
