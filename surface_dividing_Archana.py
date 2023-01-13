
def surface_dividing(triangulated_surface, sagittal_plane):
    vertices = triangulated_surface[0]
    triangles = triangulated_surface[1]
    
    left_surface_vertices = []
    right_surface_vertices = []
    left_surface_triangles = []
    right_surface_traingles =[]
    
    for vertex in vertices:
        if vertex[0] < sagittal_plane:
            left_surface_vertices.append(tuple(vertex))
        else:
            right_surface_vertices.append(tuple(vertex))
    for triangle in triangles:
        triangle_vertices = [vertices[triangle[0]], vertices[triangle[1]], vertices[triangle[2]]]
        
        left_of_sagittal = 0
        right_of_sagittal = 0
        
        for vertex in triangle_vertices:
            if vertex[0] < sagittal_plane:
                left_of_sagittal +=1
            else:
                right_of_sagittal +=1
        if left_of_sagittal == 3:
            left_indices=[]
            for i in range(len(triangle_vertices)):
                left_indices.append(left_surface_vertices.index(tuple(triangle_vertices[i])))
            left_surface_triangles.append(tuple(left_indices))
        elif right_of_sagittal == 3:
            right_indices =[]
            for i in range(len(triangle_vertices)):
                right_indices.append(right_surface_vertices.index(tuple(triangle_vertices[i])))
            right_surface_traingles.append(tuple(right_indices))
        else:
            left_indices = []
            right_indices = []
            intersection_points = []
            
            for i in range (len(triangle_vertices)):
                v1 = tuple(triangle_vertices[i])
                v2 = tuple(triangle_vertices[(i+1)%3])
                
                if v1[0] < sagittal_plane < v2[0] or v2[0] < sagittal_plane < v1[0]:
                    edge_ip = (sagittal_plane -v1[0]) / (v2[0]-v1[0])
                    intersection_point = (sagittal_plane, v1[1]+edge_ip*(v2[1]-v1[1]),v1[2]+edge_ip*(v2[2]-v1[2]))
                    if intersection_point != v1 and intersection_point != v2:
                        intersection_points.append(intersection_point)
            
            for i in range(len(intersection_points)):
                if intersection_points[i] not in left_surface_vertices:
                    left_surface_vertices.append(tuple(intersection_points[i]))
                if intersection_points[i] not in right_surface_vertices:
                    right_surface_vertices.append(tuple(intersection_points[i]))
                    
            for vertex in triangle_vertices:
                if vertex[0] < sagittal_plane:
                    left_indices.append(left_surface_vertices.index(tuple(vertex)))
                else:
                    right_indices.append(right_surface_vertices.index(tuple(vertex)))
            
            if right_of_sagittal == 1:
                right_surface_traingles.append((right_indices[0], right_surface_vertices.index((intersection_points[0])), right_surface_vertices.index((intersection_points[1]))))
            else:
                right_surface_traingles.append((right_indices[0], right_surface_vertices.index((intersection_points[0])), right_surface_vertices.index((intersection_points[1]))))
                right_surface_traingles.append((right_indices[1], right_surface_vertices.index((intersection_points[0])), right_surface_vertices.index((intersection_points[1]))))
                right_surface_traingles.append((right_indices[0], right_indices[1], right_surface_vertices.index((intersection_points[0]))))
                right_surface_traingles.append((right_indices[0], right_indices[1], right_surface_vertices.index((intersection_points[1]))))
            
            if left_of_sagittal == 1:
                left_surface_triangles.append((left_indices[0], left_surface_vertices.index((intersection_points[0])), left_surface_vertices.index((intersection_points[1]))))
            else:
                left_surface_triangles.append((left_indices[0], left_surface_vertices.index((intersection_points[0])), left_surface_vertices.index((intersection_points[1]))))
                left_surface_triangles.append((left_indices[1], left_surface_vertices.index((intersection_points[0])), left_surface_vertices.index((intersection_points[1]))))
                left_surface_triangles.append((left_indices[0], left_indices[1], left_surface_vertices.index((intersection_points[0]))))
                left_surface_triangles.append((left_indices[0], left_indices[1], left_surface_vertices.index((intersection_points[1]))))
    
    return [left_surface_vertices, left_surface_triangles], [right_surface_vertices, right_surface_traingles]

import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import scipy
from skimage import measure
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib as mpl

#Load Data file
label_train00 = np.load('label_train00.npy').T

fig = plt.figure(figsize=(10,10))  # this is converting xx to xxxx ( datatype  = mls)
ax = fig.add_subplot(111, projection = '3d')
ax.voxels(label_train00, edgecolor = 'k')
plt.show()

vertices, triangles, _, _ = measure.marching_cubes(label_train00, 0)
fig = plt.figure()
ax = fig.add_subplot(111, projection = '3d')

vertices, faces, _, _ = measure.marching_cubes(label_train00, 0, spacing=(0.5,0.5,2))
mesh = Poly3DCollection(vertices[faces], alpha=0.6, edgecolor= 'k')
mesh.set_facecolor((0.5,0.5,0.5))
ax.add_collection(mesh)

fig = plt.figure()
ax.set_xlim(vertices[:,0].min(), vertices[:,0].max())
ax.set_ylim(vertices[:,1].min(), vertices[:,1].max())
ax.set_zlim(vertices[:,2].min(), vertices[:,2].max())
ax=fig.add_subplot(111, projection='3d')
ax.plot_trisurf(vertices[:,0], vertices[:,1], vertices[:,2], triangles= triangles, alpha=0.3, edgecolor = 'k')
plt.show()

#case 1
#vertices[vertices[:,0]< 31.5]
sagittal_plane = np.median(vertices[:,0].flatten())
triangulated_surface = [vertices, triangles]
left_surfaces, right_surfaces = surface_dividing(triangulated_surface, sagittal_plane = sagittal_plane+0.1)

left_vertices = np.asarray(left_surfaces[0], dtype=np.float64)
left_triangles = np.asarray(left_surfaces[1], dtype=np.int32)

right_vertices= np.asarray(right_surfaces[0], dtype=np.float64)
right_triangles= np.asarray(right_surfaces[1], dtype=np.int32)               

#left surface
fig = plt.figure()
ax= fig.add_subplot(111, projection='3d')
ax.plot_trisurf(left_vertices[:,0], left_vertices[:,1], left_vertices[:,2], triangles = left_triangles, alpha=1, cmap = 'copper')
plt.savefig('case1_left.png',dpi=300,bbox_inches='tight')
im = Image.open('case1_left.png')
im.save('case1_left.png',dpi=[300,300])
plt.show()


#right surface
fig = plt.figure()
ax= fig.add_subplot(111, projection='3d')
ax.plot_trisurf(right_vertices[:,0], right_vertices[:,1], right_vertices[:,2], triangles = right_triangles, alpha=1, cmap = 'copper')
plt.savefig('case1_right.png',dpi=300,bbox_inches='tight')
im = Image.open('case1_right.png')
im.save('case1_right.png',dpi=[300,300])
plt.show()

#Case2
sagittal_plane = np.median(vertices[850,0].flatten())
triangulated_surface = [vertices, triangles]
left_surfaces, right_surfaces = surface_dividing(triangulated_surface, sagittal_plane = sagittal_plane+0.1)

left_vertices = np.asarray(left_surfaces[0], dtype=np.float64)
left_triangles = np.asarray(left_surfaces[1], dtype=np.int32)

right_vertices= np.asarray(right_surfaces[0], dtype=np.float64)
right_triangles= np.asarray(right_surfaces[1], dtype=np.int32)               

#left surface case 2
fig = plt.figure()
ax= fig.add_subplot(111, projection='3d')
ax.plot_trisurf(left_vertices[:,0], left_vertices[:,1], left_vertices[:,2], triangles = left_triangles, alpha=1, cmap = 'bone')
plt.savefig('case2_left.png',dpi=300,bbox_inches='tight')
im = Image.open('case2_left.png')
im.save('case2_left.png',dpi=[300,300])
plt.show()

#right surface
fig = plt.figure()
ax= fig.add_subplot(111, projection='3d')
ax.plot_trisurf(right_vertices[:,0], right_vertices[:,1], right_vertices[:,2], triangles = right_triangles, alpha=1, cmap = 'bone')
plt.savefig('case2_right.png',dpi=300,bbox_inches='tight')
im = Image.open('case2_right.png')
im.save('case2_right.png',dpi=[300,300])
plt.show()

#Case 3
sagittal_plane = np.median(vertices[2000, 0].flatten())
triangulated_surface = [vertices, triangles]
left_surfaces, right_surfaces = surface_dividing(triangulated_surface, sagittal_plane = sagittal_plane+0.1)

left_vertices = np.asarray(left_surfaces[0], dtype=np.float64)
left_triangles = np.asarray(left_surfaces[1], dtype=np.int32)

right_vertices= np.asarray(right_surfaces[0], dtype=np.float64)
right_triangles= np.asarray(right_surfaces[1], dtype=np.int32)               

#left surface
fig = plt.figure()
ax= fig.add_subplot(111, projection='3d')
ax.plot_trisurf(left_vertices[:,0], left_vertices[:,1], left_vertices[:,2], triangles = left_triangles, alpha=1, cmap = 'copper')
plt.savefig('case3_left.png',dpi=300,bbox_inches='tight')
im = Image.open('case3_left.png')
im.save('case3_left.png',dpi=[300,300])
plt.show()

#right surface
fig = plt.figure()
ax= fig.add_subplot(111, projection='3d')
ax.plot_trisurf(right_vertices[:,0], right_vertices[:,1], right_vertices[:,2], triangles = right_triangles, alpha=1, cmap = 'copper')
plt.savefig('case3_right.png',dpi=300,bbox_inches='tight')
im = Image.open('case3_right.png')
im.save('case3_right.png',dpi=[300,300])
plt.show()
