import numpy as np
from stl import mesh
from mpl_toolkits import mplot3d
from matplotlib import pyplot as plt

# https://pypi.org/project/numpy-stl/

# Create a new plot
figure = plt.figure()
axes = mplot3d.Axes3D(figure)

# Using an existing stl file:
#my_mesh = mesh.Mesh.from_file('Cube (text).stl')
my_mesh = mesh.Mesh.from_file('Frog.stl')
axes.add_collection3d(mplot3d.art3d.Poly3DCollection(my_mesh.vectors))

# Auto scale to the mesh size
scale = my_mesh.points.flatten()
axes.auto_scale_xyz(scale, scale, scale)

print(len(my_mesh.vectors))

# Show the plot to the screen
plt.show()