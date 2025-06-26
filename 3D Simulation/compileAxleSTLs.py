import pyvista as pv
import numpy as np
import argparse

def show_mesh(mesh):
    # shows mesh
    plotter = pv.Plotter()
    plotter.add_mesh(mesh, color="lightblue", show_edges=True) 
    plotter.add_axes()  
    plotter.show_grid()  
    plotter.view_yx()
    plotter.show()

def in_to_mm(inches):
    return inches*25.4

def move_item_to_center(mesh):
    meshLocation = np.array([mesh.center[0], mesh.center[1], mesh.center[2]])
    mesh.translate(-meshLocation, inplace=True)
    return mesh

def get_bounds(mesh, a, b):
    return mesh.bounds[a] - mesh.bounds[b]

def generate_full_mesh(components_directory, hub_file, bracket_file, shaft_file, arm_file, 
                       arm_angle, camber, toe, bracket_depth, bracket_bias, spindle_len, show=False):
    # load in stls
    hubMesh = pv.read(components_directory + hub_file)
    bracketMesh = pv.read(components_directory + bracket_file)
    shaftMesh = pv.read(components_directory + shaft_file)
    armMesh = pv.read(components_directory + arm_file)
    
    # centering all meshes at the origin
    bracketMesh = move_item_to_center(bracketMesh)
    hubMesh = move_item_to_center(hubMesh)
    armMesh = move_item_to_center(armMesh)
    shaftMesh = move_item_to_center(shaftMesh)
    
    # correcting for rotation in original models so all meshes are properly oriented
    hubMesh.rotate_x(270, inplace=True)
    armMesh.rotate_y(180, inplace=True)
    bracketMesh.rotate_z(270,inplace=True)
    bracketMesh.rotate_x(180, inplace=True)
    
    # it'll be better if i can somehow extract all these values with software
    arm_diameter = get_bounds(armMesh,3,2)
    arm_len = get_bounds(armMesh,1,0)
    arm_thickness = get_bounds(armMesh,5,4)
    hub_diameter = get_bounds(hubMesh,1,0)
    hub_thickness = get_bounds(hubMesh,5,4)
    shaft_length = get_bounds(shaftMesh,5,4)
    bracket_thickness = get_bounds(bracketMesh,5,4)
    bracket_bias = in_to_mm(bracket_bias)
    
    # positioning arm 
    radians = arm_angle * np.pi / 180
    armMesh.rotate_z(arm_angle, inplace=True)
    radius = arm_len - arm_diameter
    a = radius * np.cos(radians)
    b = radius * np.sin(radians)
    armMesh.translate([a/2, b/2, arm_thickness], inplace=True)
    
    # positioning shaft and bracket 
    shaftMesh.translate([a, b, shaft_length/2+arm_thickness/2], inplace=True) 
    bracketMesh.translate([a, b + bracket_bias, (spindle_len+bracket_thickness+arm_thickness)/2 
                           + bracket_depth], inplace=True)
    
    # positioning hub
    hubMesh.translate([0,0,-spindle_len], inplace=True)
    hubMesh.rotate_y(-camber, inplace=True) # camber
    hubMesh.rotate_x(-toe, inplace=True) # toe

    # creating and positioning spindle
    spindleMesh = pv.Cylinder(radius = 20, height = spindle_len, center=(0, 0, 0), direction=(0, 0, 1))
    
    # show completed mesh
    if (show):
        show_mesh(hubMesh + armMesh + shaftMesh + bracketMesh + spindleMesh)

    # return
    return hubMesh + armMesh + shaftMesh + bracketMesh + spindleMesh

if __name__ == '__main__':
    generate_full_mesh('./', 'SimplifiedHub.STL', 'Bracket.STL', 'Shaft.STL', 'CrankArm7.STL', 
                       arm_angle=36, camber=0, toe=0, bracket_depth=180, bracket_bias=2.5/2, spindle_len=100,
                       show=True) 
