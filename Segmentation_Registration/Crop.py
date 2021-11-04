
import open3d as o3d

box = "box"
filename = f"data/OtherBoxes/{box}.ply"
pointCloud = o3d.io.read_point_cloud(filename)
o3d.visualization.draw_geometries_with_editing([pointCloud])