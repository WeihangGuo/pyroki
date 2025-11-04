"""Basic IK

Simplest Inverse Kinematics Example using PyRoki.
"""

import time

import numpy as np
import pyroki as pk
import viser
from robot_descriptions.loaders.yourdfpy import load_robot_description
from viser.extras import ViserUrdf

import pyroki_snippets as pks
import yourdfpy 

def main():
    """Main function for basic IK."""
    urdf_path = "examples/assets/franka/fp3_franka_hand.urdf"
    mesh_dir = "examples/assets/franka/meshes"
    urdf = yourdfpy.URDF.load(urdf_path, mesh_dir=mesh_dir)
    target_link_name = "fp3_hand"
    # urdf = load_robot_description("ur5_description")
    # target_link_name = "ee_link"
    # Create robot.
    robot = pk.Robot.from_urdf(urdf)

    # Set up visualizer.
    server = viser.ViserServer()
    server.scene.add_grid("/ground", width=2, height=2)
    urdf_vis = ViserUrdf(server, urdf, root_node_name="/base")

    # Add the collision mesh to the visualizer.
    robot_coll = pk.collision.RobotCollision.from_urdf(urdf)

    # Create interactive controller with initial position.
    ik_target = server.scene.add_transform_controls(
        "/ik_target", scale=0.2, position=(0.61, 0.0, 0.56), wxyz=(0, 0, 1, 0)
    )
    timing_handle = server.gui.add_number("Elapsed (ms)", 0.001, disabled=True)

    while True:
        # Solve IK.
        start_time = time.time()
        solution = pks.solve_ik(
            robot=robot,
            target_link_name=target_link_name,
            target_position=np.array(ik_target.position),
            target_wxyz=np.array(ik_target.wxyz),
        )

        # Update timing handle.
        elapsed_time = time.time() - start_time
        timing_handle.value = 0.99 * timing_handle.value + 0.01 * (elapsed_time * 1000)

        # Update visualizer.
        urdf_vis.update_cfg(solution)
        # Update the collision mesh.
        robot_coll_mesh = robot_coll.at_config(robot, solution).to_trimesh()
        server.scene.add_mesh_trimesh("/robot/collision", mesh=robot_coll_mesh)
if __name__ == "__main__":
    main()
