#!/usr/bin/env python3

"""
ROS2 Isaac Sim Dual Arm Head Robot
Simple ROS2 framework to load and control dualarm_head.usd
"""

from isaacsim import SimulationApp

# Create simulation app
simulation_app = SimulationApp({"headless": False})
import numpy as np
import os
import threading
import omni.replicator.core as rep
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.extensions import enable_extension

from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation

import omni.graph.core as og

import omni.ui as ui

import omni.isaac.sensor

from isaacsim.core.api.objects import DynamicCuboid
from pxr import UsdPhysics, UsdGeom, Gf

import omni.kit.commands
from isaacsim.sensors.physics.impl.effort_sensor import EffortSensor
import random 

# Added ROS2 imports
try:
    import rclpy
    from sensor_msgs.msg import JointState
except Exception:
    rclpy = None
    JointState = None

def is_physics_rigid_body(prim):
    try:
        return UsdPhysics.RigidBodyAPI(prim).GetRigidBodyEnabledAttr().Get() is True
    except Exception:
        return False

# Global variables for user commands and home position
user_command = ""
home_positions = None
is_homing = False
homing_progress = 0.0
start_positions = None

# Randomization parameters for objects
can_RANDOMIZE_RANGE_X = (0.6, 0.7)  # meters
can_RANDOMIZE_RANGE_Y = (-0.15, -0.25)  # meters
can_RANDOMIZE_RANGE_Z = (0.9, 0.9)   # keep Z fixed or set range as needed



box_RANDOMIZE_RANGE_X = (0.65, 0.85)  # meters
box_RANDOMIZE_RANGE_Y = (0.25, 0.35)  # meters
box_RANDOMIZE_RANGE_Z = (0.9, 0.9)   # keep Z fixed or set range as needed

# Randomization parameters for lighting
LIGHT_INTENSITY_RANGE = (1.0, 4.0)  # light intensity range
LIGHT_COLOR_TEMP_RANGE = (6000.0, 8000.0)  # color temperature in Kelvin (warm to cool)
LIGHT_POSITION_OFFSET = 2.0  # max position offset in meters

def randomize_lighting():
    """Randomize lighting conditions in the scene"""
    from omni.isaac.core.utils.prims import get_prim_at_path
    from pxr import UsdLux, Sdf
    import omni.usd
    
    stage = omni.usd.get_context().get_stage()
    
    # Try to find existing lights in the scene
    lights_found = []
    for prim in stage.Traverse():
        if prim.IsA(UsdLux.DistantLight) or prim.IsA(UsdLux.SphereLight) or prim.IsA(UsdLux.DomeLight):
            lights_found.append(str(prim.GetPath()))
    
    # If no lights found, create a distant light
    if not lights_found:
        light_path = "/World/DefaultLight"
        omni.kit.commands.execute(
            'CreatePrimWithDefaultXform',
            prim_type='DistantLight',
            prim_path=light_path
        )
        lights_found.append(light_path)
        print(f"Created default light at {light_path}")
    
    # Randomize each light
    for light_path in lights_found:
        prim = get_prim_at_path(light_path)
        if not prim or not prim.IsValid():
            continue
            
        # Randomize intensity
        intensity = random.uniform(LIGHT_INTENSITY_RANGE[0], LIGHT_INTENSITY_RANGE[1])
        
        # Randomize color temperature
        color_temp = random.uniform(LIGHT_COLOR_TEMP_RANGE[0], LIGHT_COLOR_TEMP_RANGE[1])
        
        # Set light properties
        if prim.IsA(UsdLux.DistantLight):
            light = UsdLux.DistantLight(prim)
            light.GetIntensityAttr().Set(intensity)
            if light.GetColorTemperatureAttr():
                light.GetEnableColorTemperatureAttr().Set(True)
                light.GetColorTemperatureAttr().Set(color_temp)
            
            # Randomize light angle/rotation
            random_rot_x = random.uniform(-30, 30)
            random_rot_y = random.uniform(-30, 30)
            random_rot_z = random.uniform(-180, 180)
            
            omni.kit.commands.execute(
                'TransformPrimSRTCommand',
                path=light_path,
                new_rotation_euler=Gf.Vec3d(random_rot_x, random_rot_y, random_rot_z)
            )
            
        elif prim.IsA(UsdLux.SphereLight):
            light = UsdLux.SphereLight(prim)
            light.GetIntensityAttr().Set(intensity)
            if light.GetColorTemperatureAttr():
                light.GetEnableColorTemperatureAttr().Set(True)
                light.GetColorTemperatureAttr().Set(color_temp)
            
            # Randomize sphere light position
            xformable = UsdGeom.Xformable(prim)
            xform_ops = xformable.GetOrderedXformOps()
            current_translate = Gf.Vec3d(0, 0, 2)
            for op in xform_ops:
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    current_translate = op.Get()
                    break
            
            new_x = current_translate[0] + random.uniform(-LIGHT_POSITION_OFFSET, LIGHT_POSITION_OFFSET)
            new_y = current_translate[1] + random.uniform(-LIGHT_POSITION_OFFSET, LIGHT_POSITION_OFFSET)
            new_z = current_translate[2] + random.uniform(-LIGHT_POSITION_OFFSET/2, LIGHT_POSITION_OFFSET/2)
            
            omni.kit.commands.execute(
                'TransformPrimSRTCommand',
                path=light_path,
                new_translation=Gf.Vec3d(new_x, new_y, new_z)
            )
            
        elif prim.IsA(UsdLux.DomeLight):
            light = UsdLux.DomeLight(prim)
            light.GetIntensityAttr().Set(intensity * 0.5)  # Dome lights are usually brighter
            if light.GetColorTemperatureAttr():
                light.GetEnableColorTemperatureAttr().Set(True)
                light.GetColorTemperatureAttr().Set(color_temp)
        
        print(f"Randomized light {light_path}: intensity={intensity:.1f}, color_temp={color_temp:.0f}K")
    
    return len(lights_found) > 0

def randomize_object_position(prim_path, base_position=None):
    """Randomize the position of an object within defined ranges"""
    from omni.isaac.core.utils.prims import get_prim_at_path
    
    prim = get_prim_at_path(prim_path)
    if prim and prim.IsValid():
        xformable = UsdGeom.Xformable(prim)
        
        # Get current position if base_position not provided
        if base_position is None:
            xform_ops = xformable.GetOrderedXformOps()
            current_translate = None
            for op in xform_ops:
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    current_translate = op.Get()
                    break
            
            if current_translate is None:
                # If no translate op exists, use origin
                base_position = [0, 0, 0]
            else:
                base_position = [current_translate[0], current_translate[1], current_translate[2]]
        if prim_path.endswith("_05_tomato_soup_can"):
            RANDOMIZE_RANGE_X = can_RANDOMIZE_RANGE_X
            RANDOMIZE_RANGE_Y = can_RANDOMIZE_RANGE_Y
            RANDOMIZE_RANGE_Z = can_RANDOMIZE_RANGE_Z
        elif prim_path.endswith("box"):
            RANDOMIZE_RANGE_X = box_RANDOMIZE_RANGE_X
            RANDOMIZE_RANGE_Y = box_RANDOMIZE_RANGE_Y
            RANDOMIZE_RANGE_Z = box_RANDOMIZE_RANGE_Z
        # Generate random offsets
        random_x = random.uniform(RANDOMIZE_RANGE_X[0], RANDOMIZE_RANGE_X[1])
        random_y = random.uniform(RANDOMIZE_RANGE_Y[0], RANDOMIZE_RANGE_Y[1])
        random_z = random.uniform(RANDOMIZE_RANGE_Z[0], RANDOMIZE_RANGE_Z[1])
        
        # Calculate new position
        new_position = Gf.Vec3d(
            random_x,
            random_y,
            random_z
        )
        
        # Apply new position using omni commands
        omni.kit.commands.execute(
            'TransformPrimSRTCommand',
            path=prim_path,
            new_translation=new_position
        )
        
        print(f"Randomized {prim_path} to position: ({new_position[0]:.3f}, {new_position[1]:.3f}, {new_position[2]:.3f})")
        return True
    else:
        print(f"Warning: Could not find prim at {prim_path}")
        return False

def input_thread():
    """Thread function to handle user input"""
    global user_command
    while True:
        try:
            command = input("Enter command ('homing' to move to initial position, 'exit' to quit): ").strip().lower()
            user_command = command
            if command == 'exit':
                break
        except:
            break

def camera_action_graph():
    try:
        og.Controller.edit(
            {"graph_path": "/camera_ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("RenderProduct1", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                    ("RenderProduct2", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                    ("RGBPublish1","isaacsim.ros2.bridge.ROS2CameraHelper"),
                    ("DepthPublish1","isaacsim.ros2.bridge.ROS2CameraHelper"),
                    ("RGBPublish2","isaacsim.ros2.bridge.ROS2CameraHelper"),
                    ("DepthPublish2","isaacsim.ros2.bridge.ROS2CameraHelper"),

                ],
            
                og.Controller.Keys.SET_VALUES: [
                    #Camera Front
                    ("RenderProduct1.inputs:cameraPrim","/World/Camera_front"),
                    ("RenderProduct1.inputs:width", 640),
                    ("RenderProduct1.inputs:height", 480),
                    ("RGBPublish1.inputs:type", "rgb"),
                    ("RGBPublish1.inputs:topicName", "isaac_camera_front/rgb"),
                    ("DepthPublish1.inputs:type", "depth"),
                    ("DepthPublish1.inputs:topicName", "isaac_camera_front/depth"),

                    # Camera Top
                    ("RenderProduct2.inputs:cameraPrim", "/World/aidin_dsr_dualarm/dxl_head/head_link/Camera_top"),
                    ("RenderProduct2.inputs:width", 640),
                    ("RenderProduct2.inputs:height", 480),
                    ("RGBPublish2.inputs:type", "rgb"),
                    ("RGBPublish2.inputs:topicName", "isaac_camera_top/rgb"),
                    ("DepthPublish2.inputs:type", "depth"),
                    ("DepthPublish2.inputs:topicName", "isaac_camera_top/depth"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "RenderProduct1.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "RenderProduct2.inputs:execIn"),
                    # Render at Publish (Camera front)
                    ("RenderProduct1.outputs:execOut", "RGBPublish1.inputs:execIn"),
                    ("RenderProduct1.outputs:execOut", "DepthPublish1.inputs:execIn"),
                    ("RenderProduct1.outputs:renderProductPath", "RGBPublish1.inputs:renderProductPath"),
                    ("RenderProduct1.outputs:renderProductPath", "DepthPublish1.inputs:renderProductPath"),
                    # Render at Publish (Camera top)
                    ("RenderProduct2.outputs:execOut", "RGBPublish2.inputs:execIn"),
                    ("RenderProduct2.outputs:execOut", "DepthPublish2.inputs:execIn"),
                    ("RenderProduct2.outputs:renderProductPath", "RGBPublish2.inputs:renderProductPath"),
                    ("RenderProduct2.outputs:renderProductPath", "DepthPublish2.inputs:renderProductPath"),
                ]
            },
        )
    except Exception as e:
        print(f"[ERROR] Failed to setup contact sensor action graph: {e}")












def main():
    """Main function to load USD file with homing functionality"""
    
    global user_command
    
    # Enable extensions (using correct names)
    enable_extension("isaacsim.ros2.bridge") 
    enable_extension("omni.graph.window.action") 

    # Create world
    my_world = World()
    # Load USD file
    usd_path= "/home/vision/isaacsim/dualarm_test.usd"

    dualarm_head = None
    if os.path.exists(usd_path):
        # Add USD to stage
        prim_path = "/World"
        add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)


        # Create articulation 
        dualarm_head = Articulation(prim_path=prim_path, name="dualarm_head")
        my_world.scene.add(dualarm_head)
        
        print(f"Loaded USD file: {usd_path}")

    else:
        print(f"USD file not found: {usd_path}")
    
    # Reset world to initialize everything
    my_world.reset()
    camera_action_graph()
    
    # Randomize object positions and lighting on reset
    randomize_object_position("/World/aidin_dsr_dualarm/Environment/_05_tomato_soup_can")
    randomize_object_position("/World/aidin_dsr_dualarm/Environment/box")
    randomize_lighting()

    
    # Read and store initial joint positions as home position
    if dualarm_head is not None:
        # Wait a moment for the robot to settle
        for _ in range(10):
            my_world.step(render=False)
        
        # Get initial joint positions
        initial_positions = dualarm_head.get_joint_positions()
        
        global home_positions
        home_positions = list(initial_positions)

   

    # Start input thread for user commands
    input_thread_obj = threading.Thread(target=input_thread, daemon=True)
    input_thread_obj.start()
    
    print("Enter 'h' to homing robot smoothly to initial home position, 'exit' to quit")

    # Track simulation state for randomization on stop/restart
    prev_is_playing = my_world.is_playing()
    
    # Main simulation loop
    while simulation_app.is_running():
        my_world.step(render=True)
        
        # Detect simulation restart (stop -> play transition)
        current_is_playing = my_world.is_playing()
        if current_is_playing and not prev_is_playing:
            # Simulation just restarted, randomize object positions and lighting
            randomize_object_position("/World/aidin_dsr_dualarm/Environment/_05_tomato_soup_can")
            randomize_object_position("/World/aidin_dsr_dualarm/Environment/box")
            randomize_lighting()
        prev_is_playing = current_is_playing
        
       
        # Handle smooth homing motion
        global is_homing, homing_progress, start_positions
        if is_homing and dualarm_head is not None and home_positions is not None and start_positions is not None:
            homing_progress += 0.02  # Adjust speed (0.02 = ~2.5 seconds for full motion)
            if homing_progress >= 1.0:
                dualarm_head.set_joint_positions(home_positions)
                is_homing = False
                homing_progress = 0.0
                print("Robot reached home position")
            else:
                current_target = []
                for i in range(len(home_positions)):
                    interpolated_pos = start_positions[i] + homing_progress * (home_positions[i] - start_positions[i])
                    current_target.append(interpolated_pos)
                dualarm_head.set_joint_positions(current_target)

        # Check for user commands
        if user_command == "h" and dualarm_head is not None and home_positions is not None and not is_homing:
            print("Starting smooth homing motion...")
            current_positions = dualarm_head.get_joint_positions()
            if current_positions is not None:
                start_positions = list(current_positions)
                is_homing = True
                homing_progress = 0.0
                print(f"Moving from: {start_positions}")
                print(f"Moving to: {home_positions}")
            else:
                print("Could not get current joint positions")
            user_command = ""
        elif user_command == "exit":
            print("Exiting simulation...")
            break
    simulation_app.close()


if __name__ == "__main__":
    main()
