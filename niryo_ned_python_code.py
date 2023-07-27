from niryo_robot_python_ros_wrapper import *
import rospy

# observation pose
observation_pose = (-0.032,-0.168,0.184,-2.451,1.36,2.341)

# give pose
give_pose = (-0.028,-0.35,0.364,-0.002,0.651,-1.566)

# place pose on conveyor belt
place_pose = (0.236,-0.003,0.104,0.477,1.548,0.519)


ascs = "default_workspace_12"

# pick scissor pose
pick_scissor = (0.053, -0.426,0,-0.616,1.488,-2.12)

# pick knife pose
pick_knife = (-0.021,-0.423,0.002,-1.02,1.514,-2.674)

# pick injection pose
pick_injection = (-0.1,-0.421,-0.001,-0.967,1.514,-2.541)

def process(ned2):

    robot.move_pose(*observation_pose)
    robot.wait(2)
    ret = robot.get_target_pose_from_cam(ascs,height_offset=-0.2,shape=ObjectShape.ANY,color=ObjectColor.ANY)
    obj_found,obj_pose, obj_shape,obj_color = ret
    robot.open_gripper()
    if obj_shape == "SQUARE" and obj_color == "RED":
        robot.move_pose(*pick_scissor)
        robot.pick_from_pose(*pick_scissor)
        robot.place_from_pose(*give_pose)                
        
    elif obj_shape == "SQUARE" and obj_color == "BLUE":
        robot.move_pose(*pick_knife)
        robot.pick_from_pose(*pick_knife)
        robot.place_from_pose(*give_pose)
        
    elif obj_shape == "SQUARE" and obj_color == "GREEN":
        robot.move_pose(*pick_injection)
        robot.pick_from_pose(*pick_injection)
        robot.place_from_pose(*give_pose)

    elif obj_shape == "CIRCLE" and obj_color == "BLUE":
        robot.wait(6)
        # Activating connection with conveyor and storing ID
        conveyor_id = robot.set_conveyor()

        # Running conveyor at 50% of its maximum speed, in Forward direction
        robot.control_conveyor(conveyor_id, True, 50, ConveyorDirection.FORWARD)
        robot.move_pose(*give_pose)
        robot.pick_from_pose(*give_pose)
        robot.place_from_pose(*place_pose)

        # Stopping robot motor
        robot.control_conveyor(conveyor_id, True, 0, ConveyorDirection.FORWARD)
        
        # Deactivating connexion with conveyor
        robot.unset_conveyor(conveyor_id)

        robot.set_learning_mode(True)
        

if __name__ == '__main__':

    # Connect to robot
    rospy.init_node('niryo_ned_example_python_ros_wrapper')

    robot = NiryoRosWrapper()

    # Changing tool
    robot.update_tool()

    # Calibrate robot if robot needs calibration
    robot.calibrate_auto()
    
    # Launching main process
    for i in range(5):
        process(robot)
    # Ending

