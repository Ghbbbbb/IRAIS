import numpy as np
import time

from robopal.envs import RobotEnv
from robopal.robots.diana_med import DianaGraspMultiObjs

def primitive(func, checker=None):
    """ primitive flag, no practical effect. """

    def primitive_wrapper(*args, **kwargs):
        return func(*args, **kwargs)

    return primitive_wrapper


class GraspingEnv(RobotEnv):
    def __init__(self,
                 render_mode='human',
                 control_freq=100,
                 enable_camera_viewer=False,
                 controller='CARTIK',
                 is_interpolate=False,
                #  camera_name='cam'
                 ):

        robot = DianaGraspMultiObjs()
        super().__init__(
            robot=robot,
            render_mode=render_mode,
            control_freq=control_freq,
            enable_camera_viewer=enable_camera_viewer,
            controller=controller,
            is_interpolate=is_interpolate,
            # camera_name = camera_name
        )

        self.init_pos, self.init_rot = self.controller.forward_kinematics(self.robot.get_arm_qpos())
        self.action = self.init_pos

    @primitive
    def reset_robot(self):
        self.move(self.init_pos, self.init_rot)

    @primitive
    def get_obj_pose(self, obj_name):
        pos = self.mj_data.body(obj_name).xpos.copy()
        pos[2] -= 0.31
        quat = self.mj_data.body(obj_name).xquat.copy()
        return pos, quat

    @primitive
    def move(self, pos, quat):
        start_time = time.time()
        def checkArriveState(state):
            current_pos, current_quat = self.get_current_pose()
            error = np.sum(np.abs(state[:3] - current_pos)) + np.sum(np.abs(state[3:] - current_quat))
            error2 = 2*np.sum(np.abs(state[:3] - current_pos)) + np.sum(np.abs(state[3:] - current_quat)/2.5)        #跑仿真效果时用
            end_time = time.time()
            elapsed_time  = end_time - start_time
            if error2 <= 0.05 or elapsed_time  > 30:
                return True
            return False

        while True:
            self.action = np.concatenate((pos, quat), axis=0)
            self.step(self.action)
            if self.render_mode == "human":
                self.render()
            if checkArriveState(self.action):
                break

    @primitive
    def grab(self, obj_name):
        self.gripper_ctrl("open")
        obj_pos, obj_quat = self.get_obj_pose(obj_name)
        self.move(np.add(obj_pos, np.array([0, 0, 0.1])), obj_quat)
        self.move(obj_pos, obj_quat)
        self.gripper_ctrl("close")
        end_pos, end_quat = self.get_current_pose()
        self.move(np.add(end_pos, np.array([0, 0, 0.1])), end_quat)

    @primitive
    def gripper_ctrl(self, cmd: str):
        if cmd == "open":
            self.mj_data.actuator("0_gripper_l_finger_joint").ctrl = 20
        elif cmd == "close":
            self.mj_data.actuator("0_gripper_l_finger_joint").ctrl = -20
        step = 0
        while True:
            step += 1
            self.step(self.action)
            if self.render_mode == "human":
                self.render()
            if step > 100:
                break
    @primitive       
    def get_gripper_status(self):
        """
        获取夹爪状态。
        
        :return: 夹爪状态，例如 "open" 或 "close"。
        """

        if self.mj_data.actuator("0_gripper_l_finger_joint").ctrl == 20:
            return "open"
        elif self.mj_data.actuator("0_gripper_l_finger_joint").ctrl == -20:
            return "close"
        else:
            return "initial"

    @primitive
    def get_current_pose(self):
        return self.controller.forward_kinematics(self.robot.get_arm_qpos())
    
    @primitive
    def say(self,conversation):
        print(conversation)


######################env1_skill_update#####################
    @primitive
    def move_gripper_orientation(self,direction,distance):
        """
        This function moves the gripper to the corresponding position based on direction and distance. The direction is chosen from 'forward', 'backward', 'left', 'right', 'upward', 'downward', and the distance is measured in centimeters.
        """
        current_pos, current_quat = self.get_current_pose()
        
        direction_map = {
            'forward': np.array([1, 0, 0]),
            'backward': np.array([-1, 0, 0]),
            'left': np.array([0, 1, 0]),
            'right': np.array([0, -1, 0]),
            'upward': np.array([0, 0, 1]),
            'downward': np.array([0, 0, -1])
        }
        
        if direction in direction_map:
            move_vector = direction_map[direction] * distance
        else:
            raise ValueError("Invalid direction")
        
        target_pos = current_pos + move_vector
        self.move(target_pos, current_quat)

    @primitive
    def stack_object_on_object(self, top_object_name, base_object_name):
        """
        This function stack top_object on base_object. It grabs the top_object, moves it above the base_object, and 
    then places it on the base_object.
        """
        # Grab the top block
        self.grab(top_object_name)

        # Get the position and quaternion of the base block
        base_pos, base_quat = self.get_obj_pose(base_object_name)

        # Calculate the new position for placing the top block on the base block
        new_pos = np.array([base_pos[0], base_pos[1], base_pos[2]])
        if "mug" in base_object_name:
            # Move the gripper (with the top block) to the position above the base block
            self.move(np.add(new_pos, np.array([0, 0, 0.15])), base_quat)
        else:
            self.move(np.add(new_pos, np.array([0, 0, 0.1])), base_quat)

        # # Place the top block on the base block
        # self.move(new_pos, base_quat)

        # Release the top block
        self.gripper_ctrl("open")

        # # Move the gripper slightly up after placing the block
        # self.move(np.add(new_pos, np.array([0, 0, 0.1])), base_quat)

    @primitive
    def pick_and_place_next_to(self, obj_to_pick, obj_reference, direction, distance):
        """
        Picks up an object and places it next to another object in the specified direction and distance.        

        Parameters:
            obj_to_pick (str): Name of the object to pick.
            obj_reference (str): Name of the reference object next to which the picked object will be placed.   
            direction (str): Direction to place the picked object relative to the reference object. Options: 'left', 'right', 'forward', 'backward'.
            distance (float): Distance in meters from the reference object.
        """
        self.grab(obj_to_pick)
        reference_pos, reference_quat = self.get_obj_pose(obj_reference)

        direction_map = {
            'left': np.array([0, 1, 0]),
            'right': np.array([0, -1, 0]),
            'forward': np.array([1, 0, 0]),
            'backward': np.array([-1, 0, 0])
        }

        if direction in direction_map:
            displacement = direction_map[direction] * distance
        else:
            raise ValueError("Invalid direction")

        place_position = reference_pos + displacement
        self.move(np.add(place_position, np.array([0, 0, 0.1])), reference_quat)
        self.gripper_ctrl("open")
    

def make_env():
    env = GraspingEnv(
        render_mode="human",
        control_freq=200,
    )
    return env
