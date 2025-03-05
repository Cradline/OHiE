#!/usr/bin/env python3
# encoding: utf-8
import pygame
import time
import math
from common.ros_robot_controller_sdk import Board
from kinematics.arm_move_ik import ArmIK
# from kinematics.IK_servo import ArmIK

# Controller klasse med pygame input
class XboxController:
    def __init__(self):
        pygame.init()
        pygame.joystick.init()
        self.joystick = None
        self.initialize_controller()

    def initialize_controller(self):
        try:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            print("Xbox controller connected!")
        except:
            print("No controller detected!")

    def get_inputs(self):
        pygame.event.pump()
        inputs = {
            # Right analog stick (X/Y axes)
            'right_x': self.joystick.get_axis(3),
            'right_y': self.joystick.get_axis(4),
            
            # Left analog stick (for Z-axis)
            'left_y': self.joystick.get_axis(1),
            
            # Buttons (A/B for gripper, bumpers for pitch)
            'a_button': self.joystick.get_button(0),
            'b_button': self.joystick.get_button(1),
            'lb': self.joystick.get_button(4),
            'rb': self.joystick.get_button(5),
            
            # Start button for exit
            'start': self.joystick.get_button(7)
        }
        return inputs

class ArmController:
    def __init__(self):
        self.board = Board()
        self.board.enable_reception()
        self.arm_ik = ArmIK()
        self.arm_ik.board = self.board
        
        # Initial position (centered)
        self.current_pos = [0.0, 6.0, 18.0]  # X, Y, Z
        self.current_pitch = 0.0
        self.gripper_open = False
        self.speed = 0.5  # cm per update
        self.pitch_speed = 1.0  # degrees per update

    def update_position(self, dx, dy, dz):
        # Update position with speed scaling
        self.current_pos[0] += dx * self.speed
        self.current_pos[1] += dy * self.speed
        self.current_pos[2] += dz * self.speed
        
        # Begrenser arbeidsområde til gyldige posisjoner
        self.current_pos[0] = max(-10, min(10, self.current_pos[0]))  # X limits
        self.current_pos[1] = max(5, min(20, self.current_pos[1]))     # Y limits
        self.current_pos[2] = max(10, min(25, self.current_pos[2]))     # Z limits

    def update_pitch(self, delta):
        # Oppdaterer pitch til griper
        self.current_pitch += delta * self.pitch_speed
        self.current_pitch = max(-90, min(90, self.current_pitch))

    def control_gripper(self, open_gripper):
        # Åpner/lukker griperen
        position = 1300 if open_gripper else 1700
        self.board.pwm_servo_set_position(0.5, [[1, position]])
        self.gripper_open = open_gripper

    def move_arm(self):
        result = self.arm_ik.setPitchRangeMoving(
            tuple(self.current_pos),
            self.current_pitch,
            -90, 90,  # Pitch range
            200  # Movement time in ms
        )
        return result is not None

def main():
    controller = XboxController()
    arm = ArmController()
    
    print("Starting arm control...")
    print("Right stick: X/Y movement")
    print("Left stick Y: Z movement")
    print("LB/RB: Adjust pitch")
    print("A/B: Open/Close gripper")
    print("Start: Exit program")

    try:
        while True:
            if not controller.joystick:
                controller.initialize_controller()
                time.sleep(1)
                continue

            inputs = controller.get_inputs()
            
            # Handle exit
            if inputs['start']:
                break
            
            # Handle movement
            dx = inputs['right_x']
            dy = -inputs['right_y']  # Invert Y axis
            dz = -inputs['left_y']   # Z axis from left stick
            
            arm.update_position(dx, dy, dz)
            
            # Handle pitch adjustment
            if inputs['lb']:
                arm.update_pitch(-1)
            if inputs['rb']:
                arm.update_pitch(1)
            
            # Handle gripper
            if inputs['a_button'] and not arm.gripper_open:
                arm.control_gripper(True)
            if inputs['b_button'] and arm.gripper_open:
                arm.control_gripper(False)
            
            # Send movement command
            if arm.move_arm():
                print(f"Position: {arm.current_pos} | Pitch: {arm.current_pitch:.1f}°")
            else:
                print("Invalid position!")
            
            time.sleep(0.1)  # Control loop rate

    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        pygame.quit()

if __name__ == "__main__":
    main()