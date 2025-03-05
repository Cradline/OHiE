#!/usr/bin/env python3
# encoding: utf-8
import pygame
import time
import math
from common.ros_robot_controller_sdk import Board
from kinematics.arm_move_ik import ArmIK
# from kinematics.IK_servo import ArmIK

# Controller klasse 
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
            print("Kontroller tilkoblet!")
        except:
            print("Ingen kontroller tilkoblet!")

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
        
        # Initial posisjon (sentrert)
        self.current_pos = [0.0, 6.0, 18.0]  # X, Y, Z
        self.current_pitch = 0.0
        self.gripper_open = False
        self.speed = 0.5        # cm per oppdatering
        self.pitch_speed = 1.0  # grader per oppdatering

    def update_position(self, dx, dy, dz):
        # Oppdaterer posisjon med hastighetsskalering
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
            -90, 90,    # Pitch range
            200         # Bevegelsestid [ms]
        )
        return result is not None

def main():
    controller = XboxController()
    arm = ArmController()
    
    print("Starter ArmController...")
    print("Høyre stick: X/Y bevegelse")
    print("Venstre stick Y: Z bevegelse")
    print("LB/RB: Endrer pitch")
    print("A/B: Åpner/lukker griper")
    print("Start: Avslutter program")

    try:
        while True:
            if not controller.joystick:
                controller.initialize_controller()
                time.sleep(1)
                continue

            inputs = controller.get_inputs()
            
            # Avslutter programmet ved å trykke 'start'
            if inputs['start']:
                break
            
            # Endring i posisjon
            dx = inputs['right_x']
            dy = -inputs['right_y']  # Invertert Y-axse
            dz = -inputs['left_y']   # Z axis fra venstre stick
            
            arm.update_position(dx, dy, dz)
            
            # Endring i pitch (bumpers)
            if inputs['lb']:
                arm.update_pitch(-1)
            if inputs['rb']:
                arm.update_pitch(1)
            
            # Endring i griperen (A & B)
            if inputs['a_button'] and not arm.gripper_open:
                arm.control_gripper(True)
            if inputs['b_button'] and arm.gripper_open:
                arm.control_gripper(False)
            
            # Kaller bevegelsesfunksjon
            if arm.move_arm():
                print(f"Posisjon: {arm.current_pos} | Pitch: {arm.current_pitch:.1f}°")
            else:
                print("Ugyldig posisjon!")
            
            time.sleep(0.1)  # main loop hastighetskontroll i sekunder

    except KeyboardInterrupt:
        print("Avslutter...")
    finally:
        pygame.quit()

if __name__ == "__main__":
    main()