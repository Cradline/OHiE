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
            if pygame.joystick.get_count() > 0:
                self.joystick = pygame.joystick.Joystick(0)
                self.joystick.init()
                print("Kontroller tilkoblet!")
            else:
                print("Ingen kontroller tilkoblet!")
        except Exception as e:
            print(f"Kontrollerfeil: {e}")

    def apply_deadzone(self, value):
        return value if abs(value) > self.deadzone else 0.0

    def get_inputs(self):
        inputs = {}
        pygame.event.pump()
        try:
            inputs = {
                # Right analog stick (X/Y axes)
                'right_x': self.joystick.get_axis(2),
                'right_y': self.joystick.get_axis(3),
                
                # Left analog stick (for Z-axis)
                'left_y': self.joystick.get_axis(1),

                # Triggers
                'RT': self.joystick.get_axis(4),
                'LT': self.joystick.get_axis(5),
                
                # Buttons (A/B for gripper, lb/rb for pitch)
                'a_button': self.joystick.get_button(0),
                'b_button': self.joystick.get_button(1),
                'x_button': self.joystick.get_button(3),
                'y_button': self.joystick.get_button(4),
                'lb': self.joystick.get_button(6),
                'rb': self.joystick.get_button(7),
                
                # Start button for exit
                'start': self.joystick.get_button(11),
                'select': self.joystick.get_button(10)
            }
        except AttributeError:
            self.initialize_controller()
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
        '''self.speed = 0.5        # cm per oppdatering
        self.pitch_speed = 1.0  # grader per oppdatering'''

        # Sensitivitet
        self.xy_sensitivity = 0.5  # lavere = tregere
        self.z_sensitivity = 0.5
        self.pitch_sensitivity = 0.5

    def update_position(self, dx, dy, dz):
        # Oppdaterer posisjon med hastighetsskalering
        '''self.current_pos[0] += dx * self.speed
        self.current_pos[1] += dy * self.speed
        self.current_pos[2] += dz * self.speed'''
        dx *= self.xy_sensitivity
        dy *= self.xy_sensitivity
        dz *= self.z_sensitivity

        # Begrenser arbeidsområde til gyldige posisjoner
        self.current_pos[0] = max(-10, min(10, self.current_pos[0]))  # X limits
        self.current_pos[1] = max(5, min(20, self.current_pos[1]))     # Y limits
        self.current_pos[2] = max(10, min(25, self.current_pos[2]))     # Z limits

    # Fungerer ikke helt korrekt. Verdier oppdateres, men 
    # minimalt utslag i ledd C. 
    def update_pitch(self, delta):
        self.current_pitch = max(-90, min(90, 
            self.current_pitch + delta * self.pitch_sensitivity))
        # Oppdaterer pitch til griper
        '''self.current_pitch += delta * self.pitch_speed
        self.current_pitch = max(-90, min(90, self.current_pitch))'''

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
            inputs = controller.get_inputs()
            '''if not controller.joystick:
                controller.initialize_controller()
                time.sleep(1)
                continue'''

                
            # Avslutter programmet ved å trykke 'start'
            if inputs.get('start', 0):
                break
            
            # Endring i posisjon
            dx = inputs.get('right_x',0)
            dy = -inputs.get('right_y',0)  # Invertert Y-akse? TBD
            dz = -inputs.get('left_y',0)   # Z akse fra venstre stick
            
            arm.update_position(dx, dy, dz)
            
            # Endring i pitch (bumpers)
            pitch_delta = 0
            if inputs.get('lb', 0):
                pitch_delta -= 2
            if inputs.get('rb', 0):
                pitch_delta += 2
            
            # Endring i griperen (A & B)
            if inputs.get('a_button', 0):
                arm.control_gripper(True)
            if inputs.get('b_button', 0):
                arm.control_gripper(False)

            # Oppdaterer armens tilstand
            if dx != 0 or dy != 0 or dz != 0:
                arm.update_position(dx, dy, dz)
            if pitch_delta != 0:
                arm.update_pitch(pitch_delta)
            
            # Kaller bevegelsesfunksjon
            if arm.move_arm():
                print(f"X:{arm.current_pos[0]:.1f} Y:{arm.current_pos[1]:.1f} "
                      f"Z:{arm.current_pos[2]:.1f} Pitch:{arm.current_pitch:.1f}°")
            else:
                print("Ugyldig posisjon!")
            
            time.sleep(0.05)  # main loop hastighetskontroll i sekunder

    except KeyboardInterrupt:
        print("Avslutter...")
    finally:
        pygame.quit()

if __name__ == "__main__":
    main()