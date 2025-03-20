#!/usr/bin/env python3
# encoding: utf-8
import pygame
import time
import math
from common.ros_robot_controller_sdk import Board
from kinematics.arm_move_ik import ArmIK
# from controller_class import *
# from kinematics.IK_servo import ArmIK

# Controller klasse 
class XboxController:
    def __init__(self):
        pygame.init()
        pygame.joystick.init()
        self.joystick = None
        self.initialize_controller()
        self.deadzone = 0.8                 # Deadzone ratio

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
        # Kun input verdier > deadzone registreres
        return value if abs(value) > self.deadzone else 0.0

    def get_inputs(self):
        inputs = {}
        pygame.event.pump()
        try:
            inputs = {
                # Høyre analog stick (X/Y axes)
                'right_x': self.apply_deadzone(self.joystick.get_axis(2)),
                'right_y': self.apply_deadzone(self.joystick.get_axis(3)),
                
                # Venstre analog stick (for Z-axis)
                'left_y': self.apply_deadzone(self.joystick.get_axis(1)),
                'left_x': self.apply_deadzone(self.joystick.get_axis(0)),   # Ubrukt

                # Triggers
                'RT': self.joystick.get_axis(4),            # Ubrukt
                'LT': self.joystick.get_axis(5),            # Ubrukt
                
                # Knapper 
                'a_button': self.joystick.get_button(0),    # Lukker griper
                'b_button': self.joystick.get_button(1),    # Åpner griper
                'x_button': self.joystick.get_button(3),    # Ubrukt
                'y_button': self.joystick.get_button(4),    # Ubrukt
                'lb': self.joystick.get_button(6),          # Pitch ned
                'rb': self.joystick.get_button(7),          # Pitch opp

                # D-pad (Hat 0)
                'dpad_up': False,                           # Ubrukt
                'dpad_down': False,                         # Ubrukt
                'dpad_left': False,                         # Ubrukt
                'dpad_right': False,                        # Ubrukt
                
                # Start/Select
                'start': self.joystick.get_button(11),      # Avslutter main()
                'select': self.joystick.get_button(10)      # Ubrukt
            }
            # Oppdaterer D-pad inputs (Hat 0)
            if self.joystick.get_numhats() > 0:
                hat_state = self.joystick.get_hat(0)
                inputs.update({
                    'dpad_up': hat_state[1] == 1,           # (0, 1)
                    'dpad_down': hat_state[1] == -1,        # (0,-1)
                    'dpad_left': hat_state[0] == -1,        # (-1,0)
                    'dpad_right': hat_state[0] == 1         # (1, 0)
                })
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

        # Sensitivitet
        self.xy_sensitivity = 0.5  # lavere = tregere
        self.z_sensitivity = 0.5
        self.pitch_sensitivity = 0.5

    def update_position(self, dx, dy, dz):
        # Oppdaterer posisjon med hastighetsskalering

        dx *= self.xy_sensitivity
        dy *= self.xy_sensitivity
        dz *= self.z_sensitivity

        # Begrenser arbeidsområde til gyldige posisjoner
        self.current_pos[0] = max(-10, min(10, self.current_pos[0] + dx))  # X limits
        self.current_pos[1] = max(5, min(20, self.current_pos[1] + dy))     # Y limits
        self.current_pos[2] = max(10, min(25, self.current_pos[2] + dz))     # Z limits

    # Fungerer ikke helt korrekt. Verdier oppdateres, men 
    # minimalt utslag i ledd C. 
    def update_pitch(self, delta):
        # Oppdaterer pitchvinkel til griper
        self.current_pitch = max(-90, min(90, 
            self.current_pitch + delta * self.pitch_sensitivity))

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

class koraBil: #fremover, bakover, roter høyre, roter venstre
    def __init__(self):
        self.board = Board()
        self.board.enable_reception()


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
                
            # Avslutter programmet ved å trykke 'start'
            if inputs.get('start', 0):
                break
            
            if inputs['dpad_up']:
                print('dpad_up')
                #arm.update_position(0, 0.5, 0)  # Move forward in Y
            if inputs['dpad_down']:
                print('dpad_down')
                #arm.update_position(0, -0.5, 0)  # Move backward in Y
            if inputs['dpad_left']:
                print('dpad_left')
                #arm.update_position(-0.5, 0, 0)  # Move left in X
            if inputs['dpad_right']:
                print('dpad_right')
                #arm.update_position(0.5, 0, 0)  # Move right in X

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
            
            time.sleep(0.05)  # main loop delay i sekunder

    except KeyboardInterrupt:
        print("Avslutter...")
    finally:
        pygame.quit()

if __name__ == "__main__":
    main()