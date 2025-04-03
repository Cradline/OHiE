#!/usr/bin/env python3
# encoding: utf-8
import pygame
import time
import math
import cv2  # openCV
from threading import Thread  # Kjører video i separat tråd
from common.ros_robot_controller_sdk import Board
from common.mecanum import MecanumChassis
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
                'dpad_up': False,                           # Kjør frem
                'dpad_down': False,                         # Rygg
                'dpad_left': False,                         # Roter venstre
                'dpad_right': False,                        # Roter høyre
                
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

class VideoStream:
    def __init__(self, resolution=(640, 480)):
        self.stream = cv2.VideoCapture(-1, cv2.CAP_V4L2)
        self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
        self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])
        self.stopped = False
        self.frame = None

    def start(self):
        Thread(target=self.update, args=()).start()
        return self

    def update(self):
        while not self.stopped:
            ret, frame = self.stream.read()
            if not ret:
                break
            self.frame = frame

    def read(self):
        return self.frame

    def stop(self):
        self.stopped = True
        self.stream.release()

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

        # DENNE MÅ OPPDATERES
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

class koraBil:
    def __init__(self):
        self.chassis = MecanumChassis()
        self.speed = 100  # [mm/s] (Juster etter behov)
        self.rotation_speed = 50  # [mm/s] (Juster etter behov)

    def forward(self):
        """Konstant fart framover"""
        self.chassis.set_velocity(self.speed, 90, 0)  # 90° = fremover i Y-akse (bil)

    def backward(self):
        """Konstant fart bakover"""
        self.chassis.set_velocity(self.speed, 270, 0)  # 270° = bakover i Y-akse (bil)

    def rotate_left(self):
        """Roterer mot klokken"""
        self.chassis.set_velocity(0, 0, self.rotation_speed)

    def rotate_right(self):
        """Roterer med klokken"""
        self.chassis.set_velocity(0, 0, -self.rotation_speed)

    def stop(self):
        """Stopper alle motorer"""
        self.chassis.reset_motors()


def main():
    controller = XboxController()
    arm = ArmController()
    rover = koraBil()
    video_stream = VideoStream().start()
    time.sleep(1.0)
    
    print("Starter ArmController...")
    print("Høyre stick: X/Y bevegelse")
    print("Venstre stick Y: Z bevegelse")
    print("LB/RB: Endrer pitch")
    print("A/B: Åpner/lukker griper")
    print("D-pad: Rover kontroll")
    print("Start: Avslutter program")

    try:
        while True:

            # Viser video feed
            frame = video_stream.read()
            if frame is not None:
                cv2.imshow("Rover Camera", frame)
                cv2.waitKey(1)      # Kreves for å oppdatere vinduet

            inputs = controller.get_inputs()
                
            # Avslutter programmet ved å trykke 'start'
            if inputs.get('start', 0):
                break
            
            # Rover control (D-pad)
            if inputs['dpad_up']:
                rover.forward()
            elif inputs['dpad_down']:
                rover.backward()
            elif inputs['dpad_left']:
                rover.rotate_left()
            elif inputs['dpad_right']:
                rover.rotate_right()
            else:
                rover.stop()  # Stopper dersom ingen input


            # Endring i posisjon
            dx = inputs.get('right_x',0)
            dy = -inputs.get('right_y',0)  # Invertert Y-akse? TBD
            dz = -inputs.get('left_y',0)   # Z akse fra venstre stick
            
            #arm.update_position(dx, dy, dz)
            
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
        rover.stop()
        video_stream.stop()  # Frigjør camera
        cv2.destroyAllWindows()  # Lukker alle OpenCV vinduer
        pygame.quit()

if __name__ == "__main__":
    main()