#!/usr/bin/env python3
# encoding:utf-8

###########################################################################################
############################## Control by controller ######################################
###########################################################################################

import time
from common.ros_robot_controller_sdk import Board
from kinematics.arm_move_ik import ArmIK

class GamepadArmController:
    def __init__(self):
        # Initialize the motor control board
        self.board = Board()
        self.board.enable_reception()

        # Initialize the inverse kinematics solver
        self.arm_ik = ArmIK()
        self.arm_ik.board = self.board

        # Initial arm position
        self.current_position = (0, 6, 18)  # (x, y, z) in cm
        self.current_pitch = 0  # Pitch angle in degrees

    def move_arm_with_gamepad(self):
        print("Press Ctrl+C to exit.")
        try:
            while True:
                # Read gamepad input
                gamepad_data = self.board.get_gamepad()
                if gamepad_data is None:
                    time.sleep(0.01)
                    continue

                axes, buttons = gamepad_data

                # Map gamepad axes to arm movement
                # Example: Use left joystick (axes[0], axes[1]) to control X and Y
                # Use right joystick (axes[2], axes[3]) to control Z and pitch
                delta_x = axes[0] * 2  # Scale joystick input to movement in cm
                delta_y = axes[1] * 2
                delta_z = axes[3] * 2
                delta_pitch = axes[2] * 10  # Scale joystick input to pitch angle

                # Update target position
                new_x = self.current_position[0] + delta_x
                new_y = self.current_position[1] + delta_y
                new_z = self.current_position[2] + delta_z
                new_pitch = self.current_pitch + delta_pitch

                # Clamp values to safe ranges
                new_x = max(-10, min(10, new_x))   # Limit X to [-10, 10] cm
                new_y = max(5, min(20, new_y))     # Limit Y to [5, 20] cm
                new_z = max(10, min(25, new_z))    # Limit Z to [10, 25] cm
                new_pitch = max(-90, min(90, new_pitch))  # Limit pitch to [-90, 90] degrees

                # Move the arm to the new position
                result = self.arm_ik.setPitchRangeMoving(
                    (new_x, new_y, new_z), new_pitch, -90, 90, 500
                )
                if result:
                    self.current_position = (new_x, new_y, new_z)
                    self.current_pitch = new_pitch
                    print(f"Moved to: {self.current_position}, Pitch: {self.current_pitch}")

                # Small delay to avoid overwhelming the system
                time.sleep(0.1)

        except KeyboardInterrupt:
            print("Exiting gamepad control.")

# Run the controller
if __name__ == "__main__":
    controller = GamepadArmController()
    controller.move_arm_with_gamepad()