#!/usr/bin/env python3
# encoding:utf-8
import sys
from kinematics.arm_move_ik import ArmIK
from common.ros_robot_controller_sdk import Board

def get_user_input():
    """
    Prompts the user to enter the target position and pitch angle.
    Returns the coordinates (x, y, z) and pitch angle.
    """
    try:
        x = float(input("Enter target X position (cm): "))
        y = float(input("Enter target Y position (cm): "))
        z = float(input("Enter target Z position (cm): "))
        pitch = float(input("Enter target pitch angle (degrees): "))
        return (x, y, z), pitch
    except ValueError:
        print("Invalid input. Please enter numeric values.")
        return None, None

def main():
    # Initialize the motor control board and inverse kinematics solver
    board = Board()
    board.enable_reception()
    arm_ik = ArmIK()
    arm_ik.board = board

    print("Robotic Arm Interactive Control")
    print("Enter the target position and pitch angle to move the arm.")
    print("Type 'exit' to quit the program.")

    while True:
        # Get user input for target position and pitch angle
        coordinate_data, pitch = get_user_input()
        if coordinate_data is None:
            continue  # Skip invalid input

        # Move the arm to the target position
        result = arm_ik.setPitchRangeMoving(coordinate_data, pitch, -90, 90, 1000)
        if result:
            print(f"Arm moved to: {coordinate_data}, Pitch: {pitch} degrees")
        else:
            print("Failed to move the arm. No valid solution found.")

        # Ask the user if they want to continue or exit
        user_choice = input("Do you want to continue? (yes/no): ").strip().lower()
        if user_choice != "yes":
            print("Exiting the program.")
            break

if __name__ == "__main__":
    main()