from common.ros_robot_controller_sdk import Board
from kinematics.inversekinematics import IK
from arm_move_ik import ArmIK

class RoboticArmController:
    def __init__(self):
        # Initialize the motor control board
        self.board = Board()
        self.board.enable_reception()

        # Initialize the inverse kinematics solver
        self.ik = IK('arm')
        self.ik.setLinkLength(L1=self.ik.l1 + 1.3, L4=self.ik.l4)

        # Initialize the arm movement controller
        self.arm_ik = ArmIK()
        self.arm_ik.board = self.board

    def move_to_position(self, x, y, z, alpha=0):
        """
        Move the end effector to the specified (x, y, z) position.
        
        :param x: Target x-coordinate (in cm)
        :param y: Target y-coordinate (in cm)
        :param z: Target z-coordinate (in cm)
        :param alpha: Target pitch angle (in degrees, default is 0)
        """
        # Calculate the joint angles using inverse kinematics
        result = self.ik.getRotationAngle((x, y, z), alpha)
        if not result:
            print("No viable solution found for the given position.")
            return False

        # Extract the joint angles
        theta3, theta4, theta5, theta6 = result['theta3'], result['theta4'], result['theta5'], result['theta6']

        # Convert the joint angles to servo commands
        servos = self.arm_ik.transformAngelAdaptArm(theta3, theta4, theta5, theta6)
        if not servos:
            print("Servo angles out of range.")
            return False

        # Move the servos to the calculated positions
        self.arm_ik.servosMove([servos["servo3"], servos["servo4"], servos["servo5"], servos["servo6"]])

        print(f"Moved to position: ({x}, {y}, {z}) with angles: {servos}")
        return True

# Example usage
if __name__ == "__main__":
    arm_controller = RoboticArmController()
    arm_controller.move_to_position(0, 6, 10)  # Move to (0, 6, 10) cm