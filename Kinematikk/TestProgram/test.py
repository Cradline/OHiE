from ArmControllerClass import RoboticArmController
import time

def main():
    # Initialiser robotarmkontrolleren
    arm_controller = RoboticArmController()

    # Testposisjoner
    test_positions = [
        (0, 6, 10),  # Posisjon 1
        (5, 5, 15),  # Posisjon 2
        (-5, 5, 15), # Posisjon 3
    ]

    for pos in test_positions:
        print(f"Flytter til posisjon: {pos}")
        success = arm_controller.move_to_position(*pos)
        if success:
            print("Bevegelse vellykket!")
        else:
            print("Bevegelse mislyktes.")
        time.sleep(2)  # Vent 2 sekunder mellom hver bevegelse

if __name__ == "__main__":
    main()

    #test test
