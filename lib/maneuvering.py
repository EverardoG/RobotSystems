import picarx_improved
import logging
import time

def forward_backward(px):
    "Moves Picar-x forward and back, and changes the steering angle. "
    input('\nPress enter to go forward: ')
    px.forward(50)
    time.sleep(2)
    px.stop()
    input('\nPress enter to change the steering angle: ')
    px.set_dir_servo_angle(10)
    input('\nPress enter to go backward: ')
    px.backward(50)
    time.sleep(2)
    px.stop()

def parallel_park(px,direction):
    """Parallel parks given a direction ('left' or 'right')."""
    direction_map = {'left':-1,'right':1}
    dir_modifier = direction_map[direction]

    px.set_dir_servo_angle(0)
    px.forward(50)
    time.sleep(1)
    px.backward(50)
    px.set_dir_servo_angle(dir_modifier*40)
    time.sleep(0.5)
    px.set_dir_servo_angle(dir_modifier * -40)
    time.sleep(0.5)
    px.stop()

def three_point_turn(px,direction):
    """Executes 3 point turn given a direction ('left' or 'right')."""
    direction_map = {'left':-1,'right':1}
    dir_modifier = direction_map[direction]

    px.set_dir_servo_angle(0)
    px.forward(50)
    time.sleep(0.5)
    px.set_dir_servo_angle(dir_modifier*40)
    time.sleep(0.5)
    px.backward(50)
    px.set_dir_servo_angle(dir_modifier * -40)
    time.sleep(0.5)
    px.forward(50)
    px.set_dir_servo_angle(0)
    time.sleep(0.5)

def get_dir_response():
    ans_map = {'l': 'left', 'r': 'right'}
    while True:
        try:
            response = input("Left or right? (l/r)\n> ")
            direction = ans_map[response]
        except:
            print("\nInvalid entry. Enter l for left or r for right.\n")
    return direction

def get_action_response():
    ans_set = set({'1','2','3','4'})
    while True:
        try:
            response = input("Enter the number for the desired maneuver: \n"
                             "forward_backward: 1\n"
                             "parallel_park: 2\n"
                             "get_dir_response: 3\n"
                             "exit: 4\n"
                             "> ")
            assert response in ans_set
        except:
            print("\nInvalid entry. Enter 1, 2, 3 or 4.\n")
    return response

def main():
    logging.getLogger().setLevel(logging.DEBUG)
    px = picarx_improved.Picarx()

    while True:
        action = get_action_response()
        if action == '1': forward_backward(px)
        if action == '2':
            direction = get_dir_response()
            parallel_park(px,direction)
        if action == '3':
            direction = get_dir_response()
            three_point_turn(px,direction)
        if action == '4': break
        
if __name__ == '__main__':
    main()
