"""apparel_controller controller."""

#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Keyboard
# from controller import Keyboard
import struct
import time
# import sys
# sys.path.append('/home/peiyu/apparel_simulation/controllers/')
# import all_robot

robot = Robot()

TIMESTEP: int = 64
keyboard = Keyboard()
keyboard.enable(10 * TIMESTEP)

# create the Robot instance.
MAX_STL_SPEED: float = 0.3
MAX_CVY_SPEED: float = 0.3
MIN_STL_POSITION: float = 0.0
MAX_STL_POSITION: float = 1.450
ORIGIN: float = 0.3
DS_TH: float = 0.35

# connect motor and sensors
st_emt = robot.getDevice("shuttle_emitter")
ds_emt = robot.getDevice("ds_emitter")
sl_rcv = robot.getDevice("slider_receiver")
lm = robot.getDevice("linear")
cvy = robot.getDevice("track_motor")
ps = robot.getDevice("position_sensor")
ds_left = robot.getDevice("ds_left")
ds_middle = robot.getDevice("ds_middle")
ds_right = robot.getDevice("ds_right")
# st_emt = all_robot.st_emt
# ds_emt = all_robot.ds_emt
# sl_rcv = all_robot.sl_rcv
# lm = all_robot.lm
# cvy = all_robot.cvy
# ps = all_robot.ps
# ds_left = all_robot.ds_left
# ds_middle = all_robot.ds_middle
# ds_right = all_robot.ds_right

# set work mode, enable sensors
lm.setPosition(float("inf"))
cvy.setPosition(float("inf"))
ps.enable(TIMESTEP)
ds_left.enable(TIMESTEP)
ds_middle.enable(TIMESTEP)
ds_right.enable(TIMESTEP)
sl_rcv.enable(TIMESTEP)
# ds_origin_01.enable(TIMESTEP)

# set initial speed
lm.setVelocity(0.0)
cvy.setVelocity(0.0)


def test():
    print("test")


def st_send() -> None:
    """shuttle send message to the potential receivers in the world."""
    message = struct.pack("cd", "l".encode('utf-8'), ps.getValue())
    st_emt.send(message)


def ds_send() -> None:
    """distance sensor send message to the potential receivers in the world"""
    message = struct.pack("cs", "l".encode('utf-8'), "on".encode('utf-8'))
    ds_emt.send(message)


def stl_to(position: float) -> None:
    """Move the shuttle to some specific position between its min and max position."""
    # if position > MAX_STL_POSITION or position < MIN_STL_POSITION:
    #     print("Given position not valid, it should be between " + str(MIN_STL_POSITION) + "and "
    #           + str(MAX_STL_POSITION))
    #     return
    #
    # stl_current_speed: float = 0.0
    # while robot.step(TIMESTEP) != -1:
    #     if ps.getValue() < position:
    #         stl_current_speed = MAX_STL_SPEED
    #     elif ps.getValue() > position:
    #         stl_current_speed = MIN_STL_POSITION
    #     else:
    #         break
    #     lm.setVelocity(stl_current_speed)
    #
    # print("Current position: " + str(ps.getValue()))
    lm.setVelocity(MAX_STL_SPEED)
    while robot.step(TIMESTEP) != -1:
        lm.setPosition(position)
        if abs(ps.getValue() - position) < 0.0001:
            print("Shuttle reaches the target position: " + str(ps.getValue()))
            lm.setPosition(float("inf"))
            break


def reciprocate() -> None:
    """Let the shuttle do reciprocating motion"""
    current_speed = -MAX_STL_SPEED
    need_reverse: bool = False

    while robot.step(TIMESTEP) != -1:
        # print("current position: " + str(ps.getValue()))

        if ps.getValue() >= MAX_STL_POSITION or ps.getValue() <= MIN_STL_POSITION:
            need_reverse = not need_reverse
        else:
            need_reverse = False

        if need_reverse:
            current_speed = -current_speed
            need_reverse = True

        lm.setVelocity(current_speed)
        # lm.setVelocity(0.0)
        cvy.setVelocity(MAX_CVY_SPEED)


def main() -> None:
    time.sleep(3)
    is_carrying: bool = False

    # back to origin at the beginning of the simulation
    stl_to(ORIGIN)
    print("wait for command.")
    lm.setVelocity(0.0)

    while robot.step(TIMESTEP) != -1:

        # receive message from slider
        # if sl_rcv.getQueueLength() > 0:
        #     try:
        #         sl_ms = sl_rcv.getData()
        #         sl_data_list = struct.unpack("cc", sl_ms)
        #
        #         # slider loading
        #         if sl_data_list[1] == b't':
        #             print("shuttle received message: loading...")
        #             lm.setVelocity(0.0)
        #             sl_rcv.nextPacket()
        #
        #         # load complete
        #         elif sl_data_list[1] == b'f':
        #             print("shuttle received message: load complete.")
        #             is_carrying = True
        #             sl_rcv.nextPacket()
        #         sl_rcv.nextPacket()
        #     except Exception as e:
        #         print(e)

        # keyboard control
        key = keyboard.getKey()
        if key == ord('W'):
            if ps.getValue() < MAX_STL_POSITION:
                lm.setVelocity(MAX_STL_SPEED)
            else:
                lm.setVelocity(0.0)
                # print("Command cannot complete. Shuttle already in the max position")
        elif key == ord('S'):
            if ps.getValue() > MIN_STL_POSITION:
                lm.setVelocity(-MAX_STL_SPEED)
            else:
                lm.setVelocity(0.0)
                # print("Command cannot complete. Shuttle already in the min position")
        elif key == Keyboard.RIGHT:
            cvy.setVelocity(MAX_CVY_SPEED)
        elif key == Keyboard.LEFT:
            cvy.setVelocity(-MAX_CVY_SPEED)
        elif key == ord('B'):
            print("keyboard B pressed")
            stl_to(MIN_STL_POSITION)
        elif key == ord('T'):
            print("keyboard T pressed")
            stl_to(MAX_STL_POSITION)
        else:
            lm.setVelocity(0.0)
            cvy.setVelocity(0.0)

        # emit message: ready to carry objects
        # if ps.getValue() == ORIGIN:
        #     st_send()

        # emit message: object is loaded
        # if (ds_left.getValue() > DS_TH) or (ds_middle.getValue() > DS_TH) or (
        #         ds_right.getValue() > DS_TH):
        #     if is_carrying is False:
        #         ds_send()
        #         stl_to(MIN_STL_POSITION)
        #     else:
        #         pass

        # reciprocate()


if __name__ == "__main__":
    print(type(ps))
    main()
