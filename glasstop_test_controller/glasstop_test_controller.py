"""apparel_controller controller."""

#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Keyboard
import struct
import time
# import sys
# sys.path.append('/home/peiyu/apparel_simulation/controllers/lift_controller')
# sys.path.append('/home/peiyu/apparel_simulation/controllers/')
# try:
#     print("load lift_controller")
#     import all_robot
#     import lift_controller.lift_controller
#     from lift_controller.lift_controller import reciprocate
#     print("finish")
# except ModuleNotFoundError:
#     print("lift_controller not found")
#     pass

robot = Robot()

TIMESTEP: int = 32
keyboard = Keyboard()
keyboard.enable(10 * TIMESTEP)

# create the Robot instance.
MAX_SLD_SPEED: float = 0.5
MIN_BSLD_POSITION: float = 0.0
MAX_BSLD_POSITION: float = 0.5
MIN_SSLD_POSITION: float = 0.0
MAX_SSLD_POSITION: float = 0.7

# connect motor and sensors
back_lm = robot.getDevice("back_slider")
side_lm = robot.getDevice("side_slider")
ds_rcv = robot.getDevice("ds_receiver")
st_rcv = robot.getDevice("shuttle_receiver")
sl_emt = robot.getDevice("slider_emitter")

# set work mode, enable sensors
back_lm.setPosition(float("inf"))
side_lm.setPosition(float("inf"))
st_rcv.enable(TIMESTEP)
ds_rcv.enable(TIMESTEP)

# set initial speed
back_lm.setVelocity(MAX_SLD_SPEED)
side_lm.setVelocity(MAX_SLD_SPEED)


def to_conv() -> None:
    """push the object to the conveyor"""
    while robot.step(TIMESTEP) != -1:
        st_rcv.nextPacket()

        # send message to shuttle: loading
        msg_to_st = struct.pack("cc", "g".encode('utf-8'), "t".encode('utf-8'))
        sl_emt.send(msg_to_st)

        # receive message from shuttle's distance sensor(s)
        if ds_rcv.getQueueLength() > 0:
            try:
                ds_ms = ds_rcv.getData()
                ds_data_list = struct.unpack("cs", ds_ms)
                print("receive " + str(ds_rcv.getQueueLength()) + " from distance sensors")
                print("glass top received message:" + str(ds_data_list[1]))

                # send message to shuttle: load complete
                msg_to_st = struct.pack("cc", "g".encode('utf-8'), "f".encode('utf-8'))
                sl_emt.send(msg_to_st)

                break

            except Exception as e:
                print(e)
                break
        # not receive message
        else:
            side_lm.setPosition(MAX_SSLD_POSITION)
            # time.sleep(1)
            # side_lm.setPosition(MIN_BSLD_POSITION)
            # time.sleep(1)


def main() -> None:
    time.sleep(3)
    while robot.step(TIMESTEP) != -1:

        # receive message from shuttle: conveyor ready to carry
        if st_rcv.getQueueLength() > 0:
            try:
                st_ms = st_rcv.getData()
                st_data_list = struct.unpack("cd", st_ms)
                print("slider receive " + str(st_rcv.getQueueLength()) + " message from shuttle")
                print("slider received message:" + str(st_data_list[1]))
                to_conv()   # slider starts push object
                st_rcv.nextPacket()
            except Exception as e:
                print(e)

        key = keyboard.getKey()
        if key == Keyboard.UP and key != Keyboard.DOWN:
            back_lm.setPosition(MAX_BSLD_POSITION)
        elif key == Keyboard.DOWN and key != Keyboard.UP:
            side_lm.setPosition(MAX_SSLD_POSITION)
        else:
            back_lm.setPosition(MIN_BSLD_POSITION)
            side_lm.setPosition(MIN_SSLD_POSITION)


if __name__ == "__main__":
    # reciprocate()
    main()
