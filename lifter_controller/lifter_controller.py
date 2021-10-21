"""lifter_controller controller."""

import controller
from controller import Robot
from typing import Tuple
import struct
import time

TIMESTEP: int = 32

# create the Robot instance.
MAX_STL_SPEED: float = 0.3
MAX_CVY_SPEED: float = 0.3
MIN_STL_POSITION: float = 0.0
MAX_STL_POSITION: float = 1.450
ORIGIN: float = 0.3
DS_TH: float = 0.35


class Lift(Robot):
    lm: controller.Motor
    cvy: controller.Motor
    ps: controller.PositionSensor
    rec: controller.Receiver

    def __init__(self, lm_name: str, cvy_name: str, ps_name: str, rec_name):
        """Construct a point with device name"""
        super().__init__()
        self.lm = self.getDevice(lm_name)
        self.cvy = self.getDevice(cvy_name)
        self.ps = self.getDevice(ps_name)
        self.rec = self.getDevice(rec_name)

        self.lm.setPosition(float("inf"))
        self.cvy.setPosition(float("inf"))
        self.ps.enable(TIMESTEP)
        self.rec.enable(TIMESTEP)

    def reciprocate(self) -> None:
        """Let the shuttle do reciprocating motion"""
        current_speed = -MAX_STL_SPEED
        need_reverse: bool = False

        while self.step(TIMESTEP) != -1:
            # print("current position: " + str(ps.getValue()))

            if self.ps.getValue() >= MAX_STL_POSITION or self.ps.getValue() <= MIN_STL_POSITION:
                need_reverse = not need_reverse
            else:
                need_reverse = False

            if need_reverse:
                current_speed = -current_speed
                need_reverse = True

            self.lm.setVelocity(current_speed)
            # lm.setVelocity(0.0)
            self.cvy.setVelocity(MAX_CVY_SPEED)
            self.rec.nextPacket()

    def terminate(self) -> None:
        """Stop all motion"""
        self.lm.setVelocity(0.0)
        self.cvy.setVelocity(0.0)


def main() -> None:
    lift = Lift(lm_name="linear",
                cvy_name="track_motor",
                ps_name="position_sensor",
                rec_name="virtual_receiver")

    print("this is the start of the test.")
    time.sleep(1)
    while lift.step(TIMESTEP) != -1:
        print(lift.rec.getQueueLength())
        if lift.rec.getQueueLength() > 0:
            print("receiving message")
            rec_val = struct.unpack("c", lift.rec.getData())
            # print(rec_val[0])
            if rec_val[0] == b'r':
                lift.reciprocate()
            lift.rec.nextPacket()
        else:
            lift.terminate()
            print("terminated")
        

if __name__ == "__main__":
    main()


