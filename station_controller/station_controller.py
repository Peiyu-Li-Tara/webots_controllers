"""station_controller controller."""

from controller import Robot
import struct

station = Robot()

TIMESTEP: int = 32

emt = station.getDevice("world_emitter")


def main() -> None:
    while station.step(TIMESTEP) != -1:
        # print("sending")
        message = struct.pack("c", "r".encode('utf-8'))
        emt.send(message)


if __name__ == "__main__":
    main()