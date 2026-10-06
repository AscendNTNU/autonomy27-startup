from math import cos, sin, pi

from drone_fsm.drone_fsm.waypoint_node import run_figure

POINT_COUNT = 32
# The raw formula is about 32 units wide. ROS treats the resulting ENU
# coordinates as meters, so 0.1 means 0.1 m per unit and about 3.2 m wide.
SCALE = 0.1
ALTITUDE = 3.0


def make_heart(point_count, scale, altitude):
    waypoints = []

    for index in range(point_count):
        angle = 2.0 * pi * index / point_count

        # Parametric heart formula
        heart_x = 16.0 * sin(angle) ** 3
        heart_y = (
            13.0 * cos(angle)
            - 5.0 * cos(2.0 * angle)
            - 2.0 * cos(3.0 * angle)
            - cos(4.0 * angle)
        )

        # Coordinates are (east, north, up) in the local ENU frame.
        north = (heart_y - 5.0) * scale
        east = heart_x * scale

        waypoints.append((east, north, altitude))

    waypoints.append(waypoints[0])

    return waypoints


WAYPOINTS = make_heart(POINT_COUNT, SCALE, ALTITUDE)


def main(args=None):
    run_figure('heart', WAYPOINTS, args)


if __name__ == '__main__':
    main()
