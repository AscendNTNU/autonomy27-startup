from math import cos, sin, pi
from drone_fsm.waypoint import run_figure


def make_heart(point_count=32, scale=0.1, altitude=-3.0):
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

        # x = north, y = east, z = down.
        north = (heart_y - 5.0) * scale
        east = heart_x * scale

        waypoints.append((north, east, altitude))

    waypoints.append(waypoints[0])

    return waypoints

WAYPOINTS = make_heart()

def main(args=None):
    run_figure('heart', WAYPOINTS, args)

if __name__ == '__main__':
    main()