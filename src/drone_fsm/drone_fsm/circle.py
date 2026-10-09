import math

from drone_fsm.waypoint_node import run_figure

RADIUS = 20
ALTITUDE = 3.0
CIRCLE_POINTS = 36  # Number of points to approximate the circle


def make_circle_waypoints(radius, altitude, num_points):
    waypoints = []

    for i in range(num_points + 1):
        theta = math.pi + 2 * math.pi * i / num_points
        east = radius + radius * math.cos(theta)
        north = radius * math.sin(theta)

        # Coordinates are (east, north, up) in the local ENU frame.
        waypoints.append((round(east, 3) + 0.0, round(north, 3) + 0.0, altitude))

    return waypoints


WAYPOINTS = make_circle_waypoints(RADIUS, ALTITUDE, CIRCLE_POINTS)


def main(args=None):
    run_figure('circle', WAYPOINTS, args)


if __name__ == '__main__':
    main()