import math

from drone_fsm.waypoint import run_figure

RADIUS = 1.5
ALTITUDE = 3.0
CIRCLE_POINTS = 36  # Number of points to approximate the circle

def make_circle_waypoints(radius, altidude, num_points):
    waypoints = []
    for i in range(num_points + 1):
        theta = math.pi + 2 * math.pi * i / num_points
        x = radius + radius * math.cos(theta)
        y = radius * math.sin(theta)
        waypoints.append((round(x, 3) + 0.0, round(y,3) + 0.0, -altidude))
    return waypoints

WAYPOINTS = make_circle_waypoints(RADIUS, ALTITUDE, CIRCLE_POINTS)

def main(args=None):
    run_figure('circle', WAYPOINTS, args)


if __name__ == '__main__':
    main()