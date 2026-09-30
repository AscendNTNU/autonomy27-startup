from drone_fsm.waypoint import run_figure

# Square: 3 x 3 m at 3 m altitude.
# Waypoints in PX4 local NED (meters)
WAYPOINTS = [
    (0.0, 0.0, -3.0),  # Waypoint 1: above start point (where takeoff ends)
    (3.0, 0.0, -3.0),  # Waypoint 2: 3 m north
    (3.0, 3.0, -3.0),  # Waypoint 3: 3 m east
    (0.0, 3.0, -3.0),  # Waypoint 4: 3 m south
    (0.0, 0.0, -3.0),  # Waypoint 5: back to start point
]


def main(args=None):
    run_figure('square', WAYPOINTS, args)


if __name__ == '__main__':
    main()