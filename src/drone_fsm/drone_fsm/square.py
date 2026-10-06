from drone_fsm.waypoint_node import run_figure

WAYPOINTS = [
    # Coordinates are (east, north, up) in the local ENU frame.
    (0.0, 0.0, 3.0),  # Waypoint 1: above the start point
    (3.0, 0.0, 3.0),  # Waypoint 2: 3 m east
    (3.0, 3.0, 3.0),  # Waypoint 3: 3 m north
    (0.0, 3.0, 3.0),  # Waypoint 4: 3 m west
    (0.0, 0.0, 3.0),  # Waypoint 5: 3 m south, back to the start
]


def main(args=None):
    run_figure('square', WAYPOINTS, args)


if __name__ == '__main__':
    main()