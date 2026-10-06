import numpy as np
from drone_fsm.waypoint_node import run_figure

POINT_COUNT      = 20
DIGIT_LENGTH     = 20.0 # [m]
DEFAULT_ALTITUDE = 1.5  # [m]

def make_incredible_waypoints(digit_length=DIGIT_LENGTH, altitude=DEFAULT_ALTITUDE, point_count=POINT_COUNT):
    # Allocate number of points to each segment using length
    n_stem, n_loop, n_bar = (max(2, round(f * point_count)) for f in (0.18, 0.41, 0.13))
    n_diag = max(2, point_count - n_stem - n_loop - n_bar)

    # Draw the secret first digit
    stem = np.linspace(0.6 * np.pi, np.pi, n_stem)
    loop = np.linspace(np.pi, 3 * np.pi, n_loop)
    x6 = np.concatenate([1 + np.cos(stem), 0.5 + 0.5 * np.cos(loop)])
    y6 = np.concatenate([0.5 + 1.5 * np.sin(stem), 0.5 + 0.5 * np.sin(loop)])

    # Draw the even more secret second digit
    bar = np.linspace(0, 1, n_bar)
    diag = np.linspace(0, 1, n_diag)
    x7 = np.concatenate([1.5 + bar, 2.5 - 0.8 * diag])
    y7 = np.concatenate([np.full_like(bar, 2), 2 - 2 * diag + 0.3 * np.sin(np.pi * diag)])

    # Scale digits to set length
    k = digit_length / 2
    x6, y6, x7, y7 = k * x6, k * y6, k * x7, k * y7
    z6 = altitude * np.ones(len(x6))
    z7 = altitude * np.ones(len(x7))

    # Connect curves
    waypoints = []
    waypoints.extend(list(zip(x6,y6,z6)))
    waypoints.extend(list(zip(x7,y7,z7))[::-1])

    waypoints = set_start_at_origin(waypoints)

    return waypoints

def set_start_at_origin(waypoints):
    first_waypoint = waypoints[0]
    new_waypoints = [(0,0, first_waypoint[2])]

    for waypoint in waypoints[1:]:
        new_waypoints.append((
            waypoint[0] - first_waypoint[0],
            waypoint[1] - first_waypoint[1],
            waypoint[2]
        ))

    return new_waypoints

def main(args=None):
    waypoints = make_incredible_waypoints()
    run_figure('incredible', waypoints, args)

if __name__ == '__main__':
    import matplotlib.pyplot as plt

    waypoints = np.array(make_incredible_waypoints())

    ax = plt.figure().add_subplot(projection='3d')
    ax.plot(waypoints[:, 0], waypoints[:, 1], waypoints[:,2])
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_zlabel('z [m]')
    ax.set_box_aspect(np.ptp(waypoints, axis=0) + 1e-9)
    plt.savefig('figure.png')
