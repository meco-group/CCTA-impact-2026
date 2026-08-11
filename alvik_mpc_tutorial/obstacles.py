"""Shared obstacle-list parsing: 'cx,cy,r cx,cy,r ...' -> [[cx,cy,r], ...].

Used by both fake_robot.py and run_mpc.py so obstacles can be specified straight on the
command line, with no scene files and no dependency between the two on each other.
"""

DEFAULT_OBSTACLES = "0.85,0.06,0.12 1.40,-0.28,0.10"


def parse_obstacles(s):
    """'0.85,0.06,0.12 1.40,-0.28,0.10' -> [[0.85,0.06,0.12], [1.40,-0.28,0.10]]"""
    return [[float(v) for v in part.split(",")] for part in s.split()]
