"""race_tag_graph: an offline tag-aided pose graph for RACE (piccard-inc/piccard-physical-ai#265).

EKF odometry enters once, as between-factors; each AprilTag is a fixed landmark estimated jointly, initialised at
its first sighting; the output is the dock point in base_link. One factor builder (factors.py) feeds a batch
Levenberg-Marquardt solve and an ISAM2 replay (solvers.py). The estimator reads only the sensor topics
(sensors.py); truth and the lab fuser's output are read only by the evaluation (truth.py, evaluate.py).
"""
