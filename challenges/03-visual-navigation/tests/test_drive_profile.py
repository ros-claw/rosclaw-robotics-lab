from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from navigation.drive_profile import apply_drive_profile


def test_quicker_trial_matches_caps_and_extends_obstacle_lookahead():
    body = {"FollowPath": {"plugin": "nav2_mppi_controller::MPPIController", "vx_max": .45}}
    smoother = {"max_velocity": [.45, 0, .8], "max_decel": [-1.2, 0, -1.0]}
    parameters = {"controller_server": {"ros__parameters": body},
                  "velocity_smoother": {"ros__parameters": smoother}}
    guard = {"path_lookahead_m": 2.4, "recovery_local_lookahead_m": 1.5,
             "blocked_dwell_sim_s": .65}
    apply_drive_profile(parameters, guard, "baseline")
    assert body["FollowPath"]["vx_max"] == .45
    apply_drive_profile(parameters, guard, "quicker")
    assert body["FollowPath"]["vx_max"] == smoother["max_velocity"][0] == .65
    assert smoother["max_decel"] == [-1.2, 0, -1.0]
    assert guard["path_lookahead_m"] == 3.4
    assert guard["blocked_dwell_sim_s"] == .65
