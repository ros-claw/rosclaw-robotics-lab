"""Apply a named, bounded speed trial to matching MPPI and smoother limits."""


def apply_drive_profile(params, adaptive, profile):
    if profile == "baseline":
        return
    if profile != "quicker":
        raise ValueError("unknown drive profile")
    mppi = params["controller_server"]["ros__parameters"]["FollowPath"]
    if mppi["plugin"] != "nav2_mppi_controller::MPPIController":
        raise ValueError("the quicker profile requires the tested MPPI controller")
    smoother = params["velocity_smoother"]["ros__parameters"]
    mppi["vx_max"] = 0.65
    smoother["max_velocity"][0] = 0.65
    adaptive["path_lookahead_m"] = 3.4
    adaptive["recovery_local_lookahead_m"] = 2.0
