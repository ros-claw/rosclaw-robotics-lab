"""Check that new visual decisions belong to the same stationary fork context."""

import math


class FreshFrames:
    def __init__(self, episode):
        self.episode = episode
        self.previous = None
        self.latest_stamp = -1

    def after_verified_stop(self):
        self.previous = None

    def check(self, observation):
        if observation["episode_id"] != self.episode or observation["epoch"] != 0:
            raise ValueError("reobservation belongs to a different world or epoch")
        current_map = stamped_map_to_odom(observation)
        stamp = int(observation["stamp_ns"])
        if stamp <= self.latest_stamp:
            raise ValueError("reobservation must have a newer RGB stamp")
        if self.previous is not None:
            if observation["localization_revision"] != self.previous["localization_revision"]:
                raise ValueError("localization revision changed during reobservation")
            previous_map = stamped_map_to_odom(self.previous)
            delta = math.dist(current_map["xyz_m"], previous_map["xyz_m"])
            dot = abs(sum(a * b for a, b in zip(current_map["quaternion_xyzw"],
                                               previous_map["quaternion_xyzw"], strict=True)))
            if delta > 0.15 or dot < math.cos(0.15 / 2):
                raise ValueError("map to odom changed during reobservation")
        self.previous = observation
        self.latest_stamp = stamp


def stamped_map_to_odom(observation):
    transform = next((tf for tf in observation["transforms"]
                      if tf["target"] == "map" and tf["source"] == "odom"), None)
    if transform is None or transform["stamp_ns"] != observation["stamp_ns"]:
        raise ValueError("reobservation requires map to odom TF at the RGB stamp")
    xyz, quaternion = transform["xyz_m"], transform["quaternion_xyzw"]
    if (len(xyz) != 3 or len(quaternion) != 4 or not all(map(math.isfinite, xyz))
            or not math.isclose(math.hypot(*quaternion), 1.0, abs_tol=1e-3)):
        raise ValueError("invalid localization in visual reobservation")
    return transform
