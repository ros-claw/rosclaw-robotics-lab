#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
# Three short goals derived from official map pixels, for environment validation only.
# Never use this goal sender in the ROSClaw Agent demonstration.
exec "$CHALLENGE_DIR/scripts/ros-container.sh" timeout 600 ros2 launch \
  isaac_ros_navigation_goal isaac_ros_navigation_goal.launch.xml \
  goal_generator_type:=GoalReader goal_text_file_path:=/lab/config/baseline-goals.txt \
  iteration_count:=3 use_sim_time:=true initial_pose:='[-6.0,-1.0,0.0,0.0,0.0,1.0,0.0]' \
  initial_pose_settle_time_sec:=3.0
