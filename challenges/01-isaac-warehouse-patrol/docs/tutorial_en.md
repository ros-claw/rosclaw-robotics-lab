# DGX Spark local simulation operations

This guide covers the measured environment baseline. Native Agent patrol and clean-machine reproduction are pending.

The prepared machine runs Ubuntu 24.04 aarch64, GB10 driver 580.159.03, Isaac Sim 6.1.0 and ROS 2 Jazzy. Isaac is in `~/isaacsim`; the NVIDIA workspace is in `~/IsaacSim-ros_workspaces/jazzy_ws`, pinned to IsaacSim-6.1.0. Nav2 runs in a local ARM64 image because host Nav2 is absent and passwordless sudo is unavailable. The host driver/ROS installation was not changed.

```bash
cd ~/sim/rosclaw-robotics-lab/challenges/01-isaac-warehouse-patrol
./scripts/doctor.sh
./scripts/demo.sh streaming
```

Stop a previous project session first. Startup waits for fresh independent PhysX transforms, ACTIVE Nav2 and a ready NavigateToPose action server. Cold asset/shader startup previously took about 9.5 minutes; allow up to 20 minutes. Logs are stored in reports/runs/<UTC timestamp>. No automatic goal sender starts.

Connect the [official WebRTC client](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/installation/manual_livestream_clients.html) to LAN address 192.168.43.130. External client display is unverified; an actual renderer viewport PNG is available in reports/warehouse-baseline.png.

To validate the official three-goal baseline after a fresh reset:

```bash
./scripts/smoke-official.sh > reports/manual-three-goals.log 2>&1
```

This publishes the initial AMCL pose. Do not rerun after the robot has moved without resetting the simulation. Never use it as an Agent demo.

Official defaults produced physical errors 0.107 / 0.297 / 0.543 m. A separate low-noise AMCL configuration with tighter Nav2 tolerance produced 0.146 / 0.174 / 0.079 m after reset. The independent checker correlates Nav2 success logs with fresh PhysX positions; collision acceptance remains UNKNOWN. The local official-baseline.mp4 is a timestamped real viewport frame sequence, not a continuous screen recording or an Agent promotional video. Capture defaults to 180 seconds; override ROSCLAW_CAPTURE_SECONDS as needed.

The project uses domain 61, Fast DDS and host network/IPC Docker. Clock, chassis odometry, scan, pointcloud and map reception and map→odom→base_link TF were observed. Camera render products are disabled through the session layer; LiDAR remains active. NVIDIA USD files are unchanged. Four nonessential Hawk ROS variants are disabled in the session layer; a fresh startup verified zero composition errors and zero unloaded payloads.

NVIDIA's installed US/China profiles resolve the 6.1 asset root. Set ISAACSIM_ASSET_REGION_PROFILE=china or provide ISAACSIM_ASSET_ROOT ending in /Assets/Isaac/6.1. China network loading is unverified.

```bash
./scripts/stop.sh
```

Cleanup targets project-labeled containers and the recorded simulator PID/birth identity. Semantic sites, daemon patrol adapter, real Native Agent missions, bounded recovery, collision/dwell validation, three reset missions, full tutorials, videos and GitHub publication remain pending. See the root implementation report.
