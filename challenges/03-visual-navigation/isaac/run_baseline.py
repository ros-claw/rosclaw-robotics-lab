"""Own the single Isaac world and run the NVIDIA Nova Carter baseline."""

import argparse
import faulthandler
import runpy
import traceback
from pathlib import Path

from isaacsim import SimulationApp


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scene")
    parser.add_argument("asset_root")
    parser.add_argument("gpu", type=int)
    parser.add_argument("--trajectory", type=Path)
    parser.add_argument("--layout", type=Path)
    parser.add_argument("--calibration", action="store_true")
    parser.add_argument("--rgbd", action="store_true")
    parser.add_argument("--record-overview", action="store_true")
    parser.add_argument("--diagnostic-camera-yaw", type=float, default=0.0)
    parser.add_argument("--renderer", default="RaytracedLighting")
    args = parser.parse_args()

    app = SimulationApp({"headless": True, "multi_gpu": False, "active_gpu": args.gpu,
                         "physics_gpu": args.gpu, "disable_viewport_updates": True,
                         "renderer": args.renderer})
    exit_code = 0
    witness = None
    try:
        import carb
        import omni.timeline
        import omni.usd
        from isaacsim.core.experimental.utils.app import enable_extension

        settings = carb.settings.get_settings()
        settings.set("/exts/isaacsim.core.simulation_manager/default_engine", "physx")
        settings.set("/exts/isaacsim.physics.newton/auto_switch_on_startup", False)
        settings.set("/persistent/isaac/asset_root/default", args.asset_root)
        enable_extension("isaacsim.ros2.bridge")
        app.update()

        if not omni.usd.get_context().open_stage(args.scene):
            raise RuntimeError(f"Isaac could not open the baseline stage: {args.scene}")
        runpy.run_path(str(Path(__file__).with_name("baseline_stage.py")))
        if args.layout:
            from graybox import apply_graybox

            apply_graybox(omni.usd.get_context().get_stage(), args.layout)
        if args.trajectory:
            from graybox import configure_final_velocity
            from physical_witness import PhysicalWitness

            configure_final_velocity(omni.usd.get_context().get_stage())
            witness = PhysicalWitness(args.trajectory)
        if args.rgbd or args.calibration:
            from scene_and_sensors import add_front_rgbd

            add_front_rgbd(omni.usd.get_context().get_stage(), args.calibration,
                          args.diagnostic_camera_yaw)
        if args.record_overview:
            from display_camera import add_display_camera

            add_display_camera(omni.usd.get_context().get_stage())
        timeline = omni.timeline.get_timeline_interface()
        # The stock warehouse authors 1000 frames at 60 Hz. Its playback
        # loop must not wrap the physical state during a navigation episode.
        timeline.set_end_time(3600.0)
        timeline.set_looping(False)
        timeline.play()
        print("Luna baseline timeline play requested", flush=True)
        faulthandler.dump_traceback_later(45, repeat=True)
        for _ in range(3):
            app.update()
        faulthandler.cancel_dump_traceback_later()
        print(f"Isaac first updates returned, timeline={timeline.get_current_time()}", flush=True)
        while app.is_running():
            app.update()
            if witness:
                witness.sample(timeline.get_current_time())
    except KeyboardInterrupt:
        pass
    except Exception:
        traceback.print_exc()
        exit_code = 1
    finally:
        faulthandler.cancel_dump_traceback_later()
        if witness:
            witness.close()
        app.close(exit_code=exit_code)


if __name__ == "__main__":
    main()
