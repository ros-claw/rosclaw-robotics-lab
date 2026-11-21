"""Export the neutral layout, visual variant and Nav2 map from one geometry recipe."""

import argparse
import hashlib
import json
from pathlib import Path
import sys

import yaml

ROOT = Path(__file__).resolve().parents[1]
GLYPHS = {
    "A": ("01110", "10001", "10001", "11111", "10001", "10001", "10001"),
    "B": ("11110", "10001", "10001", "11110", "10001", "10001", "11110"),
}
COLORS = {"A": [0.68, 0.21, 0.06], "B": [0.045, 0.25, 0.86]}


def cube(xyz, size, color, collision):
    return {"kind": "box", "position": xyz, "size": size,
            "color": color, "collision": collision}


def occupancy(layout, destination):
    resolution = 0.05
    x0, y0, x1, y1 = layout["bounds_xyxy"]
    width, height = round((x1 - x0) / resolution), round((y1 - y0) / resolution)
    boxes = [(x - sx / 2, y - sy / 2, x + sx / 2, y + sy / 2)
             for barrier in layout["obstacles"]
             for (x, y, _), (sx, sy, _) in [(barrier["xyz"], barrier["size"])]]
    pixels = bytearray()
    for row in range(height):
        y = y1 - (row + 0.5) * resolution
        for column in range(width):
            x = x0 + (column + 0.5) * resolution
            blocked = any(a <= x <= c and b <= y <= d for a, b, c, d in boxes)
            pixels.append(0 if blocked else 254)
    (destination / "map.pgm").write_bytes(f"P5\n{width} {height}\n255\n".encode() + pixels)
    (destination / "map.yaml").write_text(yaml.safe_dump({
        "image": "map.pgm", "resolution": resolution, "origin": [x0, y0, 0.0],
        "negate": 0, "occupied_thresh": 0.65, "free_thresh": 0.196,
    }, sort_keys=False))


def build(recipe, variant, out, nvidia, controller):
    from simscene.usd_export import export_scene

    layout = yaml.safe_load(recipe.read_text())
    nav = out / "nav"
    sim = out / "sim"
    nav.mkdir(parents=True, exist_ok=False)
    sim.mkdir()
    objects = {
        "floor": cube([0, 0, -0.06], [13.0, 9.0, 0.12], [0.36, 0.39, 0.43], True),
        "room_light": {"kind": "light", "light": {"type": "dome"}, "intensity": 1500.0},
    }
    for obstacle in layout["obstacles"]:
        objects[obstacle["id"]] = cube(obstacle["xyz"], obstacle["size"], [0.58, 0.61, 0.66], True)
    labels = ("A", "B") if variant == "original" else ("B", "A")
    result = {}
    for gate, label in zip(layout["gates"], labels, strict=True):
        gate_id = gate["gate_id"]
        # Mount both signs on the divider face so high LiDAR returns do not block a lane.
        y = 0.48 if gate["lower_y"] > 0 else -0.48
        objects[f"panel-{gate_id}"] = cube([-0.222, y, 0.61], [0.040, 0.68, 0.52], COLORS[label], False)
        for row, pixels in enumerate(GLYPHS[label]):
            for column, filled in enumerate(pixels):
                if filled == "1":
                    objects[f"glyph-{gate_id}-{row}-{column}"] = cube(
                        [-0.245, y + (2 - column) * 0.054, 0.61 + (3 - row) * 0.054],
                        [0.007, 0.054, 0.054], [0.94, 0.94, 0.94], False,
                    )
        result[gate_id] = label
    export_scene({"revision": 0, "objects": objects}, sim / "layout.usda")
    occupancy(layout, nav)
    parameters = yaml.safe_load((nvidia / "humble_ws/src/navigation/carter_navigation/params/"
                                  "carter_navigation_params.yaml").read_text())
    x, y, yaw = layout["spawn_xy_yaw"]
    parameters["amcl"]["ros__parameters"]["initial_pose"] = {"x": x, "y": y, "z": 0.0, "yaw": yaw}
    for server in ("global_costmap", "local_costmap"):
        # Replace the warehouse preset's 25 cm padding for the measured graybox lanes.
        parameters[server][server]["ros__parameters"]["footprint_padding"] = 0.05
    if controller == "mppi":
        overlay = yaml.safe_load((ROOT / "config/nav2_mppi.yaml").read_text())
        for node, patch in overlay.items():
            parameters[node]["ros__parameters"].update(patch["ros__parameters"])
    (nav / "params.yaml").write_text(yaml.safe_dump(parameters, sort_keys=False))
    public = {**layout, "frame_id": "map", "resolution_m": 0.05,
              "map_sha256": hashlib.sha256((nav / "map.pgm").read_bytes()).hexdigest()}
    (nav / "public_scene_manifest.json").write_text(json.dumps(public, indent=2) + "\n")
    (sim / "spawn.json").write_text(json.dumps(layout["spawn_xy_yaw"]) + "\n")
    (out / "evaluator_manifest.json").write_text(json.dumps({
        "label_by_gate": result,
        "expected_gate": next(gate for gate, label in result.items() if label == "B"),
        "variant": variant,
    }, indent=2) + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("out", type=Path)
    parser.add_argument("--recipe", type=Path, default=ROOT / "scene/layouts/fork.yaml")
    parser.add_argument("--variant", choices=["original", "swapped"], default="original")
    parser.add_argument("--controller", choices=["dwb", "mppi"], default="dwb")
    parser.add_argument("--nvidia", type=Path, required=True)
    parser.add_argument("--simscene", type=Path, required=True)
    args = parser.parse_args()
    sys.path.insert(0, str(args.simscene / "src"))
    build(args.recipe, args.variant, args.out, args.nvidia, args.controller)
