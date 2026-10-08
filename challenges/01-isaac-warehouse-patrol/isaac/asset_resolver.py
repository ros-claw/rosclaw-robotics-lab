"""Resolve 6.1 assets using the installed NVIDIA region profiles, without guessed CDNs."""

import argparse
import os
import tomllib
from pathlib import Path
from urllib.parse import urlsplit

SCENE = "Isaac/Samples/ROS2/Scenario/carter_warehouse_navigation.usd"
ROBOT = "Isaac/Samples/ROS2/Robots/Nova_Carter_ROS.usd"


def resolve_root(installation, region="us", explicit=None):
    if region not in {"us", "china"}:
        raise ValueError("Only NVIDIA us/china profiles are supported")
    config = Path(installation) / "exts/isaacsim.storage.native/config/extension.toml"
    profiles = tomllib.loads(config.read_text())["settings"]["exts"][
        "isaacsim.storage.native"
    ]["asset_region_profiles"]
    root = (explicit or profiles[region]["asset_root"]).rstrip("/")
    if not urlsplit(root).path.endswith("/Assets/Isaac/6.1"):
        raise ValueError("Asset root must remain pinned to /Assets/Isaac/6.1")
    return root


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--installation",
        default=os.environ.get("ISAACSIM_PATH", str(Path.home() / "isaacsim")),
    )
    parser.add_argument(
        "--region", default=os.environ.get("ISAACSIM_ASSET_REGION_PROFILE", "us")
    )
    parser.add_argument("--root", default=os.environ.get("ISAACSIM_ASSET_ROOT"))
    parser.add_argument("--scene", action="store_true")
    args = parser.parse_args()
    root = resolve_root(args.installation, args.region, args.root)
    print(root + "/" + SCENE if args.scene else root)


if __name__ == "__main__":
    main()
