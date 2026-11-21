"""Sample PhysX state independently from Nav2 and ROS odometry."""

import json
import time

import omni.physics.tensors as tensors
import omni.usd
from omni.physx import get_physx_simulation_interface
from pxr import PhysxSchema, PhysicsSchemaTools, Usd, UsdPhysics

ROBOT = "/World/Nova_Carter_ROS/"
FLOOR = "/World/LunaGraybox/floor"


class PhysicalWitness:
    def __init__(self, output):
        self.stream = output.open("x")
        self.contacts = output.with_name("physx-contacts.jsonl").open("x")
        self.stage = omni.usd.get_context().get_stage()
        self.contact_pairs = set()
        with Usd.EditContext(self.stage, self.stage.GetSessionLayer()):
            for prim in self.stage.Traverse():
                if str(prim.GetPath()).startswith(ROBOT) and prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr().Set(0.0)
        self.contact_subscription = get_physx_simulation_interface().subscribe_contact_report_events(
            self.on_contact
        )
        self.next_sample = 0.1
        self.view = None
        self.body = None

    def on_contact(self, headers, _):
        for header in headers:
            if not header.num_contact_data:
                continue
            a = str(PhysicsSchemaTools.intToSdfPath(header.collider0))
            b = str(PhysicsSchemaTools.intToSdfPath(header.collider1))
            if not (a.startswith(ROBOT) or b.startswith(ROBOT)):
                continue
            pair = tuple(sorted((a, b)))
            if pair in self.contact_pairs:
                continue
            self.contact_pairs.add(pair)
            self.contacts.write(json.dumps({
                "source": "physx", "host_mono_ns": str(time.monotonic_ns()),
                "colliders": pair, "classification": "floor" if FLOOR in pair else "non_floor",
                "points": header.num_contact_data,
            }) + "\n")
            self.contacts.flush()

    def sample(self, sim_time):
        if sim_time < self.next_sample:
            return
        if self.body is None:
            self.view = tensors.create_simulation_view(
                "numpy", stage_id=omni.usd.get_context().get_stage_id(), backend="physx"
            )
            self.view.set_subspace_roots("/")
            self.body = self.view.create_rigid_body_view("/World/Nova_Carter_ROS/chassis_link")
            if self.body.count != 1:
                raise RuntimeError("PhysX chassis witness is ambiguous")
        pose = self.body.get_transforms()[0].tolist()
        velocity = self.body.get_velocities()[0].tolist()
        self.stream.write(json.dumps({
            "source": "physx", "sim_time_s": sim_time, "host_mono_ns": str(time.monotonic_ns()),
            "pose_xyz_xyzw": pose, "linear_angular_velocity": velocity,
        }, allow_nan=False) + "\n")
        self.stream.flush()
        self.next_sample = sim_time + 0.05

    def close(self):
        self.contact_subscription = None
        self.contacts.close()
        self.stream.close()
