"""Bounded complete model-facing semantic catalog; full proofs stay daemon-owned."""


def compact_catalog(catalog, physics):
    targets = []
    for row in catalog["targets"]:
        targets.append(
            {
                "target_prim": row["target_prim"],
                "center_xy": row["center_xy"],
                "distance_from_initial_pose_m": round(
                    row["distance_from_initial_pose_m"], 3
                ),
                "west_of_initial_pose": row["west_of_initial_pose"],
                "status": row["status"],
                "proposals": [
                    {
                        k: round(v, 4) if isinstance(v, float) else v
                        for k, v in c.items()
                        if k in {"proposal_id", "x", "y", "yaw"}
                    }
                    for c in row["proposals"][:2]
                ],
            }
        )
    return {
        "requirements": [
            "Choose the user-requested USD-known shelf yourself; map west is -X, north is +Y.",
            "One proposal_id per navigation. Observe fresh candidates before each action; proposals expire.",
            "After shelf observation, navigate using return_to_initial_pose.proposal_id.",
            "Then mission.verify_and_remember with exact canonical action_ids in order; register its artifact and close TaskKernel.",
            "USD supplies semantics. Inspection scope is explicit; display cameras are not Agent sensor observations.",
        ],
        "return_to_initial_pose": catalog["return_to_initial_pose"],
        "initial_pose": catalog["initial_pose"],
        "generated_at_wall": catalog["generated_at_wall"],
        "proposal_max_wall_age_seconds": catalog["proposal_max_wall_age_seconds"],
        "body_snapshot_hash": catalog["body_snapshot_hash"],
        "inspection_scope": catalog["inspection_scope"],
        "physics": {
            k: physics[k]
            for k in [
                "observer_id",
                "wall_time",
                "sim_time",
                "physics_transforms_xyzw",
                "timeline_playing",
                "collision_count",
            ]
        },
        "targets": targets,
        "candidate_limit_per_target": 2,
        "holdout_exposed": False,
        "authorization": False,
        "evidence_domain": "SIMULATION",
    }
