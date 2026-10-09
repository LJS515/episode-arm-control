#!/usr/bin/env python3
"""Generate the SRDF collision matrix for the official model (Setup-Assistant method).

Run while move_group is up with the official model and an SRDF that disables
only the Adjacent pairs:

    python3 moveit_config/tools/generate_collision_matrix.py --samples 2000 \
        --srdf moveit_config/config/episode1_official.srdf --write

For each sampled joint state (uniform within the URDF limits) it calls
/check_state_validity and records which link pairs are in contact.
  * pair in contact in the default (all-zero) posture  -> reason="Default"
  * pair never in contact in any sample                -> reason="Never"
  * pairs sometimes in contact stay ENABLED (real self-collision risk)
Joint limits are read from the official URDF (evidence B).
"""

import argparse
import itertools
import math
import random
import re
import sys
from pathlib import Path

import rclpy
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetStateValidity
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
LINKS = ["base_link", "link1", "link2", "link3", "link4", "link5", "link6"]
LIMITS_DEG = [(-180, 160), (-90, 90), (-83, 80), (-30, 305), (-110, 110), (-30, 305)]  # official URDF
ADJACENT = {tuple(sorted(p)) for p in zip(LINKS, LINKS[1:])}


def contacts(node, cli, q):
    req = GetStateValidity.Request()
    req.group_name = "arm"
    req.robot_state = RobotState(joint_state=JointState(name=JOINTS, position=[float(v) for v in q]))
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    res = fut.result()
    pairs = set()
    for c in res.contacts:
        a, b = c.contact_body_1, c.contact_body_2
        if a in LINKS and b in LINKS and a != b:
            pairs.add(tuple(sorted((a, b))))
    return pairs


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--samples", type=int, default=2000)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--srdf", default=None)
    ap.add_argument("--write", action="store_true")
    args = ap.parse_args()
    rclpy.init()
    node = Node("episode_collision_matrix")
    # spin_until_future_complete per call (no background spin thread: a daemon
    # spin thread raced with shutdown and aborted the process at exit)
    cli = node.create_client(GetStateValidity, "/check_state_validity")
    if not cli.wait_for_service(timeout_sec=30):
        print("check_state_validity not available", file=sys.stderr)
        return 2
    default = contacts(node, cli, [0.0] * 6)
    rng = random.Random(args.seed)
    seen = set(default)
    for _ in range(args.samples):
        q = [math.radians(rng.uniform(lo, hi)) for lo, hi in LIMITS_DEG]
        seen |= contacts(node, cli, q)
    all_pairs = {tuple(sorted(p)) for p in itertools.combinations(LINKS, 2)}
    default_pairs = sorted(default - ADJACENT)
    never_pairs = sorted(all_pairs - seen - ADJACENT - set(default_pairs))
    enabled = sorted(all_pairs - ADJACENT - set(default_pairs) - set(never_pairs))
    print(f"samples={args.samples} seed={args.seed}")
    print("Default (in contact at zero pose):", default_pairs)
    print("Never in contact:", never_pairs)
    print("Kept enabled (can collide):", enabled)
    lines = [f'  <disable_collisions link1="{a}" link2="{b}" reason="Default"/>' for a, b in default_pairs]
    lines += [f'  <disable_collisions link1="{a}" link2="{b}" reason="Never"/>' for a, b in never_pairs]
    if args.write and args.srdf:
        p = Path(args.srdf)
        s = p.read_text(encoding="utf-8")
        block = "  <!-- GENERATED-PAIRS-BEGIN -->\n" + "\n".join(lines) + ("\n" if lines else "") + "  <!-- GENERATED-PAIRS-END -->"
        s = re.sub(r"  <!-- GENERATED-PAIRS-BEGIN -->.*?  <!-- GENERATED-PAIRS-END -->", block, s, flags=re.S)
        p.write_text(s, encoding="utf-8")
        print(f"wrote {len(lines)} pairs into {p}")
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
