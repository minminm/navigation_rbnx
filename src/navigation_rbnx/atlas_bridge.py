#!/usr/bin/env python3
"""Atlas bridge for navigation_rbnx.

Registers navigate/status/cancel contracts with robonix-atlas (MCP),
bridges to Nav2 NavigateToPose action client over ROS2.

Pattern follows tiago_bridge/node.py (navigate section, lines 401-670).
"""
import json
import math
import os
import sys
import threading
import time
import uuid
from pathlib import Path
from queue import Queue

# ── Proto / codegen imports ──────────────────────────────────────────────────

def _ensure_proto():
    d = Path(__file__).resolve().parent
    while d.parent != d:
        for name in ("proto_stubs", "proto_gen"):
            p = d / name
            if p.is_dir() and (p / "robonix_runtime_pb2.py").exists():
                sys.path.insert(0, str(p))
                return
        d = d.parent

_ensure_proto()

# Attempt to import MCP codegen types; fall back gracefully
try:
    from robonix_py import mcp_contract
    from mcp.server.fastmcp import FastMCP
    _HAS_MCP = True
except ImportError:
    _HAS_MCP = False

import grpc
import robonix_runtime_pb2 as pb
import robonix_runtime_pb2_grpc as pb_grpc

# ── Lazy ROS2 imports (only in ROS2 thread) ──────────────────────────────────

_rclpy = None
_PoseStamped = None
_Twist = None
_NavigateToPose = None

def _import_ros2():
    global _rclpy, _PoseStamped, _Twist, _NavigateToPose
    import rclpy as _rclpy
    from geometry_msgs.msg import PoseStamped as _PS, Twist as _T
    _PoseStamped = _PS
    _Twist = _T
    try:
        from nav2_msgs.action import NavigateToPose as _NTP
        _NavigateToPose = _NTP
    except ImportError:
        _NavigateToPose = None

# ── Shared state ─────────────────────────────────────────────────────────────

_ros_node = None
_nav_client = None
_nav_action_ready = False
_goal_pub = None
_lock = threading.Lock()
_goal_states: dict[str, dict] = {}
_goal_handles: dict = {}
_nav_queue: Queue = Queue()

NODE_ID = "com.go2.navigation"
NAMESPACE = "robonix/srv/navigation"

# ── MCP tool handlers ────────────────────────────────────────────────────────

if _HAS_MCP:
    mcp_app = FastMCP("go2-navigation")

    # Codegen types will be available after rbnx codegen --mcp
    try:
        mcp_gen_dir = Path(__file__).resolve().parent.parent.parent
        for name in ("robonix_mcp_types", "proto_stubs", "proto_gen"):
            p = mcp_gen_dir / name
            if p.is_dir():
                sys.path.insert(0, str(p))

        from geometry_msgs_mcp import PoseStamped
        from std_msgs_mcp import String
    except ImportError:
        PoseStamped = None
        String = None

    if PoseStamped and String:
        @mcp_contract(mcp_app, contract_id="robonix/srv/navigation/navigate")
        def base_navigate(msg: PoseStamped) -> String:
            """Navigate the robot to a target pose.
            Contract: robonix/srv/navigation/navigate.
            Returns JSON with goal_id."""
            frame_id = msg.header.frame_id or "map"
            x = msg.pose.position.x
            y = msg.pose.position.y
            qz = msg.pose.orientation.z
            qw = msg.pose.orientation.w
            yaw = 2.0 * math.atan2(qz, qw)

            gid = str(uuid.uuid4())
            _nav_queue.put((gid, float(x), float(y), float(yaw), frame_id))
            with _lock:
                _goal_states[gid] = {"status": "QUEUED", "accepted": False}
            return String(data=json.dumps({
                "goal_id": gid, "status": "queued",
                "nav_action": _nav_action_ready,
            }))

        @mcp_contract(mcp_app, contract_id="robonix/srv/navigation/status")
        def base_nav_status(msg: String) -> String:
            """Get navigation status for a goal_id.
            Contract: robonix/srv/navigation/status."""
            gid = msg.data
            with _lock:
                st = _goal_states.get(gid)
            if st is None:
                result = {"error": "unknown goal_id", "goal_id": gid}
            else:
                result = {"goal_id": gid, **st}
            return String(data=json.dumps(result))

        @mcp_contract(mcp_app, contract_id="robonix/srv/navigation/cancel")
        def base_nav_cancel(msg: String) -> String:
            """Cancel a navigation goal.
            Contract: robonix/srv/navigation/cancel."""
            gid = msg.data
            with _lock:
                gh = _goal_handles.get(gid)
            if gh is None:
                result = {"error": "no active goal handle", "goal_id": gid}
            else:
                gh.cancel_goal_async()
                result = {"goal_id": gid, "status": "cancel_requested"}
            return String(data=json.dumps(result))

# ── Nav2 ROS2 action client ─────────────────────────────────────────────────

def _goal_status_name(status_code):
    names = {1: "ACCEPTED", 2: "EXECUTING", 3: "CANCELING",
             4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}
    return names.get(status_code, f"UNKNOWN({status_code})")


def _feedback_cb(gid, _feedback):
    with _lock:
        if gid in _goal_states:
            _goal_states[gid]["feedback"] = "navigating"


def _goal_response_cb(fut, gid):
    try:
        gh = fut.result()
    except Exception as e:
        with _lock:
            _goal_states[gid] = {"status": "FAILED", "accepted": False, "error": str(e)}
        return
    if not gh.accepted:
        with _lock:
            _goal_states[gid] = {"status": "REJECTED", "accepted": False}
        return
    with _lock:
        _goal_handles[gid] = gh
        _goal_states[gid] = {"status": "ACCEPTED", "accepted": True}
    res_fut = gh.get_result_async()
    res_fut.add_done_callback(lambda f: _result_cb(f, gid))


def _result_cb(fut, gid):
    try:
        res = fut.result()
        status = getattr(res, "status", None)
        st_name = _goal_status_name(status) if status is not None else "UNKNOWN"
        with _lock:
            _goal_states[gid] = {"status": st_name, "accepted": True, "terminal": True}
            _goal_handles.pop(gid, None)
    except Exception as e:
        with _lock:
            _goal_states[gid] = {"status": "FAILED", "accepted": True,
                                 "error": str(e), "terminal": True}
            _goal_handles.pop(gid, None)


def _make_pose_stamped(node, frame_id, x, y, yaw):
    goal = _PoseStamped()
    goal.header.frame_id = frame_id
    goal.header.stamp = node.get_clock().now().to_msg()
    goal.pose.position.x = float(x)
    goal.pose.position.y = float(y)
    goal.pose.position.z = 0.0
    goal.pose.orientation.z = math.sin(yaw / 2.0)
    goal.pose.orientation.w = math.cos(yaw / 2.0)
    return goal


def _dispatch_nav_goal(node, gid, x, y, yaw, frame_id):
    pose = _make_pose_stamped(node, frame_id, x, y, yaw)
    if _nav_client is not None and _nav_action_ready:
        goal_msg = _NavigateToPose.Goal()
        goal_msg.pose = pose
        send_future = _nav_client.send_goal_async(
            goal_msg,
            feedback_callback=lambda fb: _feedback_cb(gid, fb),
        )
        send_future.add_done_callback(lambda f, g=gid: _goal_response_cb(f, g))
        with _lock:
            _goal_states[gid] = {"status": "SENT", "accepted": False}
        return
    _goal_pub.publish(pose)
    with _lock:
        _goal_states[gid] = {"status": "PUBLISHED_TOPIC", "accepted": True,
                              "topic": "/goal_pose"}


def _start_ros2():
    global _ros_node, _goal_pub, _nav_client, _nav_action_ready
    _import_ros2()
    _rclpy.init()

    from rclpy.executors import MultiThreadedExecutor
    from rclpy.action import ActionClient

    node = _rclpy.create_node("navigation_rbnx_bridge")
    _ros_node = node

    _goal_pub = node.create_publisher(_PoseStamped, "/goal_pose", 1)

    if _NavigateToPose is not None:
        _nav_client = ActionClient(node, _NavigateToPose, "navigate_to_pose")
        wait_sec = float(os.environ.get("NAV2_ACTION_WAIT_SEC", "30"))
        _nav_action_ready = _nav_client.wait_for_server(timeout_sec=wait_sec)
        if not _nav_action_ready:
            print("[navigation_rbnx] WARNING: NavigateToPose action not ready; using /goal_pose fallback")
        else:
            print("[navigation_rbnx] NavigateToPose action client connected")

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    print("[navigation_rbnx] ROS2 bridge active")
    while _rclpy.ok():
        executor.spin_once(timeout_sec=0.05)
        while not _nav_queue.empty():
            gid, x, y, yaw, frame_id = _nav_queue.get_nowait()
            _dispatch_nav_goal(node, gid, x, y, yaw, frame_id)


# ── MCP HTTP server ──────────────────────────────────────────────────────────

def _start_mcp_http(port):
    import uvicorn
    app = mcp_app.streamable_http_app()
    uvicorn.run(app, host="0.0.0.0", port=port, log_level="warning")


def _pick_mcp_listen_port():
    import socket
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


# ── Atlas registration helpers ───────────────────────────────────────────────

def _single_tool_meta(name, description, input_schema):
    return json.dumps({
        "transport": "mcp",
        "tools": [{
            "name": name,
            "description": description,
            "input_schema": input_schema,
        }],
    })


def _heartbeat_loop(stub, node_id):
    while True:
        time.sleep(15.0)
        try:
            stub.NodeHeartbeat(pb.NodeHeartbeatRequest(node_id=node_id))
        except Exception as e:
            print(f"[navigation_rbnx] heartbeat failed: {e}")


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    atlas_addr = os.environ.get("ROBONIX_ATLAS", "localhost:50051")
    channel = grpc.insecure_channel(atlas_addr)
    stub = pb_grpc.RobonixRuntimeStub(channel)

    skill_md = ""
    skill_path = Path(__file__).resolve().parent.parent.parent / "skills" / "navigation" / "SKILL.md"
    if skill_path.is_file():
        skill_md = skill_path.read_text(encoding="utf-8")

    stub.RegisterNode(pb.RegisterNodeRequest(
        node_id=NODE_ID,
        namespace=NAMESPACE,
        kind="service",
        skill_md=skill_md,
        distro=os.environ.get("ROBONIX_DISTRO", "humble"),
        container_id=os.environ.get("ROBONIX_CONTAINER_ID", ""),
    ))
    print(f"[navigation_rbnx] registered node {NODE_ID}")

    mcp_port = _pick_mcp_listen_port()

    if _HAS_MCP and PoseStamped and String:
        stub.DeclareInterface(pb.DeclareInterfaceRequest(
            node_id=NODE_ID, name="base_navigate",
            supported_transports=["mcp"],
            metadata_json=_single_tool_meta(
                "base_navigate",
                "Navigate to pose. geometry_msgs/PoseStamped → std_msgs/String (goal_id).",
                PoseStamped.json_schema(),
            ),
            listen_port=mcp_port,
            contract_id="robonix/srv/navigation/navigate",
        ))

        stub.DeclareInterface(pb.DeclareInterfaceRequest(
            node_id=NODE_ID, name="base_nav_status",
            supported_transports=["mcp"],
            metadata_json=_single_tool_meta(
                "base_nav_status",
                "Get navigation status. std_msgs/String (goal_id) → std_msgs/String (status JSON).",
                String.json_schema(),
            ),
            listen_port=mcp_port,
            contract_id="robonix/srv/navigation/status",
        ))

        stub.DeclareInterface(pb.DeclareInterfaceRequest(
            node_id=NODE_ID, name="base_nav_cancel",
            supported_transports=["mcp"],
            metadata_json=_single_tool_meta(
                "base_nav_cancel",
                "Cancel navigation goal. std_msgs/String (goal_id) → std_msgs/String.",
                String.json_schema(),
            ),
            listen_port=mcp_port,
            contract_id="robonix/srv/navigation/cancel",
        ))
        print(f"[navigation_rbnx] declared 3 MCP contracts on port {mcp_port}")
    else:
        print("[navigation_rbnx] WARNING: MCP codegen types not found, skipping MCP registration")

    threading.Thread(target=_heartbeat_loop, args=(stub, NODE_ID), daemon=True).start()
    threading.Thread(target=_start_ros2, daemon=True).start()

    if _HAS_MCP and PoseStamped and String:
        threading.Thread(target=_start_mcp_http, args=(mcp_port,), daemon=True).start()

    print("[navigation_rbnx] ready")
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
