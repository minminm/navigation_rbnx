# navigation_rbnx

基于 [Nav2](https://nav2.org/) 的 Robonix 导航系统服务。

提供目标式导航（navigate）、状态查询（status）、取消（cancel）三个标准契约，通过 MCP 工具暴露给 Robonix Pilot/Executor。

默认在 Docker 容器中运行，内含 Nav2 全套节点（planner、controller、BT navigator、behavior server）。

## 架构

```
Agent ──MCP──► navigation/navigate(pose)
               │
               ▼
    atlas_bridge ──rclpy ActionClient──► Nav2 BT navigator
                                              │
                                              ▼
                                         planner_server
                                         controller_server
                                         behavior_server
                                              │
                                              ▼
                                         cmd_vel ──► prm/base/twist_in
```

## 契约

### 提供

| 契约 ID | 模式 | 说明 |
|---------|------|------|
| `robonix/srv/navigation/navigate` | rpc (MCP) | 发送导航目标，返回 goal_id |
| `robonix/srv/navigation/status` | rpc (MCP) | 查询 goal 状态 |
| `robonix/srv/navigation/cancel` | rpc (MCP) | 取消导航 goal |

### 消费

| 契约 ID | 来源 | 说明 |
|---------|------|------|
| `robonix/srv/common/map/occupancy_grid` | mapping_rbnx | Nav2 global costmap static layer |
| `robonix/srv/common/map/scan_2d` | mapping_rbnx | Nav2 costmap obstacle layer |
| `robonix/prm/base/odom` | mapping_rbnx | Nav2 odometry 输入 |

## 使用

### Docker（默认）

```bash
rbnx build -p .
rbnx start -p . -n com.go2.navigation
```

### 配置

Nav2 参数文件：`config/nav2_params.yaml`（从 GO2 move_base 配置迁移，已适配 DWB）。

关键参数（GO2 特有）：
- footprint: `[[0.2, 0.3], [0.2, -0.3], [-0.5, -0.3], [-0.5, 0.3]]`
- max_vel_x: 1.2 m/s, max_vel_y: 0.3 m/s（全向移动）
- inflation_radius: 0.55 m

## 依赖

- mapping_rbnx（提供定位和地图）
- robonix-atlas（控制面注册）

## 许可证

MulanPSL-2.0
