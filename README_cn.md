# Car 基础信息
**参赛队伍：比好就去开派队  
****学校名称：哈尔滨工业大学（威海）  
****队长姓名：李成谋  
****联系电话：15266101491  
****队员姓名：林印、刘宛韵、李政硕、汤奕乔  
****使用型号：RDK S100  
****参赛模式：全自动模式  
**
整体技术路线与系统构成概述：<br>
1.用nav2来完成整图的导航和避障，手绘地图<br>
2.微信opencv库完成扫码<br>
3.云端+本地部署大模型<br>

---

# RDK S100 智能车竞赛赛后报告
  
**目标平台**: 地瓜机器人RDK系列  
**竞赛场景**: 全国大学生智能汽车竞赛 — 智慧医疗赛项  
**分析日期**: 2026-07-27

---

## 一、整体系统架构
### 1.0 项目概览
本系统是一个面向 **全国大学生智能汽车竞赛智慧医疗赛项** 的完整 RDK X5 自主导航方案。项目基于 **ROS2 Humble** + **地平线 TROS SDK**，覆盖了从传感器驱动、感知、SLAM、导航规划、运动控制到行为决策的完整机器人软件栈。



### 1.1 硬件架构
```plain
┌─────────────────────────────────────────────────────────────┐
│                  RDK S100 主控 (BPU + CPU)                   │
│  ┌──────────────────────────────────────────────────────┐   │
│  │  ROS2 Humble + TROS (Horizon Robotics SDK)           │   │
│  │  ├─ BPU: YOLO 目标检测 (硬件加速推理)                   │   │
│  │  ├─ CPU: SLAM / 导航 / 决策 / 通信                     │   │
│  │  └─ BPU/CPU: VLM 大模型推理（本地版）                   │   │
│  └──────────────────────────────────────────────────────┘   │
└──────────┬──────────┬──────────┬──────────┬─────────────────┘
           │          │          │          │
     ┌─────┴────┐ ┌───┴───┐ ┌───┴───┐ ┌───┴────┐
     │ LSLIDAR  │ │Aurora │ │ USB   │ │OriginCar│
     │  LSN10   │ │ 930   │ │Camera │ │  底盘   │
     │ 360°激光  │ │深度相机│ │       │ │(阿克曼) │
     │          │ │			  │ │		    │ │+IMU+Odom│
     └──────────┘ └───────┘ └───────┘ └─────────┘
```

| 硬件 | 型号/规格 | 用途 |
| --- | --- | --- |
| 主控 | RDK S100 (BPU 80 TOPS) | 整体计算、AI推理 |
| 激光雷达 | LSLIDAR LSN10 | 360° 环境感知、SLAM建图 |
| 深度相机 | Aurora 930  | RGB-D 深度感知、避障 |
| USB相机 | 1920×1080 | 二维码识别、VLM场景理解 |
| 底盘 | OriginCar Pro(阿克曼转向) | 运动执行、里程计反馈 |


### 1.2 软件架构 (ROS2 Workspace 总览)
```plain
rdk-x5-main/
├── ros2_ws/                    # 【核心】主导航与控制工作空间
│   └── src/
│       ├── nav_pkg/             # 自定义导航插件集
│       ├── nav2_bringup/        # Nav2 启动配置 (launch)
│       ├── nav2_costmap/     ， # 代价地图 
│       ├── nav2_controller/     # 控制器
│       ├── obstacle_info_bridge/ # YOLO检测→障碍物坐标桥接
│       ├── visual_avoidance_controller/ # 视觉避障控制器
│       └── nodes/               # BehaviorTree 行为节点
│           ├── Initialise.cpp   # 初始化
│           ├── IsQRCodeReceived.cpp # 二维码方向判断
│           ├── IsHalf.cpp       # 半场检测 (触发QR)
│           ├── IsOpen.cpp       # VLM触发
│           ├── IsOpen_new.cpp   # VLM触发2.0
│           └── letsgoback.cpp   # 阿克曼倒车 
├── lidar_ws/                    # 激光雷达驱动
│   └── src/lslidar_driver/      # LSN10 系列驱动
├── slam_test_ws/                # SLAM 工作空间
│   └── src/slam_pkg/            # SLAM Toolbox 在线异步建图
├── VLMmodel_ws/                 # 视觉大模型工作空间
│   └── src/large_model/         
├── qrcode_decoding/             # 二维码解码
│   └── src/qrcode_decoding/     # 图像预处理+解码
└── llama.cpp/                   # 本地LLM推理目录 
```

---

## 二、软件系统设计思路
### 2.1 分层架构设计
系统采用 **感知—规划—控制** 三层经典机器人架构，每一层内部再细分为多个功能模块：

| 层次 | 模块 | 功能 |
| --- | --- | --- |
| **感知层** | SLAM (slam_toolbox) | 实时建图与定位 |
|  | YOLO  | 目标检测  |
|  | QRCode Decoder | 二维码方向指令解析 |
|  | VLM  | 医疗场景语义理解 |
| **规划层** | Planner | 规划路径 |
|  | Keepout Filter | 禁行区域过滤 |
|  | Relocalization  | 初始位姿估计 |
| **控制层** | Controller | 局部避障 |
|  | AckermannSmoother | 阿克曼转向运动学平滑 |
| **决策层** | BehaviorTree (BT.CPP) | 任务编排与状态转移 |


### 2.2 部分设计理念
#### 2.2.1 "继承优于重写" 的 Nav2 插件化扩展
所有自定义导航功能均以 **Nav2 插件** 形式实现，充分利用 Nav2 框架的生命周期管理和参数系统

#### 2.2.2 阿克曼倒车碰撞保护 (AckermannCurveBackupAction)
倒车安全机制：

```plain
代价地图快照 → 轨迹模拟 
  → 对每个步进: 采样车尾矩形区域 (3排×5列)
  → 变换到map坐标系查询代价
  → 检测到障碍物: 计算安全角度 
  → 无碰撞: 全量执行
```

### 2.3 技术选型策略
| 技术 | 选型理由 |
| --- | --- |
| ROS2 Humble + TROS | RDK S100 官方支持, 硬件加速 (BPU) 集成 |
| Nav2 | 成熟导航框架, 插件化扩展性强 |
| BehaviorTree.CPP  | 可视化任务编排, 替代硬编码状态机 |
| SLAM Toolbox (online_async) | 支持大场景在线建图, Karto 后端 |
| Qwen | 云端大模型, 医疗场景语义理解能力强 |
| WeChatQRCode (OpenCV) | 轻量级, 中文场景优化 |
| SmacPlanner2D | 全局规划 |


---

## 三、关键任务实现策略
### 3.1 任务一：自主导航与避障
**实现路径**:

```plain
SLAM建图/画图 → 预设路径点(YAML) → BehaviorTree导航 
```

### 3.2 任务二：二维码识别与方向选择
**实现路径**:

```plain
过中点检测(IsHalf) → 触发QR解码 → 解析奇偶(方向) → BT选择左/右路径
```

**核心代码链路**:

1. **触发条件** (`IsHalf`):

```plain
条件: 过中场
动作: 发布 QR_trigger = true
```

2. **QR解码** (`qrcode_decoding`):
    - USB Camera → `aurora/rgb/image_raw` 订阅
    - WeChatQRCode 解码
    - 结果: 奇数→顺时针(`1`), 偶数→逆时针(`0`)
3. **方向决策** (`IsQRCodeReceived` BT节点):

```plain
接收 /qrcode_direction (Int32)
→ 0: setOutput("direction", "left")   (逆时针)
→ 1: setOutput("direction", "right")  (顺时针)
→ BehaviorTree 据此选择 goals_left 或 goals_right 路径
```

### 3.3 任务三：视觉场景理解 (VLM)
**实现路径**:

```plain
区域检测(IsOpen) → 触发VLM → Qwen→ 医疗场景描述
```

**核心代码链路**:

1. **触发条件** (`IsOpen`):

```plain
条件达到
动作: 发布 VLM_trigger = true
```

    - 有`IsOpen_new` 版本
2. **VLM推理** (`large_model.py`):
    - USB Camera → 编码 → 图像队列
    - `VLM_trigger` 为 true 时启动推理线程
    - 调用Qwen模型
    - 提示词
    - 结果发布到 `/output_complete` (String)

### 3.4 禁区与减速区域
+ **Keepout Filter** : 设定永久禁行区
+ **Slow Area Publisher**: 在特定区域发布速度限制指令
+ **排除清除层** (`exclusion_clear_layer`): 清除非物理障碍物的标记

---

## 四、与竞赛任务与规则的适配说明
对于省赛筒较少的时候可以通过倒车更快的面向通道用更少的时间，并选择一个合适的扫码点位进行追踪，使小车在进入通道之前获得更好的位姿，因为山东的发车规则，可以采用更激进的提速策略，尝试较高速度的完成，根据现场网络情况调整大模型的触发，在不停车情况下也可以保证高速下扫到清晰的大模型图像

---

## 五、硬件选型与连接方式
选择了一款温漂和零漂较好的六轴imu模块，通过杜邦线连接到STM32,stm32与s100通过数据线连接，雷达、相机、语音模块等通过usb连接到s100上，均选择的是具有较好的性能，且可以调整参数的模块，针对于s100的供电，我们通过降压模块将25.1v的电源降至约18v保证供电稳定，用另外一条dc线连接stm32，使32板同时稳定供电

## 总结
本系统是一套基于 **RDK X5 + ROS2 Humble + Nav2** 的完整自主导航方案，覆盖了建图、定位、规划、控制、感知、决策六大模块。

**设计亮点**:

1. **插件化扩展**
2. **渐进式迭代**
3. **双阶段参数**
4. **三态状态机**
5. **多控制器并存**
6. **延迟启动**
7. **硬件加速**
8. **安全保障**

**部署环境**: RDK S100 (Ubuntu 22.04, ROS2 Humble), 以 root 用户运行



### 
  



