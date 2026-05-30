# PCD-to-OctoMap Pro: 高效点云转换与三维地图构建工具

### 🌟 项目概述
本项目是一个开箱即用的 C++ 工具库，旨在解决机器人导航中 **PCD (Point Cloud Data)** 与 **OctoMap (八叉树地图)** 转换过程中的配置繁琐、噪声干扰等痛点。本工具为 **3D A* 路径规划**、**四足机器人跨楼层导航**及环境建模提供了高性能的底层支持。

> **💡 核心设计理念**：极简部署，深度滤波，为规划而生。

---

### ✨ 核心亮点

*   **📦 零配置，开箱即用**：
    *   项目内置了封装好的 OctoMap 头文件及预编译的 **x86_64 动态库**。
    *   **无需**在系统中全局安装 OctoMap 源码，避免版本冲突，极大降低集成门槛。
*   **🛡️ 双重深度去噪**：
    *   **体素点数过滤 (Voxel Filtering)**：自动剔除点数稀疏的无效体素。
    *   **连通域聚类过滤 (Cluster Filtering)**：基于 BFS 算法识别并移除悬浮在空中的孤立噪点，生成“纯净”的 .bt 地图。
*   **🧩 易于集成**：
    *   仅依赖 PCL (Point Cloud Library)，适合快速嵌入各种机器人导航架构。

---

### 🛠️ 环境要求

在开始构建前，请确保您的开发环境满足以下要求：

1.  **PCL 库** (系统基础依赖)：
    ```bash
    sudo apt update && sudo apt install libpcl-dev
    ```
2.  **Octovis** (地图可视化必备)：
    ```bash
    sudo apt install octovis
    ```

---

### 📂 项目架构
```text
.
├── include/           # OctoMap 核心头文件
├── lib/               # 预编译动态库 (已封装 liboctomap.so / liboctomath.so)
├── pcd_files/         # 存放待转换的 PCD 源文件
├── pcd2octomap.cpp     # 核心转换逻辑（含双重滤波算法）
└── CMakeLists.txt     # 优化的自动构建脚本（支持 RPATH 自动关联）
```

---

### 🔨 编译构建

采用标准 CMake 工作流，数秒内即可完成构建：

```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
```

---

### 🏃 运行与可视化

1.  **启动转换**：
    ```bash
    ./pcd2octomap
    ```
    *程序将处理指定的 PCD 文件并输出经过清洗的 `result_cleaned.bt` 二进制地图。*

2.  **可视化分析**：
    使用 Octomap 官方工具查看转换后的 3D 效果：
    ```bash
    octovis result_cleaned.bt
    ```
    *可视化技巧：在 `octovis` 中按 `F` 键查看空闲空间，拖动 `Depth` 滑块观察多分辨率下的地图特征。*

---

### 🧭 后续应用：3D A* 路径规划
生成的 `.bt` 文件是 3D 导航的基石。在 A* 搜索中，您可以直接调用：
*   **快速碰撞检测**：通过 `tree->search(p)` 实现 $O(\log N)$ 级的点阻碍查询。
*   **安全距离考量**：结合 `leaf_bbx_iterator`，完美适配无人机或四足机器人的包围盒（Bounding Box）检测。

---

### 📖 深度解析与实战案例

本项目是**四足机器人跨楼层导航**方案的重要组成部分。关于如何利用本项目进行更复杂的 3D 全局路径规划，请参考我的深度解析文章：

🔗 **[【四足跨楼层三维全局路径规划】深入理解并使用OctoMap地图](https://mp.weixin.qq.com/s/6THL9OLRxGVo8kOa2LUtLA)**

---

### 🤝 联系与支持

欢迎扫描下方二维码或搜索关注我的微信公众号，获取更多关于**机器人运动规划、四足仿生控制**的前沿干货：

**公众号：机器人规划与控制研究所**

---

### 📝 版权声明
Copyright (c) 2025-2026 Institute of Robotics Planning and Control (IRPC).  
All rights reserved.

---

### 💡 小贴士
*   **运行报错？**：若提示 `.so` 库找不到，项目已配置 RPATH，但您也可以手动指定：`export LD_LIBRARY_PATH=$(pwd)/lib/pcd_convert_octomap/lib:$LD_LIBRARY_PATH`。
*   **分辨率设置**：在 `pcd2octomap.cpp` 中通过 `resolution` 修改精度。大型室外场景建议设为 `0.2m - 0.5m`，室内精细操作建议 `0.05m`。
