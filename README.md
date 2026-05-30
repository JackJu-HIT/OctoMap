这份 README 可以按照以下结构进行润色，使其显得更加专业且易于被其他开发者使用。

---

# PCD to OctoMap Converter

### 🚀 项目简介
本项目是一个轻量级的 C++ 工具，专门用于将 **PCD (Point Cloud Data)** 点云文件转换为 **OctoMap (.bt)** 二进制地图。本项目旨在为 **3D A* 路径规划**、机器人导航及环境建模提供高效的底层地图转换支持。

**核心优势：**
*   **零配置依赖**：已将 OctoMap 头文件及 x86_64 动态库进行封装，无需在系统中预装 OctoMap 源码。
*   **易于集成**：仅需 PCL (Point Cloud Library) 基础依赖，即可快速嵌入到你的机器人开发框架中。
*   **去噪处理**：内置体素点数过滤与连通域去噪算法，生成的 OctoMap 更加干净。

---

### 🛠 环境要求

在开始编译前，请确保系统中已安装 PCL 库及可视化工具。

1. **安装 PCL 库：**
   ```bash
   sudo apt update
   sudo apt install libpcl-dev
   ```

2. **安装 Octovis 可视化工具：**
   ```bash
   sudo apt install octovis
   ```

---

### 📂 项目结构
```text
.
├── include/       # OctoMap 头文件
├── lib/           # 预编译的 OctoMap 动态库 (x86_64)
├── pcd_files/     # 存放待转换的 PCD 文件
├── pcd2octomap.cpp # 核心转换程序
└── CMakeLists.txt # 项目自动化构建脚本
```

---

### 🔨 编译构建

本项目使用标准的 CMake 构建流程：

```bash
# 1. 创建构建目录
mkdir build && cd build

# 2. 生成 Makefile
cmake ..

# 3. 编译
make
```

---

### 🏃 执行转换

编译完成后，直接运行生成的可执行程序即可完成转换。

1. **运行程序：**
   ```bash
   ./pcd2octomap
   ```
   *程序会自动读取配置好的 PCD 文件，并在当前目录下生成 `result_cleaned.bt` 文件。*

2. **可视化结果：**
   转换完成后，使用 `octovis` 查看生成的地图：
   ```bash
   octovis result_cleaned.bt
   ```
   *提示：在 octovis 界面中，可以通过拖动左侧 "Depth" 滑块查看不同层级的地图精细度。*

---

### 🧭 后续计划：3D A* 规划
本项目生成的 `.bt` 文件可直接用于 A* 算法的碰撞检测。
*   **点查询**：利用 `tree->search(p)` 快速判断坐标是否被占据。
*   **体积查询**：利用 `leaf_bbx_iterator` 进行无人机包围盒碰撞检测。

---

### 💡 小贴士
*   **库路径问题**：如果运行时提示找不到 `.so` 文件，请尝试在执行前设置：  
    `export LD_LIBRARY_PATH=$(pwd)/lib/pcd_convert_octomap/lib:$LD_LIBRARY_PATH`
*   **分辨率调整**：可在 `pcd2octomap.cpp` 中修改 `resolution` 参数，以适应不同尺寸的场景。
