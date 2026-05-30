cmake_minimum_required(VERSION 3.0)
project(pcd_test)

set(CMAKE_CXX_STANDARD 14)

# ================= 配置 OctoMap 的本地路径 =================
include_directories(./include)

# 指定库文件路径 (通常在源码下的 lib 目录里)

# 寻找 PCL (假设 PCL 是系统安装的)
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcd2octomap pcd2octomap.cpp)

# 手动链接具体的库文件名
# OctoMap 编译后通常生成 liboctomap.so 和 liboctomath.so
target_link_libraries(pcd2octomap 
    ${PCL_LIBRARIES}
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/liboctomap.so.1.10
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/liboctomath.so.1.10
)

# 3. 设置 RPATH（加在这里！）
set_target_properties(pcd2octomap PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "$ORIGIN/../lib"
)
