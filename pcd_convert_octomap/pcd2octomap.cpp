/**
 * @file      pcd2octomap.cpp
 * @brief     pcd Convert To OctoMap
 * @author    juchunyu <juchunyu@qq.com>
 * @date      2026-05-30 09:00:01 
 * @copyright Copyright (c) 2025-2026 Institute of Robotics Planning and Control (IRPC). 
 *            All rights reserved.
 */

#include <iostream>
#include <vector>
#include <deque>
#include <unordered_map>
#include <unordered_set>
#include <memory>

// PCL 
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// Octomap
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

// --- 定义 Key 结构，用于在 STL 容器中存储 OctoMap 的 Key ---
struct Key {
    unsigned int k[3];
    bool operator==(const Key& other) const {
        return k[0] == other.k[0] && k[1] == other.k[1] && k[2] == other.k[2];
    }
};

// 为 Key 提供 Hash 函数
struct KeyHash {
    std::size_t operator()(const Key& key) const {
        std::size_t seed = std::hash<unsigned int>{}(key.k[0]);
        seed ^= std::hash<unsigned int>{}(key.k[1]) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= std::hash<unsigned int>{}(key.k[2]) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        return seed;
    }
};

bool isSpaceFree(const octomap::point3d& min_pt, const octomap::point3d& max_pt, octomap::OcTree* tree) {
    // 检查给定的长方体区域内是否有任何被占据的节点
    for (octomap::OcTree::leaf_bbx_iterator it = tree->begin_leafs_bbx(min_pt, max_pt),
         end = tree->end_leafs_bbx(); it != end; ++it) {
        if (tree->isNodeOccupied(*it)) {
            return false; // 只要有一个方块被占，就说明会碰撞
        }
    }
    return true;
}

bool isPointFree(const octomap::point3d& p, octomap::OcTree* tree) {
    octomap::OcTreeNode* node = tree->search(p);
    // 如果是空指针（未知），在全局规划中我们通常认为它是可以通过的
    if (node == nullptr) {
        return true; 
    }
    // 只有明确被标记为占据的才返回 false
    return !tree->isNodeOccupied(node);
}

int main() {
    // ================= 配置区域 =================
    std::string input_pcd  = "../pcd_files/building2_9.pcd";
    std::string output_bt  = "result_cleaned.bt";
    
    double resolution           = 0.2;   // Octomap 分辨率 (米)
    int min_points_per_voxel    = 3;     // 每个方块内至少有多少个点才算存在
    int min_cluster_voxels      = 4;     // 连通的方块数少于这个值会被当做噪点删除
    // ===========================================

    // 1. 使用 PCL 加载点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_pcd, *cloud) == -1) {
        std::cerr << "Couldn't read file " << input_pcd << std::endl;
        return -1;
    }
    std::cout << "Loaded " << cloud->size() << " points." << std::endl;

    // 2. 初始化 OcTree
    auto tree = std::make_shared<octomap::OcTree>(resolution);

    // 3. 统计每个体素内的点数 (Voxel Filtering)
    std::unordered_map<Key, std::size_t, KeyHash> voxel_counts;
    for (const auto& p : cloud->points) {
        if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z)) continue;
        
        octomap::OcTreeKey raw_key;
        if (tree->coordToKeyChecked(p.x, p.y, p.z, raw_key)) {
            Key key{{raw_key.k[0], raw_key.k[1], raw_key.k[2]}};
            ++voxel_counts[key];
        }
    }

    // 4. 筛选点数达标的体素
    std::unordered_set<Key, KeyHash> occupied_keys;
    for (const auto& entry : voxel_counts) {
        if (static_cast<int>(entry.second) >= min_points_per_voxel) {
            occupied_keys.insert(entry.first);
        }
    }
    std::cout << "Voxels after point-count filtering: " << occupied_keys.size() << std::endl;

    // 5. 连通域过滤 (Cluster Filtering - BFS算法)
    if (min_cluster_voxels > 1 && !occupied_keys.empty()) {
        std::unordered_set<Key, KeyHash> filtered_keys;
        std::unordered_set<Key, KeyHash> visited;
        filtered_keys.reserve(occupied_keys.size());

        for (const auto& seed : occupied_keys) {
            if (visited.count(seed)) continue;

            std::deque<Key> queue;
            std::vector<Key> cluster;
            queue.push_back(seed);
            visited.insert(seed);

            while (!queue.empty()) {
                Key current = queue.front();
                queue.pop_front();
                cluster.push_back(current);

                // 检查 26 邻域 (3x3x3 范围)
                for (int dx = -1; dx <= 1; ++dx) {
                    for (int dy = -1; dy <= 1; ++dy) {
                        for (int dz = -1; dz <= 1; ++dz) {
                            if (dx == 0 && dy == 0 && dz == 0) continue;
                            
                            Key neighbor{{
                                static_cast<unsigned int>(current.k[0] + dx),
                                static_cast<unsigned int>(current.k[1] + dy),
                                static_cast<unsigned int>(current.k[2] + dz)
                            }};

                            if (occupied_keys.count(neighbor) && !visited.count(neighbor)) {
                                visited.insert(neighbor);
                                queue.push_back(neighbor);
                            }
                        }
                    }
                }
            }

            // 只有足够大的簇才保留
            if (static_cast<int>(cluster.size()) >= min_cluster_voxels) {
                for(const auto& k : cluster) filtered_keys.insert(k);
            }
        }
        occupied_keys = std::move(filtered_keys);
    }
    std::cout << "Voxels after cluster filtering: " << occupied_keys.size() << std::endl;

    // 6. 填充 OcTree
    for (const auto& key : occupied_keys) {
        octomap::OcTreeKey octo_key;
        octo_key.k[0] = key.k[0]; octo_key.k[1] = key.k[1]; octo_key.k[2] = key.k[2];
        tree->updateNode(tree->keyToCoord(octo_key), true);
    }

    // tree->updateNode(octomap::point3d(1, 1,1), false);


    tree->updateInnerOccupancy();
    
    // 7. 保存结果
    if (tree->writeBinary(output_bt)) {
        std::cout << "Success! Saved to " << output_bt << std::endl;
    }
     // 6. (可选) 尝试自动调用 octovis 可视化
    std::cout << "\nConversion finished. To visualize, run:\n";
    std::cout << "octovis " << output_bt << std::endl;

    int count = 0;
    // 2. 使用叶子节点迭代器遍历整个树
    // tree->begin_leafs() 会遍历所有存在的节点（包括空闲和占据）
    for (octomap::OcTree::leaf_iterator it = tree->begin_leafs(), end = tree->end_leafs(); it != end; ++it) {
        
        // 3. 只打印“被占据”的节点
        if (tree->isNodeOccupied(*it)) {
            // 获取当前方块的中心坐标
            octomap::point3d p = it.getCoordinate();
            
            // 获取当前节点的大小（反映了该节点在八叉树中的深度）
            double size = it.getSize();

            // 打印坐标
            std::cout << "Node [" << count << "]: " 
                      << "x=" << p.x() << ", y=" << p.y() << ", z=" << p.z() 
                      << " (Size: " << size << ")" << std::endl;

            count++;

            // 限制一下打印数量，防止屏幕刷爆
            // if (count > 100) {
            //     std::cout << "... and more nodes (total occupied: " << tree->size() << ")" << std::endl;
            //     break;
            // }
        }
    }
    
    //3.点查询 (Point Query) 
    octomap::point3d current_pos(13.1 ,4.1, 14);

    if (isPointFree(current_pos, tree.get())) {
        std::cout << " free" << std::endl;
    }
    else
    {
        std::cout << "occupy" << std::endl;
    }

    //4.  定义机器人的包围盒（以当前点为中心）
    octomap::point3d min_pt(13.1, 4.1, 10);
    octomap::point3d max_pt(13.1, 4.1, 16);

    if (isSpaceFree(min_pt, max_pt, tree.get())) {
        // 逻辑：这个 0.5x0.5x0.3 的空间里没有任何一个八叉树方块是占据状态
        // 无人机飞过去很安全！
        std::cout << " free" << std::endl;
    } else {
        // 逻辑：哪怕空间里只有一个小碎块，为了安全，A* 也会放弃这个节点
        std::cout << " occupancy" << std::endl;
    }
    // 如果你已经安装了 octovis，取消下面这行的注释可以直接弹窗显示
    system(("octovis " + output_bt).c_str());
    return 0;
}