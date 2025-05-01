import open3d as o3d
import pandas as pd
import numpy as np

def ply_to_csv(ply_file, csv_file):
    # 读取 PLY 文件
    pcd = o3d.io.read_point_cloud(ply_file)
    
    # 获取点的坐标
    points = np.asarray(pcd.points)
    
    # 获取点的颜色（如果存在）
    if pcd.has_colors():
        colors = np.asarray(pcd.colors)
        # 将颜色值转换为整数（0-255）
        colors = (colors * 255).astype(int)
    else:
        raise ValueError("PLY 文件中没有颜色信息")
    
    # 将颜色分类为唯一的类别
    unique_colors, color_labels = np.unique(colors, axis=0, return_inverse=True)
    
    # 构建 DataFrame
    data = pd.DataFrame(points, columns=["x", "y", "z"])
    data["r"] = colors[:, 0]  # 添加 R 值
    data["g"] = colors[:, 1]  # 添加 G 值
    data["b"] = colors[:, 2]  # 添加 B 值
    data["color_class"] = color_labels  # 添加颜色分类列
    
    # 保存为 CSV 文件
    data.to_csv(csv_file, index=False)
    print(f"成功将 {ply_file} 转换为 {csv_file}")

# 示例用法
ply_file = "./output/smoothness_constraint_segmentation.ply"  # 替换为你的 PLY 文件路径
csv_file = "output.csv"   # 替换为输出的 CSV 文件路径
ply_to_csv(ply_file, csv_file)