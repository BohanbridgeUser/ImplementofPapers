import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import convolve

# 参数设置
box_size = 3      # 盒子滤波器大小
n_points = 1000   # 曲线采样点数

# 初始化盒子滤波器
kernel = np.ones(box_size) / box_size  # 归一化的盒子滤波器
stages = [kernel.copy()]  # 保存每次卷积后的滤波器

# 对盒子滤波器进行 1 到 3 次卷积
for _ in range(2):  # 进行两次卷积（总共 3 个阶段：1 次、2 次、3 次）
    kernel = convolve(kernel, stages[0], mode='full')  # 与原始盒子滤波器卷积
    kernel /= kernel.sum()  # 归一化
    stages.append(kernel)

# 创建可视化
plt.figure(figsize=(12, 6), dpi=100)

# 绘制每次卷积后的滤波器分布
colors = ['blue', 'green', 'red']  # 每次卷积的颜色
for i, (stage, color) in enumerate(zip(stages, colors), start=1):
    x = np.linspace(-len(stage)//2, len(stage)//2, len(stage))  # 中心对称坐标
    plt.plot(x, stage, color=color, linestyle='-', alpha=0.8, label=f'{i} Convolution(s)')  # 使用曲线绘制盒子滤波器

# 图像设置
plt.title(f'Box Filter Convolution (size={box_size})')
plt.xlabel('Position')
plt.ylabel('Amplitude')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()