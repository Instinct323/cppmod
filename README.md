# Deployment

- ├── `cmake`: 使 find_package 支持自建库的 Find*.cmake
- └── `docker`: 用于创建 ubuntu 容器的 Dockerfile, 以及配置 C++ 开发环境的脚本

# Package

- ├── `utils`: 第三方库的增强功能, 以及个人开发的实用工具
- $~~~~~~~~~~~~~~$└── `parallel`: 全局优先级线程池, 自动限定线程总数
- └── `zjcv`: 计算机视觉相关工具
- $~~~~~~~~~~~~~~$├── `camera`: 相机模型 (Pinhole, Kannala-Brandt)
- $~~~~~~~~~~~~~~$├── `dataset`: 数据集加载器
- $~~~~~~~~~~~~~~$└── `imu`: IMU 数据处理 (预积分)

# ORB-SLAM3

基于 `utils` 和 `zjcv` 开发的 ORB-SLAM3 框架

## New Feature

- 使用 `yaml-cpp` 替代 `cv::FileStorage`, 支持最新的 YAML 文件格式
- 引入全局优先级线程池, 自动限定线程总数
- 为 Pinhole 的双目特征点匹配添加了基于视差的筛选 (Pauta Criterion)

## Change
- 简化了 IMU 的预积分过程
- 使用运算速度更快的 `fbow` 替代 `DBoW2`
