# UCA-SLAM++

UCA-SLAM++ is a visual–LiDAR SLAM system with uncertainty-aware estimation.

## Current release: SLAM interface preview

This repository currently publishes **only the C++ header declarations of the SLAM component**. It provides an initial view of the system interfaces and data structures while the paper is under review.

Function bodies, inline and template implementations, member initializers, and default argument values have been omitted from this preview. This is **not a header-only library or a buildable SDK**. It cannot be compiled into a working SLAM system or used to reproduce the paper's results. Implementation files, build scripts, binaries, experiment data, and bundled third-party implementation code are not included. Some declarations refer to external dependencies that are not bundled here.

## Release plan

After the paper is accepted, we plan to release:

- The full implementation of the SLAM component, with build and usage instructions.
- The **LoCoU-Net** network component.
- The implementations required to integrate LoCoU-Net with the SLAM system.

The network and its integration implementation are not part of this initial release. Interfaces may change before the full release.

## Header layout

| Location | Contents |
| --- | --- |
| `include/System.h` | System entry points and lifecycle |
| `include/Tracking.h` | Tracking and consolidated LiDAR frontend declarations |
| `include/Frame.h`, `KeyFrame.h`, `Map.h`, `MapPoint.h` | Frame and map data structures |
| `include/LocalBAV3.h`, `LocalMapping.h` | Local mapping and bundle adjustment interfaces |
| `include/LocalBAGeometry.h`, `LocalBAV3Edges.h` | Local geometry, plane constraints and optimization factors |
| `include/LocalBAV3Covariance.h` | Covariance, uncertainty and anchor information |
| `include/LocalBALifecycle.h` | Scheduling, cancellation, replay policy and commit coordination |
| `include/LiDARIncludes/` | LiDAR types, voxel-map and registration declarations |

`HEADER_MANIFEST.json` records the SHA-256 digest of each published header.

## License and acknowledgements

The released SLAM declarations retain the project's GPLv3 licensing and upstream attribution. See [LICENSE.txt](LICENSE.txt), [the GPLv3 text](License-gpl.txt), and [ACKNOWLEDGEMENTS.md](ACKNOWLEDGEMENTS.md). Third-party components retain their respective terms.

## 中文说明

当前仓库仅公开 **UCA-SLAM++ 的 SLAM 部分 C++ 头文件接口声明**，用于在论文审稿期间展示系统接口与数据结构。函数体、inline/template 实现、成员初值及默认参数已从预览版本中移除，因此当前内容不是可编译运行的完整系统，也不能据此复现论文结果。

**SLAM 部分的完整实现、LoCoU-Net 网络部分，以及 LoCoU-Net 与 SLAM 系统接入相关的实现，计划在论文被接收后公开。** 完整发布时将补充构建与使用说明。
