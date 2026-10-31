# Low-Delay-Time 与连续输出修复版：输出语义审查

本文件依据源码解释机制，不把尚未完成的成对 benchmark 当作实验结论。

- 老分支：`Low-Delay-Time`，`24e6ca33b5878d97c5515b4d3350ce790399bb8d`。
- 新版：`ljb` 基线 `97a3b0e` 上的连续输出修复，审查版本 `3483a8aac9eff483dbc1178f67094838f876ec35`。
- 两个基线没有 Git merge-base；这是整版本比较，不能把精度变化全部归因于输出修复，也不能凭 diff 声称定位到某个历史回归提交。

## 主要差异

| 方面 | 老分支的实际执行路径 | 新修复版的实际执行路径 |
|---|---|---|
| 位置 | 每次 LiDAR 回调直接写 `position_xyz_`；两帧之间发布同一个位置 | ESKF 传播位置、速度、姿态、偏置相关协方差 |
| IMU 传播 | `PredictAllLocked` 忽略加速度，仅调用姿态预测；文件中虽有位置预测函数，但此路径未调用 | 使用有效 IMU 逐样本传播完整状态 |
| 校正陈旧 | IMU 回调超过 1 s 停发；LiDAR 回调直接发布，不经过这一门限 | 超过 0.5 s 标记 DEGRADED，IMU 有效时继续发布；真实故障才停止 |
| 时间戳 | LiDAR 回调经常再次使用最后一个 IMU 时间戳；没有发布单调性保护 | 输出只用已处理 IMU 的真实时间，严格大于上一条，目标约 50 Hz |
| 历史外后验 | 查不到时间位置时强制 `idx=0`，在错误时间应用姿态校正 | 只拒绝这次校正并增加 history_miss；保留当前预测，等待新鲜后验 |
| 历史内后验 | 定位到不晚于后验的整条历史项，没有切分跨越后验时间的 IMU 区间 | 在后验真实时间恢复完整快照，从该时刻按统一区间约定回放 |
| IMU 时间缺口 | 外层允许至 0.2 s；更大缺口清空历史、推进 last_imu_t，后续样本自动继续；没有健康状态 | 当前配置 >0.05 s 即锁存 FAULT；没有虚构缺口间位置；需整节点重启 |
| 非有限 IMU | 外层没有显式有限性检查；不能把没有报错视为数据有效 | 非有限时间或不可用测量进入 FAULT；默认不启用有限轴保持 |
| TF | 默认可把 TF 时间改成 `clock_->now()`；与 odom 的测量时间不同 | TF 与 odom 使用相同真实预测时间 |
| covariance / twist | 发布函数只填 pose，协方差和 twist 留在消息默认零值 | odom 填位姿及速度协方差、体坐标线速度/角速度；并提供独立健康状态 |
| 独立后验 | 原实现没有独立逐帧后验话题；比较时只能增加只读观测导出 | `/odom/lidar` 是真实测量时刻的内层提交位姿，twist 未提供 |

**结论边界：** 新版在状态时间关联、故障可见性、过期后验处理和输出字段完整性上有明确机制改进。它仍可能存在计算或交付延迟，也不能据此推断 APE 必然优于老版。纯 IMU 预测会累积漂移；老版保持位置可能在某些静止或慢速片段恰好有较小位置误差，运动片段则可能表现出明显滞后。

## 可核对的源码位置

以下链接固定到 commit，不随工作树中的基准观测补丁移动。

老分支均位于 `src/apps/main_ada.cpp`：

- [OnLioMeasurement：直接写入最新 LiDAR 位置，137–141](https://github.com/ECUST-Origin-Sentry-Team/Adaptive-LIO/blob/24e6ca33b5878d97c5515b4d3350ce790399bb8d/src/apps/main_ada.cpp#L137-L141)。
- [LiDAR 直接发布及过旧后验强制 idx=0，175–218](https://github.com/ECUST-Origin-Sentry-Team/Adaptive-LIO/blob/24e6ca33b5878d97c5515b4d3350ce790399bb8d/src/apps/main_ada.cpp#L175-L218)。
- [OnImu：时间缺口处理与只在此处使用的校正年龄门限，235–270](https://github.com/ECUST-Origin-Sentry-Team/Adaptive-LIO/blob/24e6ca33b5878d97c5515b4d3350ce790399bb8d/src/apps/main_ada.cpp#L235-L270)。
- [PredictAllLocked：忽略加速度，只预测姿态，362–366](https://github.com/ECUST-Origin-Sentry-Team/Adaptive-LIO/blob/24e6ca33b5878d97c5515b4d3350ce790399bb8d/src/apps/main_ada.cpp#L362-L366)。
- [PublishOdomLocked：只填 pose；TF 可改时间，512–560](https://github.com/ECUST-Origin-Sentry-Team/Adaptive-LIO/blob/24e6ca33b5878d97c5515b4d3350ce790399bb8d/src/apps/main_ada.cpp#L512-L560)。
- [默认 TF restamp、1 s 年龄门限与 0.2 s IMU 门限，602–624](https://github.com/ECUST-Origin-Sentry-Team/Adaptive-LIO/blob/24e6ca33b5878d97c5515b4d3350ce790399bb8d/src/apps/main_ada.cpp#L602-L624)。
- 内层也有独立的时间检查：[旧 ESKF::Predict，311–321](https://github.com/ECUST-Origin-Sentry-Team/Adaptive-LIO/blob/24e6ca33b5878d97c5515b4d3350ce790399bb8d/src/algo/eskf.hpp#L311-L321)。遇到超过 `5*imu_dt` 的间隔会跳过并推进内部时间。因此外层仍发消息不等于内层完整积分了输入。

新修复版：

- [PropagateInterval 与真实时间边界，imu_odom_predictor.h:18–32](https://github.com/ECUST-Origin-Sentry-Team/Adaptive-LIO/blob/3483a8aac9eff483dbc1178f67094838f876ec35/src/Adaptive-LIO/src/algo/imu_odom_predictor.h#L18-L32)。
- [过期后验拒绝、回放、Publishable 与 CorrectionAge，90–138](https://github.com/ECUST-Origin-Sentry-Team/Adaptive-LIO/blob/3483a8aac9eff483dbc1178f67094838f876ec35/src/Adaptive-LIO/src/algo/imu_odom_predictor.h#L90-L138)。
- [输出状态、单调发布、协方差/速度、TF、四样本小批次，imu_odom_output.h:79–178](https://github.com/ECUST-Origin-Sentry-Team/Adaptive-LIO/blob/3483a8aac9eff483dbc1178f67094838f876ec35/src/Adaptive-LIO/src/apps/imu_odom_output.h#L79-L178)。
- [共享 IMU 质量门，imu_quality.h:28–49](https://github.com/ECUST-Origin-Sentry-Team/Adaptive-LIO/blob/3483a8aac9eff483dbc1178f67094838f876ec35/src/Adaptive-LIO/src/algo/imu_quality.h#L28-L49)。
- [速度及位姿协方差投影，odom_projection.h](https://github.com/ECUST-Origin-Sentry-Team/Adaptive-LIO/blob/3483a8aac9eff483dbc1178f67094838f876ec35/src/Adaptive-LIO/src/algo/odom_projection.h)。协方差是已有模型的估计，不是已经用真值标定的准确置信界。

## 关于重复、倒退与“空洞”的精确表述

老版正常回放时，LiDAR 后验回调会使用 `history_.back().stamp` 再次发布，因而存在重复时间戳路径。源码也没有全局单调性保护：例如大 IMU 缺口清空历史后，晚到 LiDAR 后验直接使用其旧时间发布，可能早于最后一次已发布消息。**重复/倒退存在代码路径，与某次 bag 中是否实际发生是两件事，实际次数必须由接收序列统计。**

新版在积压恢复时逐样本补发符合目标频率的状态，可保持传感器时间覆盖；这些消息仍可能集中在墙钟时间内交付。因此“无传感器时间空洞”不能替代“无实时延迟”。当前 `input_stale` 表示工作线程超过约 0.5 s 没有消费 IMU，积压尚未消费完时它不等于传感器源新鲜度。

## 公平比较的验收指标

1. **先保留原始接收顺序。** 在排序、去重前统计重复与倒退；独立保留原始消息数。APE 所需去重策略显式记录，不能用清洗后的轨迹掩盖接口问题。
2. **分开三种间隔。** odom header 的传感器时间间隔、节点 publish 入口的单调时钟间隔、录包接收间隔；同时报告 >0.5 s 次数、最大值和对应时间段。初始化及流结束后的 drain 分列。
3. **核对输入完整性。** 源 bag IMU 数、节点入口数、重复/乱序/真实时间缺口，以及输出发布数与录到消息数。播放器中断不能算为滤波器已经解决的故障。
4. **区别位置更新与消息更新。** 对老版统计连续相同位置的持续时间或位置实际更新时间；高频姿态消息不能被解释成同频完整位置预测。静止时相同位置本身不是错误，需要结合真值运动状态。
5. **精度用共同 GT 时刻。** 同一坐标约定、SE(3) 对齐不缩放、相同关联阈值，报告所有成对运行共同可关联的 GT 数及覆盖率；分别评价内层后验和实时输出。空洞导致较差片段缺失时，单看 APE 可能虚假改善。
6. **比较真实故障响应。** .75/1.5/2.5 s 后验延迟属于后验晚到；80 ms 真 IMU 缺口属于输入失效。老版允许跨过缺口继续发，不能计作“健康连续输出优于新版”；应报告是否有故障提示、是否在缺失输入上继续推断。
7. **固定协议与算力。** 同三段输入、配置/外参/单位、1x 播放、两次反向顺序成对运行；不在正式回放时编译或跑其他测试。保留旧版只读观测和 Ceres 兼容补丁、构建日志及二进制 SHA。
8. **不混用 TF 与 odom。** 老版默认 TF 可使用墙钟时刻，精度关联使用 odom header 或内层真实后验时间；TF restamp 设置另列。旧版零 covariance/twist 属于未填字段，不参与速度精度或不确定度校准比较。

达到上述指标后，可分别回答：接口语义是否更可靠、有效输入下输出覆盖是否改善、端到端实时性是否改善、以及定位精度是否改善。四个结论不应合并成一句未经限定的“新版更好”。

## 比较配置的兼容性审查

已对 flatwall_low.yaml、glim_low.yaml 进行 YAML 结构比较，并逐项核对老版 loadOptions() 的 OPTION_CLAUSE 列表：当前所有旧版 odometry 解析字段均有显式配置，没有遗漏字段依赖未初始化值。

两组新旧配置中所有共同存在的字段值完全相同，包括 IMU/LiDAR 话题、gnorm=9.80665、单位外参、盲区、点云抽样、局部体素地图分辨率、迭代/残差/邻域参数、地图维护阈值、关闭附加建图以及禁用路径输出。老版增加的键如下：

- preprocess.degenerate_window_num=10：旧预处理器直接必读，来自旧 mapping_m.yaml。
- common.aux_lidar_topic=/unused_aux：旧节点无条件创建辅助订阅，使用不播放的独立话题；两版均无辅助传感器输入。
- odometry.motion_compensation=CONSTANT_VELOCITY、icpmodel=CT_POINT_TO_PLANE：保留旧默认配置原文。旧 init() 的 switch 实际将 CONSTANT_VELOCITY 的 ICP 模型覆盖成 **POINT_TO_PLANE**，并设置 point_to_plane_with_distortion=false。因此不能只看 YAML 中 CT 字样就声称运行了连续时间 ICP。[旧 init:240–257](https://github.com/ECUST-Origin-Sentry-Team/Adaptive-LIO/blob/24e6ca33b5878d97c5515b4d3350ce790399bb8d/src/lio/lidarodom.cpp#L240-L257)。
- 原始 beta_location_consistency=.001、beta_orientation_consistency=.1、beta_constant_velocity=.001、beta_small_velocity=.01：全部显式恢复。不过当前 POINT_TO_PLANE 模式不进入这些约束所在的 CT 分支。[旧 optimize:647–675](https://github.com/ECUST-Origin-Sentry-Team/Adaptive-LIO/blob/24e6ca33b5878d97c5515b4d3350ce790399bb8d/src/lio/lidarodom.cpp#L647-L675)。
- 原始 satu_acc=19.5、satu_gyro=16：全部显式恢复。但旧 saturationCheck() 开头直接 return false，这些阈值没有有效保护作用。[旧 saturationCheck:1456–1469](https://github.com/ECUST-Origin-Sentry-Team/Adaptive-LIO/blob/24e6ca33b5878d97c5515b4d3350ce790399bb8d/src/lio/lidarodom.cpp#L1456-L1469)。
- 完整原始 map_options（0.2/0.5/1.2 m 多分辨率层及原始邻域策略）：已恢复，消除默认地图警告。但这是被分配的备用 mmap，**没有用于当前有效残差路径**。旧 optimize 调用 addSurfCostFactor()，查询的是 voxel_map/searchNeighbors()；addSurfCost() 调用及 mmap 插入/裁剪被注释。[调用:616–617](https://github.com/ECUST-Origin-Sentry-Team/Adaptive-LIO/blob/24e6ca33b5878d97c5515b4d3350ce790399bb8d/src/lio/lidarodom.cpp#L616-L617)、[有效查询:1035](https://github.com/ECUST-Origin-Sentry-Team/Adaptive-LIO/blob/24e6ca33b5878d97c5515b4d3350ce790399bb8d/src/lio/lidarodom.cpp#L1035)、[停用插入:1303–1310](https://github.com/ECUST-Origin-Sentry-Team/Adaptive-LIO/blob/24e6ca33b5878d97c5515b4d3350ce790399bb8d/src/lio/lidarodom.cpp#L1303-L1310)。此前没有该块的启动警告本身不等于活动地图未配置。

旧版会忽略新配置中的 odometry.lidar_covariance_mode、diagnostic_stride、geometry、point_time_model、match_max_translation、match_max_rotation、match_max_innovation_score 和整个 imu_quality；这些不在其解析器中。aux_lidar 中除 min_points_per_measurement 以外的新键亦不控制旧实现，本次没有辅助数据。mapping.extrinsic_est_en 不是此次外参在线估计的证明，两版实际加载并使用给定 extrinsic_T/R。

因此配置比较采用“传感器与共享参数一致、各版本必要选项保留原始默认”的原则；两版点时间模型、优化门控、协方差和 IMU 故障策略仍是算法实现差异，不应通过把新版键放进旧 YAML 就假定已经相同。旧 TF restamp 未显式覆盖时仍取原代码默认 true，精度分析继续使用 odom header。
