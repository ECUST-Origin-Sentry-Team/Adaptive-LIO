# Adaptive-LIO 实车优化与遗留接口清理完整报告

日期：2026-10-10。状态：**代码实现与独立 C++ 核心测试完成；ROS 全包构建、真实 bag、实车性能及安全验收未完成。** 本报告合并前一轮实车优化和本轮《Adaptive-LIO 遗留接口与死代码清理执行计划》的实施结果；前一轮 `Adaptive-LIO_实车优化实施报告.md` 仅作历史记录。本报告不能视为已经上车验收。

## 1. 源头、范围和交付形式

| 项目 | 内容 |
| --- | --- |
| 实车仓库 | `https://github.com/ECUST-Origin-Sentry-Team/origin_sentry_nav2026`，先 clone 再在实车包上修改 |
| 分支和原始提交 | `agent/navigation-stack-update`，`e0339963ab35f2827c83fcf0298cf32c57ad4907` |
| 工作包 | `src/localization/Adaptive-LIO`，ROS 包及可执行文件名 `adaptive_lio` |
| 两轮修改 | 同一工作树中先完成融合输出优化，再在优化版本上执行 P0–P6 遗留清理；**不是**用旧独立仓库覆盖实车代码 |
| Git 操作 | 未提交、未推送、未创建 PR；变更仅限此包；保留原仓库历史 |
| 配置 | `mapping_m.yaml` 保留 0.05 s 等待；`mapping_m_low_delay.yaml` 为 0.0 s 对照；不合并配置 |
| 交付 | 完整 Adaptive-LIO 包、相对原始提交的可审查补丁、原始包归档、测试日志和本报告 |

本轮清理不做“状态统一”：`current_state`、`all_state_frame`、`p_frame->p_state` 与 ESKF 后验依旧承担各自既有职责。不改 `optimize()`、`AcceptMatch()`、`ObserveSE3()`、`poseEstimation()`、`map_incremental()` 的算法含义及接受/拒绝顺序；不改双雷达匹配、ScanContext、时间同步等待、IMU 历史重放、TF 拓扑或导航侧代码。ESKF 18 维传播、Q 与 Joseph 更新在**清理轮**中不变。上一轮优化有明确的传播和观测变化，详见第 2 节，不能把“两轮总体行为”误称为完全不变。

## 2. 第一轮实车优化：完整工作链路

原实车包有外层 RPY/位置平滑及历史索引校正，原动态 TF 可能按发布时刻 restamp；内层 IMU 传播、LiDAR 匹配与外层 odom 不完全同源。本次把高频 odom 驱动改为内层完整 ESKF 快照经独立 worker 传播；既有 LiDAR 匹配/地图职责不因此合并。

### 2.1 快照、传播和后验回放

`src/algo/eskf.hpp` 的 `Snapshot` 按值携带名义状态（时间、R/p/v/bg/ba）、重力、18×18 协方差、Options 与过程噪声 Q。`ValidSnapshot` 核查有限值、SO(3) 正交性、矩阵对称性和半正定性；`RestoreSnapshot` 先验证再赋值，失败时原滤波状态不动，成功时清零误差状态。`Predict` 对重复、倒退、非有限输入及超过五个参考 IMU 周期的间隔返回失败且不跳时钟；合法间隔联合传播 p/v/R/P。两个 `ObserveSE3` 重载继续用 LDLT 和 Joseph 形式更新。

`src/algo/imu_odom_predictor.h` 的单所有者核心接收 `LioStateUpdate`，按 `sequence` 和状态时间拒绝重复/过期状态，允许迟到后验在历史 IMU 时间轴上安装并重放。帧末落在原始 IMU 间隔内部时，`PropagateInterval` 插值右端值；之后仅积分剩余区间，不重复计入帧前部分。历史约 2 s，保留左边界样本且最多 20000 条；没有有效 LiDAR 校正超过 0.5 s 则停发预测输出，后续有效校正可恢复。输入大间隔/时钟倒退等错误采取停机并重启**整个节点**的策略，无隐式单滤波器 epoch 重置。

上一轮明确将参考 10 ms 离散 Q 按 `dt / imu_dt_` 缩放；数值没有平方或改成已标定的连续谱密度。此改动会影响 P，须用实车数据比较。`ObserveSE3` 默认融合观测噪声的实际量纲仍须现场核实。清理轮没有修改这些数值方程。

### 2.2 线程、LiDAR 接受门和地图

`src/apps/main_ada.cpp` 的 IMU 入口做一次校验/单位换算，再将样本投递给传播 worker 与 LIO；回调不再自行积分或发布 TF。`src/apps/imu_odom_output.h` 的 worker 单独拥有预测器，是 `/odom`、路径、动态 TF 的唯一代码发布者。LIO 匹配完成后 `SetStateCallback` 传出完整值快照。线程退出顺序为 executor 停止、LIO 生产线程 join、高频输出线程 join、点云与 ScanContext 消费线程停止；原未 join 的路径已处理。主副 LiDAR 保持互斥回调组，IMU 使用单独回调组；尚未测量锁等待或 DDS 抖动。

`src/lio/lidarodom.cpp` 先预测、匹配，再经有效支持量、求解代价和同刻先验创新判断，通过后才进行内部 ESKF 位姿观测和地图/点云/ScanContext 维护。空地图只依有效点数做种子门控。拒绝帧恢复匹配前状态，不写图、裁图或发该帧注册点云/关键帧；可把仅预测快照送高频输出，但不更新最近**有效**校正时间。`match_max_innovation_score=100` 是工程初值，不是经现场标定的卡方门限；日志保留 `match_posterior_dp` 供对比地图匹配位姿与滤波后验。

### 2.3 双雷达、输出坐标和性能边界

副雷达点入队前过滤非有限值并排序，按主雷达半开窗口 `[begin,end)` 消费，未来点留下一窗；太少的已消费点不回退游标。主、副点在运动补偿前限定到已有 IMU 轨迹覆盖。副雷达输出仍按实车外参进入主雷达输出坐标；`enable_bundle_fusion: false` 时仅输出辅助点云，不进入优化/地图；保留可选联合匹配路径。主帧取走时间不晚于末端的 IMU，右邻样本只借用不消费。部分副云裁剪仍在队列锁内，不能据此宣称调度延迟已全面解决。

保持 `/odom`、`/odometry_path`、`/livox/scan` 与 `odom → aft_mapped → base_link` 拓扑。正式 odom 的子坐标系仍为 aft_mapped（假定 IMU 原点与之重合）：姿态位置来自同一状态时刻，线速度为 `Rᵀv_world`，角速度为 `gyro-bg`；pose/twist 协方差用 Jacobian 投影，包含相应交叉项。通用 `ProjectOdometry` 支持非零外参及杆臂并有 18 列有限差分测试；正式 `/odom` **未**被偷偷重命名为 base_link 或二次施加杆臂。TF 与 odom 使用状态时间，不按系统当前时间重打戳；固定外参仍沿原发布机制，未改 `/tf_static`。

新配置与边界：主雷达队列 200 帧，IMU 队列 20000；超限停机；可选副雷达队列 200 包并丢弃最旧包。路径最多 300 点并限制在 10 Hz；计时统计有锁且每项最多 10000 条。0.05 s 配置用于阶段 A，0.0 s 配置仅作阶段 B 对照；两个配置外参、话题和 `enable_bundle_fusion: false` 保持一致。`fusion_aft_to_base_xyzrpy=[0.0,0.257,-0.236,0,0,3.1415]`、`gnorm=1` 保留实车设定，其实际物理单位尚须现场确认。

## 3. 第二轮：逐项删除记录（P0–P6）

以下“已验证”指全仓静态引用检查、核心编译和单元测试；**不代表 ROS 二进制集成或运行时验证**。所有删除都是真删声明/定义/调用，不留空 lambda、弃用桩或兼容桥。

| 旧符号/路径 | 原用途与旧调用者 | 新实现或保留对象 | 删除位置和结果 | 验证 |
| --- | --- | --- | --- | --- |
| P0 `pose_pub_func`、SE3 `setFunc`、`pub_pose_to_ros("laser"/"world")` | LIO 每帧调用两次，主函数注册的 lambda 只返回 `true` | `SetStateCallback → ImuOdomOutput` 负责正式 odom/TF；`pose_of_lo_` 仍供日志 | `main_ada.cpp`、`lidarodom.h/.cpp` 整链删除 | 引用零命中，核心测试通过；ROS 未测 |
| P0 `RIG_`、`SafeSO3FromMatrix` | 只给无效果的 world 回调构造位姿 | `TIL_` 和 LiDAR–IMU 外参不动；独立的 `g2R` 工具定义不误删 | `lidarodom.h/.cpp` 删除成员、唯一赋值及私有 helper | 全仓引用零命中 |
| P1 `Back_Predict`、`Predict_Cont`、所有 Back/Cont getter、`prepareBack/Cont`、`_unc/_cont` 状态 | 旧辅助递推；`RestoreSnapshot` 仍为其复制状态，无实际消费者 | `RestoreSnapshot` 保留名义状态、重力、P、Q、Options 赋值及 `dx_.setZero()`；`Predict/PropagateInterval` 真正运行 | `eskf.hpp` 删除整套接口、模板定义、专属成员与初始化 | 核心传播/后验重放测试通过 |
| P2 `saturationCheck` 和恒速回退 | 函数入口恒 `return false`；其 30 帧判断、ESKF 复位和 `SetX` 永不可达 | `stateInitialization` 直接执行原 else 的 IMU 预测赋值；帧首/帧末姿态位置来源保持原样 | `lidarodom.h/.cpp` 删除函数、死分支和注释 | 静态证明原入口恒假；ROS 未测 |
| P2 `cache_vel`、`satu_acc/satu_gyro` | 只服务不可达回退及其 YAML 参数 | 无替代；**不宣称已经实现 IMU 饱和检测/恢复** | `lidarodom.h/.cpp` 和双 YAML 中字段、赋值、加载均删除 | 符号零命中 |
| P3 `fusion_restamp_tf_to_now`、`SetTfRestampToNow` | 配置被读入，API 仅发弃用警告，TF 仍用状态时间 | 直接保留当前 odom/TF 同源时间戳实现 | `main_ada.cpp`、`imu_odom_output.h`、双 YAML 整链删除 | 符号零命中；运行时间戳待测 |
| P4 `epoch` | 生产者永远写 0，消费者拒绝非零；无真实代际切换 | `sequence`、`pending_` 和时间顺序检查 | `imu_odom_predictor.h`、`lidarodom.cpp`、`test_core.cpp` 删除字段、判断和聚合槽位 | 零/重复/旧序号拒绝测试通过 |
| P4 `lidar_correction_accepted` | 生产者赋值，消费者从不读取 | `last_accepted_lidar_stamp` 实际控制有效校正年龄与超时 | 同上删除字段、聚合槽位和测试赋值；保留本地 `accepted` 门控/日志 | 拒绝帧超时与恢复测试通过 |
| P6 `PR_begin/PR_end` | 只出现在注释掉的 Ceres 残差代码 | 当前实际速度一致性残差不动 | `lidarodom.h/.cpp` 删除数组及整段旧注释 | 符号零命中 |
| P6 `mtx_aux_buf/mtx_state`、`all_cloud_frame`、`time_curr`、`imu_cont` | 无锁使用、无缓存消费者、只写未读、只 clear | `mtx_buf`、`all_state_frame`、`measures_.imu_` 仍实际使用 | `lidarodom.h/.cpp`、`lio_utils.h` 删除字段与唯一写入 | 符号零命中 |
| P6 `getIndex()<20 ? 0.01 : 0.01` 两处、`imu_repub` 注释链 | 恒等采样分支和完全注释的发布链 | 两处仍取 `0.01`；标准点云 `<30 ? 0.02 : 0.1` 未改 | `main_ada.cpp` 简化并删旧注释 | 源码审查；核心测试不覆盖 ROS 回调 |
| P1/P6 `SetX` 两个重载、`SetCov`、ESKF 自身 `GetGravity` | 一个只在死回退中用，其余全仓无调用 | `StaticIMUInit::GetGravity` 仍用于初始化；`SetInitialTime` 等保留 | `eskf.hpp` 删除 API；不删初始化器同名接口 | 全仓引用检查，核心编译通过 |
| P6 可选 `ObserveWheelSpeed`、`OdomNoiseT/odom_noise_`、轮速 Options | 当前整个工作仓库无轮速调用/输入接入 | `GetNominalVel` 因标量速度输出保留；`tools/odom.h` 文件保留供其他头文件引用 | `eskf.hpp` 删除方法、专属噪声和配置、校验项及 include | 全仓引用零命中，核心编译通过 |

`common/mutexDeque.hpp` 在 LIO 头中原只为死饱和分支而包含，现移除此 include；**没有**删该通用文件。`g2R` 的工具声明/定义仍留仓库，未跨出本轮定向清理范围。头文件包含清理未做全项目格式化。

## 4. 不删的旧接口及生产者—消费者核验

已从仓库根目录对 `velocity`、`move_dist`、相关注册和回调做全文搜索，而非仅查看 Adaptive-LIO 的 C++。结果：RViz 配置 `src/localization/Adaptive-LIO/launch/rviz.rviz` 中存在两个 **Enabled: true、Value: true** 的 `jsk_rviz_plugin/Plotter2D`，分别订阅 `/velocity` 与 `/move_dist`。完整仓库内未发现其他代码/脚本订阅者；车上实际启动的外部客户端、遥测桥和运行时订阅关系**未能在此环境核实**。因此两路都有明确仓内可视化消费者，本轮均保留，绝不谎称“已迁移到 /odom”。

| 路径 | 生产者 | 仓内已知消费者及用途 | 结果与外部边界 |
| --- | --- | --- | --- |
| `/velocity` (`std_msgs/Float32`) | `lidarodom.cpp` 的 `Rᵀv_world` x 分量 → `pub_data_to_ros` → `main_ada.cpp::data_pub_func`、`vel_pub` | `launch/rviz.rviz` 的已启用 Vel Plotter2D | 保留。它是低频帧处理得到的标量，不能未经验证机械替换为高频三维 `/odom.twist`；外部订阅未确认 |
| `/move_dist` (`std_msgs/Float32`) | 每 8 帧累积位置变化距离 → 同一数据回调 → `dist_pub` | 同一 RViz 的已启用 Dist Plotter2D | 保留。累积路程不是瞬时 `/odom.twist`；外部订阅未确认 |
| `/livox/scan` 与云回调 | `pub_cloud_to_ros`、`cloud_pub_func` | 点云发布/可视化链 | 保留；高频里程计不代替点云 |
| ScanContext 回调 | `pub_scantext_data`、`scantext_cbk` | 地图/关键帧消费线程 | 保留 |
| 完整后验事件 | `state_callback_`、`SetStateCallback` | `ImuOdomOutput::OnState` / `ImuOdomPredictor` | 保留，替代的是无效果的旧 SE3 回调 |
| ESKF 名义状态与历史 | `Predict`、`ObserveSE3`、`all_state_frame`、`imu_states_` 等 | 匹配、去畸变、地图和高频预测 | 保留；不做状态统一 |

最终上线前如需删除两路旧标量，应先确认实际 RViz 配置是否随车启用、外部订阅者和消息语义，明确迁移需求后另起变更；这不是本轮“尚未清理掉的死代码”。

## 5. 静态检查与测试的真实结果

工作环境具备 GCC 13/C++17 和 Eigen（本次独立测试使用运行时自带 Eigen 头）及包内 Sophus；缺少完整 ROS 发行版、`ros2`、`colcon`、CMake、PCL/Ceres 等完整构建条件。运行 `EIGEN3_INCLUDE_DIR=<Eigen 头目录> ./test/run_core_tests.sh`：**全部核心测试通过**。`git diff --check` 通过。编译出现第三方 Sophus 宏的 4 条既有 `-Wpedantic` 多余分号警告；未改第三方代码。原始输出在交付包 `validation/core_test.log`。

| 测试范围 | 结果 |
| --- | --- |
| 快照导出/恢复、非法 P 原子拒绝、零/负/大间隔、两个 Joseph 位姿观测 | 通过 |
| 静止、匀速、加速、三轴旋转，后验迟到 83 ms、轨迹重放和 P 一致 | 通过 |
| pending 后验、零/重复/旧 sequence、历史过期、NaN、IMU 缺口失败 | 通过 |
| 拒绝匹配不刷新校正年龄、超时停发及新有效校正恢复 | 通过 |
| 非零外参杆臂、pose/twist 协方差 18 列有限差分 | 通过 |
| 匹配支持量/代价/创新门、副雷达排序/有限值/窗口/偏移、帧末插值 | 通过 |
| yaw 分支、近垂直姿态、运行中迟到校正 | 通过 |
| 全包 ROS 构建、colcon test、消息/TF 集成、bag/实车 | **未执行/未验证**；环境和数据不具备 |

清理项残余搜索（`src`/`config`/`test`）：`pub_pose_to_ros|pose_pub_func|RIG_`、旧 Back/Cont 组、`saturationCheck|cache_vel|satu_acc|satu_gyro`、`fusion_restamp_tf_to_now|SetTfRestampToNow`、`lidar_correction_accepted|epoch`、`PR_begin|PR_end|mtx_aux_buf|mtx_state|all_cloud_frame|time_curr|imu_cont` 均为 **0 命中**。`StaticIMUInit::GetGravity`、`SetStateCallback`、标量/点云/ScanContext 回调仍有真实命中。历史文档中会出现旧名称，属变更记录而不是生产引用。

上述核心测试直接编译本包交付的真实头文件，不是 ROS 代码的替代测试；未编译 `main_ada.cpp`、`lidarodom.cpp` 所需的全部 ROS/PCL/Ceres 集成。没有声称定位精度、CPU、p50/p95/p99 延迟改进百分比，亦未做线程竞争分析。

## 6. 上车前验证、回退和残余风险

先用真实 ROS 工作空间安装齐依赖，在仓库根运行 `colcon build --packages-select adaptive_lio --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON`、`colcon test --packages-select adaptive_lio --event-handlers console_direct+`、`colcon test-result --verbose`。原 CMake 使用 `/opt/ceres-2.2`，需与车上实际路径核对。确认 `/odom` 唯一发布者、`/odom` 与动态 TF 同一状态时间、`aft_mapped → base_link` 唯一且方向正确、RViz 两个标量绘图仍有数据、点云和 ScanContext 仍生成。

先用同一 bag 比较实车原版与 `mapping_m.yaml`（0.05 s），核对静止/起停/遮挡恢复、`lio_quality` 和 `imu_output` 日志，再完整停机切 `mapping_m_low_delay.yaml`（0.0 s）比较；回放须配置同源 `/clock`。检查副雷达点数与锁等待，故障时是否停发而非输出伪时间；单纯画面更顺滑不能等同于定位更准。门限 100、IMU 加速度单位、Q/R 和机器人实车动态均待标定。真正 IMU 饱和检测/恢复需要驱动量程及单位设计，本轮删除的恒假函数并未提供保护。

回退时保留当前工作包及对应配置快照，用包内 `baseline_e0339963.tar.gz` 整包恢复原始实车代码、重建，然后再测原版；不能只恢复 `fusion_restamp_tf_to_now` 一项。交付 `changes.patch` 以仓库根为路径基准，包含两轮所有修改和新增文件，建议先 `git apply --check` 并核对本地 HEAD 与私有修改，避免覆盖车上差异。不要在同一 `src` 下并列安装两个同名 `adaptive_lio` 包。

后续独立议题：匹配/滤波/地图状态的架构统一、最终同步延迟选择、固定 TF 是否改 `/tf_static`、安全的 IMU 饱和处理、实车外部标量消费者清单和性能测试。这些均**未在本轮以清理名义擅自实施**。
