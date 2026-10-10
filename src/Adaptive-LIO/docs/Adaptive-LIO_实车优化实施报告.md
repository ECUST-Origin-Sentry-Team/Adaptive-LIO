# Adaptive-LIO 实车优化实施报告

> 此文档记录第一轮优化时的实现状态。遗留接口与死代码清理后，以 `Adaptive-LIO_完整改动与清理报告_2026-10-10.md` 为当前交付说明；下文旧参数及旧测试描述只作历史记录。

日期：2026-10-10。交付状态：已完成代码实现与核心 C++ 合成测试；未完成 ROS 全包构建、真实 rosbag 回放、实车验证。不是已经完成性能验收的上车版本。

## 1. 基线与修改范围

| 项目 | 实际结果 |
| --- | --- |
| 用户指定仓库 | https://github.com/ECUST-Origin-Sentry-Team/origin_sentry_nav2026 |
| 克隆方式 | `git clone`，保留仓库远端默认分支 |
| 分支 | `agent/navigation-stack-update` |
| 基线 HEAD | `e0339963ab35f2827c83fcf0298cf32c57ad4907` |
| 包目录 | `src/localization/Adaptive-LIO` |
| 包名 / 可执行文件 | `adaptive_lio` / `adaptive_lio` |
| 工作树 | 克隆后原本干净；本次只修改该包 |
| Git 操作 | 克隆、只读核查；未切分支、提交、推送或发起 PR |
| 适用 AGENTS.md | 目标仓库及包内未找到 |

用户这次明确要求修改实车仓库，因此上传方案中独立仓库 `Adaptive-LIO/Low-Delay-Time` 的路径和分支要求不作为本次目标。不用旧独立版覆盖实车新版。原版包从上述 HEAD 原样导出为 `baseline_e0339963.tar.gz`。

保留了主副雷达话题、实车外参、ScanContext、点云滤波、地图维护节流、可选双雷达联合匹配以及 Nav2/FakeBaselink 接口。没有修改导航、重定位、驱动或 FakeBaselink 包。原有联合匹配能力保留，两个交付配置仍为 `aux_lidar.enable_bundle_fusion: false`。

## 2. F1–F8 复核与处理

| 问题 | 实车原版情况 | 本次处理与限制 |
| --- | --- | --- |
| F1 平移零阶保持 | 仍存在；外层位置直接来自匹配 | 移除旧外层滤波；完整后验驱动 p/v/R 联合传播；合成匀速、加速测试通过 |
| F2 等待 / 重写时间 | 50 ms 等待与 restamp=true 均存在 | TF 与 odom 均使用状态时刻；旧 true 也被忽略并告警；保留 50 ms 阶段 A，另给零等待阶段 B 配置 |
| F3 外层 RPY 模型 | 仍存在 | 移除 RPY 二次融合；统一 SO(3) ESKF；跨 yaw 分支与近垂直姿态测试通过 |
| F4 迟到观测 | 外层仍按历史离散索引校正 | 完整后验精确安装，首段按后验时刻截断，缓存重放到最新有效 IMU；迟到前后状态及 P 一致 |
| F5 异常匹配 / 地图污染 | 支持量不足只打印，地图先于 ESKF 更新 | 匹配支持量、求解有效性、代价变化、同刻创新检查均先于地图；拒绝时恢复候选前状态，不发点云或 ScanContext 关键帧；真实错误匹配恢复仍待 bag |
| F6 调度 | 主副雷达和 IMU 同回调组 | IMU 独立互斥组，主副雷达共享原互斥组；输出 worker 单独拥有传播状态；锁等待及 DDS 实测未完成 |
| F7 副雷达排序 | 实车已修复排序，真实帧窗裁剪也已存在 | 不重复当成新发现；将排序与有限值过滤集中至入队前，明确半开边界、低点数丢弃策略，约束轨迹覆盖；合成窗口测试通过 |
| F8 条件问题 | 实车外参并非零旋转；twist/P 缺失；饱和检测仍关闭 | aft_mapped 原点发布，补齐 child-frame twist 与 Jacobian 协方差；不再错误地将 IMU 转 base 再转回；饱和阈值/恢复未启用，需核对硬件量程及单位 |

实车 ESKF 原来已经修复负 dt/重复 IMU，并补上 `Predict_Cont()` 成功返回；这些不计为本次新增修复。原版遇大间隔仍只推进时间，本次取消这种行为，改为失败时保持原有效状态并停止链路。

## 3. 逐步实现

### 阶段 1：完整快照和传播契约

`ESKF::Snapshot` 包含时间、R/p/v/bg/ba、重力、18×18 P、Options 和离散 Q。Export 按值返回；Restore 先检查有限值、旋转、矩阵对称性和半正定性，成功后一次性恢复并清空误差状态、同步辅助状态。失败不污染原状态。

`Predict()` 不再跨缺失区间改写时间；零 dt、负 dt、非有限输入、大于五个参考周期的间隔均返回失败。合法传播同时更新 p/v/R/P，并补齐位置关于姿态、加速度偏置及重力的二阶 dt 项。两种 `ObserveSE3()` 统一使用 LDLT 求解、Joseph 协方差更新及误差重置后的对称化。

核心过滤器移除了不必要的 OpenCV/glog 头依赖，使用仓库自带 Sophus 和 Eigen 即可独立测试。

### 阶段 2：完整状态回调与独立 worker

新链路为：IMU 统一验证与单位换算 → 投递传播队列、送入 LIO → LIO 精确预测至帧末 → 匹配与质量决策 → 导出完整快照 → 输出 worker 安装快照、重放、发布。

worker 单独拥有高频 ESKF。IMU 回调不积分、不发 TF；LIO 回调只投递值快照。pending 后验可合并为最新版本，IMU 样本不随意抽样。输出只在状态时刻严格大于上次输出时发布；同一时刻的后验会先更新内部状态，等下一条更晚 IMU 再输出。

线程关闭顺序：executor 结束 → 停止并 join LIO 生产线程 → 停止并 join 高频输出线程 → 停止点云和 ScanContext 消费线程 → 销毁对象。修复原来 measurement_process 未 join 的退出路径。

### 阶段 3：匹配和写图分离

`optimize()` 返回是否可用，并记录支持量、初末代价、法向量最小奇异值。地图为空时通过有效点数检查建立种子地图，不要求不存在的匹配残差。正常帧先做质量检查，再调用内部 `ObserveSE3()`，成功后才进入原地图与点云维护函数。

拒绝匹配时恢复匹配前的 current_state 和 frame state，不写图、不裁图、不产出该帧注册点云或 ScanContext 数据。下一帧历史不保存被拒绝候选。仍可导出“仅预测”快照，但不刷新 last_accepted_lidar_stamp。

`match_max_innovation_score=100` 是工程初值，针对同一时刻先验与候选的位姿差，并非对机器人帧间移动距离限速。尚未做现场分布标定；因匹配和先验共享 IMU 信息，也不宣称严格卡方一致性。退化奇异值目前用于诊断，没有未经验证地加硬拒绝阈值或自适应 R。

### 阶段 4：同步与副雷达

主帧取走 `IMU.time <= lidar_end` 的样本；存在右邻居时只引用、不消费。恰好有帧末样本即可覆盖，不能因为队列消耗至空而跳过副雷达处理。内外通过同一个 `PropagateInterval()` 使用右端样本；内层遇帧末在两样本之间时插值该端点，外层只积分后验之后的剩余时长。

副雷达入队前过滤非有限值并排序；窗口使用 `[begin,end)`，end 点留给下一窗。重叠主窗由先处理的窗口拥有已消费点；间隙内过期点丢弃；少于 min_points 的已消费片段丢弃，不回退游标。保留未来点，不等待副雷达齐帧。进入去畸变前再次限定主、副点云时间在已有 IMU 轨迹内，避免外推到轨迹之外。

锁内一次只取一个主帧。副雷达点级裁剪目前仍在队列锁内；尚未做锁等待基准，不宣称所有调度延迟已经消除。

### 阶段 5：状态和坐标一致的输出

保持 `/odom`、`/livox/scan`、`odom → aft_mapped → base_link`。内部状态对应 IMU 原点，实车配置定义 IMU 与 aft_mapped 一致，因此 `/odom.child_frame_id` 仍是 aft_mapped。

- pose：同一状态时刻的 p 与 R。
- twist.linear：`Rᵀ v_world`，表达在 aft_mapped。
- twist.angular：`gyro-bg`，表达在 aft_mapped。
- pose covariance：将世界系位置误差和局部右扰动姿态误差投影到 ROS 固定世界轴扰动，保留交叉项。
- twist covariance：包含世界速度到局部速度的姿态 Jacobian、bias 协方差与初始化估计的 gyro 测量噪声；瞬时测量噪声与 P 的独立性仍是一阶近似。

通用 `ProjectOdometry()` 也实现了非零外参和杆臂项，并做有限差分测试；正式 `/odom` 没有改成 base_link，所以不额外施加一次杆臂。固定 `aft_mapped → base_link` 继续沿原发布链路发送，避免擅自改动 TF 拓扑。

地图、点云和 ScanContext 保留已接受的匹配位姿；高频 odom 使用 ESKF 后验。两者不保证数值完全相等，日志 `match_posterior_dp` 记录位置差。没有通过修改 TF 假装点云也变成后验坐标。

## 4. 参数和行为迁移

| 配置 / 限制 | 新含义 |
| --- | --- |
| `mapping_m.yaml` | 阶段 A：新完整状态传播 + 原 0.05 s 等待，便于隔离改动 |
| `mapping_m_low_delay.yaml` | 阶段 B：仅把等待改为 0.0 s；同样外参、话题和联合匹配开关 |
| `fusion_restamp_tf_to_now` | 弃用；false 为默认，旧 true 仅告警且不生效 |
| `fusion_aft_to_base_xyzrpy` | 原样保留 `[0.0,0.257,-0.236,0,0,3.1415]`；单位 m/rad |
| `gnorm` | 原样保留 1，只在 IMU 入口换算一次；仍需现场确认加速度实际单位 |
| `match_max_innovation_score` | 新增，默认 100，无量纲工程门限，需回放调节 |
| ESKF `imu_dt_` | 保留默认 0.01 s，作为离散 Q 的参考周期 |
| ESKF Q | 原数值不平方；每次采用 `Q * dt / 0.01` 的一阶时长缩放 |
| ESKF 观测噪声 | 原 0.01 仍作为方差，等效位置标准差 0.1 m、姿态标准差 0.1 rad |
| 历史 | 2 s，保留一个左边界样本，同时最多 20000 条 |
| 无有效雷达校正 | 超过 0.5 s 不再发布预测状态；后续有效校正允许恢复 |
| 输入异常 | IMU 倒退超过 1 s 或间隔超过 0.05 s，停止两条链路，要求整节点重启 |
| 版本 | 同次运行 epoch=0，sequence 递增；重复/过期版本拒绝，外国 epoch 拒绝 |
| 队列 | 主雷达上限 200 帧、IMU 上限 20000；溢出停止。可选副雷达上限 200 包，丢弃最旧包 |
| 路径/计时 | 路径最多 300 点、最多 10 Hz；耗时记录每项保留最近 10000 条并加锁 |

**Q 的语义发生了明确变化**：以前每条样本固定加 Q；现在按 10 ms 参考周期缩放。若实际输入为 200 Hz，每条约加 0.5Q。因此不是完全保持旧滤波权重，更不是已完成连续噪声模型标定；必须同 bag 比较响应和不确定性。没有同时平方噪声、重调数值或换成不明单位的 Qc。

选择了“错误时停用并整节点重启”的恢复策略，没有实现 bag seek 时自动清图、自动重初始化与新 epoch 切换。这样不会只重置外层而继续使用旧内层地图。短暂乱序/重复 IMU 丢弃，不清空正常历史。

饱和阈值在原 YAML 中写着不明确的单位，本次没有直接启用原不可达常速度回退分支。已知实际饱和的输入仍需由经核验的驱动/后续饱和策略处理，本版本不覆盖该项现场验收。

## 5. 实际验证

环境：Ubuntu 24.04.3，GCC 13.3.0，C++17，`-O1 -Wall -Wextra -pedantic -pthread`，仓库自带 Sophus，运行环境附带 Eigen。无 `/opt/ros`、`ros2`、`colcon`、CMake；没有用户 bag 或真值数据。

已运行 `test/run_core_tests.sh`。独立测试直接包含交付头文件，不是 Python 重写模型，也未用假的 ROS/PCL/Ceres 头代替实际依赖。完整输出在 `validation/core_test.log`。

| 检查 | 结果 |
| --- | --- |
| 快照往返、非法 P 原子拒绝、重复/负 dt/大间隔 | 通过 |
| 两个 SE3 观测重载，Joseph 更新 | 通过 |
| 静止 2 s | 无系统性位置漂移，绝对容差 1e-9 |
| 初速度 1 m/s、2 s | x=2 m，容差 1e-9 |
| 恒加速度 2 m/s²、2 s | x=4 m、vx=4 m/s，容差 1e-9 |
| 三轴固定角速度 | 与 SO(3) Exp 参考解一致，旋转距离容差 1e-9 |
| yaw 跨分支、近垂直姿态 | 通过，不用 RPY 相减 |
| 初始化后验在 1.002 s、5 ms IMU、迟到 83 ms | 及时/迟到的最终 p/P 一致，P 容差 1e-12 |
| 已运行轨迹接收迟到后验、偏置随状态恢复 | 及时/迟到最终 p/R/P 一致，容差 1e-12 |
| 未来 pending、重复版本、异 epoch、历史不足、NaN | 按预期拒绝/等待，缓存有界 |
| 缺失校正超时、拒绝帧不刷新年龄、有效校正恢复 | 通过 |
| 内外共用帧末插值与截断积分 | 状态及 P 一致 |
| 非零旋转、杆臂速度、pose/twist 协方差 | 18 列 Jacobian 中心有限差分检验，步长 1e-6，协方差容差 1e-8 |
| 支持量/代价/创新门限、正常高速运动 | 通过核心判定测试 |
| 副云排序、NaN、相邻半开窗口、偏移、未来保留 | 通过 |
| `git diff --check` | 通过 |
| 全包 `colcon build` | 未执行成功：`colcon: command not found`，退出 127 |

编译显示 4 条原有 Sophus 宏多余分号的 `-Wpedantic` 警告；未屏蔽警告，未修改第三方头。未发现本次新代码的编译警告。未运行完整 ROS 节点、Ceres 匹配集成、线程竞争检测或真实地图拒绝/恢复测试。没有可报告的 p50/p95/p99、CPU 改善、真实定位精度或抖动改善百分比。

## 6. 在实车工作空间验证

先将当前使用的 Adaptive-LIO 目录备份到工作空间 `src` 之外，再用交付 `Adaptive-LIO/` 替换目标包。不要将原版和新版两份同名 package 同时放入 `src`。如果本地代码不是报告的 HEAD，请先比较补丁，合并本地修改；不要直接覆盖新改动。

在已安装 Eigen 的机器上可先运行：

```bash
cd <工作空间>/src/localization/Adaptive-LIO
./test/run_core_tests.sh
```

`EIGEN3_INCLUDE_DIR` 可显式指向 Eigen 头目录；默认 `/usr/include/eigen3`。`CXX` 和 `BUILD_DIR` 可覆盖编译器及临时输出目录。独立测试只需要 g++、Eigen 和包内 Sophus。

进入你实际使用的 ROS 环境，例如只有确认安装 Jazzy 后才 source `/opt/ros/jazzy/setup.bash`。上传方案写 Humble，本实车实际发行版不能据此猜测。然后在工作空间根执行：

```bash
colcon list
colcon build --packages-select adaptive_lio --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select adaptive_lio --event-handlers console_direct+
colcon test-result --verbose
ros2 pkg prefix adaptive_lio
ros2 pkg executables adaptive_lio
```

依赖包未构建时先补齐依赖闭包。原工程 CMake 仍使用 `/opt/ceres-2.2`，并依赖 PCL/OpenCV/CSparse/Cholmod/ROS 消息包；这些未在本次环境装齐。修复了第三方配置把 `BUILD_TESTING` 强制关掉的问题，新核心测试已注册到 CTest。

先跑阶段 A（使用实际绝对 YAML 路径）：

```bash
ros2 run adaptive_lio adaptive_lio --config_file=<绝对路径>/config/mapping_m.yaml
```

确认静止、起步、刹停时输出正确后，再完全停止进程并改为阶段 B：

```bash
ros2 run adaptive_lio adaptive_lio --config_file=<绝对路径>/config/mapping_m_low_delay.yaml
```

bag 回放时给节点加 `--ros-args -p use_sim_time:=true` 并回放 `/clock`。配置经文件加载，普通 ROS 参数不自动覆盖 YAML。不要只编辑 source 配置却实际加载旧 install 中的另一份文件；查看程序打印的 `config_file`。

第一轮仅做两项现场核验：

1. 同 bag 分别运行原版、阶段 A、阶段 B，记录 `/odom`、`/tf` 和 `lio_quality`、`imu_output` 日志。核对匀速位置不再保持不变，odom/TF 时间戳相同，`twist` 与运动方向相符。使用 `ros2 topic info /odom --verbose` 检查只有一个发布者。
2. 检查场景遮挡/恢复段的 accepted、support、innovation_score 与 match_posterior_dp。确认拒绝时不继续刷新雷达校正年龄；超过 0.5 s 的停发应被控制侧正确识别，随后校正恢复。阶段 B 的副云点数可能变化，需要同时检查 `/livox/scan`。

后续根据这两项反馈再调 Q/R 或门限。不要仅以发布频率高、RViz 更顺滑认定定位更准。`state_age_s` 只有在 ROS 时间与传感器时间同源时才代表延迟；`match_ms` 用 steady_clock，可独立比较计算时间。DDS 丢包、锁等待和设备时钟偏差仍需现场补测。

## 7. 文件和回退

新增：`src/algo/imu_odom_predictor.h`、`odom_projection.h`、`match_quality.h`、`aux_time_crop.h`、`src/apps/imu_odom_output.h`、`test/test_core.cpp`、`test/run_core_tests.sh`、低等待配置及本说明。

修改：`eskf.hpp`、`main_ada.cpp`、`lidarodom.h/.cpp`、`lio_utils.h`、`common/timer/timer.h/.cc`、根 CMake、`cmake/packages.cmake`、原配置。计时器修改是为修复原来主线程与 LIO 并发写全局统计容器，以及记录无限增长的问题。

完整文件差异见 `changes.patch`，包括新增文件；它以实车仓库根目录为路径基准。`baseline_e0339963.tar.gz` 包含修改前的包，可用于回退；回退需恢复整个包并重新构建，不能只把 restamp=true 或旧 YAML 改回来。

未完成项明确包括：ROS 集成构建、实车 TF 唯一性、同 bag 性能对照、Ceres/地图异常注入、线程压力测试、现场门限标定、IMU 饱和处理、自动 epoch 重启。当前交付是可审查、可复现核心验证的候选实现，不宣称这些事项已验收。
