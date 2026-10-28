# depth_handle_detect

L515 原始深度横杆检测；Python / OpenCV / NumPy / SciPy，CPU 运行，不使用神经网络、不控制机械臂。

项目：`/home/pc/realman_RML63/src/usr/depth_handle_detect`，与 pnp_detect 独立；没有修改后者。
本机已创建 `.venv`，使用 Python 3.8、NumPy 1.24.4、SciPy 1.10.1、OpenCV 4.10.0、pyrealsense2 2.50.0.3812。

当前对象是用户确认的略带弧度木条，用户提供近似尺寸 200×16×7 mm。默认使用 `configs/real_handle.json`：
人工确认的深度 ROI + 曲面拟合 + 水平桌面法线 + **竖直优先，必要时选最小几何倾角**。

## 当前已验证

最新真实原始数据：`data/captures/live_vertical_first_final/`。
连续 30 帧（帧号 31–60），30/30 检测、姿态计算成功，全部选择 0° 竖直接近。
位置各轴标准差约 0.410 / 0.114 / 0.360 mm，轴向最大变化约 0.273°；这些是重复性，不是绝对精度。
检测含桌面法线和姿态计算，均值约 15.01 ms；实时流约 30 FPS。详见 REPORT.md。
按当前要求，outputs 仅保留最终检测效果图；统计、位姿和点云保存到 reports/runs/，原始数据保存到 data/。

仍需验证：实物截面中心绝对误差、不同位置/朝向、夹爪空间间隙。完整自动篮子定位尚未实现。

## ROS 统一采集入口（当前默认）

相机统一发布、检测与实机抓取的启动命令见 [ROS_USAGE.md](../demo/depth_handle_grasp/ROS_USAGE.md)。
ROS 相机运行时，使用 `detect_ros.py` 或 `grasp_demo.py` 订阅，不能同时运行下面的 SDK live/capture 命令。
原离线检测和几何算法保持不变。

## 独立 SDK / 离线运行

```bash
cd /home/pc/realman_RML63/src/usr/depth_handle_detect
source .venv/bin/activate
python -m unittest discover -s tests -v

# 相机实时检测：每次使用新的输出目录
python run.py live --output outputs/live_new --config configs/real_handle.json \
  --frames 30 --width 320 --height 240 --min-distance-mm 250

# 只采集，不运行检测；加 --color-preview 可保存原生 RGB 参考
python run.py capture --output data/basket_new --frames 30 \
  --width 320 --height 240 --min-distance-mm 250 --color-preview

# 离线复现最终真实序列
python run.py stability --input data/captures/live_vertical_first_final \
  --config configs/real_handle.json --output outputs/replay

# 单帧检测
python run.py detect --input data/captures/live_vertical_first_final/frame_000000 \
  --config configs/real_handle.json --output outputs/single
```

本机实际连接 USB 2.1，SDK 当前仅提供 **320×240@30** 深度流，不支持当前连接下的 640×480。
请求不支持的分辨率会明确列出可用模式。CLI 通用默认值仍为 640×480，本机请使用上述明确的 320×240 参数。

相机此前设置为最小距离 490 mm，横杆约 400 mm 处的深度因此被过滤。`--min-distance-mm 250` 临时调整硬件裁剪，
采集结束或异常退出时恢复原最小距离和预设；本次实测恢复为 Max Range / 490 mm。
没有降低置信度阈值，没有使用无效像素绕过功能或填洞生成三维点。
SDK 原生设置、深度单位、USB 类型、帧号和时间戳均记录在每帧元数据中。

采集模式预热 30 帧；live 还在正式记录前预热算法，避免首次线性代数初始化造成跳帧。
硬件选项在会话开始查询一次，避免每帧 USB 查询阻塞。实时模式每帧检测和保存最后结果，结束后离线统计同一原始序列。

## 配置与适用范围

`configs/default.json` 是未确认参数模板，禁止据此直接生成真实抓取位姿。
`configs/real_handle.json` 使用本次用户提供的近似实物尺寸与确认的水平桌面条件。

- `roi_xywh`：原生深度网格 `[x,y,w,h]`；完整包含目标并留边缘。
- `roi_confirms_handle_identity`：操作者确认 ROI 内目标身份。
- `length_m / width_m / thickness_m`：实物先验；当前 .200 / .016 / .007 m。
- `length_tolerance_m / width_tolerance_m`：当前 .025 / .006 m，是筛选容差，不是已验证的测量误差界。
- `dimensions_verified`：当前 true，表示形状和近似尺寸来自用户；不表示量具精度已验证。
- `dimension_uncertainty_m`：当前 null，未编造用户没有提供的尺寸误差界。
- `shape`：`rectangular_top_face` 为平直矩形上表面；`curved_rectangular_top_face` 为轻微弯曲木条上表面。
- `depth_range_m`：只隔离横杆，不包含参考平面。机械狗安装视角当前使用 .40–.54 m，篮子移动后必须重新检查。
- `segmentation_max_local_depth_range_m`：可选的 3×3 原生深度邻域极差上限，用于剔除前景与背景交界的混合深度边缘；不填洞或修改参与拟合的深度值。当前使用 .025 m，改变视角后需重新验证。分割后以相邻原始测量点恢复尺寸边界，并保留原有曲面内点与质量门槛。
- `include_full_roi_component`：当前 true，在指定深度范围内先拟合完整连通区域，避免深度切层截断弯曲横杆。
- `max_bow_sag_m`：允许的中心线最大弓高，当前 15 mm；超出返回失败。
- `support_plane`：用户确认水平桌面的深度 ROI；拟合平面法线作为竖直接近方向，未假定相机 Z 就是竖直方向。

篮子位置/相机视角改变后，须同时检查横杆 ROI、工作深度范围和桌面 ROI。当前桌面 ROI 是右侧裸露桌面。
桌面被遮挡、非平面、内点不足或残差超限时，姿态返回失败。程序不自动证明一个平面就是水平桌面。

安装位置变化后，可用 `python3 scripts/select_roi.py --live --support-plane --write-config`
重新选择已确认水平的裸露参考面（地面或水平平台）。此模式更新 `support_plane.roi_xywh`
及根据原生深度估计的范围，保留把手 ROI 和原有平面质量门槛。选择参考面即表示操作者确认其水平；
机械狗身体倾斜时，不能把其平台直接当作水平面。保存后仍须通过平面拟合及新采集序列验证。

全图几何候选搜索与可选 `basket_relation` 位置/轴向约束仍可用，但不等于完整自动定位篮子。
圆柱、严重弯曲木条或只见侧面，需要相应模型；不把它们冒充已验证矩形截面。

## 竖直优先和小角度回退

默认 `approach_policy="vertical_first"`：

1. 先用水平桌面向下法线生成 0° 竖直接近。
2. 若表面/轴向几何条件不满足，再按绝对角度从小到大测试 ±5°、±10°、±15°、±20°、±25°。
3. 选第一个满足几何门限的方向；都不满足时返回失败。

倾斜轴为杆切线在水平面内的投影；工具 X 尽量沿局部杆切线，工具 Y 为闭合方向，工具 Z 为接近方向。
当前真实篮子满足竖直接近，因此默认结果倾角为 **0°，不是固定 15°**。
`configs/real_handle_tilt15.json` 和相应 reports/archive_outputs_20261024/live_tilt15_final 仅用于固定倾角测试，不是生产默认策略。

**目前回退依据几何条件，不依据未知的夹爪与篮框碰撞。** JSON 明确输出 `clearance_checked=false`。
要自动判断“竖直会碰篮框、倾斜可以通过”，需要夹爪外形、张开宽度和 TCP 等尺寸及障碍间隙检查；本项目尚未实现该判断。
没有控制机械臂，也没有把视觉位姿标成可直接执行的机器人动作。

## 几何与坐标约定

原始格式：

```
frame_000000/
  depth_raw.npy   # 原生 Z16 / uint16；0 无效
  metadata.json  # 原生深度内参、畸变、米/单位、设备和帧信息
  rgb.png        # 可选；原生彩色，仅供看场景，不是深度对齐图
```

`z_m = depth_raw * depth_scale_m_per_unit`，单位从 SDK 读取。不能使用伪彩色/8-bit 截图计算三维坐标。
RGB 与深度不是同一网格，ROI 始终使用原生深度坐标；无畸变用针孔反投影，非零畸变调用 SDK。
约定参考 [RealSense 官方 Projection 文档](https://github.com/realsenseai/librealsense/wiki/Projection-in-RealSense-SDK-2.0)。

预处理中值滤波只用于分割。曲面模型先对候选内部原始点做 RANSAC 和稳健拟合，排除 L515 边界混合深度；
再将模型用于完整候选的原始点，恢复表面内点，不能直接把腐蚀后宽度当成横杆宽度。
曲面模型为局部 `z=a+b*u+c*v+d*u²`，中心线允许轻微横向曲率，检查轴向覆盖、长宽、残差、内点比例和弓高。
杆方向取弧长中点的切线，长度是可信可见中心线弧长，不一定包含被连接部位遮住的整根木条。

可见表面中心与实体截面中心分开保存：实体中心估计 = 表面中心 − 局部朝相机外法线 × 厚度/2。
采用用户给定的 7 mm 厚度，厚度误差会直接影响实体中心；无法从单侧深度独立验证背面位置。

所有点和矩阵使用 **depth_optical：X 右、Y 下、Z 前，单位米**。
`R_camera_tool` 的列为工具 X/Y/Z，四元数 XYZW。没有变换到机器人基坐标系，未添加具体夹爪 TCP 偏移。

## 输出和失败语义

```
outputs/
  handle_detection.png           # 最终检测效果，当前仅保留此文件
reports/runs/latest/
  depth_visualization.png
  handle_mask.png
  handle_points.ply
  handle_pose.json
  test_results.csv
  stability_summary.json
  geometry_summary.json
  run_metadata.json
data/captures/<运行名>/             # 原始深度和相机元数据
```

使用 `--output outputs/<运行名>` 时，只有检测效果图进入该子目录，其他文件自动进入
`reports/runs/<运行名>/`，实时采集的原始数据进入 `data/captures/<运行名>/`。
原 outputs 下的旧实验内容已移到 `reports/archive_outputs_20261024/`，没有删除原始测量证据。

可视化：红点为可见表面中心，白箭头为局部切线，紫线为拟合中心线，青色箭头为接近方向。
PLY 为原始测量的表面拟合内点；无有效拟合时为空点云。

`success=true` 表示几何检测和视觉抓取姿态构造均通过；失败时 `pose=null`，并给出 reason。
`detection_success=true` 但 `success=false` 可能表示检测到目标，而桌面方向/姿态尚不可靠；两种状态分开记录。
只在身份和物理截面已确认时计算实体中心估计；其精度仍受近似尺寸和深度误差影响。
多个有效候选、无效深度、截断、模型不适合、拟合失败均返回失败；空输入也会清理旧位姿与点云。
单帧检测不伪造稳定性 CSV。退出码 2 表示失败或序列验证不通过。

稳定性默认要求：30 帧、无重复/跳帧、内参与设备一致，检测成功率≥95%，位置各轴样本标准差≤3 mm，
轴向相对平均轴最大偏差≤3°。工具姿态另统计整体旋转变化，结果见 grasp_pose_stability_pass。
真实静止性由操作者保证；低方差不代表绝对准确，也不证明篮子未移动。

## 合成数据、深度质量与环境复现

```bash
python run.py synthetic --output data/synthetic_new --frames 30
python run.py stability --input data/synthetic_new \
  --config data/synthetic_new/synthetic_config.json --output outputs/synthetic_new
python scripts/inspect_capture.py --input data/captures/live_vertical_first_final \
  --config configs/real_handle.json --output outputs/quality_new
```

19 项 unittest 覆盖平直/弯曲真值、混合边缘深度、桌面法线、竖直优先与倾斜回退、异常数据、多候选、跳帧、旧位姿清理。

重建：`python3 -m venv .venv` 后 `pip install -r requirements-camera.txt`。
本机系统缺 ensurepip，已通过项目内 get-pip 引导，使用 PyPI wheel 并核对 SHA256 后安装，没有修改系统 Python。
`wheelhouse/` 留存依赖，可离线安装：`pip install --no-index --find-links wheelhouse -r requirements-camera.txt`。
模块位于 handle_detect/，采集、几何、桌面法线、检测、可视化、统计分离，便于后续 ROS 适配。

## 新增：横杆简化 GeoGrasp 对比

已自行实现局部法线/曲率和接触点对评分方案，不复制或依赖上游 GeoGrasp 代码。

```bash
.venv/bin/python scripts/compare_grasps.py
```

使用相同真实 30 帧：两者均生成 30/30 个逐帧候选；原方案位置标准差最大约 0.41 mm，
简化 GeoGrasp 因一帧候选切换升到约 4.45 mm，序列稳定性未通过。原方案继续作为默认。
详情见 [GEOGRASP_COMPARISON.md](GEOGRASP_COMPARISON.md)。对比图在 outputs/grasp_comparison.png，
数值在 reports/runs/latest/geograsp_comparison/；outputs 仍只存效果图。
