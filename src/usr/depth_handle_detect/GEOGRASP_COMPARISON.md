# 横杆简化 GeoGrasp 对比（2026-10-24）

已新增独立的抓取候选规划器，保留现有方案作为默认。本次对比中，现有方案的重复性更好；简化方案能生成候选，但一帧发生候选切换，未通过序列稳定性门限。

## 来源与自主实现范围

只参考用户提供的 [GeoGrasp 项目](https://github.com/yayaneath/GeoGrasp) 的 README 及其对应
[原论文：Fast geometry-based computation of grasping points on three-dimensional point clouds](https://journals.sagepub.com/doi/10.1177/1729881419831846)。
借鉴局部 PCA 法线、邻域曲率和接触候选评分的核心思想。
没有克隆、复制、编译或依赖上游实现；没有引入 ROS、PCL、神经网络或其他新依赖。
代码由本项目重新编写，是针对固定矩形横杆的简化改编，不声称复现原论文的完整算法或实验效果。

实现文件：`handle_detect/geograsp.py`；参数：`configs/geograsp.json`；对比入口：`scripts/compare_grasps.py`。

## 本项目的具体改编

1. 共用现有横杆检测 mask 和桌面法线，以比较抓取规划差异。新方法从 mask 内的原始深度点重新估计局部几何，不复用现有曲面拟合的中心、局部法线或切线。
2. 用 SciPy cKDTree 查邻域，排除缺少近邻支持的点；批量 PCA 得到法线和 `λ_min / Σλ` 曲率指标。法线朝向相机，退化线状邻域被拒绝。
3. 在杆的中央范围采样五个轴向位置：−25%、−12.5%、0、12.5%、25%；每个位置检查约 24 mm 长的局部上表面窗口。这是分析窗口，不是已验证的夹爪指腹尺寸。
4. 检查局部点数、法线一致性、平面内点、残差、宽度及左右边缘支持。中心缺点、单侧支持不足、靠近连接端均拒绝，不能跨过未观测的空洞构造接触点。
5. 从两条可见上边缘估计局部宽度，结合用户提供的 7 mm 厚度推断两侧截面中部接触点。抓取中心是接触点对中点。
6. 自定义评分综合中心偏好、低曲率、法线一致性、宽度匹配、左右支持平衡、残差。优先竖直接近；不满足几何条件时沿用现有最小倾角策略。

这套流程与原方案的区别是：原方案拟合整段弯曲上表面并使用弧长中点；新方案独立比较多个局部区域，再选局部接触点对。
本次没有添加时间平滑、候选锁定或人为删除跳变帧，避免将稳定性问题藏在滤波后。

## 单视角接触证据的限制

目前主要看到木条上表面。上边缘点可以约束横向范围，但两侧中部接触点和侧壁法线依赖矩形截面先验，不能当成实际观测。
因此输出明确区分：

- `inferred_contacts_m`：推断的侧壁中部接触点。
- `observed_top_edge_estimates_m`：由观测点估计的上边缘位置，不声称每个估计坐标恰是某个原始像素点。
- `observed_antipodal_verified=false`、`force_closure_verified=false`：没有证明观测到对向法线或力闭合。
- `clearance_checked=false`：缺少夹爪实体几何，未验证篮框碰撞。
- `gripper_opening_verified=false`：当前未提供夹爪最大张开宽度。建议张开量是局部宽度加每侧 2 mm 几何余量，不表示实际夹爪可执行。

`configs/geograsp.json` 可填写 `gripper_max_opening_m`，提供后才检查张开上限；这是行程检查，仍不是实体碰撞检查。
所有位姿都在深度光学坐标系，未接机器人、未做真实抓取实验。

## 相同 30 帧真实数据对比

输入：`data/captures/live_vertical_first_final/`，帧号 31–60，连续无跳帧。
现有 mask 的身份/几何筛选和桌面参考被两者共享，因此这比较的是两种抓取候选策略，不是两种独立检测器。

| 指标 | 原有弯曲木条拟合 | 简化 GeoGrasp |
|---|---:|---:|
| 几何候选生成 | 30/30 | 30/30 |
| X 标准差 | 0.410 mm | 4.447 mm |
| Y 标准差 | 0.114 mm | 0.116 mm |
| Z 标准差 | 0.360 mm | 0.398 mm |
| 最大无向轴变化 | 0.273° | 2.815° |
| 最大工具旋转变化 | 0.155° | 0.470° |
| 选择倾角 | 全部 0° | 全部 0° |
| 默认序列稳定性门限 | 通过 | 不通过 |

两者对应帧抓取点的平均距离约 4.03 mm；这是方法间差异，不是绝对定位误差。
没有量具中心真值，也没有真实抓取成功率数据，不能据此宣称任何方法更准确或抓得更牢。

简化方案 29 帧选中间候选，`frame_000006` 的中间区域因局部平面内点不足被拒绝，切换到 −12.5% 的相邻候选。
该帧的跳变使沿杆方向的标准差升到约 4.45 mm，超过 3 mm 门限。这是候选选择不连续，不是所有帧都没有几何解。
最终 `geograsp_pose.json` 因序列不稳定明确为 `success=false, pose=null`；逐帧候选仍保留在 frame_results.json 供分析。

## CPU 开销

最近一次本机离线重放：共用检测和原有姿态计算均值约 16.97 ms；简化 GeoGrasp 在该共享阶段之后额外增加约 10.43 ms，P95 约 12.07 ms。
两段时间口径不同，不将“原方案全流程”和“新方法追加规划”直接当作两个独立算法总耗时。
数值随主机负载变化，复现时以 comparison_summary.json 为准。当前新增方案没有进行实时相机长时间验证。

## 效果图、数值文件与复现

`outputs` 只保留效果图：

- `handle_detection.png`：原方案已有检测图，未替换。
- `geograsp_detection.png`：局部候选及所选接触点，序列失败时明确标记 UNSTABLE。
- `grasp_comparison.png`：左右并排对比；红点为截面抓取中心，白箭头为局部杆向，青色为接近方向，紫色接触线是推断接触点对，灰点为其他通过局部检查的候选。

数值输出位于 `reports/runs/latest/geograsp_comparison/`：
comparison_summary.json、comparison.csv、frame_results.json、geograsp_pose.json。

```bash
cd /home/pc/realman_RML63/src/usr/depth_handle_detect
.venv/bin/python scripts/compare_grasps.py
.venv/bin/python -m unittest discover -s tests -v

# 也可指定其他相同格式的离线序列
.venv/bin/python scripts/compare_grasps.py \
  --input data/captures/live_vertical_first_final \
  --config configs/real_handle.json --geograsp-config configs/geograsp.json
```

脚本退出码表示对比任务是否生成结果；是否接受该序列看 stability_pass 和最终位姿的 success/pose。
原有 run.py 默认行为保持不变。现阶段继续使用原方案，简化 GeoGrasp 保留作局部候选研究对照。
