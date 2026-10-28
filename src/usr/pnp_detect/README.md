# pnp_detect：传统 OpenCV 篮口定位与平面 PnP

工程位于 `/home/qi/Project/pnp_detect`，Ubuntu 22.04 / Python 3.10，独立 `.venv`，仅需 OpenCV headless、NumPy、PyYAML。没有深度学习、ROS 或机械臂控制。

当前完成了用户截图和一张公开网络篮图的检测，以及经过合成数据验证的 PnP 模块。**相机型号是 L515，但目前没有真实 RGB 内参、实测篮子外缘尺寸和截图坐标变换信息，因此尚未输出真实篮子的 6D 位姿。**

## GitHub 开源算法对照

新增原版 `docdetect`（MIT）对照后端，固定源码 commit，源码未修改，同一套上游参数处理全部 5 张图片。自动几何检查有 1 个候选通过，但人工视觉核对发现混用内外缘并否决，因此当前没有确认的篮口定位成功结果。未将它替换为默认后端。

详见 [GitHub 算法评估记录](reports/github_algorithm_review.md)，含其他候选项目、许可证、排除原因、复现命令与适配边界。`third_party/docdetect/UPSTREAM.json` 和 LICENSE 保留上游版本及逐文件哈希。比较结果最终图在 `outputs/docdetect_20261024/`，JSON 在 reports 下，原图未修改。

## 统一批量评估（2026-10-24）

当前默认使用 `configs/general.yaml`：所有图片使用同一份参数，不指定颜色、位置 ROI 或逐图片覆盖配置。参考 [OpenCV 官方 squares.cpp（4.13.0）](https://github.com/opencv/opencv/blob/4.13.0/samples/cpp/squares.cpp) 的金字塔平滑、B/G/R 多通道、多阈值、Canny 和轮廓四边形候选流程；本项目额外沿用原图边缘/直线/几何检查。为允许透视，采用已有凸四边形角度范围，不照搬示例的近 90° 假设。它是四边形基线，不是已验证的通用篮子识别器，也不是原封不动运行上游程序的结果。

```bash
cd /home/qi/Project/pnp_detect
source .venv/bin/activate
python scripts/detect_batch.py \
  --input data/images/web \
  --config configs/general.yaml \
  --output outputs/web_batch_next \
  --report reports/web_batch_next.json

# 单张图片使用相同默认配置：
python scripts/detect_image.py --image data/images/web/image.png \
  --output outputs/single_next
```

用户本次所说的目录实际为 `data/images/web`。本轮 3 张新增图 + 1 张旧黑篮对照，全部使用统一配置，**0/4 通过图像可靠性检查**。新增红白篮、白篮无可靠四边形；装物品的红篮触及轮廓搜索预算，不能在未完成搜索时选取四点；旧黑篮候选未通过真实边缘支持检查。处理完成但不代表识别成功，不按单图更改参数来制造通过结果。

结果：`outputs/web_batch_20261024/`，只含 4 张最终 PNG；失败图标记 FAILED 并保持角点为空。统计与失败原因在 `reports/web_batch_20261024.json`。原始图片未覆盖、未修改，图片上的原有文字/水印保留。统一配置 SHA256 写入报告，所有图片没有逐图片配置覆盖。

`basket_real.yaml` / `dark_region.yaml` 等旧配置保留作历史诊断对照；此前单图检测成功不能当作当前统一配置的效果。下一步改进必须有算法出处、保持固定配置，并在独立图片集上评估误检与漏检。单元测试通过只证明代码行为，不证明场景泛化。

按用户偏好，检测入口**默认只保存 `detected_corners.png`**。成功返回 0，拒绝检测/输入错误返回 2；失败图不画任意四点。缺失/不可解码图像时只能打印原因，不能制造 PNG。非空输出目录拒绝覆盖。

需要机器可读结果时显式加 `--json reports/detection_next.json`，JSON 保存至 outputs 之外。调试时显式使用 `--debug --output /tmp/pnp_debug_next`，才保存 original、gray、blurred、search_mask、edges、closed_edges、contours、hough_lines、candidates、rejected_XX、corners.json 等中间结果。不要将调试输出混进最终图片目录。

outputs 中仅保存各次运行的最终检测 PNG（历史两张以及本轮四张失败标注图）。JSON、来源与验证摘要在 reports / SOURCES.md；临时实验在 /tmp。

## 检测算法与适用边界

基础步骤：校验图像尺寸/通道/格式，灰度 GaussianBlur/Canny，轮廓 approxPolyDP，凸性/面积/边长/角度检查，Huber 直线拟合及求交点，再检查真实边缘重合、逐边 Hough 支持、缺口和歧义。没有使用外接矩形四点替代篮口。

三个候选来源：

- 闭合轮廓近似：适合边界完整的四边形。
- 有预算限制的 Hough 四线组合：适合被把手打断的篮口；复杂背景候选太多或多个候选接近时拒绝输出。
- 新增颜色区域外边界：HSV 分割连通区域，取外轮廓/凸包近似作为直边种子，仅用外轮廓中段点拟合四条直线，再求交点。网格孔洞不会直接成为外边界，圆角的轮廓端点也不会直接充当角点。最终仍在原图 Canny 上独立验证。

`configs/dark_region.yaml` 的区域方法针对暗色连通边框，像素阈值按 `reference_long_side_px: 1000` 自动缩放；**不改变图像分辨率或输出坐标**。平移/透视合成变换、500/750/1200 分辨率测试已通过，但颜色、连通性或严重遮挡不满足时仍可能失败。

`configs/basket_real.yaml` 仍使用手工搜索区域 `[165,25,195,180]` 和至少一组对边夹角 <=8° 的近俯视先验。这些是场景先验，**不能说是全图自动找到了篮子**。用户原图的无 ROI 配置 `complex_dark_rim.yaml` 仍可能报告线缆竞争候选歧义；新增方法没有证明已解决用户现场的全部背景干扰。

结果 `success` 只表示通过配置的图像检查，支持度分数不是准确率/概率。圆角采用**拟合直边的虚拟交点**，点可能落在可见圆角轮廓之外；后续三维模型必须采用同一定义。仅凭几何无法普遍区分篮子上沿、侧面和背景矩形。

## 角点顺序与人工标注

输入像素原点在左上角，x 向右、y 向下。四点按图像顺时针排列，从 x+y 最小的点开始，平局按 y、x；图像标签依次为 image_top_left / image_top_right / image_bottom_right / image_bottom_left。

同一图片结果确定，但物体旋转后同一个编号可能对应另一个物理角。**不能把图像排序当成篮子模型方向。** 检测结果默认 `physical_corner_mapping: null`、`orientation_ambiguous: true`、`pnp_ready: false`。

无远程桌面也可人工点击：

```bash
python scripts/manual_annotate.py --image data/images/basket.png \
  --export-html /tmp/basket_annotation.html --output /tmp/manual_basket
# 下载 HTML 到本地浏览器打开，点击四点并导出 manual_points.json；再上传 JSON：
python scripts/manual_annotate.py --image data/images/basket.png \
  --points-file data/images/manual_points.json --output /tmp/manual_basket
```

人工输出明确标记 `source: manual`；可用 `--compare` 对同一 SHA256 原图的自动结果做逐角像素差对照。人工标注也不会自动解决物理方向对应。

## PnP 已实现的内容

`src/pnp_detect/pnp_estimator.py` 使用 `solvePnPGeneric(..., SOLVEPNP_IPPE)` 保留平面候选，检查正深度/有限值；两个不同解的重投影误差接近时报告歧义，返回 `pose: null`。通过检查后，对最佳解做 LM 精化并验证重投影 RMSE/最大点误差。具体方法见 [OpenCV 4.13 官方 PnP 文档](https://docs.opencv.org/4.13.0/d5/d1f/calib3d_solvePnP.html)。

- 返回 `T_camera_object`，满足 `X_camera = R @ X_object + t`；平移单位米，rvec 单位弧度。相机光学坐标为 x 向右、y 向下、z 向前。
- 篮口模型中心为原点，X 沿实测 length，Y 沿 width，Z 按右手系。模型点顺序：`[-L/2,-W/2,0]`、`[L/2,-W/2,0]`、`[L/2,W/2,0]`、`[-L/2,W/2,0]`。
- 必须显式填写经确认的 `model_to_image_indices` 与确认依据。算法不会尝试任意交换点来“得到一个好看的位姿”。
- 相机 K/畸变对应源相机分辨率。已知裁剪/缩放时，用 `source_to_input_3x3` 的逆将检测像素还原到源像素，再求解；重投影误差在当前输入图片像素上计算。未知变换直接拒绝。
- 只支持明确兼容 OpenCV pinhole 的畸变参数，不将其他畸变模型的系数直接当作 Brown 参数。
- `robot_ready: false`、`depth_validated: false`。低重投影误差不能证明标定、物体尺寸或物理角对应正确，也不能证明可以抓取。

### 先运行合成验证

```bash
python scripts/demo_pnp.py --output reports/pnp_synthetic_next.json
```

该 demo 使用明确标记的合成相机、合成尺寸、已知位姿投影。结果标记 `SYNTHETIC_ONLY`，没有使用用户或网络照片的伪造内参。

### 真实 L515 数据准备后运行

填写 `configs/pnp.yaml`，目前未知值为 null / confirmed=false，默认无法计算真实位姿。需要：

1. 实际 RGB 流分辨率对应的 fx/fy/cx/cy、畸变模型/系数与标定来源；不要使用深度流内参或通用“L515 默认内参”。
2. 当前篮子直边外缘虚拟交点之间的实测长、宽（米），不能使用网络上另一款篮子的商品尺寸。
3. 可辨识结构确认的模型角到检测编号对应；矩形的旋转对称性不能靠图像左上角消除。
4. 截图是否裁剪、缩放；源分辨率和从源像素到截图像素的确切 3×3 变换。native 要求同分辨率与单位变换。

```bash
python scripts/estimate_pose.py \
  --image data/images/basket.png \
  --detection-config configs/basket_real.yaml \
  --pnp-config configs/pnp.yaml \
  --output reports/pnp_real_next.json
```

当前执行应拒绝并说明缺少真实标定。填写真实数据后可用 `--visualization outputs/pose_next.png` 保存通过校验的坐标轴图。也支持 `--corners-file reports/detection_next.json`，此时校验原图 SHA256 和分辨率，避免误用另一张图的角点。JSON/PNG 已存在时拒绝覆盖。

## 验证与样本来源

```bash
python -m unittest discover -s tests -v
```

覆盖角点排列/退化四边形、基础与复杂合成检测、重复运行、遮挡/缺边/双目标拒绝、预算耗尽、网络样本及已知变换的一致性、不同分辨率、PNG 精简输出；PnP 覆盖已知合成位姿、畸变、非等比例缩放/裁剪还原、显式点映射、平面歧义、重投影拒绝及完整 CLI。

网络样本为 Regency 黑色购物篮近俯视商品照片，详见 [SOURCES.md](SOURCES.md)。这是新增的一张调参样本，透视/平移等版本由它生成，**不是更多独立实拍场景，也不是独立测试集准确率**。没有为网络照片计算真实位姿。

重新配置环境可用 `python3 -m venv .venv && .venv/bin/python -m pip install -r requirements.txt`；系统缺少 ensurepip 时，可使用已有 uv：`uv venv .venv --python python3`，再用 `uv pip install --python .venv/bin/python -r requirements.txt`。当前虚拟环境已配置好。

L515 深度校验、手眼标定、抓取偏移、机器人运动仍未实现，后续独立接入。
