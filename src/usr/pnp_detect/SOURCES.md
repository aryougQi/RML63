# 网络测试图来源

- 图片：[Regency 黑色购物篮近俯视商品照片](https://www.webstaurantstore.com/images/products/extra_large/446885/1779114.jpg)。
- 来源页：[WebstaurantStore 产品页](https://www.webstaurantstore.com/regency-black-16-1-8-x-11-plastic-grocery-market-shopping-basket-with-plastic-handles-pack/176MSB24LBK.html)。
- 获取日期：2026-10-23；保存为 `data/images/web/regency_top_view.jpg`，1000×1000，SHA256 见 `reports/web_source.json`。
- 权利仍属原权利人；未核实开放许可，仅作本地算法评估，不声称可自由再分发。
- 用于检测调参与已知像素变换测试，没有真实角点/位姿真值或内参；未把商品尺寸或假内参用于真实 PnP。

PnP 实现参考：[OpenCV 4.13 Perspective-n-Point](https://docs.opencv.org/4.13.0/d5/d1f/calib3d_solvePnP.html)。

## 统一四边形候选基线

参考：[OpenCV 4.13.0 squares.cpp](https://github.com/opencv/opencv/blob/4.13.0/samples/cpp/squares.cpp)，查阅日期 2026-10-24。`opencv_quads.py` 是流程的独立 Python 实现：金字塔平滑、多颜色通道、多阈值/Canny、轮廓与多边形近似。新增预算限制和原图边缘验证；将上游近直角过滤替换为现有透视四边形几何检查。不是直接移植一个通用篮子检测模型。

本次另外 3 张图由用户放入 data/images/web；原始网页 URL 未提供，不杜撰来源。未移除水印，未为这些图估计真实 PnP。

## GitHub 原版 docdetect 评估后端

[alessandrozamberletti/docdetect](https://github.com/alessandrozamberletti/docdetect/tree/dc29391bcc1494d662e3fe7cabd630b708ab2a70)，MIT；原版源码保存在 third_party/docdetect，逐文件 SHA256 和 commit 见 UPSTREAM.json，LICENSE 原样保留。源码未经修改；包装层只处理统一缩放、重复/退化过滤、超时隔离与既有质量验证。其他项目及适用性比较见 reports/github_algorithm_review.md。
