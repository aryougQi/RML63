# 本机部署与抓取点仿真

部署于 `/home/qi/Project/RML63/fixed_grasp`，复用已有
`rml63/noetic-sim:local` 镜像中的 ROS Noetic、Gazebo 11、MoveIt 和依赖。
原始源码、冻结 URDF、两套示教点和标定参数保留。新增仿真源码位于
`simulation/`，不加入原工程的七个包，也不调用真机启动脚本。

```bash
cd /home/qi/Project/RML63/fixed_grasp
bash deploy.sh
# 验证当前场景
docker compose exec sim /workspace/simulation/env.sh rosrun rml63_fixed_sim check_scene.py
# 日志和停止
docker compose logs --tail=80
docker compose down
```

默认打开 Gazebo 和 RViz。无图形界面：

```bash
SIM_GUI=false SIM_RVIZ=false docker compose up -d
```

切换第二套抓取位置（场景只放置一个长方体）：

```bash
GRASP_PROFILE=2 docker compose up -d --force-recreate
```

仿真独立使用 `http://127.0.0.1:11321`，不会占用之前工程的 ROS 11311。
机器人 base_link 在 world 中位于 `(0,0,0.5)`，朝向相同；原 URDF 的
`dummy_joint` 3.14 rad 旋转由 world→dummy 的 -3.14 rad 抵消。

长方体中心对应选中的 `waypoints*.yaml / Grasp / tcp_base` 平移；
姿态保持水平，长轴沿 TCP Y 在水平面的投影。默认尺寸 120×30×60 mm，
为缺少物体 CAD 时的可修改假设，定义在 `simulation/rml63_fixed_sim/scripts/scene.py`。
支撑台高度根据长方体底面计算，也是近似场景。

Gazebo 使用原冻结模型的派生副本，固定机器人基座、去掉不稳定的网格接触，
沿用 position 轨迹控制器；MoveIt 保留完整原模型碰撞几何。
仿真 Home 命令将示教关节读数加上配套零位修正，再交给 nominal URDF；
不修改原始关节参数或原始模型。红球标出示教抓取 TCP。

此仿真已包含机械臂运动、动态长方体、静态支撑、MoveIt 场景和位置自检。
冻结模型的夹爪保持固定；当前不验证接触夹持、抓起或搬运成功。
真机 `start.sh`、`client.sh` 保留在源码中，本次部署没有运行它们。

源码离线验证和原有单元测试：

```bash
docker compose exec sim /workspace/simulation/env.sh bash -c 'python3 verify_package.py && python3 -m unittest discover -s src/usr/fixed_grasp/tests -p "test_*.py"'
```

原始包：`fixed_grasp_standalone_20261103_source.zip`

SHA256：`7187832192d39aeb5153953d3a9de92980b9facaf8dcb4925520eea32f1b1ce9`

`MANIFEST.sha256` 覆盖原包文件，新部署和仿真文件不在其中。
本地 `outputs/scene_report.json` 记录当前实际物体位置及验证结果。

`deploy.sh` 优先复用本地基础镜像；其他机器缺少时使用 `simulation/Dockerfile.base` 重建依赖。
