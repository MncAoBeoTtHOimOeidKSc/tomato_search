# Tomato Search：高位臂植株搜寻

本工程用高位 Piper (`can_piper_high`) 和 D435i (`317622073937`) 查看绿色植株区域，记录深度点在导航地图上的位置，并通过现有导航适配器到达接近点。当前是单个绿色区域的调试流程，不做植物分类、多视角搜索或抓取。

先启动高位 Piper ROS 驱动、Ranger V3、Helios 和 Jetson 导航适配器。导航使用当前地图，先在 RViz 确认 AMCL 定位与扫描重合。有正在运行的节点时不要重复启动。导航期间不要用键盘、手柄或其他程序向底盘发送速度指令。

```bash
cd ~/vlm-jetson
bash scripts/start_navigation_ros.sh
```

手动把高位臂放到观察位并停稳，再录制。另录一次现场确认适合底盘移动的行驶收臂位。录制操作只读反馈，不发送动作，不使能电机。

```bash
cd ~/tomato_search
./run.sh record-pose observation
./run.sh record-pose travel
```

只看高位 D435i 时，在图形终端运行 `./run.sh camera-preview`，左侧为原始 RGB，右侧为绿色识别；不需要启动 ROS 驱动。按 Q / Esc 退出。`--check` 可在无桌面终端读一帧。

退出示教模式并确认高位臂已使能；在 Jetson 图形终端可先独立验证绿色分割，不需要车体外参和 AMCL，但需要底盘里程计证明车体停稳：

```bash
./run.sh capture --execute --preview-only
```

当前 `config.json` 已记录高位臂基座相对 `base_link` 的平移 `[0.150, 0, 0.439]` m，姿态与车体同向，RPY 为 `[0, 0, 0]`°。已有手眼文件给出的是相机到高位臂末端，双臂基座关系文件也不能代替车体外参。标定变更后重新采集目标。

```bash
./run.sh capture --execute
```

程序到观察位后打开相机，使用 HSV 筛选绿色和 RGB-D 深度。对选中的绿色区域采样约 2 秒，至少有 3 帧有效深度即可取平均；画面显示平均深度后可按 `S` 记录（`Q` / `Esc` 退出）。转换顺序为 `camera → arm_ee → piper_high_base → base_link → map`，车体位姿取自导航当前的 `map → base_link` TF，无需触发 AMCL 重新发布；输出 `runs/<时间>/target.json`，同时打印实际路径。建议接近点沿当前车体到植株的方向生成，默认车体中心距绿色区域的深度点 0.69 m，朝向植株。按导航车头轮廓 0.39 m（含 padding）估算，名义前方净距约 0.30 m；定位、到点误差和植株边缘都会影响实测净距。

```bash
./run.sh navigate --execute runs/<时间>/target.json
```

先核对终端打印的接近点和 RViz 路径，输入 `GO` 才会收臂并提交导航。程序检查记录时效、地图内容、车体 TF、目标格是否空闲以及导航适配器套接字，随后由现有导航适配器负责执行和确认停止。接近距离是车体中心到识别点的平面距离，静态空闲格检查不能证明路径无碰撞。超时或停止未确认时不自动重试。`Ctrl+C` 会请求取消正在执行的动作。

离线测试：`/usr/bin/python3 -m unittest discover -v`。

## 单次运行完整流程

先退出独立相机预览。首次使用需在现场确认行驶收臂位，从该姿态执行 `./run.sh record-pose travel`。Ranger 和 Helios 保持运行；每个终端使用同一个 ROS master。高位臂 CAN 已正常启用时，在独立终端启动：

```bash
source /opt/ros/melodic/setup.bash
source ~/piper_ws/devel/setup.bash --extend
roslaunch ~/tomato_grasp/piper_high.launch
```

该 launch 默认会使能并运行到六轴零位；启动前确认复位路径净空。另一个终端运行 `~/vlm-jetson/scripts/start_navigation_ros.sh`，完成新图的 AMCL 初始化后，从图形终端运行：

```bash
cd ~/tomato_search
./run.sh run --execute
```

该入口在移动观察前检查导航适配器套接字和行驶位配置。观察后自动对选中的绿色区域采样，允许间歇丢帧；有足够样本后保存平均深度和地图坐标，无需按 S。30 秒内未采集成功则退出，不提交导航。终端显示地图坐标和接近点，输入 `GO` 后才收臂并导航。单独调试仍可使用 `capture` 和 `navigate`。
