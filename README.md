# 🦾 BUAA Robotic Simulation Major Assignment  
**北航《机器人仿真》课程大作业**

---

## 📌 Project Overview | 项目简介

本项目为**北京航空航天大学《机器人仿真》课程大作业**，  
以 **Epson C3-A601 六自由度工业机械臂** 为研究对象，完成以下内容：

- 运动学建模与解析逆解推导  
- 工作空间分析  
- 关节空间与笛卡尔空间轨迹规划  
- ADAMS–MATLAB 联合动力学仿真  
- 图形化 GUI 控制与可视化动画演示  

项目基于 **MATLAB Robotics Toolbox (RTB)** 实现，  
并结合 **SolidWorks / ADAMS** 完成几何与动力学验证。

---

## 🧩 Assignment Tasks | 作业内容与完成情况

### 1️⃣ 运动学仿真与验证（15%）
- ✔ 使用 **Robotics Toolbox** 建立正运动学模型  
- ✔ 编写 **自定义 Modified DH 正运动学函数**  
- ✔ 推导并实现 **解析逆运动学闭式解**
- ✔ 与 RTB 结果进行数值与动画验证  

---

### 2️⃣ 工作空间分析（5%）
- ✔ 设定六关节转角范围  
- ✔ 通过随机采样法绘制机械臂工作空间  
- ✔ 与 Epson 官方 CAD 工作空间进行对比  

---

### 3️⃣ 轨迹规划与优化（30%）

#### 🔹 关节空间规划
- ✔ PTP 任务规划  
- ✔ 三次多项式 / 五次多项式 / S 曲线插补  
- ✔ 末端轨迹、速度、加速度对比分析  

#### 🔹 笛卡尔空间规划
- ✔ PTP 轨迹规划  
- ✔ CP 直线轨迹规划  
- ✔ CP 圆弧轨迹规划  
- ✔ 多点连续作业与速度连续性约束  
- ✔ 码垛（Stacking）完整任务规划  

---

### 4️⃣ ADAMS–MATLAB 联合仿真与动力学验证（25%）
- ✔ 建立 ADAMS 机械臂动力学模型  
- ✔ MATLAB–ADAMS 数据交互  
- ✔ 输出关节角、角速度、角加速度、驱动力矩  
- ✔ 与自编动力学程序结果进行对比验证  

---

### 5️⃣ GUI 图形界面交互（25%）
- ✔ 单关节 / 多关节控制  
- ✔ 任务参数输入  
- ✔ 实时显示末端位姿  
- ✔ 轨迹动画与可视化  
- ✔ 支持正运动学与逆运动学计算  

---

## 🖥️ Software Requirements | 软件环境

建议软件版本 **不低于** 以下配置：

- **MATLAB R2020a 或更高**
  - Robotics Toolbox for MATLAB **v10.4**
- **SolidWorks 2024**
- **AutoCAD Mechanical 2025**
- **Python 3.10**（可选）
  - 用于 SymPy 推导解析逆解  
  - 支持 Peter Corke 的 RVC3（非课程强制）

---

## 📦 Robotics Toolbox 安装说明

### ✔ 正版 MATLAB
使用 MATLAB Driver 安装：

- 安装程序：  
  `RTB安装/mathworksservicehost_2024.10.0.3_win64_installer.exe`
- MATLAB Drive 下载：  
  https://www.mathworks.com/products/matlab-drive.html

---

### ✔ 非正版 MATLAB
- 使用 `RTB安装/RTB.mltbx` 直接安装  
- 安装教程参考：  
  https://blog.csdn.net/ooorczgc/article/details/125110656

📘 推荐资料：
- `RTB用户手册.pdf`
- B站教程合集（RTB）：  
  https://www.bilibili.com/list/watchlater?oid=976064387

---

## 📂 Project Structure | 项目结构说明

```text
├─ Epson.mlx                      % 第1、2问总程序
├─ Modified_DH_Transform.m        % 正运动学（自编）
├─ my_ikine_epson.m               % 解析逆运动学（自编）
├─ Trajectory.mlx                 % 关节空间规划
├─ cartesion_PTP.mlx              % 笛卡尔 PTP
├─ cartesion_CP_line.mlx          % 笛卡尔 CP 直线
├─ cartesion_CP_circle.mlx        % 笛卡尔 CP 圆弧
├─ Stacking.mlx                   % 码垛任务
├─ ptp_trajectory.m               % PTP 规划函数
├─ line_trajectory.m              % 直线 CP 规划函数
├─ arc_trajectory.m               % 圆弧 CP 规划函数
│
├─ GUI/                            % GUI 界面
│  ├─ GUI.m
│  ├─ GUI.fig
│  └─ ...
│
├─ videos/                         % 输出动画视频
├─ figures/                        % 输出图片
├─ datas/                          % ADAMS 联合仿真数据
│
├─ EpsonC3A601_SW/                 % SolidWorks 模型
│  ├─ link0.STL
│  ├─ ...
│  └─ link6.STL
│
├─ EpsonC3A601_CAD/                % 官方工作空间 CAD
└─ README.md
```

**⚠️ 注意：STL 文件名不可更改，否则 RTB 无法加载模型**

---

## 🚀 Quick Start | 快速开始
### ▶ 运动学 & 工作空间
    run Epson.mlx

### ▶ 轨迹规划

依次运行：

* `Trajectory.mlx`

* `cartesion_PTP.mlx`

* `cartesion_CP_line.mlx`

* `cartesion_CP_circle.mlx`

* `Stacking.mlx`

### ▶ ADAMS 联合仿真

将模型导入 ADAMS

使用 datas/ 中的数据进行联合仿真

### ▶ GUI 交互
    cd GUI
    run GUI.m
---

## ⚠️ Notes | 注意事项

* 本项目使用 **Standard DH** 参数（后置 DH）

* 我们使用的是 2020a 版本的MATLAB，相同的项目程序，在更高版本的MATLAB下运行，可能会出现**腕关节朝向无法改变的问题**，目前不太清楚这是什么原因导致的。

* 所有角度单位默认 **弧度**

* GUI 中输入角度为 **角度制**

* GUI 中使用 `plot3d` 绘制动画时，光照调整比较困难，因此地面比较暗

* 码垛采用的路径规划每一段都是 **PTP**，经验证 **PTP** 效果最好，既不会又CP直线规划的加速度突变，也不会有CP曲线规划的不易求解的问题