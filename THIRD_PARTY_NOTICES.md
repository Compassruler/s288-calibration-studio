# 来源与第三方依赖

本项目是社区维护的S288调试与软件零位标定工具，不由宇树发布或维护。S288、Unitree等名称仅用于说明兼容设备，相关商标属于各自权利人。

## 项目代码

根目录的界面、通信、配置与标定代码按[MIT许可证](LICENSE)提供。通信实现使用S288报文格式、CRC与单位约定；部署标定导出保留14关节参考例程的字段与关节布局兼容定义。本仓库没有随包分发宇树SDK、部署例程或BAM源码，也不依赖它们运行。

## 机械模型

`assets/robot/`保留本项目使用者提供的机械装配显示快照，包含STL/OBJ网格及关节拓扑。网格未重做；显示数据增加了项目零位支架设计的静态姿态偏移及模型兼容指纹白名单，溯源摘要保存在`model.json`的`reference_pose`中，独立几何回归依据保存在`tests/fixtures/jig_zero_targets.json`。

软件代码的MIT许可不自动覆盖机械网格。机械零件涉及各自的作者和来源，本仓库当前未给这些素材指定统一的再许可；发布者需按其来源单列相应授权和署名。具体见[模型说明](assets/README.md)。

## 运行依赖

源码版的依赖由使用者通过Python包管理器安装；Windows便携版内置所需依赖，许可声明保存在随包的`licenses/`目录。各依赖以对应发行包中的声明为准，本项目的MIT许可证不替代这些依赖的许可。

| 依赖 | 用途 | 项目来源 |
|---|---|---|
| Python / Tkinter | 程序与窗口界面 | [Python](https://www.python.org/)、[Tcl/Tk](https://www.tcl.tk/) |
| pyserial | 串口通信 | [pyserial](https://github.com/pyserial/pyserial) |
| Pillow | 界面图片与圆角 | [Pillow](https://github.com/python-pillow/Pillow) |
| NumPy | 三维矩阵运算 | [NumPy](https://github.com/numpy/numpy) |
| MuJoCo | 只读三维预览渲染 | [MuJoCo](https://github.com/google-deepmind/mujoco) |
| GLFW | MuJoCo预览的窗口与图形上下文依赖 | [GLFW](https://www.glfw.org/)、[Python绑定](https://github.com/FlorianRhiem/pyGLFW) |

构建工具由`requirements-build.txt`声明，PyInstaller用于生成Windows便携版，不是源码运行所需的额外依赖。构建时保留实际发行包附带的许可，不能用本项目MIT声明替代依赖许可。

字体使用Windows已安装字体，不随源码包分发字体文件。自行打包EXE时应保留随附依赖的许可文件。
