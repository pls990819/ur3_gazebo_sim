# 安装Pinocchio

## 添加robotpkg 的软件仓库

```bash
sudo tee /etc/apt/sources.list.d/robotpkg.list <<EOF
deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -sc) robotpkg
EOF
 
curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key | sudo apt-key add -
sudo apt update
```

## 安装Pinocchio 及其依赖项

```bash
 sudo apt install -qqy robotpkg-py3*-pinocchio
```

## 在~/.bashrc中添加（可以不添加）

```
export PATH=/opt/openrobots/bin:$PATH
export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
export PYTHONPATH=/opt/openrobots/lib/python3.8/site-packages:$PYTHONPATH
export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH
```

# 使用

## 打开Gazeboz的ur3仿真环境

```bash
roslaunch ur3_gazebo ur3.launch
```

## 控制器

- 打开控制器节点

```bash
rosrun ur3_controller ur3_adaptive_controller
```

- 打开运动规划节点

```bash
rosrun ur3_controller ur3_motion_planner
```

## 生成关节角运动轨迹并跟踪

- 在interface_node界面输入期望关节角，前六个对应关节角姿态，最后一个为运动时间，如：1,-1,1,-1,1,-1,10
用10秒钟，从初始位置，运动到关节位形为[1,-1,1,-1,1,-1]的状态
