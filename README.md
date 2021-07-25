# imusim
imu和cam数据仿真，用于vio算法测试，代码有任何问题都欢迎交流 heyijia_2013@163.com。

we also create a ros_version in ros_version branch.

![demo pic](./bin/demo.png)

## 坐标系
- **B**ody frame: imu坐标系

- **C**am frame: 相机坐标系

- **W**orld frame: imu坐标系的第一帧位置

- **N**avigation frame: NED（北东地）or ENU（东北天），本代码采用的是ENU，重力向量在该坐标系下为$(0,0,-9.81)$

目前，imu的z轴向上，xy平面内做椭圆运动，z轴做正弦运动，x轴沿着圆周向外。外参数Tbc将相机坐标旋转，使得相机朝向特征点。

## 代码结构
main/gener_alldata.cpp : 用于生成imu数据，相机轨迹，特征点像素坐标，特征点的3d坐标

src/paramc.h：imu噪声参数，imu频率，相机内参数等等

src/camera_model.cpp：相机模型，调用的svo，目前代码里这个文件删掉了

python_tool/：文件夹里为可视化工具，draw_points.py就是动态绘制相机轨迹和观测到的特征点。如果是ubuntu不需额外安装，windows需要安装python matplot等依赖项

## 数据存储的格式
### 特征点
> x，y，z，1，u，v

每个特征出现在文件里的顺序，就是他们独立的id，可用来检索特征匹配

### imu data
> timestamp (1)，imu quaternion(4)，imu position(3)，imu gyro(3)，imu acc(3)

### cam data
> timestamp (1)，cam quaternion(4)，cam position(3)，imu gyro(3)，imu acc(3)

注意，由于imu和cam的存储采用的是同一个函数，所以cam也会存储一些gyro,acc这些数据，但是没用，是多余存储的。



# MY README

## 说明

`gener_alldata.cpp`文件根据一些参数来仿真IMU的数据，并根据这些数据进行中值积分，得到IMU的odom。

在`imu.cpp`中的`testImu`实现中值积分（积分的目的在于根据IMU的测量数据，包括角速度和线加速度，获得不同时刻IMU在world坐标系下的位置，速度，和姿态），效果非常好

## Quick Demo

```bash
cmakefull

cd vio_data_simulation/
python python_tool/draw_trajcory.py
```



原来的效果如下

![](./bin/Figure_2.png)

![](./bin/Figure_1.png)