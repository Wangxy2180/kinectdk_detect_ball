# obj_detected
[toc]
## 2020.10.11新增

- 新增参数`obj_pos_non_move_range`，float型，单位“米”，当前设定值为`0.01m`。在函数`moving_or_stationary`中进行判断只有当xyz任一维度的移动距离超过此值时，才认定球的位置发生了变化，并发送数据。但是当前这个函数好像有很大的问题(ps:给变量起名真是一大难题)
- 新增参数`distance_to_wall`，float型，单位“米”，当前设定值为`3m`。用来设定到墙的距离，如果判定距离超过此值，则认定此次位置坐标无效，不进行发送

## 一些新的修改

- 将时间由ms改为us/1000，也就是说原来是int型的毫秒，现在是小数点后还有数值的毫秒。
- README新增加了**深度处理**和**cameraInfo**的内容。
- callback中增加了一句ROS_INFO，唯一的作用就是查看两次callback之间到底隔了多少，是图像获取慢的问题还是处理慢的问题

## 说明
- 应该是需要创建package，然后把src里的内容和msg，xml，CMakeLists复制过去，然后注释掉CMakeLists.txt里的`add_executable()`，然后`catkin_make`以生成头文件，然后再放开`add_executable`的注释，`catkin_make`。
- 当然也可以调用其他包已经编译好的`ball_pose.msg`所生成的头文件。参考[这里](https://www.cnblogs.com/long5683/p/11450472.html)。
- 程序源文件 `src/obj_detected.cpp`


## 一些信息

- 程序没有输入参数

- 作为订阅者，程序默认接收`qhd(960*540)`图像，订阅topic为`"/kinect2/qhd/image_color_rect"`和`"/kinect2/qhd/image_depth_rect"`

- 作为发布者，发布的topic名为`"/obj_detect/tennis_pose"`

- 如果获取到的深度信息为0,则不会publish

- 可视化时屏幕上的单位，坐标的单位为米，时间的单位为秒

- 发送的信息单位为
    >从图像输出的小球三维坐标
    >float64 x	米
    >float64 y	米
    >float64 z	米
    >float32 T   毫秒
    >int8 count  

## 使用方法

1. 先开启bridge

```
roslaunch kinect2_bridge kinect2_bridge.launch
```

2. 开启obj_detected

   **注意，一定要在你的catkin_workspace目录下运行此条(也就是运行catkin_make的那个目录)，命令，否则就会因为当前路径错误而找不到手眼标定文件。水平有限，凑合先用**

```
rosrun obj_detected obj_detected
```

3. 可以先用下命令进行topic查看

```
rostopic echo /obj_detect/tennis_pose
```
或者使用下列命令将其重定向到文件output.txt中，并同时在屏幕输出
```
rostopic echo /obj_detect/tennis_pose >&1 | tee output.txt
```

## 手眼标定相关
**手眼标定相关文件请放于`/home/yourname/catkin_ws/obj_detected`文件夹下**，如果你的包名不叫`obj_detected`，或者你想修改放置文件的路径，请修改源码中的手眼标定文件路径，直接在源代码中`ctrl + f`搜索`手眼标定文件路径`即可找到。
**注意**

- 每个数字之间采用空格分割，每行之间采用一个回车，即每行3个空格1个回车。

- 当然，如果你多敲几个空格，或者干脆把数据直接写成一行，理论上可能也不会有事，但是，我没试过，以防万一，还是写成4×4。(其实好像也没什么必要，16×1也可以)

- **在第一次运行时请检查终端输出的外参矩阵是否正确**

- 该文件中存放的矩阵为公式(1)中的矩阵A
  $$
  coord_{world} = A*coord_{camera}\tag{1}
  $$
  

## 深度处理

新增深度处理，圆心为中心点，半径×2为变长的矩形内的深度信息进行遍历，并选出最小的，实际实验过程中发现会出现0的情况，没太搞懂为什么，又加了一步判断，代码中直接搜`最小深度`可达，**我将最小深度的赋值注释掉了**(我又给他取消注释了)，两个误差小的时候2～3，大的时候10+。

- `obj_output.txt`为obj_detected程序在控制台输出的内容，`topic_send`是obj_detected发送的topic，b.txt是另一个obj_detected程序在控制台输出的内容。其中`obj_output.txt`和`topic_send`是同步的。

- `obj_output.txt`中detected_frame_count: 44 47 48的位置可以看出他确实有个很好的矫正。
- **但是，确实还存在整个矩形的深度都在背景板上的情况**，比如`b.txt`中的detected_frame_count: 78。但是基本上是小概率事件，有可能使因为我的实验环境导致网球发白，识别面积太小，全都跑偏了。

## cameraInfo

viewer中有一步是readCameraInfo()，这个就是在读你标定的信息，topic所广播的是`D,K,R,P`，分别对应`calib_color.yaml`中的

topic:`/kinect2/hd/camera_info` :`1920*1080`

```
D - distortionCoefficients
K - cameraMatrix
R - rotation
P - projection
```

topic:`/kinect2/hd/camera_info`:`960*540`

```
D - distortionCoefficients
K - cameraMatrix/2
R - rotation
P - projection/2
```

topic:`/kinect2/hd/camera_info`:`512*424`

这个对应`calib_ir.yaml`中的。。。这个怎么不一样？为啥呢

```
D - distortionCoefficients
K - cameraMatrix
R - rotation
P - projection
```

## 编译报错

有可能`Eigen库`编译时会报错，提示找不到头文件，可能是因为路径不对，尝试下列命令，创建一个软连接（类似于快捷方式）。如果没能解决问题，那我也不知道了。

```
sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen
```

我为什么一定要用这个矩阵库，写他三个公式不好吗，坑死我了