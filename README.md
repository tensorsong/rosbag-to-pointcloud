# <center> Rosbag点云文件生成工具 </center> #
## 1. 工具简介 ##
这是一个用于将 **rosbag** 文件中特定点云消息转成 ***.pcd或者.txt*** 存储格式的点云数据的小工具，目前只支持格式为 ***pcl::PointXYZI 和 pcl::PointXYZRGB*** 的点云。
## 2. 工具依赖的环境与库 ##
工具需要在 **Linux** 环境下编译使用，软件架构和通信机制基于 **ROS**，使用到的开源库为 **PCL**。总结如下：
- **Linux操作系统，推荐Ubuntu16.04/Ubuntu18.04**
- **ROS**
- **PCL**
## 3. 工具的下载与安装 ##
- 从代码仓库下载源代码；
```bash
git clone https://github.com/PySonger/rosbag-to-pointcloud.git
```
- 进入源码目录，使用 catkin_make 编译源代码；
```bash
cd rosbag-to-pointcloud && catkin_make
```
- 播放发送点云数据的 **rosbag** 文件；
```bash
roscore && rosbag play ROSBAG_FILE
```
- 启动源代码内的 **bag_to_pointcloud.launch** 文件，开始生成点云存储文件；
```bash
source devel/setup.bash
roslaunch src/bag_to_pointcloud/bag_to_pointcloud.launch
```
- **rosbag** 播放完毕，键入 **ctrl + C** 关闭生成节点；

## 4. 启动文件说明 ##
工具由 **bag_to_pointcloud.launch** 文件启动，内容如下：
```html
<launch>
    <arg name="input_topic" default="/hesai/pandar"/>
    <arg name="point_type" default="XYZI"/>
    <arg name="save_type" default="txt"/>
    <arg name="save_dir" default="./outputs"/>

    <node pkg="bag_to_pointcloud" name="bag_to_pointcloud" type="bag_to_pointcloud" output="screen">
        <param name="input_topic" type="string" value="$(arg input_topic)"/>
        <param name="point_type" type="string" value="$(arg point_type)"/>
        <param name="save_dir" type="string" value="$(arg save_dir)"/>
        <param name="save_type" type="string" value="$(arg save_type)"/>
    </node>
</launch>
```
文件内的参数说明如下：
- input_topic: 启动节点需要订阅的话题名称；
- point_type: 点云的类型，可以选择为 ***XYZI/XYZRGB***；
- save_dir: 生成文件的存放目录，注意一定要填写为 ***绝对路径*** ；
- save_type: 需要保存的文件格式，可以选择为 **txt或者pcd** 格式；

## 4. 工具的说明事项 ##
对该工具有以下几点需要说明：
1. 目前工具只支持 **pcl::PointXYZI、pcl::PointXYZRGB** 格式的点云数据；
2. 生成的点云存储文件会从 **1.txt/1.pcd** 依次升序存储;
3. 上述的 **save_dir** 一定要事先存在，填写时必须为绝对路径；