# hokuyo_rsf

Version: ROS1 Noetic

### ビルド

```shell
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/Hokuyo-aut/hokuyo_rsf.git
cd ~/catkin_ws

# GPZDA は hokuyo_rsf/Gpzda としてこのパッケージ内で生成されます。
# nmea_msgs 側に Gpzda が含まれていない環境でも追加の nmea_msgs checkout は不要です。

rosdep install --from-paths src --ignore-src -r -y
catkin_build hokuyo_rsf
source devel/setup.bash
```

### Docker

```shell
cd hokuyo_rsf
# Build Image
docker build --network host -f docker/Dockerfile -t hokuyo_rsf:release .
# Enter the Container
./docker/run.bash -n hokuyo_rsf_release -s /path/to/your/share_folder
# After you exit the container, the endpoint is generated automatically.
# You can use this script when you want to enter the container again.
~/CONTAINER_NAME.bash
```

### 実行

```shell
# Node only
roslaunch hokuyo_rsf hokuyo_rsf.launch

# With Visualization
roslaunch hokuyo_rsf hokuyo_rsf_sample.launch
```

### コマンド送信

```shell
rosrun hokuyo_rsf send_uint8_command 1
```

### 自律走行サンプル

解説：https://sourceforge.net/p/urgnetwork/wiki/rsf_app_info_jp/

ソース：https://github.com/Hokuyo-aut/hokuyo_navigation2
