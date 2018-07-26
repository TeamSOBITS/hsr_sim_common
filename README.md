# Common ROS package for HSR

ロボットを動かすのに必要なPKGのインストールスクリプトがPKG内の直下にあります。

下記のコマンドで必要な依存関係がインストールできます。

roscd hsr_ros
chmod 755 install.sh
sudo ./install.sh

#dependency

ros-kinetic-jsk-rviz-plugins

ros-kinetic-rosbridge-suite

ros-kinetic-roswww