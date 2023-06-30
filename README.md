# Common ROS package for HSR

This package is a common package for HSR in SIGVerse.

## dependency

ros-noetic-jsk-rviz-plugins

ros-noetic-rosbridge-suite

ros-noetic-roswww

下記のコマンドで必要な依存関係がすべてインストールできます。

- hsr_rosに移動
```bash
$ roscd hsr_ros
```
- install.shに権限を付与
```bash
$ chmod 755 install.sh
```
- install.shを実行する
```bash
$ sudo ./install.sh
```


以下も忘れずに。
- hsr_rosのsrcに移動
```bash
$ roscd hsr_ros/src
```
- 権限を付与
```bash
$ chmod 755 *
```
