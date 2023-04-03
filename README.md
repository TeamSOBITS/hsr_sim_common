# Common ROS package for HSR in SIGVerse

HSRをSIGVerse上で動かすためのリポジトリです．

## Prerequisites

以下の環境で動作します．  
・OS: Ubuntu 20.04  
・ROS distribution: Noetic Ninjemys  

## How to use
まず，以下のコマンドを入力して，HSRを動かすための環境設定を行います．  
この設定は，初回のみに行う作業ですので，1度行ったことのある人は飛ばしてください．

```bash:
$ roscd hsr_ros
$ chmod 755 install.sh
$ sudo ./install.sh
```

以下のコマンドを端末から入力することで，SIGVerse環境と接続することができます．  

```bash:
$ roslaunch hsr_ros sigverse.launch
```

さらに，以下のコマンドを各端末から入力することで，SIGVerse環境上のHSRを擬似的に起動することができます．  
これにより，各センサ情報を扱ったり，制御を行うことができます．

```bash:
$ roslaunch hsr_ros minimal.launch
```