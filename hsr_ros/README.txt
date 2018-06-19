【Turtlebot_EDUの使い方】
ロボットを動かすのに必要なPKGのインストールスクリプトがPKG内の直下にあります。
下記のコマンドで必要な依存関係がインストールできます。

roscd turtlebot_edu
chmod 755 install.sh
sudo ./install.sh

【変更必須箇所】
src/pose_saver_tf.cpp
39行目のユーザーネームを変更
