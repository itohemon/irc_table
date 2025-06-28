# irc_table
知能ロボコンのステージ向けGazeboシミュレーション環境

![image](https://github.com/user-attachments/assets/6ee09abf-9487-4390-a282-44e01bc1987e)

## メモ
- チャレンジコースのみです。マスターズコースのオプジェクトのspawnはできません
- ボール配置ルールに基づいて生成しているつもりです
- サンプルのロボットとしてturtlebot3が出るようにしています
- ```irc_table.launch.py```に```# for spawn turtlebot3```とあるコードブロックを自分のロボットに差し替えてください
- ```turtlebot3_gazebo```パッケージのインストールが必要だと思います

## 実行環境
- ROS 2(Jazzy)
- Gazebo Harmonic

## 実行方法
```sh-session
$ export TURTLEBOT3_MODEL=burger
$ ros2 launch irc_table irc_table.launch.py 
```
