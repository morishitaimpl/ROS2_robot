# ROS2_robot_Gazebo

# 環境構築
macOS（Apple Silicon）上で **ROS2 Humble + Gazebo（Ignition Fortress）** のシミュレーション開発を行う手順です。  

## 前提

- **Docker Desktop** をインストール: `https://www.docker.com/products/docker-desktop/`
- **Apple Siliconの場合**: Docker Desktop の設定 `Use Rosetta for x86/amd64 emulation` は **OFF推奨**

## 手順（確実に「4輪モデルが表示」まで）

# 1. Dockerイメージをビルド

```bash
docker build -t ros2-humble-gazebo .
```

# 2. noVNCコンテナを起動

リポジトリを `/work` にマウントします（`ros2_ws/` をコンテナ内でビルドするため）。

```bash
docker run -it --rm \
  --platform linux/arm64 \
  -p 6080:6080 \
  -p 5900:5900 \
  -v "$PWD":/work \
  -e GEOMETRY=1920x1080 \
  --name ros2_sim \
  ros2-humble-gazebo /usr/local/bin/start-vnc.sh
```

起動したらブラウザで **`http://localhost:6080/vnc.html`** を開きます（ここにGazeboのウィンドウが出ます）。

# 3. ワークスペースをビルド（コンテナ内）

上の `docker run` で開いている **コンテナのシェル（`root@...`）** で実行します。

```bash
cd /work/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

# 4. シミュレーター起動（4輪モデルをスポーン）

```bash
ros2 launch raspbot_sim sim.launch.py
```

#### 確認ポイント

- noVNCのGazebo画面が開く
- 右側の **Entity Tree** に **`raspbot`** が出る（4輪モデルがスポーンされている）

# 5. ロボットを動かす（`cmd_vel`）

別のmacターミナルを開いて、同じコンテナに入ります。

```bash
docker exec -it ros2_sim bash
cd /work/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic pub -r 10 /model/raspbot/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.3}}"
```

> `raspbot_sim` はGazeboの DiffDrive システムを使っているため、制御トピックは `/model/raspbot/cmd_vel` です。

## noVNCの基本操作

- **マウス/キーボード**: そのままブラウザ上で操作できます
- **noVNCサイドバー**: 左端のタブ（矢印）から開けます（特殊キー送信 / フルスクリーン等）
- **ウィンドウ切り替え**: `Alt+Tab`

## トラブルシュート

- **`http://localhost:6080/vnc.html` が開けない**
  - `docker ps` で `ros2_sim` が起動しているか確認
- **Gazeboが前面に出ない**
  - `Alt+Tab`、または右クリックメニューからウィンドウを選択
- **`raspbot` が出ない**
  - `ros2 launch raspbot_sim sim.launch.py` のログにエラーがないか確認（特に `ros2 run ros_gz_sim create ...`）
