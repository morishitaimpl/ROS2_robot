# ROS2_robot_Gazebo

mac bookで作業する場合のubuntu22.04とROS2によるロボット開発

# 環境構築

- 実機の環境であるubuntu22.04とROS2 Humbleをローカルで動かすためのシミュレーション環境の構築手順をまとめます。

## ロボットのシミュレーション環境構築手順

## 1. リポジトリのクローン

- このページのURLをgit cloneしてください。

# 2. Dockerインストール

https://www.docker.com/products/docker-desktop/

## 2.1 Docker Desktopで「Use Rosetta for x86/amd64 emulation」をOFFにする方法（macOS）

- windows環境の場合は不要

1. Docker Desktop を起動
2. 画面右上（メニューバー or アプリ内）の 歯車アイコン（Settings） をクリック
3. 左サイドバーから General を選択
4. Use Rosetta for x86/amd64 emulation on Apple Silicon
   というチェック項目を探す
5. チェックを外す（OFF）
6. 右下の Apply & Restart をクリック
   - Docker Desktop が再起動される

# 3. XQuartz（Gazebo GUI用）

```bash
brew install --cask xquartz
```

- XQuartz 起動後：

1. 設定 → Security
2. Allow connections from network clientsにチェック
3. 一度 XQuartz を終了 → 再起動

```bash
xhost + 127.0.0.1
```

# 4. Docker イメージのビルド

- 10分ほどかかるので待機\

```bash
docker build -t ros2-humble-gazebo .
```

# 5. コンテナ起動（Gazebo GUI 対応）

```bash
docker run -it --rm \
  --platform linux/arm64 \
  -e DISPLAY=host.docker.internal:0 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --name ros2_sim \
  ros2-humble-gazebo
```

# 6. 動作確認（必ずこの順で）

## 6.1 X11 疎通確認

```bash
xeyes
```

## 6.2 Gazebo 起動

ROS経由:

```bash
ros2 launch ros_gz_sim gz_sim.launch.py
```

# 7. ROS2 × Gazebo 連携確認

```bash
ros2 topic list
ros2 run ros_gz_bridge parameter_bridge
```
