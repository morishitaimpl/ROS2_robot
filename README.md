# ROS2_robot_Gazebo

mac bookで作業する場合のubuntu22.04とROS2によるロボット開発

## 環境構築

- 実機の環境であるubuntu22.04とROS2 Humbleをローカルで動かすためのシミュレーション環境の構築手順をまとめます。

## ロボットのシミュレーション環境構築手順（macOS + Apple Silicon想定）

### 1. リポジトリのクローン

- このページのURLをgit cloneしてください。

### 2. Dockerインストール

Docker Desktop をインストールします（リンク: `https://www.docker.com/products/docker-desktop/`）。

#### 2.1 Docker Desktopで「Use Rosetta for x86/amd64 emulation」をOFF（macOS）

- windows環境の場合は不要

1. Docker Desktop を起動
2. 画面右上（メニューバー or アプリ内）の 歯車アイコン（Settings） をクリック
3. 左サイドバーから General を選択
4. Use Rosetta for x86/amd64 emulation on Apple Silicon
   というチェック項目を探す
5. チェックを外す（OFF）
6. 右下の Apply & Restart をクリック
   - Docker Desktop が再起動される

### 3. XQuartz（Gazebo GUI表示用）

```bash
brew install --cask xquartz
```

- XQuartz 起動後：

1. 設定 → Security
2. Allow connections from network clientsにチェック
3. 一度 XQuartz を終了 → 再起動

```bash
xhost +localhost
```

表示できない場合（`No protocol specified` / `Can't open display` 等）は、暫定対応として下記も試してください（セキュリティ的に緩くなるので、検証時のみ推奨）。

```bash
xhost + 127.0.0.1
```

> 補足: Gazebo（`gz`）は**コンテナ内**で動かします。macOS側にGazeboを別途インストールする必要はありません。

### 4. Docker イメージのビルド

- 10分ほどかかるので待機\

```bash
docker build -t ros2-humble-gazebo .
```

```bash
docker build --no-cache -t ros2-humble-gazebo .
```

- キャッシュを使わない場合のコマンド例

### 5. コンテナ起動（Gazebo GUI 対応 / macOS）

```bash
docker run -it --rm \
  --platform linux/arm64 \
  -e DISPLAY=host.docker.internal:0 \
  -e QT_X11_NO_MITSHM=1 \
  -e LIBGL_ALWAYS_INDIRECT=1 \
  -e QT_OPENGL=software \
  -e QT_QUICK_BACKEND=software \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  --name ros2_sim \
  ros2-humble-gazebo
```

> macOSでは `/tmp/.X11-unix` をマウントする方式は基本的に効きません（Linux向け手順）。XQuartzへTCPで描画します。
>
> `Failed to create OpenGL context` が出る場合は、上記のように **Qt/Mesaをソフトウェアレンダリング**に寄せるとGUIが起動しやすいです（速度は落ちます）。

### 6. 動作確認（この順で）

#### 6.1 Gazebo（Ignition Fortress）の確認

```bash
ign gazebo --versions
ign gazebo -v 4 -r /usr/share/ignition/ignition-gazebo6/worlds/empty.sdf
```

#### 6.2 X11 疎通確認（コンテナ内で）

```bash
xeyes
```

#### 6.2.1 OpenGL/GLX 疎通確認（コンテナ内で・GUIクラッシュ時に有効）

GazeboのGUIが `Failed to create OpenGL context` や `Segmentation fault` で落ちる場合、まずGLXが有効か確認します。

```bash
glxinfo -B
glxinfo -B -i
```

`OpenGL renderer string` が表示されずエラーになる場合は、macOS側のXQuartz設定（Indirect GLX）が原因のことが多いです。

#### 6.2.2 macOS（XQuartz）側のIndirect GLXを有効化（必要な場合）

XQuartzを終了した上で、macOS側のターミナルで実行 → XQuartzを再起動してください。

```bash
defaults write org.xquartz.X11 enable_iglx -bool true
```

その後、XQuartz側で `xhost +localhost`（もしくは一時的に `xhost + 127.0.0.1`）を実行してから再度コンテナを起動します。

#### 6.3 ROS2経由でGazebo起動（コンテナ内）

ROS経由:

```bash
source /opt/ros/humble/setup.bash
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="-v 4 -r /usr/share/ignition/ignition-gazebo6/worlds/empty.sdf"
```

### 7. ROS2 × Gazebo 連携確認（例）

```bash
ros2 topic list
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock
```
