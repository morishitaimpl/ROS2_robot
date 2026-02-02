# ROS2_robot_Gazebo

## 環境構築

- 実機の環境であるubuntu22.04とROS2 Humbleをローカルで動かすためのシミュレーション環境の構築手順をまとめます。

## ロボットのシミュレーション環境構築手順（macOS + Apple Silicon想定）

### 1. リポジトリのクローン

- このページのURLをgit cloneしてください。

### 2. Dockerインストール

Docker Desktop をインストールします
`https://www.docker.com/products/docker-desktop/`

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

> 補足: Gazebo（Ignition Fortress）は**コンテナ内**で動かします。macOS側にGazeboを別途インストールする必要はありません。

### 3. Docker イメージのビルド

- 10分ほどかかるので待機\

```bash
docker build -t ros2-humble-gazebo .
```

```bash
docker build --no-cache -t ros2-humble-gazebo .
```

- キャッシュを使わない場合のコマンド例

### 4. コンテナGUIをブラウザで見る（noVNC方式 / 推奨）

XQuartzは使わず、コンテナ内で仮想ディスプレイ（Xvfb）を起動し、noVNCでブラウザ表示します。

```bash
docker build -t ros2-humble-gazebo .
docker run -it --rm \
  --platform linux/arm64 \
  -p 6080:6080 \
  -p 5900:5900 \
  --name ros2_sim \
  ros2-humble-gazebo /usr/local/bin/start-vnc.sh
```

ブラウザで `http://localhost:6080/vnc.html` を開きます（ここにGazeboのウィンドウが出ます）。

その後、上の `docker run ...` で開いている**コンテナのシェル（root@...）**でGazeboを起動します。

```bash
ign gazebo --versions
ign gazebo -v 4 -r /usr/share/ignition/ignition-gazebo6/worlds/empty.sdf
```

noVNC画面（添付のようなUbuntuロゴ/黒背景）は「デスクトップが起動した状態」です。Gazeboを起動するとここにウィンドウが出ます。

- **操作**: マウス/キーボード操作はそのままブラウザ上で可能です
- **noVNCのサイドバー**: 左端のタブ（矢印）から開けます（特殊キー送信 / フルスクリーン等）
- **ウィンドウ操作（Fluxbox）**: 右クリックでメニュー、ウィンドウが前面に出ない時は `Alt+Tab`
- **別コマンドを打ちたい**: mac側の別ターミナルで `docker exec -it ros2_sim bash` を使うと、同じコンテナにもう1つシェルを開けます

#### 4.1 ロボットが最初から出るデモワールド例

`empty.sdf` は空のワールドなので、ロボットは出てきません。まずはロボットが含まれるデモで動作確認します。

```bash
ign gazebo -v 4 -r /usr/share/ignition/ignition-gazebo6/worlds/diff_drive.sdf
```

同梱ワールド一覧:

```bash
ls /usr/share/ignition/ignition-gazebo6/worlds
```

### 5. 動作確認（この順で）

#### 6.1 Gazebo（Ignition Fortress）の確認

```bash
ign gazebo -v 4 -r /usr/share/ignition/ignition-gazebo6/worlds/empty.sdf
```

#### 6.2 X11 疎通確認（コンテナ内で）

```bash
xeyes
```

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
