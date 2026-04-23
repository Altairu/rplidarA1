# RPLIDAR A1 + Cartographer 2D SLAM for ROS2 Humble

このプロジェクトは、ROS2 Humble 環境において、タイヤの回転情報（オドメトリ）を使用せず、RPLIDAR A1 のスキャンデータのみで高精度な自己位置推定および地図生成(SLAM)を行うためのパッケージ群です。

SLAMアルゴリズムとしてGoogleが開発した強力な **Cartographer** を採用しており、LiDARを手で持って歩くような環境でも、スキャン形状を自動的にマッチングさせて追従・地図作成を行います。

## 📁 パッケージ構成

- `sllidar_ros2`: Slamtec公式のRPLIDAR ROS2ドライバー。
- `lidar_processing`: 独自パッケージ。
  - `launch/rplidar_slam_launch.py`: LiDARとCartographerを一括起動するファイル。
  - `config/lidar_only_2d.lua`: オドメトリレスで動かすためのCartographer専用チューニング設定。
  - `lidar_processing/ransac_node.py`: 点群から直線を抽出する解析ノード。

---

## 🛠 環境構築 (Setup)

### 1. 前提条件
- **OS**: Ubuntu 22.04
- **ROS2**: Humble
- **ハードウェア**: RPLIDAR A1 (USB接続)

### 2. 必要なライブラリ・パッケージのインストール
ターミナルを開き、以下のコマンドで必要な依存関係をインストールします。

```bash
# Cartographer パッケージのインストール
sudo apt update
sudo apt install ros-humble-cartographer-ros

# RANSAC直線抽出用のPythonライブラリ
pip install scikit-learn numpy scipy
```

### 3. USBポートへの権限付与
RPLIDARはデフォルトで `/dev/ttyUSB1` (または `ttyUSB0`) として認識されます。読み書き権限を与えてください。

```bash
sudo chmod 666 /dev/ttyUSB1
```
*(※接続ポートが `ttyUSB0` の場合は数字を読み替えてください)*

### 4. ワークスペースのビルド
```bash
cd ~/rplidarA1/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## 🚀 動かし方 (Usage)

### 1. システムの一括起動
以下のLaunchコマンドを実行するだけで、LiDARの通信、SLAM(Cartographer)、直線抽出ノード、および可視化ツール(RViz2)がすべて立ち上がります。

```bash
ros2 launch lidar_processing rplidar_slam_launch.py
```

### 2. RViz2 での可視化設定
RViz2のウィンドウが開いたら、左側のパネルで以下の設定を行ってください。

1. **Fixed Frame の変更**:
   - `Global Options` > `Fixed Frame` を `laser` から **`map`** に変更します。（必須）
2. **マップ (地図) の表示**:
   - 左下の `Add` ボタンをクリック -> `By topic` タブ -> `/map` の `Map` (OccupancyGrid) を選択。
3. **レーザースキャン (点群) の表示**:
   - `Add` ボタン -> `By topic` タブ -> `/scan` の `LaserScan` を選択。
   - ※見やすくするために、設定ツリーの中の `Size (m)` を `0.05` などに変更すると良いです。
4. **抽出された直線の表示 (任意)**:
   - `Add` ボタン -> `By topic` タブ -> `/ransac_lines` の `MarkerArray` を選択。

### 3. マッピングのコツ
オドメトリ（車輪の回転情報）がないため、Cartographerは壁の形状だけを頼りに現在位置を割り出します。
そのため、以下の点に注意してLiDARを動かしてください。
- **ゆっくり動かす**: 急に振り向いたり、走ったりするとスキャンが追いつかず見失います。
- **特徴のある場所を歩く**: まっすぐで何もない長い廊下などでは「滑り」が発生することがあります。角や家具がある部屋のほうが精度が高くなります。

---

## 🔧 カスタマイズ

Cartographerの挙動（探索範囲やペナルティ）を調整したい場合は、以下のファイルを編集して再ビルドしてください。
`~/rplidarA1/ros2_ws/src/lidar_processing/config/lidar_only_2d.lua`
