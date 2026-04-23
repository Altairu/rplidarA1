# RPLIDAR A1 + Cartographer 2D SLAM for ROS2 Humble

このプロジェクトは、ROS2 Humble環境において、オドメトリ（車輪のエンコーダー情報など）を使用せず、**RPLIDAR A1 のスキャンデータのみ**で高精度な2D自己位置推定および地図生成（SLAM）を行うためのパッケージです。

SLAMアルゴリズムとしてGoogleの **Cartographer** を採用しており、LiDARを手で持って歩くような環境でも、スキャン形状を自動的にマッチングさせて追従・地図作成を行います。さらに、RANSACを用いた直線（壁）抽出機能も含まれています。

---

## 🛠 環境構築 (Setup)

他のPCで一から環境を構築し、このパッケージを動かすための手順です。

### 1. 前提条件 (Requirements)
- **OS**: Ubuntu 22.04
- **ROS2**: Humble
- **ハードウェア**: RPLIDAR A1 (USB接続)

### 2. 依存パッケージのインストール
まず、SLAMのコアとなるCartographerや、Pythonの解析ライブラリをインストールします。

```bash
# パッケージリストの更新
sudo apt update

# Cartographer パッケージのインストール
sudo apt install ros-humble-cartographer-ros

# RANSAC直線抽出用のPythonライブラリ
pip3 install scikit-learn numpy scipy
```

### 3. ワークスペースの構築とリポジトリのクローン
ROS2のワークスペースを作成し、本リポジトリと、RPLIDARを動かすための公式ドライバー（`sllidar_ros2`）を `src` ディレクトリ内にクローンします。

```bash
# ワークスペースとsrcディレクトリの作成
mkdir -p ~/rplidar_ws/src
cd ~/rplidar_ws/src

# 本リポジトリのクローン (※URLはご自身の公開リポジトリのURLに変更してください)
git clone <YOUR_GITHUB_REPOSITORY_URL>

# Slamtec公式ドライバー (sllidar_ros2) のクローン
git clone https://github.com/Slamtec/sllidar_ros2.git
```

### 4. ハードウェア(USBポート)の権限付与
RPLIDARをPCのUSBポートに接続します。通常 `/dev/ttyUSB0` または `/dev/ttyUSB1` として認識されます。
LiDARのデータを読み書きできるように権限を付与してください。

```bash
# ポート番号が ttyUSB0 の場合
sudo chmod 666 /dev/ttyUSB0

# ポート番号が ttyUSB1 の場合
sudo chmod 666 /dev/ttyUSB1
```
> **注意**: `launch` ファイルではデフォルトで `/dev/ttyUSB1` を参照するように設定されている場合があります。もし `ttyUSB0` に接続されている場合は、Launchファイル内のパラメータを変更するか、一時的にポートをリンクさせてください。

### 5. ビルドとセットアップ
```bash
cd ~/rplidar_ws
# 依存関係の解決 (初回のみ)
rosdep update
rosdep install --from-paths src -y --ignore-src

# ビルド
colcon build --symlink-install

# 設定の反映
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
起動後、自動的に開いたRViz2ウィンドウで以下の設定を行ってください。

1. **Fixed Frame の変更**:
   - `Global Options` > `Fixed Frame` を `laser` 等から **`map`** に変更します。（**必須**: これを行わないと地図が追従しません）
2. **マップ (地図) の表示**:
   - 左下の `Add` ボタンをクリック -> `By topic` タブ -> `/map` の `Map` (OccupancyGrid) を選択。
3. **レーザースキャン (点群) の表示**:
   - `Add` ボタン -> `By topic` タブ -> `/scan` の `LaserScan` を選択。
   - ※見やすくするために、追加されたLaserScanの設定ツリーの中の `Size (m)` を `0.05` などに変更すると良いです。
4. **抽出された直線の表示 (任意)**:
   - `Add` ボタン -> `By topic` タブ -> `/ransac_lines` の `MarkerArray` を選択。

### 3. マッピングのコツ (LiDARオンリーSLAM)
オドメトリ（車輪の回転情報）がないため、Cartographerは壁の形状だけを頼りに現在位置を割り出します。
- **ゆっくり動かす**: 急に振り向いたり、走ったりするとスキャンマッチングが追いつかず、現在地を見失う（地図が破綻する）原因になります。
- **特徴のある場所を歩く**: 何もない長い廊下などでは、LiDARのデータだけでは前後の移動を検知できず「滑り」が発生することがあります。角や家具がある部屋のほうが精度が高くなります。

---

## 📁 主要なファイル構成

- **`launch/rplidar_slam_launch.py`**:
  LiDARドライバー、Cartographer、RANSAC、RViz2をすべて起動する一括起動ファイル。
- **`config/lidar_only_2d.lua`**:
  オドメトリを無効化(`use_odometry = false`)し、LiDARのデータのみで地図を作成・更新するためのCartographer専用チューニング設定。
- **`lidar_processing/ransac_node.py`**:
  `/scan` トピックを受信し、scikit-learnを用いて点群から直線（壁）を抽出する解析ノード。
