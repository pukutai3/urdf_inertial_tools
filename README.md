# pukutai3 / urdf_xacro_tuner

GitHub アカウント `pukutai3` で公開している `urdf_xacro_tuner` は、URDF / xacro の link 質量、慣性、joint 設定を編集し、その場で確認するための ROS 2 ツールです。

- `質量/慣性` タブで link ごとの質量を入力し、手元の STL から `<inertial>` を自動計算して元ファイルへ反映できます。
- `ジョイント` タブで joint type、limit、dynamics を編集できます。
- 3D 確認は GUI 内で完結します。RViz は必須ではありません。
- xacro を開いた場合は、展開済み URDF を別生成するのではなく、元の xacro / include を直接編集します。

## 画面構成

- 左: link / joint 一覧
- 中央: 編集欄
- 右: 3D 確認と慣性情報
- 下: 反映ログ

`質量/慣性` タブでは、選択した link を中心に数値編集と 3D 確認を連続して行えます。

## 動作環境

確認環境:

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10

必要なもの:

- `git`
- `python3-pip`
- `python3-setuptools`
- `python3-tk`
- `python3-numpy`
- `python3-vtk9`
- `python3-colcon-common-extensions`
- `python3-rosdep`
- `trimesh`
- ROS 2 の以下のパッケージ
  - `launch`
  - `launch_ros`
  - `robot_state_publisher`
  - `rviz2`
  - `ament_index_python`

`python3-vtk9` が見つからない場合は、`universe` リポジトリを有効にしてから再実行してください。

```bash
sudo add-apt-repository universe
sudo apt update
```

## インストール

### 1. システム依存の導入

```bash
sudo apt update
sudo apt install -y \
  git \
  python3-pip \
  python3-setuptools \
  python3-tk \
  python3-numpy \
  python3-vtk9 \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-humble-launch \
  ros-humble-launch-ros \
  ros-humble-robot-state-publisher \
  ros-humble-rviz2 \
  ros-humble-ament-index-python
```

### 2. Python 依存の導入

```bash
python3 -m pip install --user --upgrade pip
python3 -m pip install --user --upgrade trimesh
```

### 3. ワークスペースへ配置

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/pukutai3/urdf_xacro_tuner.git urdf_xacro_tuner
```

### 4. rosdep で不足依存を補完

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 5. ビルド

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select urdf_xacro_tuner
source install/setup.bash
```

`launch/` と `rviz/` を share 配下へ入れるため、この手順では通常 build を使います。

## 起動方法

### GUI を launch で起動

GUI は ROS 2 launch から起動できます。これが推奨です。

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch urdf_xacro_tuner gui.launch.py \
  urdf:=/path/to/your_robot_description/urdf/your_robot.xacro \
  package_root:=/path/to/your_workspace/src
```

- `urdf` は開きたい URDF / xacro のパスです。
- `package_root` は `package://` の解決に使うワークスペース root です。
- GUI 内の 3D 確認は内蔵ビューで行うため、メインの編集作業に RViz は不要です。

### 直接起動

```bash
ros2 run urdf_xacro_tuner urdf-xacro-tuner-gui --urdf /path/to/your_robot_description/urdf/your_robot.xacro --package-root /path/to/your_workspace/src
```

### CLI で使う

```bash
ros2 run urdf_xacro_tuner urdf-xacro-tuner-cli scan /path/to/model.xacro --package-root /path/to/your_workspace/src --expand-xacro
ros2 run urdf_xacro_tuner urdf-xacro-tuner-cli apply /path/to/model.xacro --package-root /path/to/your_workspace/src --mass base_link=3.2
ros2 run urdf_xacro_tuner urdf-xacro-tuner-cli check /path/to/model.urdf
```

## GUI の使い方

1. `URDF/xacro` に対象ファイルを指定します。
2. `パッケージroot` にワークスペースの `src` を指定します。
3. `再読込` を押します。
4. link 一覧から編集したい link を選びます。
5. `入力質量 kg` に値を入力します。
6. `反映` を押すと、その link の `<inertial>` が元ファイルへ反映されます。
7. `3D確認` を押すと、選択 link の STL、重心、慣性主軸、慣性楕円体をその場で確認できます。

補足:

- `自動反映` を有効にすると、質量入力の変更後に自動で反映されます。
- `反映` 時にはバックアップ `.bak` を作成します。
- `ジョイント` タブでは movable joint の type / limit / dynamics を編集できます。
- mimic / gripper / counterweight 周辺などは、初期状態では編集ロックされることがあります。必要なら許可を有効にしてから反映してください。

## xacro の扱い

このツールは、xacro を開いた場合でも展開済み URDF を別ファイルとして生成しません。

- include / macro の元定義ファイルを直接更新します。
- `dual_arm_robot.xacro` を開いた場合は、呼び出し先の xacro に反映されるように編集します。
- 既存の xacro 構成を壊しにくいように、expanded URDF の一時生成は内部処理に限定しています。

## launch ファイル

- `gui.launch.py`: 編集 GUI を起動します。
- `preview_link.launch.py`: 単一 link のプレビュー用です。
- `view_stl_auto.launch.py`: 任意の STL ファイルと慣性情報の確認用です。`stl:=/path/to/model.stl` を指定してください。

## よくある確認

### GUI が起動しない

```bash
echo $DISPLAY
python3 - <<'PY'
import tkinter as tk
root = tk.Tk()
print('Tk OK')
root.destroy()
PY
```

### `python3-vtk9` が見つからない

```bash
sudo add-apt-repository universe
sudo apt update
sudo apt install -y python3-vtk9
```

### `package://` が解決できない

`package_root` にワークスペースの `src` を指定してください。

## 備考

- GitHub の所有者は `pukutai3`、リポジトリ名、Python module 名、ROS package 名は `urdf_xacro_tuner` で揃えています。
- 既存コマンド互換の entry point も残しています。
- 編集対象の元ファイルは、GUI 上で選択した xacro / URDF そのものです。
