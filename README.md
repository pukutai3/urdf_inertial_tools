# urdf_xacro_tuner

`urdf_xacro_tuner` は、URDF / xacro の link 質量、慣性情報、joint 設定を編集し、その場で確認するためのGUI/CLIツールです。

主な用途は、STL meshから `<inertial>` を自動計算し、元の URDF / xacro に反映することです。GUIには3D確認ビューを内蔵しており、RVizを起動せずに選択中linkのSTL、重心、慣性主軸、等価慣性楕円体を確認できます。

## Features

- URDF / xacro の読み込み
- xacro include / macro の元定義ファイルへの直接反映
- link単位の質量指定
- STL meshから以下を自動計算
  - mass
  - center of mass
  - inertia tensor
- 選択linkのみの3D確認
  - STL表示
  - link原点軸
  - 重心マーカー
  - 慣性主軸
  - 等価慣性楕円体
- joint limit / dynamics の編集
  - lower / upper
  - effort / velocity
  - damping / friction
- mimic / gripper / counterweight 系jointの編集ロック
- 旧コマンド互換

STLファイルは読込専用です。表示用変換や慣性計算はメモリ上で行い、元のSTLは変更しません。

## Environment

確認環境:

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10

前提:

- ROS 2 Humble がインストール済み
- `source /opt/ros/humble/setup.bash` が実行できる
- GUIを表示できるデスクトップ環境、VNC、X11 forwardingのいずれかがある

このツールが使う主な依存:

- `ament_python`: ROS 2 Python package build
- `colcon`: ROS 2 workspace build
- `tkinter`: GUI
- `VTK`: GUI内蔵3D表示
- `numpy`: 数値計算
- `trimesh`: STL読み込み、体積、重心、慣性計算
- `git`: clone
- `pip`: Python package追加

## Install

現在の公開リポジトリ名は `urdf_inertial_tools` ですが、ROS package名とPython module名は `urdf_xacro_tuner` です。

### 1. System packages

Ubuntu 22.04 / ROS 2 Humble 環境で、先に必要なapt packageを入れます。

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
  python3-ament-package
```

`python3-vtk9` が見つからない場合は、Ubuntuの `universe` repository を有効にしてから再実行してください。

```bash
sudo add-apt-repository universe
sudo apt update
sudo apt install -y python3-vtk9
```

### 2. Python packages

`trimesh` はpipで入れます。

```bash
python3 -m pip install --user --upgrade trimesh
```

必要なら `numpy` もpip側で更新できますが、通常はaptの `python3-numpy` で動作します。

```bash
python3 -m pip install --user --upgrade numpy trimesh
```

### 3. Check dependencies

以下がすべて `OK` になれば、GUIとSTL計算に必要なPython依存は揃っています。

```bash
python3 - <<'PY'
mods = ["tkinter", "vtkmodules", "numpy", "trimesh"]
for mod in mods:
    try:
        m = __import__(mod)
        print(mod, "OK", getattr(m, "__version__", ""))
    except Exception as exc:
        print(mod, "NG", exc)
PY
```

### 4. Build

```bash
cd ~/ros2_ws/src
git clone https://github.com/pukutai3/urdf_inertial_tools.git

cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select urdf_xacro_tuner
source install/setup.bash
```

このREADMEでは、必要な依存をapt/pipで明示しているため `rosdep install` は必須にしていません。

### 5. Build check

```bash
ros2 pkg list | grep urdf_xacro_tuner
ros2 pkg executables urdf_xacro_tuner
```

## GUI Usage

ROS 2環境を読み込んでから起動します。

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run urdf_xacro_tuner urdf-xacro-tuner
```

Python moduleとして直接起動する場合:

```bash
python3 -m urdf_xacro_tuner.gui
```

GUIが起動しない場合は、まずDISPLAYを確認してください。

```bash
echo $DISPLAY
python3 - <<'PY'
import tkinter as tk
root = tk.Tk()
print("Tk OK")
root.destroy()
PY
```

互換用に旧コマンドも残しています。

```bash
ros2 run urdf_xacro_tuner urdf-inertia-gui
python3 -m urdf_inertial_tools.gui
```

## GUI Workflow

1. `URDF/xacro` に対象ファイルを指定します。
2. `パッケージroot` にROS workspaceの `src` を指定します。
   - 例: `/home/ubuntu/ros2_ws/src`
3. `再読込` を押します。
4. link一覧から編集するlinkを選択します。
5. `質量 kg` にlink単体の質量を入力します。
6. `反映` を押すと、そのlinkの `<inertial>` を計算して元ファイルへ反映します。
7. `3D確認` タブでSTL、重心、慣性主軸、慣性楕円体を確認します。

`自動反映` を有効にすると、質量入力の変更後に選択linkへ自動反映し、3D確認も更新します。

## GUI Tabs

### 質量・慣性

linkごとの質量入力と `<inertial>` 反映を行います。

- 質量指定単位はlink名です。
- `collision` meshを優先し、無い場合は `visual` meshを使います。
- `type="fixed"` のjointでつながるlinkはグループ表示します。
- グループ行は表示整理用です。編集は子link行を選択して行います。
- 反映時には `.bak` バックアップを作成します。

### 3D確認

選択中linkのSTLだけをGUI内に表示します。

- メインSTLは不透明表示です。
- 重心は赤い球で表示します。
- link原点軸と慣性主軸を矢印で表示します。
- 等価慣性楕円体は透過ワイヤーフレームで表示します。
- 表示右側にmass、center of mass、inertia tensor、principal inertiaを表示します。
- RViz、robot_state_publisher、launchは使用しません。

### ジョイント

movable jointのlimit/dynamicsを編集します。

- fixed jointは編集対象外です。
- mimic joint、gripper周辺、counterweight周辺は表示しますが、初期状態では編集ロックします。
- ロック対象は `このジョイントの編集を許可` を有効にした場合だけ反映できます。

## CLI Usage

scan:

```bash
python3 -m urdf_xacro_tuner.urdf_mass_inertia scan \
  path/to/model.urdf.xacro \
  --package-root ~/ros2_ws/src \
  --expand-xacro
```

apply:

```bash
python3 -m urdf_xacro_tuner.urdf_mass_inertia apply \
  path/to/model.urdf.xacro \
  --package-root ~/ros2_ws/src \
  --mass base_link=3.2
```

check:

```bash
python3 -m urdf_xacro_tuner.urdf_mass_inertia check path/to/model.urdf
```

CLIコマンドとして実行する場合:

```bash
ros2 run urdf_xacro_tuner urdf-xacro-tuner-cli scan \
  path/to/model.urdf.xacro \
  --package-root ~/ros2_ws/src \
  --expand-xacro
```

## xacro Handling

GUIでxacroを開いた場合、展開URDFを成果物として生成せず、include / macro の元定義ファイルを直接編集します。

例:

- `main.xacro` が `arm.xacro` をinclude
- 対象linkが `arm.xacro` のmacro内で定義されている
- GUIで対象linkへ質量を反映
- 実際に更新されるのは `arm.xacro`

同じmacroから左右など複数インスタンスが生成される場合、元定義を更新するため、同じmacroを使うインスタンスにも同じ慣性定義が反映されます。

## Mesh Support

v1で対応するmeshはSTLのみです。

対応URI:

- relative path
- absolute path
- `package://package_name/...`
- `$(find package_name)/...`

`package://` と `$(find ...)` は、GUIの `パッケージroot` またはCLIの `--package-root` で解決します。

## Write Behavior

- URDF / xacro は上書き更新します。
- 更新前に `.bak` を作成します。
- STLは上書きしません。
- GUIの3D確認では一時expanded URDFを生成しません。
- CLIの `expand` と `preview-link` は明示実行した場合だけ出力ファイルを生成します。

## Limitations

- mesh単位系はURDF / xacro側の記述に従います。STL自体に単位情報はありません。
- STLがwatertightでない場合、体積と慣性計算に失敗することがあります。
- 複数meshを持つlinkでは、mesh体積比で指定質量を分配して慣性を合成します。
- xacro式は本ツール内の簡易評価で処理します。複雑なxacro構文は対応できない場合があります。
- 本ツールはURDF / xacroの調整支援ツールです。動力学シミュレーション上の妥当性は別途確認してください。

## Troubleshooting

### `ModuleNotFoundError: No module named 'trimesh'`

```bash
python3 -m pip install --user --upgrade trimesh
```

### `ModuleNotFoundError: No module named 'vtkmodules'`

```bash
sudo apt update
sudo apt install -y python3-vtk9
```

Ubuntuで `python3-vtk9` が見つからない場合:

```bash
sudo add-apt-repository universe
sudo apt update
sudo apt install -y python3-vtk9
```

### `_tkinter.TclError` or GUI window does not open

GUIを表示できる環境が必要です。ローカルUbuntuデスクトップ、VNC、X11 forwardingのいずれかを使ってください。

```bash
echo $DISPLAY
sudo apt install -y python3-tk
```

### `ros2 run` cannot find package

workspaceをbuildして、setupを読み込んでください。

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select urdf_xacro_tuner
source install/setup.bash
ros2 pkg list | grep urdf_xacro_tuner
```

### `package://...` mesh is not found

GUIの `パッケージroot` またはCLIの `--package-root` に、ROS workspaceの `src` を指定してください。

```bash
--package-root ~/ros2_ws/src
```

## Public Package Notes

公開パッケージには以下を含めません。

- サンプルSTL
- RViz設定
- 旧RViz launch
- ROS marker preview node

## License

MIT License
