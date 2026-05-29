# urdf_xacro_tuner

urdf_xacro_tuner は、URDF / xacro の link 質量・慣性情報・joint 設定を
編集し、その場で確認するためのユーティリティパッケージです。

STL メッシュから慣性パラメータを計算し、元の xacro 定義ファイルへ直接反映できます。

---

## 機能

* STL メッシュから慣性パラメータを計算

  * 質量（mass）
  * 重心位置（Center of Mass）
  * 慣性テンソル（ixx, iyy, izz, ixy, ixz, iyz）
* GUI 内蔵 3D ビューによる可視化

  * STL モデル表示
  * 重心位置表示
  * 慣性テンソルに基づく慣性楕円体表示
* URDF / xacro に記述可能な inertial 情報の取得
* joint limit / dynamics の編集
※ 本パッケージは URDF / xacro の調整支援ツールです。STL ファイル自体は変更しません。

---

## 対応環境

* Ubuntu 22.04
* ROS 2 Humble Hawksbill
* Python 3
* python3-vtk9

---

## インストールとセットアップ

以下を上から順に実行してください。

```bash
cd ~/ros2_ws/src
git clone https://github.com/pukutai3/urdf_xacro_tuner.git

cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## 使い方

### URDF の link ごとに質量を指定して inertial を上書きする

ROS を使わずに Python だけで URDF / xacro を読み込み、各 link の STL mesh から
`<inertial>` を計算して URDF に上書きできます。

GUI:

```bash
python3 -m urdf_xacro_tuner.gui
# または
urdf-xacro-tuner
```

互換用に旧コマンド `urdf-inertia-gui` と `python3 -m urdf_inertial_tools.gui` も残しています。

GUI の `反映更新` ボタンは、反映後すぐに再 scan と inertial check を実行し、
更新後の質量と確認結果を同じ画面に表示します。
include / macro 型の xacro では、元の xacro 定義ファイルへ直接反映します。
GUI のlink一覧は `type="fixed"` のjointでつながるlinkをグループ行にまとめます。
グループ行は表示整理用で、質量入力、Set、3D確認は子link行を選択した場合だけ実行します。
`Set` ボタンは選択中linkのmassを確定し、そのlinkだけ反映して GUI 内蔵 3D ビューを更新します。

GUI の `3D確認` タブは、選択中の link の STL だけを表示し、重心、リンク原点軸、
慣性主軸、等価慣性楕円体を重ねて確認できます。RViz は起動しません。
`自動反映` が有効な場合、mass入力の変更後に自動で反映し、同じ3Dビューを更新します。
モデルを読み替えた場合は、以前の選択、preview、auto反映タイマーを破棄します。

CLI:

```bash
python3 -m urdf_xacro_tuner.urdf_mass_inertia scan path/to/model.urdf.xacro --package-root ~/ros2_ws/src
python3 -m urdf_xacro_tuner.urdf_mass_inertia apply path/to/model.urdf.xacro --package-root ~/ros2_ws/src --mass base_link=3.2
python3 -m urdf_xacro_tuner.urdf_mass_inertia check path/to/model.urdf.xacro
```

include / macro を使う xacro は、ROS を使わずに展開した一時 URDF として確認できます。

```bash
python3 -m urdf_xacro_tuner.urdf_mass_inertia scan path/to/model.urdf.xacro --package-root ~/ros2_ws/src --expand-xacro
python3 -m urdf_xacro_tuner.urdf_mass_inertia expand path/to/model.urdf.xacro /tmp/model.expanded.urdf --package-root ~/ros2_ws/src
python3 -m urdf_xacro_tuner.urdf_mass_inertia apply path/to/model.urdf.xacro --package-root ~/ros2_ws/src --expanded-output /tmp/model.expanded.inertial.urdf --mass base_link=3.2
```

* 質量指定単位は URDF の `link` 名です。
* v1 で対応する mesh は STL のみです。
* `collision` mesh を優先し、無い場合は `visual` mesh を使います。
* STL は読込専用です。単位変換や表示用変換はメモリ上で行い、元の STL ファイルは変更しません。
* `package://` は `--package-root` または GUI の package root で解決します。
* `$(find package_name)/...` 形式も `--package-root` から解決します。
* xacro は include / macro の元定義ファイルを直接編集します。
* 上書き前に `model.urdf.bak` を作成します。

## 出力される情報

* 計算された質量
* 重心位置（X, Y, Z）
* 慣性テンソル
* GUI 内蔵 3D ビューでの可視化結果

これらの値を URDF の `<inertial>` 要素に記述することで、
物理シミュレーションに利用できます。

---

## 注意事項

* STL の単位系（mm / m など）は結果に影響します。
* 公開パッケージにはサンプル STL、RViz 設定、旧 launch ファイルは含めません。
* 本ツールは慣性情報の計算・確認を目的とした補助ツールです。

---

## ライセンス

本リポジトリは、同梱の LICENSE ファイルに記載された条件の下で公開されています。

---

## 出典・参考資料

* ROS Wiki – urdf_inertial_tools
  [https://wiki.ros.org/urdf_inertial_tools](https://wiki.ros.org/urdf_inertial_tools)
  要約：STL 形状から URDF 用の慣性パラメータを計算・可視化するツール。

* ROS 2 Documentation – URDF
  [https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html)
  要約：URDF における inertial 要素の役割と記述方法。
