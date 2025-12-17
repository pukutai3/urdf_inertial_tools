# urdf_inertial_tools

urdf_inertial_tools は、STL メッシュから URDF 用の慣性パラメータ
（質量・重心位置・慣性テンソル）を計算・可視化するための
ROS 2 向けユーティリティパッケージです。

URDF モデル作成時に、形状に基づいた慣性情報を設定・確認することを
目的としています。

---

## 機能

* STL メッシュから慣性パラメータを計算

  * 質量（mass）
  * 重心位置（Center of Mass）
  * 慣性テンソル（ixx, iyy, izz, ixy, ixz, iyz）
* RViz2 による可視化

  * STL モデル表示
  * 重心位置表示
  * 慣性テンソルに基づく慣性楕円体表示
* URDF に記述可能な inertial 情報の取得
* ROS 2 launch ファイルによる実行

※ 本パッケージは URDF 全体（リンク構造・ジョイント）を生成しません。

---

## 対応環境

* Ubuntu 22.04
* ROS 2 Humble Hawksbill
* Python 3
* RViz2

---

## インストールとセットアップ

以下を上から順に実行してください。

```bash
cd ~/ros2_ws/src
git clone https://github.com/pukutai3/urdf_inertial_tools.git

cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## 使い方

### STL モデルの慣性情報を可視化する例

```bash
ros2 launch urdf_inertial_tools view_stl_auto.launch.py \
  stl:=model.stl \
  mass:=3.2 \
  unit:=kg
```

* `stl` : 解析対象の STL ファイル
* `mass` : リンク全体の質量
* `unit` : 質量の単位（例: kg）

---

## 出力される情報

* 計算された質量
* 重心位置（X, Y, Z）
* 慣性テンソル
* RViz2 上での可視化結果

これらの値を URDF の `<inertial>` 要素に記述することで、
物理シミュレーションに利用できます。

---

## 注意事項

* STL の単位系（mm / m など）は結果に影響します。
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
