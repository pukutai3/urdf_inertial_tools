#!/usr/bin/env python3
"""Simple Tk GUI for editing URDF inertial values and previewing one link."""

from __future__ import annotations

import os
import signal
import shutil
import subprocess
import tempfile
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import numpy as np

from urdf_xacro_tuner.urdf_mass_inertia import (
    apply_joint_properties_to_urdf,
    apply_joint_properties_to_xacro_sources,
    apply_masses_to_xacro_sources,
    apply_masses_to_urdf,
    calculate_link_inertial,
    expand_xacro_to_tree,
    extract_mesh_refs,
    find_direct_link,
    find_existing_mass,
    fixed_joint_groups,
    format_float,
    guess_package_root,
    load_transformed_stl,
    looks_like_xacro,
    parse_urdf,
    scan_urdf_joints,
    scan_xacro_source_joints,
    scan_xacro_source_links,
    scan_urdf,
    validate_urdf_inertials,
    validate_xacro_inertials,
    write_expanded_xacro,
    write_single_link_preview_urdf,
)


PREVIEW_LAUNCH_PATTERN = "ros2 launch urdf_xacro_tuner preview_link.launch.py"

try:
    from vtkmodules import vtkInteractionStyle, vtkRenderingOpenGL2  # noqa: F401
    from vtkmodules.tk.vtkTkRenderWindowInteractor import vtkTkRenderWindowInteractor
    from vtkmodules.vtkCommonCore import vtkIdList, vtkPoints
    from vtkmodules.vtkCommonDataModel import vtkCellArray, vtkPolyData
    from vtkmodules.vtkCommonMath import vtkMatrix4x4
    from vtkmodules.vtkCommonTransforms import vtkTransform
    from vtkmodules.vtkFiltersGeneral import vtkTransformPolyDataFilter
    from vtkmodules.vtkFiltersSources import vtkArrowSource, vtkSphereSource
    from vtkmodules.vtkRenderingCore import vtkActor, vtkPolyDataMapper, vtkRenderer

    VTK_AVAILABLE = True
    VTK_IMPORT_ERROR: Exception | None = None
except Exception as exc:  # noqa: BLE001 - GUI shows the reason if preview is unavailable
    VTK_AVAILABLE = False
    VTK_IMPORT_ERROR = exc


def cleanup_stale_preview_launches() -> int:
    try:
        result = subprocess.run(
            ["pgrep", "-f", PREVIEW_LAUNCH_PATTERN],
            check=False,
            capture_output=True,
            text=True,
        )
    except OSError:
        return 0
    stopped = 0
    for line in result.stdout.splitlines():
        try:
            pid = int(line.strip())
        except ValueError:
            continue
        if pid == os.getpid():
            continue
        try:
            os.killpg(os.getpgid(pid), signal.SIGTERM)
        except ProcessLookupError:
            continue
        except PermissionError:
            continue
        stopped += 1
    return stopped


class InertiaEditor(tk.Tk):
    def __init__(self) -> None:
        stale_preview_count = cleanup_stale_preview_launches()
        super().__init__()
        self.title("URDF/xacro Tuner")
        self.geometry("1040x720")
        self.urdf_path: Path | None = None
        self.mass_values: dict[str, str] = {}
        self.last_applied_path: Path | None = None
        self.rviz_process: subprocess.Popen[str] | None = None
        self.active_preview_link: str | None = None
        self.active_preview_urdf: Path | None = None
        self.auto_apply_after_id: str | None = None
        self.loading_selection = False
        self.auto_backup_paths: set[Path] = set()
        self.tree_link_by_item: dict[str, str] = {}
        self.tree_item_by_link: dict[str, str] = {}
        self.tree_display_by_item: dict[str, str] = {}
        self.tree_group_items: set[str] = set()
        self.joint_by_item: dict[str, str] = {}
        self.joint_item_by_name: dict[str, str] = {}
        self.joint_group_items: set[str] = set()
        self.joint_requires_confirmation: dict[str, bool] = {}
        self.preview_tab: ttk.Frame | None = None
        self.vtk_widget: object | None = None
        self.vtk_renderer: object | None = None
        self.preview_info: tk.Text | None = None
        self._shutting_down = False
        self._signal_pipe: tuple[int, int] | None = None
        self._signal_poll_after_id: str | None = None
        self._build_widgets()
        self.protocol("WM_DELETE_WINDOW", self.request_close)
        self.install_signal_handlers()
        if stale_preview_count:
            self.log_line(f"残留プレビューを停止: {stale_preview_count}件")

    def _build_widgets(self) -> None:
        root = ttk.Frame(self, padding=10)
        root.pack(fill=tk.BOTH, expand=True)

        file_row = ttk.Frame(root)
        file_row.pack(fill=tk.X)
        ttk.Label(file_row, text="URDF/xacro").pack(side=tk.LEFT)
        self.urdf_var = tk.StringVar()
        ttk.Entry(file_row, textvariable=self.urdf_var).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=6)
        ttk.Button(file_row, text="開く", command=self.open_urdf).pack(side=tk.LEFT)
        ttk.Button(file_row, text="再読込", command=self.scan).pack(side=tk.LEFT, padx=(6, 0))

        package_row = ttk.Frame(root)
        package_row.pack(fill=tk.X, pady=(8, 0))
        ttk.Label(package_row, text="パッケージroot").pack(side=tk.LEFT)
        self.package_root_var = tk.StringVar()
        ttk.Entry(package_row, textvariable=self.package_root_var).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=6
        )
        ttk.Button(package_row, text="参照", command=self.browse_package_root).pack(side=tk.LEFT)

        self.notebook = ttk.Notebook(root)
        self.notebook.pack(fill=tk.BOTH, expand=True, pady=10)

        inertia_tab = ttk.Frame(self.notebook)
        joint_tab = ttk.Frame(self.notebook)
        preview_tab = ttk.Frame(self.notebook)
        self.preview_tab = preview_tab
        self.notebook.add(inertia_tab, text="質量・慣性")
        self.notebook.add(joint_tab, text="ジョイント")
        self.notebook.add(preview_tab, text="3D確認")

        middle = ttk.PanedWindow(inertia_tab, orient=tk.HORIZONTAL)
        middle.pack(fill=tk.BOTH, expand=True)

        table_frame = ttk.Frame(middle)
        middle.add(table_frame, weight=4)
        columns = ("mass", "existing", "meshes", "status")
        self.tree = ttk.Treeview(table_frame, columns=columns, show="tree headings", selectmode="browse")
        self.tree.heading("#0", text="リンク")
        self.tree.heading("mass", text="入力質量 kg")
        self.tree.heading("existing", text="現在値 kg")
        self.tree.heading("meshes", text="mesh数")
        self.tree.heading("status", text="状態")
        self.tree.column("#0", width=260, stretch=True)
        self.tree.column("mass", width=110, anchor=tk.E)
        self.tree.column("existing", width=110, anchor=tk.E)
        self.tree.column("meshes", width=80, anchor=tk.E)
        self.tree.column("status", width=260, stretch=True)
        yscroll = ttk.Scrollbar(table_frame, orient=tk.VERTICAL, command=self.tree.yview)
        self.tree.configure(yscrollcommand=yscroll.set)
        self.tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        yscroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.tree.bind("<<TreeviewSelect>>", self.on_select)

        edit_frame = ttk.Frame(middle, padding=(10, 0, 0, 0))
        middle.add(edit_frame, weight=1)
        ttk.Label(edit_frame, text="選択リンク").pack(anchor=tk.W)
        self.selected_var = tk.StringVar()
        ttk.Entry(edit_frame, textvariable=self.selected_var, state="readonly").pack(fill=tk.X, pady=(2, 10))
        ttk.Label(edit_frame, text="質量 kg").pack(anchor=tk.W)
        self.mass_var = tk.StringVar()
        mass_entry = ttk.Entry(edit_frame, textvariable=self.mass_var)
        mass_entry.pack(fill=tk.X, pady=(2, 8))
        mass_entry.bind("<Return>", lambda _event: self.set_mass())
        self.mass_var.trace_add("write", self.on_mass_changed)
        ttk.Button(edit_frame, text="反映", command=self.set_mass).pack(fill=tk.X)
        ttk.Button(edit_frame, text="現在値を使用", command=self.use_current_mass).pack(fill=tk.X, pady=(6, 0))
        ttk.Button(edit_frame, text="入力クリア", command=self.clear_mass).pack(fill=tk.X, pady=(6, 0))
        self.auto_apply_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(edit_frame, text="自動反映", variable=self.auto_apply_var).pack(
            anchor=tk.W, pady=(8, 0)
        )
        ttk.Separator(edit_frame).pack(fill=tk.X, pady=14)
        ttk.Button(edit_frame, text="一括反映", command=self.apply_update).pack(fill=tk.X)
        ttk.Button(edit_frame, text="3D確認", command=self.preview_selected_link).pack(fill=tk.X, pady=(6, 0))
        self.status_var = tk.StringVar(value="")
        ttk.Label(edit_frame, textvariable=self.status_var, wraplength=220).pack(fill=tk.X, pady=(10, 0))

        self.build_joint_tab(joint_tab)
        self.build_preview_tab(preview_tab)

        self.log = tk.Text(root, height=10, wrap=tk.WORD)
        self.log.pack(fill=tk.BOTH)

    def build_preview_tab(self, parent: ttk.Frame) -> None:
        controls = ttk.Frame(parent)
        controls.pack(fill=tk.X, pady=(0, 8))
        ttk.Button(controls, text="選択リンクを表示", command=self.preview_selected_link).pack(side=tk.LEFT)
        ttk.Button(controls, text="表示クリア", command=lambda: self.clear_preview_scene("表示をクリアしました")).pack(
            side=tk.LEFT,
            padx=(6, 0),
        )
        self.preview_status_var = tk.StringVar(value="リンクを選択してください。")
        ttk.Label(controls, textvariable=self.preview_status_var).pack(side=tk.LEFT, padx=(10, 0), fill=tk.X)

        middle = ttk.PanedWindow(parent, orient=tk.HORIZONTAL)
        middle.pack(fill=tk.BOTH, expand=True)

        viewer_frame = ttk.Frame(middle)
        middle.add(viewer_frame, weight=4)
        info_frame = ttk.Frame(middle, padding=(10, 0, 0, 0))
        middle.add(info_frame, weight=1)

        self.preview_info = tk.Text(info_frame, width=42, height=18, wrap=tk.NONE)
        self.preview_info.pack(fill=tk.BOTH, expand=True)
        self.preview_info.configure(state=tk.DISABLED)

        if not VTK_AVAILABLE:
            reason = f"VTKを読み込めません: {VTK_IMPORT_ERROR}"
            ttk.Label(viewer_frame, text=reason, wraplength=560).pack(fill=tk.BOTH, expand=True)
            self.preview_status_var.set(reason)
            return

        self.vtk_renderer = vtkRenderer()
        self.vtk_renderer.SetBackground(0.07, 0.08, 0.09)
        self.vtk_widget = vtkTkRenderWindowInteractor(viewer_frame, width=720, height=500)
        self.vtk_widget.pack(fill=tk.BOTH, expand=True)
        render_window = self.vtk_widget.GetRenderWindow()
        render_window.AddRenderer(self.vtk_renderer)
        render_window.SetMultiSamples(4)
        self.vtk_widget.Initialize()
        self.vtk_widget.Start()
        self.clear_preview_scene("リンクを選択してください。")

    def set_preview_info(self, text: str) -> None:
        if self.preview_info is None:
            return
        self.preview_info.configure(state=tk.NORMAL)
        self.preview_info.delete("1.0", tk.END)
        self.preview_info.insert(tk.END, text)
        self.preview_info.configure(state=tk.DISABLED)

    def clear_preview_scene(self, message: str = "") -> None:
        if hasattr(self, "preview_status_var"):
            self.preview_status_var.set(message)
        self.set_preview_info("")
        if self.vtk_renderer is None:
            return
        self.vtk_renderer.RemoveAllViewProps()
        if self.vtk_widget is not None:
            self.vtk_widget.GetRenderWindow().Render()

    def mesh_to_polydata(self, mesh: object) -> object:
        points = vtkPoints()
        for vertex in np.asarray(mesh.vertices, dtype=float):
            points.InsertNextPoint(float(vertex[0]), float(vertex[1]), float(vertex[2]))

        polys = vtkCellArray()
        for face in np.asarray(mesh.faces, dtype=np.int64):
            ids = vtkIdList()
            ids.SetNumberOfIds(int(len(face)))
            for index, point_id in enumerate(face):
                ids.SetId(index, int(point_id))
            polys.InsertNextCell(ids)

        polydata = vtkPolyData()
        polydata.SetPoints(points)
        polydata.SetPolys(polys)
        return polydata

    def add_mesh_actor(self, mesh: object) -> None:
        mapper = vtkPolyDataMapper()
        mapper.SetInputData(self.mesh_to_polydata(mesh))
        actor = vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(0.72, 0.78, 0.84)
        actor.GetProperty().SetOpacity(1.0)
        self.vtk_renderer.AddActor(actor)

    def add_sphere_actor(self, center: np.ndarray, radius: float, color: tuple[float, float, float]) -> None:
        sphere = vtkSphereSource()
        sphere.SetCenter(float(center[0]), float(center[1]), float(center[2]))
        sphere.SetRadius(float(radius))
        sphere.SetPhiResolution(24)
        sphere.SetThetaResolution(24)
        mapper = vtkPolyDataMapper()
        mapper.SetInputConnection(sphere.GetOutputPort())
        actor = vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(*color)
        self.vtk_renderer.AddActor(actor)

    def vector_basis_from_x(self, direction: np.ndarray) -> np.ndarray:
        x_axis = np.array(direction, dtype=float)
        norm = float(np.linalg.norm(x_axis))
        if norm <= 1.0e-12:
            x_axis = np.array([1.0, 0.0, 0.0])
        else:
            x_axis /= norm
        helper = np.array([0.0, 0.0, 1.0])
        if abs(float(np.dot(x_axis, helper))) > 0.9:
            helper = np.array([0.0, 1.0, 0.0])
        z_axis = np.cross(x_axis, helper)
        z_axis /= float(np.linalg.norm(z_axis))
        y_axis = np.cross(z_axis, x_axis)
        return np.column_stack((x_axis, y_axis, z_axis))

    def add_arrow_actor(
        self,
        start: np.ndarray,
        direction: np.ndarray,
        length: float,
        color: tuple[float, float, float],
        thickness: float = 0.08,
    ) -> None:
        basis = self.vector_basis_from_x(direction)
        matrix = vtkMatrix4x4()
        for row in range(3):
            matrix.SetElement(row, 0, float(basis[row, 0] * length))
            matrix.SetElement(row, 1, float(basis[row, 1] * length * thickness))
            matrix.SetElement(row, 2, float(basis[row, 2] * length * thickness))
            matrix.SetElement(row, 3, float(start[row]))
        matrix.SetElement(3, 0, 0.0)
        matrix.SetElement(3, 1, 0.0)
        matrix.SetElement(3, 2, 0.0)
        matrix.SetElement(3, 3, 1.0)

        source = vtkArrowSource()
        source.SetShaftRadius(0.04)
        source.SetTipRadius(0.12)
        source.SetTipLength(0.32)
        transform = vtkTransform()
        transform.SetMatrix(matrix)
        transform_filter = vtkTransformPolyDataFilter()
        transform_filter.SetTransform(transform)
        transform_filter.SetInputConnection(source.GetOutputPort())
        mapper = vtkPolyDataMapper()
        mapper.SetInputConnection(transform_filter.GetOutputPort())
        actor = vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(*color)
        self.vtk_renderer.AddActor(actor)

    def add_inertia_ellipsoid(self, center: np.ndarray, mass: float, inertia: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        moments, axes = np.linalg.eigh(inertia)
        ix, iy, iz = [float(value) for value in moments]
        radius_sq = np.array(
            [
                5.0 * (-ix + iy + iz) / (2.0 * mass),
                5.0 * (ix - iy + iz) / (2.0 * mass),
                5.0 * (ix + iy - iz) / (2.0 * mass),
            ],
            dtype=float,
        )
        radius_sq = np.maximum(radius_sq, 1.0e-12)
        radii = np.sqrt(radius_sq)

        matrix = vtkMatrix4x4()
        for row in range(3):
            for column in range(3):
                matrix.SetElement(row, column, float(axes[row, column] * radii[column]))
            matrix.SetElement(row, 3, float(center[row]))
        matrix.SetElement(3, 0, 0.0)
        matrix.SetElement(3, 1, 0.0)
        matrix.SetElement(3, 2, 0.0)
        matrix.SetElement(3, 3, 1.0)

        sphere = vtkSphereSource()
        sphere.SetRadius(1.0)
        sphere.SetPhiResolution(32)
        sphere.SetThetaResolution(32)
        transform = vtkTransform()
        transform.SetMatrix(matrix)
        transform_filter = vtkTransformPolyDataFilter()
        transform_filter.SetTransform(transform)
        transform_filter.SetInputConnection(sphere.GetOutputPort())
        mapper = vtkPolyDataMapper()
        mapper.SetInputConnection(transform_filter.GetOutputPort())
        actor = vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(1.0, 0.78, 0.22)
        actor.GetProperty().SetOpacity(0.42)
        actor.GetProperty().SetRepresentationToWireframe()
        self.vtk_renderer.AddActor(actor)
        return moments, axes

    def preview_mass_for_link(self, link: str, link_element: object) -> tuple[float | None, str]:
        value = self.mass_values.get(link, "").strip()
        if value:
            return float(value), "入力質量"
        existing = find_existing_mass(link_element)
        if existing is not None and existing > 0.0:
            return existing, "現在値"
        return None, "未設定"

    def render_link_preview(self, link: str) -> None:
        if self.urdf_path is None:
            raise ValueError("URDF/xacroファイルを選択してください。")
        if not VTK_AVAILABLE:
            raise RuntimeError(f"VTKを読み込めません: {VTK_IMPORT_ERROR}")
        if self.vtk_renderer is None:
            raise RuntimeError("3Dビューが初期化されていません。")

        package_roots = self.package_roots()
        tree = expand_xacro_to_tree(self.urdf_path, package_roots) if self.expanded_mode() else parse_urdf(self.urdf_path)
        link_element = find_direct_link(tree, link)
        if link_element is None:
            raise ValueError(f"リンクが見つかりません: {link}")

        refs = extract_mesh_refs(link_element, self.urdf_path, package_roots)
        if not refs:
            raise ValueError(f"STL meshがありません: {link}")
        meshes = [load_transformed_stl(ref) for ref in refs]

        self.clear_preview_scene("")
        for mesh in meshes:
            self.add_mesh_actor(mesh)

        all_vertices = np.concatenate([np.asarray(mesh.vertices, dtype=float) for mesh in meshes], axis=0)
        bounds_min = all_vertices.min(axis=0)
        bounds_max = all_vertices.max(axis=0)
        diagonal = float(np.linalg.norm(bounds_max - bounds_min))
        if diagonal <= 1.0e-9:
            diagonal = 1.0
        axis_length = diagonal * 0.28
        marker_radius = diagonal * 0.025
        for direction, color in (
            (np.array([1.0, 0.0, 0.0]), (0.9, 0.2, 0.2)),
            (np.array([0.0, 1.0, 0.0]), (0.2, 0.8, 0.3)),
            (np.array([0.0, 0.0, 1.0]), (0.25, 0.45, 1.0)),
        ):
            self.add_arrow_actor(np.zeros(3), direction, axis_length, color, thickness=0.06)

        mass, mass_source = self.preview_mass_for_link(link, link_element)
        result = None
        inertia_error = ""
        if mass is not None:
            try:
                result = calculate_link_inertial(link_element, self.urdf_path, mass, package_roots)
            except Exception as exc:  # noqa: BLE001 - mesh display still helps inspection
                inertia_error = str(exc)

        info_lines = [
            f"リンク: {link}",
            f"mesh数: {len(meshes)}",
            "STL処理: 読込専用（元ファイルは変更しません）",
            "STL:",
            *[f"  {ref.filename}" for ref in refs],
            "",
            f"質量: {format_float(mass)} kg ({mass_source})" if mass is not None else "質量: 未設定",
        ]
        if result is not None:
            self.add_sphere_actor(result.center, marker_radius, (1.0, 0.15, 0.15))
            moments, axes = self.add_inertia_ellipsoid(result.center, result.mass, result.inertia)
            principal_axis_length = max(axis_length, diagonal * 0.18)
            for index, color in enumerate(((1.0, 0.25, 0.25), (0.25, 1.0, 0.35), (0.35, 0.55, 1.0))):
                self.add_arrow_actor(result.center, axes[:, index], principal_axis_length, color, thickness=0.045)
            inertia = result.inertia
            info_lines.extend(
                [
                    f"重心: {' '.join(format_float(float(v)) for v in result.center)}",
                    "慣性:",
                    f"  ixx {format_float(float(inertia[0, 0]))}",
                    f"  ixy {format_float(float(inertia[0, 1]))}",
                    f"  ixz {format_float(float(inertia[0, 2]))}",
                    f"  iyy {format_float(float(inertia[1, 1]))}",
                    f"  iyz {format_float(float(inertia[1, 2]))}",
                    f"  izz {format_float(float(inertia[2, 2]))}",
                    "主慣性:",
                    f"  {' '.join(format_float(float(v)) for v in moments)}",
                ]
            )
        elif inertia_error:
            info_lines.extend(["", f"慣性計算不可: {inertia_error}"])

        self.set_preview_info("\n".join(info_lines))
        self.vtk_renderer.ResetCamera()
        self.vtk_widget.GetRenderWindow().Render()
        self.active_preview_link = link
        self.preview_status_var.set(f"3D確認: {link}")

    def build_joint_tab(self, parent: ttk.Frame) -> None:
        middle = ttk.PanedWindow(parent, orient=tk.HORIZONTAL)
        middle.pack(fill=tk.BOTH, expand=True)

        table_frame = ttk.Frame(middle)
        middle.add(table_frame, weight=4)
        columns = ("type", "lower", "upper", "effort", "velocity", "damping", "friction", "status")
        self.joint_tree = ttk.Treeview(table_frame, columns=columns, show="tree headings", selectmode="browse")
        self.joint_tree.heading("#0", text="ジョイント")
        for column in columns:
            self.joint_tree.heading(column, text=column)
        self.joint_tree.heading("type", text="種類")
        self.joint_tree.heading("lower", text="下限")
        self.joint_tree.heading("upper", text="上限")
        self.joint_tree.heading("effort", text="トルク/力")
        self.joint_tree.heading("velocity", text="速度")
        self.joint_tree.heading("damping", text="減衰")
        self.joint_tree.heading("friction", text="摩擦")
        self.joint_tree.heading("status", text="状態")
        self.joint_tree.column("#0", width=280, stretch=True)
        for column in ("type", "lower", "upper", "effort", "velocity", "damping", "friction"):
            self.joint_tree.column(column, width=90, anchor=tk.E if column != "type" else tk.W)
        self.joint_tree.column("status", width=260, stretch=True)
        yscroll = ttk.Scrollbar(table_frame, orient=tk.VERTICAL, command=self.joint_tree.yview)
        self.joint_tree.configure(yscrollcommand=yscroll.set)
        self.joint_tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        yscroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.joint_tree.bind("<<TreeviewSelect>>", self.on_joint_select)

        edit_frame = ttk.Frame(middle, padding=(10, 0, 0, 0))
        middle.add(edit_frame, weight=1)
        ttk.Label(edit_frame, text="選択ジョイント").pack(anchor=tk.W)
        self.selected_joint_var = tk.StringVar()
        ttk.Entry(edit_frame, textvariable=self.selected_joint_var, state="readonly").pack(fill=tk.X, pady=(2, 10))
        self.allow_joint_edit_var = tk.BooleanVar(value=False)
        self.allow_joint_edit_check = ttk.Checkbutton(
            edit_frame,
            text="このジョイントの編集を許可",
            variable=self.allow_joint_edit_var,
            command=self.update_joint_edit_state,
        )
        self.allow_joint_edit_check.pack(anchor=tk.W, pady=(0, 10))
        self.joint_edit_widgets: list[tk.Widget] = []
        self.joint_field_vars: dict[str, tk.StringVar] = {}
        for label, key in (
            ("下限 rad/m", "lower"),
            ("上限 rad/m", "upper"),
            ("トルク/力", "effort"),
            ("速度", "velocity"),
            ("減衰", "damping"),
            ("摩擦", "friction"),
        ):
            ttk.Label(edit_frame, text=label).pack(anchor=tk.W)
            var = tk.StringVar()
            self.joint_field_vars[key] = var
            entry = ttk.Entry(edit_frame, textvariable=var)
            entry.pack(fill=tk.X, pady=(2, 6))
            entry.bind("<Return>", lambda _event: self.set_joint_properties())
            self.joint_edit_widgets.append(entry)
        self.joint_set_button = ttk.Button(edit_frame, text="ジョイント反映", command=self.set_joint_properties)
        self.joint_set_button.pack(fill=tk.X, pady=(4, 0))
        self.joint_edit_widgets.append(self.joint_set_button)
        self.joint_clear_button = ttk.Button(edit_frame, text="入力クリア", command=self.clear_joint_fields)
        self.joint_clear_button.pack(fill=tk.X, pady=(6, 0))
        self.joint_edit_widgets.append(self.joint_clear_button)
        self.joint_status_var = tk.StringVar(value="")
        ttk.Label(edit_frame, textvariable=self.joint_status_var, wraplength=220).pack(fill=tk.X, pady=(10, 0))
        self.update_joint_edit_state()

    def log_line(self, message: str) -> None:
        self.log.insert(tk.END, message + "\n")
        self.log.see(tk.END)

    def package_roots(self) -> list[Path]:
        value = self.package_root_var.get().strip()
        return [Path(value).expanduser().resolve()] if value else []

    def open_urdf(self) -> None:
        path = filedialog.askopenfilename(
            title="URDF/xacroを開く",
            filetypes=[("URDF/xacroファイル", "*.urdf *.xacro *.xml"), ("すべてのファイル", "*.*")],
        )
        if not path:
            return
        new_path = Path(path).expanduser().resolve()
        if self.urdf_path != new_path:
            self.reset_model_state(stop_preview=True)
        self.urdf_path = new_path
        self.urdf_var.set(str(self.urdf_path))
        guessed = guess_package_root(self.urdf_path)
        if guessed is not None:
            self.package_root_var.set(str(guessed))
        self.scan()

    def browse_package_root(self) -> None:
        path = filedialog.askdirectory(title="ROSワークスペースのsrcを選択")
        if path:
            self.package_root_var.set(path)

    def reset_model_state(self, stop_preview: bool = True) -> None:
        if self.auto_apply_after_id is not None:
            self.after_cancel(self.auto_apply_after_id)
            self.auto_apply_after_id = None
        if stop_preview:
            self.stop_rviz_preview()
        self.tree.delete(*self.tree.get_children())
        self.mass_values.clear()
        self.tree_link_by_item.clear()
        self.tree_item_by_link.clear()
        self.tree_display_by_item.clear()
        self.tree_group_items.clear()
        if hasattr(self, "joint_tree"):
            self.joint_tree.delete(*self.joint_tree.get_children())
        self.joint_by_item.clear()
        self.joint_item_by_name.clear()
        self.joint_group_items.clear()
        self.joint_requires_confirmation.clear()
        self.last_applied_path = None
        self.active_preview_link = None
        self.active_preview_urdf = None
        self.auto_backup_paths.clear()
        self.selected_var.set("")
        self.loading_selection = True
        self.mass_var.set("")
        self.loading_selection = False
        self.status_var.set("")
        if hasattr(self, "selected_joint_var"):
            self.selected_joint_var.set("")
        if hasattr(self, "joint_field_vars"):
            for var in self.joint_field_vars.values():
                var.set("")
        if hasattr(self, "joint_status_var"):
            self.joint_status_var.set("")
        if hasattr(self, "allow_joint_edit_var"):
            self.allow_joint_edit_var.set(False)
            self.update_joint_edit_state()
        if hasattr(self, "preview_status_var"):
            self.clear_preview_scene("モデル未読込")

    def stop_rviz_preview(self) -> None:
        if self.rviz_process is not None and self.rviz_process.poll() is None:
            try:
                os.killpg(os.getpgid(self.rviz_process.pid), signal.SIGTERM)
                self.rviz_process.wait(timeout=3)
            except ProcessLookupError:
                pass
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(self.rviz_process.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass
                self.rviz_process.wait(timeout=3)
        self.rviz_process = None
        self.active_preview_link = None
        self.active_preview_urdf = None

    def rviz_preview_running(self) -> bool:
        return self.rviz_process is not None and self.rviz_process.poll() is None

    def install_signal_handlers(self) -> None:
        try:
            read_fd, write_fd = os.pipe()
            os.set_blocking(read_fd, False)
            os.set_blocking(write_fd, False)
            signal.set_wakeup_fd(write_fd)
            self._signal_pipe = (read_fd, write_fd)
            if hasattr(self, "createfilehandler"):
                self.createfilehandler(read_fd, tk.READABLE, self.on_signal_pipe_readable)
            else:
                self.schedule_signal_poll()
        except (OSError, ValueError):
            self._signal_pipe = None
            self.schedule_signal_poll()

        for signal_name in ("SIGINT", "SIGTERM", "SIGTSTP"):
            sig = getattr(signal, signal_name, None)
            if sig is None:
                continue
            signal.signal(sig, self.handle_process_signal)

    def handle_process_signal(self, signum: int, _frame: object | None) -> None:
        if self._signal_pipe is None:
            self.after(0, lambda: self.close_from_signal(signum))

    def close_from_signal(self, signum: int) -> None:
        try:
            signal_name = signal.Signals(signum).name
        except ValueError:
            signal_name = str(signum)
        self.request_close(f"received {signal_name}")

    def on_signal_pipe_readable(self, fd: int, _mask: int) -> None:
        try:
            data = os.read(fd, 1024)
        except BlockingIOError:
            return
        except OSError:
            return
        if data:
            self.close_from_signal(data[-1])

    def schedule_signal_poll(self) -> None:
        if self._shutting_down:
            return
        self._signal_poll_after_id = self.after(100, self.poll_signal_pipe)

    def poll_signal_pipe(self) -> None:
        self._signal_poll_after_id = None
        if self._signal_pipe is None:
            self.schedule_signal_poll()
            return
        self.on_signal_pipe_readable(self._signal_pipe[0], tk.READABLE)
        if not self._shutting_down:
            self.schedule_signal_poll()

    def cleanup_signal_handlers(self) -> None:
        if self._signal_poll_after_id is not None:
            try:
                self.after_cancel(self._signal_poll_after_id)
            except tk.TclError:
                pass
            self._signal_poll_after_id = None
        if self._signal_pipe is None:
            return
        read_fd, write_fd = self._signal_pipe
        try:
            if hasattr(self, "deletefilehandler"):
                self.deletefilehandler(read_fd)
        except tk.TclError:
            pass
        try:
            signal.set_wakeup_fd(-1)
        except ValueError:
            pass
        for fd in (read_fd, write_fd):
            try:
                os.close(fd)
            except OSError:
                pass
        self._signal_pipe = None

    def request_close(self, reason: str = "close") -> None:
        if self._shutting_down:
            return
        self._shutting_down = True
        self.status_var.set(reason)
        if self.auto_apply_after_id is not None:
            self.after_cancel(self.auto_apply_after_id)
            self.auto_apply_after_id = None
        self.stop_rviz_preview()
        cleanup_stale_preview_launches()
        self.cleanup_signal_handlers()
        self.shutdown_embedded_preview()
        self.destroy()

    def shutdown_embedded_preview(self) -> None:
        if self.vtk_widget is None:
            return
        try:
            self.vtk_widget.GetRenderWindow().Finalize()
        except Exception:  # noqa: BLE001 - best effort during shutdown
            pass

    def scan_path(self, path: Path, expand_xacro: bool) -> None:
        self.tree.delete(*self.tree.get_children())
        self.mass_values.clear()
        self.tree_link_by_item.clear()
        self.tree_item_by_link.clear()
        self.tree_display_by_item.clear()
        self.tree_group_items.clear()
        self.selected_var.set("")
        self.loading_selection = True
        self.mass_var.set("")
        self.loading_selection = False
        if expand_xacro and looks_like_xacro(path):
            source_summaries = scan_xacro_source_links(path, self.package_roots())
            source_group_items: dict[Path, str] = {}
            for index, summary in enumerate(source_summaries):
                source_file = summary.source_file
                source_label = source_file.name
                group_item = source_group_items.get(source_file)
                if group_item is None:
                    group_item = f"__xacro_source__{len(source_group_items) + 1}"
                    source_group_items[source_file] = group_item
                    self.tree_group_items.add(group_item)
                    self.tree.insert(
                        "",
                        tk.END,
                        iid=group_item,
                        text=source_label,
                        values=("", "", "", "xacro元定義"),
                        open=True,
                    )
                current = "" if summary.existing_mass is None else format_float(summary.existing_mass)
                instances = ", ".join(summary.instance_names[:2])
                if len(summary.instance_names) > 2:
                    instances += f", ... ({len(summary.instance_names)})"
                status = f"instances: {instances}" if instances else "OK"
                if summary.warnings:
                    status = f"{status}; {'; '.join(summary.warnings)}"
                self.mass_values[summary.representative_name] = current
                item_id = f"source::{index}"
                self.tree_link_by_item[item_id] = summary.representative_name
                self.tree_item_by_link[summary.representative_name] = item_id
                self.tree_display_by_item[item_id] = f"{source_label}:{summary.source_link_name}"
                self.tree.insert(
                    group_item,
                    tk.END,
                    iid=item_id,
                    text=summary.source_link_name,
                    values=(current, current, str(summary.mesh_count), status),
                )
            self.log_line(f"読込完了（xacro元定義）: {path}")
            return

        summaries = scan_urdf(path, self.package_roots(), expand_xacro=expand_xacro)
        summary_by_name = {summary.name: summary for summary in summaries}
        groups = fixed_joint_groups(path, self.package_roots(), expand_xacro=expand_xacro)
        grouped_links: set[str] = set()
        for group in groups:
            if len(group.link_names) <= 1:
                continue
            group_mass = "" if group.existing_mass is None else format_float(group.existing_mass)
            group_status = "; ".join(group.warnings) if group.warnings else "OK"
            group_item = f"__group__{group.group_id}"
            self.tree_group_items.add(group_item)
            self.tree.insert(
                "",
                tk.END,
                iid=group_item,
                text=group.label,
                values=("", group_mass, str(group.mesh_count), group_status),
                open=len(group.link_names) <= 8,
            )
            for link_name in group.link_names:
                summary = summary_by_name.get(link_name)
                if summary is None:
                    continue
                grouped_links.add(link_name)
                current = "" if summary.existing_mass is None else format_float(summary.existing_mass)
                status = "; ".join(summary.warnings) if summary.warnings else "OK"
                self.mass_values[summary.name] = current
                item_id = f"link::{summary.name}"
                self.tree_link_by_item[item_id] = summary.name
                self.tree_item_by_link[summary.name] = item_id
                self.tree.insert(
                    group_item,
                    tk.END,
                    iid=item_id,
                    text=summary.name,
                    values=(current, current, str(summary.mesh_count), status),
                )
        for summary in summaries:
            if summary.name in grouped_links:
                continue
            current = "" if summary.existing_mass is None else format_float(summary.existing_mass)
            status = "; ".join(summary.warnings) if summary.warnings else "OK"
            self.mass_values[summary.name] = current
            item_id = f"link::{summary.name}"
            self.tree_link_by_item[item_id] = summary.name
            self.tree_item_by_link[summary.name] = item_id
            self.tree.insert(
                "",
                tk.END,
                iid=item_id,
                text=summary.name,
                values=(current, current, str(summary.mesh_count), status),
            )
        mode = "xacro展開" if expand_xacro else "直接"
        self.log_line(f"読込完了（{mode}）: {path}")

    def scan_joint_path(self, path: Path, expand_xacro: bool) -> None:
        self.joint_tree.delete(*self.joint_tree.get_children())
        self.joint_by_item.clear()
        self.joint_item_by_name.clear()
        self.joint_group_items.clear()
        self.joint_requires_confirmation.clear()
        self.selected_joint_var.set("")
        for var in self.joint_field_vars.values():
            var.set("")
        self.allow_joint_edit_var.set(False)
        self.update_joint_edit_state()

        if expand_xacro and looks_like_xacro(path):
            summaries = scan_xacro_source_joints(path, self.package_roots(), movable_only=True)
            source_group_items: dict[Path, str] = {}
            for index, summary in enumerate(summaries):
                group_item = source_group_items.get(summary.source_file)
                if group_item is None:
                    group_item = f"__joint_source__{len(source_group_items) + 1}"
                    source_group_items[summary.source_file] = group_item
                    self.joint_group_items.add(group_item)
                    self.joint_tree.insert(
                        "",
                        tk.END,
                        iid=group_item,
                        text=summary.source_file.name,
                        values=("", "", "", "", "", "", "", "xacro元定義"),
                        open=True,
                    )
                status = ", ".join(summary.instance_names[:2])
                if len(summary.instance_names) > 2:
                    status += f", ... ({len(summary.instance_names)})"
                if summary.warnings:
                    status = f"{status}; {'; '.join(summary.warnings)}" if status else "; ".join(summary.warnings)
                item_id = f"joint_source::{index}"
                self.joint_by_item[item_id] = summary.representative_name
                self.joint_item_by_name[summary.representative_name] = item_id
                self.joint_requires_confirmation[summary.representative_name] = summary.requires_confirmation
                self.joint_tree.insert(
                    group_item,
                    tk.END,
                    iid=item_id,
                    text=summary.source_joint_name,
                    values=(
                        summary.joint_type,
                        summary.lower,
                        summary.upper,
                        summary.effort,
                        summary.velocity,
                        summary.damping,
                        summary.friction,
                        status,
                    ),
                )
            self.log_line(f"ジョイント読込完了（xacro元定義）: {path}")
            self.update_joint_edit_state()
            return

        for index, summary in enumerate(scan_urdf_joints(path, self.package_roots(), expand_xacro=False)):
            item_id = f"joint::{index}"
            self.joint_by_item[item_id] = summary.name
            self.joint_item_by_name[summary.name] = item_id
            self.joint_requires_confirmation[summary.name] = summary.requires_confirmation
            status = "; ".join(summary.warnings) if summary.warnings else "OK"
            self.joint_tree.insert(
                "",
                tk.END,
                iid=item_id,
                text=summary.name,
                values=(
                    summary.joint_type,
                    summary.lower,
                    summary.upper,
                    summary.effort,
                    summary.velocity,
                    summary.damping,
                    summary.friction,
                    status,
                ),
            )
        self.log_line(f"ジョイント読込完了（直接）: {path}")
        self.update_joint_edit_state()

    def scan(self) -> None:
        path_text = self.urdf_var.get().strip()
        if not path_text:
            messagebox.showerror("URDF", "URDF/xacroファイルを選択してください。")
            return
        new_path = Path(path_text).expanduser().resolve()
        if self.urdf_path != new_path:
            self.reset_model_state(stop_preview=True)
            self.urdf_path = new_path
            self.urdf_var.set(str(self.urdf_path))
        else:
            self.urdf_path = new_path
        try:
            self.scan_path(
                self.urdf_path,
                expand_xacro=looks_like_xacro(self.urdf_path),
            )
            self.scan_joint_path(
                self.urdf_path,
                expand_xacro=looks_like_xacro(self.urdf_path),
            )
        except Exception as exc:  # noqa: BLE001 - shown in GUI
            messagebox.showerror("読込失敗", str(exc))
            return

    def selected_link(self) -> str | None:
        selection = self.tree.selection()
        if not selection:
            return None
        return self.tree_link_by_item.get(str(selection[0]))

    def tree_item_for_link(self, link: str) -> str | None:
        item_id = self.tree_item_by_link.get(link)
        if item_id and self.tree.exists(item_id):
            return item_id
        return None

    def selected_joint(self) -> str | None:
        selection = self.joint_tree.selection()
        if not selection:
            return None
        return self.joint_by_item.get(str(selection[0]))

    def joint_item_for_name(self, joint_name: str) -> str | None:
        item_id = self.joint_item_by_name.get(joint_name)
        if item_id and self.joint_tree.exists(item_id):
            return item_id
        return None

    def on_joint_select(self, _event: object | None = None) -> None:
        joint = self.selected_joint()
        if joint is None:
            self.selected_joint_var.set("")
            for var in self.joint_field_vars.values():
                var.set("")
            self.allow_joint_edit_var.set(False)
            self.update_joint_edit_state()
            return
        item_id = self.joint_item_for_name(joint)
        if item_id is None:
            return
        self.selected_joint_var.set(joint)
        for key in ("lower", "upper", "effort", "velocity", "damping", "friction"):
            self.joint_field_vars[key].set(self.joint_tree.set(item_id, key))
        self.allow_joint_edit_var.set(not self.joint_requires_confirmation.get(joint, False))
        self.update_joint_edit_state()

    def update_joint_edit_state(self) -> None:
        if not hasattr(self, "joint_edit_widgets"):
            return
        joint = self.selected_joint()
        has_joint = joint is not None
        if hasattr(self, "allow_joint_edit_check"):
            self.allow_joint_edit_check.configure(state=tk.NORMAL if has_joint else tk.DISABLED)
        enabled = has_joint and self.allow_joint_edit_var.get()
        for widget in self.joint_edit_widgets:
            widget.configure(state=tk.NORMAL if enabled else tk.DISABLED)
        if not hasattr(self, "joint_status_var"):
            return
        if not has_joint:
            self.joint_status_var.set("")
        elif self.joint_requires_confirmation.get(joint, False) and not self.allow_joint_edit_var.get():
            self.joint_status_var.set("mimic/補助ジョイントの可能性があるため、編集許可が必要です。")
        elif not self.allow_joint_edit_var.get():
            self.joint_status_var.set("編集ロック中です。")

    def collect_joint_values(self) -> dict[str, str]:
        return {key: var.get().strip() for key, var in self.joint_field_vars.items()}

    def set_joint_properties(self) -> None:
        if self.urdf_path is None:
            messagebox.showerror("URDF", "URDF/xacroファイルを選択してください。")
            return
        joint = self.selected_joint()
        if joint is None:
            messagebox.showerror("ジョイント", "ジョイントを選択してください。")
            return
        if not self.allow_joint_edit_var.get():
            messagebox.showerror("ジョイント", "編集を許可してから反映してください。")
            return
        values = self.collect_joint_values()
        try:
            if self.expanded_mode():
                report = apply_joint_properties_to_xacro_sources(
                    self.urdf_path,
                    {joint: values},
                    self.package_roots(),
                    backup=True,
                )
            else:
                report = apply_joint_properties_to_urdf(self.urdf_path, {joint: values}, backup=True)
        except Exception as exc:  # noqa: BLE001 - shown in GUI
            messagebox.showerror("ジョイント反映失敗", str(exc))
            return

        for name in report.updated:
            self.log_line(f"ジョイント更新: {name}")
        for name, warnings in report.skipped.items():
            self.log_line(f"ジョイント未更新 {name}: {'; '.join(warnings)}")
        for backup_path in report.backup_paths:
            self.log_line(f"バックアップ: {backup_path}")
        self.joint_status_var.set(f"更新={len(report.updated)} 未更新={len(report.skipped)}")
        self.scan_joint_path(self.urdf_path, expand_xacro=self.expanded_mode())
        item_id = self.joint_item_by_name.get(joint)
        if item_id and self.joint_tree.exists(item_id):
            self.joint_tree.selection_set(item_id)
            self.joint_tree.see(item_id)
            self.on_joint_select()

    def clear_joint_fields(self) -> None:
        for var in self.joint_field_vars.values():
            var.set("")

    def on_select(self, _event: object | None = None) -> None:
        link = self.selected_link()
        if link is None:
            selection = self.tree.selection()
            if selection and str(selection[0]) in self.tree_group_items:
                self.selected_var.set("")
                self.loading_selection = True
                self.mass_var.set("")
                self.loading_selection = False
            return
        self.loading_selection = True
        selection = self.tree.selection()
        display = self.tree_display_by_item.get(str(selection[0]), link) if selection else link
        self.selected_var.set(display if display == link else f"{display} -> {link}")
        self.mass_var.set(self.mass_values.get(link, ""))
        self.loading_selection = False
        self.refresh_preview_for_selection(link)

    def refresh_preview_for_selection(self, link: str) -> None:
        if not hasattr(self, "preview_status_var") or not VTK_AVAILABLE:
            return
        try:
            self.render_link_preview(link)
        except Exception as exc:  # noqa: BLE001 - selection should not interrupt editing
            self.active_preview_link = None
            self.preview_status_var.set(f"3D確認不可: {exc}")
            self.set_preview_info(f"リンク: {link}\n3D確認不可: {exc}")

    def store_current_mass(self, show_errors: bool) -> bool:
        link = self.selected_link()
        if link is None:
            return False
        value = self.mass_var.get().strip()
        if value:
            try:
                mass = float(value)
            except ValueError:
                if show_errors:
                    messagebox.showerror("質量", "質量は数値で入力してください。")
                return False
            if mass <= 0.0:
                if show_errors:
                    messagebox.showerror("質量", "質量は正の値で入力してください。")
                return False
            value = format_float(mass)
        item_id = self.tree_item_for_link(link)
        if item_id is None:
            if show_errors:
                messagebox.showerror("リンク", f"選択リンクが現在のリストにありません: {link}")
            return False
        self.mass_values[link] = value
        existing = self.tree.set(item_id, "existing")
        meshes = self.tree.set(item_id, "meshes")
        status = self.tree.set(item_id, "status")
        self.tree.item(item_id, values=(value, existing, meshes, status))
        return True

    def set_mass(self) -> None:
        if self.auto_apply_after_id is not None:
            self.after_cancel(self.auto_apply_after_id)
            self.auto_apply_after_id = None
        link = self.selected_link()
        if link is None or not self.store_current_mass(show_errors=True):
            return
        if not self.mass_values.get(link, "").strip():
            return
        self.reflect_link_mass(link, start_preview=True, label="set")

    def on_mass_changed(self, *_args: object) -> None:
        if self.loading_selection or not self.auto_apply_var.get():
            return
        if self.auto_apply_after_id is not None:
            self.after_cancel(self.auto_apply_after_id)
            self.auto_apply_after_id = None
        if not self.store_current_mass(show_errors=False):
            return
        if not self.mass_var.get().strip():
            return
        self.auto_apply_after_id = self.after(450, self.auto_apply_selected_link)

    def use_current_mass(self) -> None:
        link = self.selected_link()
        if link is None:
            return
        item_id = self.tree_item_for_link(link)
        if item_id is None:
            return
        self.mass_var.set(self.tree.set(item_id, "existing"))
        self.set_mass()

    def clear_mass(self) -> None:
        link = self.selected_link()
        if link is None:
            return
        self.mass_var.set("")
        self.set_mass()

    def collect_masses(self) -> dict[str, float]:
        masses: dict[str, float] = {}
        for link, value in self.mass_values.items():
            if value.strip():
                masses[link] = float(value)
        return masses

    def expanded_mode(self) -> bool:
        return self.urdf_path is not None and looks_like_xacro(self.urdf_path)

    def apply_target_path(self) -> Path:
        if self.urdf_path is None:
            raise ValueError("URDF is not selected")
        if not self.expanded_mode():
            return self.urdf_path
        return self.urdf_path

    def apply_masses_to_target(self, masses: dict[str, float], backup: bool) -> tuple[Path, object, list[str]]:
        target_path = self.apply_target_path()
        package_roots = self.package_roots()
        if self.expanded_mode():
            report = apply_masses_to_xacro_sources(self.urdf_path, masses, package_roots, backup=backup)
            check_messages = validate_xacro_inertials(
                self.urdf_path,
                package_roots,
                [result.link_name for result in report.updated],
            )
        else:
            report = apply_masses_to_urdf(target_path, masses, package_roots, backup=backup)
            check_messages = validate_urdf_inertials(
                target_path,
                [result.link_name for result in report.updated],
            )
        self.last_applied_path = target_path
        return target_path, report, check_messages

    def apply_update(self) -> None:
        if self.urdf_path is None:
            messagebox.showerror("URDF", "URDF/xacroファイルを選択してください。")
            return
        selected = self.selected_link()
        self.store_current_mass(show_errors=True)
        masses = self.collect_masses()
        if not masses:
            messagebox.showerror("質量", "少なくとも1つのリンク質量を入力してください。")
            return
        try:
            target_path, report, check_messages = self.apply_masses_to_target(masses, backup=True)
            self.log_apply_report(target_path, report, check_messages)
            if self.expanded_mode():
                self.scan_path(self.urdf_path, expand_xacro=True)
            else:
                self.scan_path(target_path, expand_xacro=False)
            selected_item = self.tree_item_by_link.get(selected or "")
            if selected_item and self.tree.exists(selected_item):
                self.tree.selection_set(selected_item)
                self.tree.see(selected_item)
                self.on_select()
        except Exception as exc:  # noqa: BLE001 - shown in GUI
            messagebox.showerror("一括反映失敗", str(exc))
            return

    def auto_apply_selected_link(self) -> None:
        self.auto_apply_after_id = None
        if self.urdf_path is None:
            return
        link = self.selected_link()
        if link is None or not self.store_current_mass(show_errors=False):
            return
        value = self.mass_values.get(link, "").strip()
        if not value:
            return
        self.reflect_link_mass(link, start_preview=False, label="auto")

    def reflect_link_mass(self, link: str, start_preview: bool, label: str) -> None:
        try:
            target_path = self.apply_target_path()
            backup = target_path not in self.auto_backup_paths
            value = self.mass_values.get(link, "").strip()
            _, report, check_messages = self.apply_masses_to_target({link: float(value)}, backup=backup)
            if report.updated and backup:
                self.auto_backup_paths.add(target_path)
            self.update_row_from_target(target_path, link)
            if start_preview:
                self.start_or_refresh_preview(link)
            else:
                self.refresh_active_preview_if_needed(link)
            status = "OK" if check_messages == ["OK"] else "; ".join(check_messages[:2])
            self.status_var.set(f"{label}: {link} 更新={len(report.updated)} 確認={status}")
        except Exception as exc:  # noqa: BLE001 - shown in status
            if label == "set":
                messagebox.showerror("反映失敗", str(exc))
            else:
                self.status_var.set(f"{label} 失敗: {exc}")

    def update_row_from_target(self, target_path: Path, link: str) -> None:
        if self.expanded_mode() and self.urdf_path is not None:
            for summary in scan_xacro_source_links(self.urdf_path, self.package_roots()):
                if summary.representative_name != link:
                    continue
                current = "" if summary.existing_mass is None else format_float(summary.existing_mass)
                value = self.mass_values.get(link, current)
                item_id = self.tree_item_by_link.get(link)
                if item_id and self.tree.exists(item_id):
                    self.tree.item(item_id, values=(value, current, str(summary.mesh_count), "OK"))
                break
            return

        for summary in scan_urdf(target_path, self.package_roots(), expand_xacro=False):
            if summary.name != link:
                continue
            current = "" if summary.existing_mass is None else format_float(summary.existing_mass)
            value = self.mass_values.get(link, current)
            status = "; ".join(summary.warnings) if summary.warnings else "OK"
            item_id = self.tree_item_by_link.get(link)
            if item_id and self.tree.exists(item_id):
                self.tree.item(item_id, values=(value, current, str(summary.mesh_count), status))
            break

    def log_apply_report(self, target_path: Path, report: object, check_messages: list[str]) -> None:
        self.log_line(f"更新: {len(report.updated)}")
        for result in report.updated:
            self.log_line(
                f"  {result.link_name}: 質量={format_float(result.mass)} "
                f"com={' '.join(format_float(float(v)) for v in result.center)}"
            )
        for link, warnings in report.skipped.items():
            self.log_line(f"  未更新 {link}: {'; '.join(warnings)}")
        if report.backup_path:
            self.log_line(f"バックアップ: {report.backup_path}")
        for backup_path in getattr(report, "backup_paths", []):
            if backup_path != report.backup_path:
                self.log_line(f"バックアップ: {backup_path}")
        for message in check_messages:
            self.log_line(f"更新リンク確認: {message}")
        status = "OK" if check_messages == ["OK"] else "; ".join(check_messages[:3])
        self.status_var.set(f"反映完了: 更新={len(report.updated)} 未更新={len(report.skipped)} 確認={status}")
        self.log_line(f"反映先: {target_path}")

    def preview_paths(self, link: str) -> tuple[Path, Path]:
        preview_dir = Path(tempfile.gettempdir()) / "urdf_xacro_tuner_preview"
        preview_dir.mkdir(parents=True, exist_ok=True)
        safe_link = "".join(ch if ch.isalnum() or ch in ("_", "-") else "_" for ch in link)
        return preview_dir / f"{safe_link}.work.urdf", preview_dir / f"{safe_link}.preview.urdf"

    def write_preview_urdf(self, link: str) -> Path:
        if self.urdf_path is None:
            raise ValueError("URDF is not selected")
        package_roots = self.package_roots()
        work_urdf, preview_urdf = self.preview_paths(link)
        if self.expanded_mode():
            write_expanded_xacro(self.urdf_path, work_urdf, package_roots)
        else:
            shutil.copy2(self.urdf_path, work_urdf)

        value = self.mass_values.get(link, "").strip()
        if value:
            apply_masses_to_urdf(work_urdf, {link: float(value)}, package_roots, backup=False)

        write_single_link_preview_urdf(
            work_urdf,
            link,
            preview_urdf,
            package_roots,
            expand_xacro=False,
        )
        return preview_urdf

    def preview_selected_link(self) -> None:
        if self.urdf_path is None:
            messagebox.showerror("URDF", "URDF/xacroファイルを選択してください。")
            return
        link = self.selected_link()
        if link is None:
            messagebox.showerror("リンク", "リンクを選択してください。")
            return
        self.store_current_mass(show_errors=True)

        try:
            if self.preview_tab is not None:
                self.notebook.select(self.preview_tab)
            self.start_or_refresh_preview(link)
        except Exception as exc:  # noqa: BLE001 - shown in GUI
            messagebox.showerror("3D確認失敗", str(exc))
            return

    def start_or_refresh_preview(self, link: str) -> None:
        self.render_link_preview(link)
        self.status_var.set(f"3D確認更新: {link}")
        self.log_line(f"3D確認更新: {link}")

    def refresh_active_preview_if_needed(self, link: str) -> None:
        if self.active_preview_link != link:
            return
        try:
            self.render_link_preview(link)
            self.log_line(f"3D確認更新: {link}")
        except Exception as exc:  # noqa: BLE001 - keep the editor usable
            self.preview_status_var.set(f"3D確認不可: {exc}")


def main() -> None:
    app = InertiaEditor()
    try:
        app.mainloop()
    except KeyboardInterrupt:
        app.request_close("received KeyboardInterrupt")


if __name__ == "__main__":
    main()
