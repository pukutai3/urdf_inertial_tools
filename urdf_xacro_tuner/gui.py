#!/usr/bin/env python3
"""Simple Tk GUI for editing URDF inertial values and previewing one link."""

from __future__ import annotations

import os
import argparse
import sys
import shlex
import signal
import shutil
import subprocess
import tempfile
import threading
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from tkinter import font as tkfont
import xml.etree.ElementTree as ET

import numpy as np

from urdf_xacro_tuner.urdf_mass_inertia import (
    apply_joint_properties_to_urdf,
    apply_joint_properties_to_xacro_sources,
    apply_masses_to_xacro_sources,
    apply_masses_to_urdf,
    apply_mesh_visibility_to_urdf,
    apply_mesh_visibility_to_xacro_sources,
    calculate_link_inertial,
    link_mesh_visible,
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
ANGLE_JOINT_TYPES = {"revolute", "continuous"}
JOINT_TYPE_OPTIONS = ("fixed", "revolute", "continuous", "prismatic", "floating", "planar")
JOINT_ANGLE_UNITS = ("rad", "deg")

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


class FrozenTreeTable:
    def __init__(self, master: tk.Widget, name_heading: str, columns: tuple[str, ...], name_width: int = 220, right_width: int = 220) -> None:
        self.frame = ttk.Frame(master)
        self._selection_callback = None
        self._syncing_selection = False
        self._right_width = right_width

        self.frame.columnconfigure(0, weight=0, minsize=name_width)
        self.frame.columnconfigure(1, weight=0, minsize=right_width)
        self.frame.rowconfigure(0, weight=1)

        self.left_frame = ttk.Frame(self.frame, width=name_width)
        self.left_frame.grid(row=0, column=0, sticky='ns')
        self.left_frame.grid_propagate(False)
        self.right_holder = ttk.Frame(self.frame, width=right_width)
        self.right_holder.grid(row=0, column=1, sticky='ns')
        self.right_holder.grid_propagate(False)

        self.left = ttk.Treeview(self.left_frame, columns=(), show='tree headings', selectmode='browse')
        self.left.heading('#0', text=name_heading)
        self.left.column('#0', width=name_width, stretch=False)
        self.left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.right_frame = ttk.Frame(self.right_holder)
        self.right_frame.pack(fill=tk.BOTH, expand=True)

        self.right = ttk.Treeview(self.right_frame, columns=columns, show='headings', selectmode='browse')
        self.right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        vscroll = ttk.Scrollbar(self.right_frame, orient=tk.VERTICAL, command=self.yview)
        vscroll.pack(side=tk.RIGHT, fill=tk.Y)
        hscroll = ttk.Scrollbar(self.frame, orient=tk.HORIZONTAL, command=self.right.xview)
        hscroll.grid(row=1, column=0, columnspan=2, sticky='ew')
        self.right.configure(xscrollcommand=hscroll.set)

        self.left.configure(yscrollcommand=vscroll.set)
        self.right.configure(yscrollcommand=vscroll.set)

        self.left.bind('<<TreeviewSelect>>', self._on_tree_select, add='+')
        self.right.bind('<<TreeviewSelect>>', self._on_tree_select, add='+')
        for widget in (self.left, self.left_frame):
            widget.bind('<MouseWheel>', self._on_left_mousewheel, add='+')
            widget.bind('<Button-4>', self._on_left_button4, add='+')
            widget.bind('<Button-5>', self._on_left_button5, add='+')
        for widget in (self.right, self.right_frame, self.right_holder):
            widget.bind('<MouseWheel>', self._on_right_mousewheel, add='+')
            widget.bind('<Button-4>', self._on_right_button4, add='+')
            widget.bind('<Button-5>', self._on_right_button5, add='+')

    def set_selection_callback(self, callback) -> None:
        self._selection_callback = callback

    def _clear_selection_sync(self) -> None:
        self._syncing_selection = False

    def _sync_selection(self, iid: str) -> None:
        if self._syncing_selection:
            return
        self._syncing_selection = True
        self.left.selection_set(iid)
        self.right.selection_set(iid)
        self.left.see(iid)
        self.right.see(iid)
        self.frame.after_idle(self._clear_selection_sync)

    def _on_tree_select(self, event: object | None = None) -> None:
        if self._syncing_selection:
            return
        source = self.left if event is not None and getattr(event, 'widget', None) is self.left else self.right
        selection = source.selection()
        if not selection:
            return
        iid = selection[0]
        self._sync_selection(iid)
        if self._selection_callback is not None:
            self._selection_callback(event)

    def _on_left_mousewheel(self, event: tk.Event) -> str:
        delta = -1 if getattr(event, 'delta', 0) > 0 else 1
        self.yview_scroll(delta, 'units')
        return 'break'

    def _on_left_button4(self, _event: tk.Event) -> str:
        self.yview_scroll(-1, 'units')
        return 'break'

    def _on_left_button5(self, _event: tk.Event) -> str:
        self.yview_scroll(1, 'units')
        return 'break'

    def _on_right_mousewheel(self, event: tk.Event) -> str:
        delta = -1 if getattr(event, 'delta', 0) > 0 else 1
        self.right.xview_scroll(delta, 'units')
        return 'break'

    def _on_right_button4(self, _event: tk.Event) -> str:
        self.right.xview_scroll(-1, 'units')
        return 'break'

    def _on_right_button5(self, _event: tk.Event) -> str:
        self.right.xview_scroll(1, 'units')
        return 'break'

    def pack(self, *args, **kwargs) -> None:
        self.frame.pack(*args, **kwargs)

    def bind(self, *args, **kwargs):
        self.left.bind(*args, **kwargs)
        return self.right.bind(*args, **kwargs)

    def heading(self, column: str, **kwargs) -> None:
        if column == '#0':
            self.left.heading(column, **kwargs)
        else:
            self.right.heading(column, **kwargs)

    def column(self, column: str, **kwargs) -> None:
        if column == '#0':
            self.left.column(column, **kwargs)
        else:
            self.right.column(column, **kwargs)

    def set_right_width(self, width: int) -> None:
        self._right_width = width
        self.right_frame.configure(width=width)

    def configure(self, **kwargs) -> None:
        self.left.configure(**kwargs)
        self.right.configure(**kwargs)

    def delete(self, *items) -> None:
        self.left.delete(*items)
        self.right.delete(*items)

    def get_children(self, item: str = ''):
        return self.left.get_children(item)

    def exists(self, item: str) -> bool:
        return self.left.exists(item) or self.right.exists(item)

    def insert(self, parent: str, index: str, iid: str | None = None, **kwargs):
        left_kwargs = dict(kwargs)
        left_kwargs.setdefault('values', ())
        right_kwargs = dict(kwargs)
        right_kwargs.pop('text', None)
        self.left.insert(parent, index, iid=iid, **left_kwargs)
        self.right.insert(parent, index, iid=iid, **right_kwargs)
        return iid

    def item(self, item: str, **kwargs):
        if kwargs:
            left_kwargs = dict(kwargs)
            right_kwargs = dict(kwargs)
            if 'values' in kwargs:
                left_kwargs.pop('values', None)
            if 'text' in kwargs:
                right_kwargs.pop('text', None)
            self.left.item(item, **left_kwargs)
            self.right.item(item, **right_kwargs)
            return
        data = self.right.item(item)
        data['text'] = self.left.item(item, 'text')
        return data

    def set(self, item: str, column: str, value=None):
        if value is None:
            if column == '#0':
                return self.left.item(item, 'text')
            return self.right.set(item, column)
        self.right.set(item, column, value)

    def selection(self):
        return self.left.selection()

    def selection_set(self, items) -> None:
        if not items:
            return
        item = items[0] if isinstance(items, (tuple, list)) else items
        self._sync_selection(item)

    def see(self, item: str) -> None:
        self.left.see(item)
        self.right.see(item)

    def yview(self, *args):
        self.left.yview(*args)
        self.right.yview(*args)

    def yview_scroll(self, number: int, what: str) -> None:
        self.left.yview_scroll(number, what)
        self.right.yview_scroll(number, what)

    def yview_moveto(self, fraction: float) -> None:
        self.left.yview_moveto(fraction)
        self.right.yview_moveto(fraction)

    def xview(self, *args):
        self.right.xview(*args)


class InertiaEditor(tk.Tk):
    def _configure_fonts(self) -> None:
        try:
            default_font = tkfont.nametofont("TkDefaultFont")
            default_font.configure(size=8)
            text_font = tkfont.nametofont("TkTextFont")
            text_font.configure(size=8)
            fixed_font = tkfont.nametofont("TkFixedFont")
            fixed_font.configure(size=8)
            heading_font = tkfont.nametofont("TkHeadingFont")
            heading_font.configure(size=8)
            tab_font = default_font.copy()
            tab_font.configure(size=9)
            style = ttk.Style(self)
            style.configure('Treeview', rowheight=20, font=default_font)
            style.configure('Treeview.Heading', font=heading_font)
            style.configure('TNotebook.Tab', padding=(8, 3), font=tab_font)
            style.configure('TLabel', font=default_font)
            style.configure('TButton', font=default_font)
            style.configure('TCheckbutton', font=default_font)
            style.configure('TEntry', font=default_font)
            style.configure('TCombobox', font=default_font)
            self.option_add('*TCombobox*Listbox.font', default_font)
        except Exception:
            pass

    def __init__(self) -> None:
        stale_preview_count = cleanup_stale_preview_launches()
        super().__init__()
        self.title("URDF/xacro Tuner")
        self.geometry("1020x660")
        self.urdf_path: Path | None = None
        self.mass_values: dict[str, str] = {}
        self.last_applied_path: Path | None = None
        self.rviz_process: subprocess.Popen[str] | None = None
        self.active_preview_link: str | None = None
        self.active_preview_urdf: Path | None = None
        self.auto_apply_after_id: str | None = None
        self.loading_selection = False
        self.auto_backup_paths: set[Path] = set()
        self.auto_rebuild_var = tk.BooleanVar(value=True)
        self._rebuild_thread: threading.Thread | None = None
        self._preview_worker: threading.Thread | None = None
        self._preview_render_token = 0
        self._preview_requested_link: str | None = None
        self._pending_preview_payload: tuple[str, dict[str, object]] | None = None
        self._preview_refresh_after_id: str | None = None
        self._preview_render_after_id: str | None = None
        self._preview_payload_cache: dict[str, dict[str, object]] = {}
        self.preview_model_tree: ET.ElementTree | None = None
        self.preview_model_key: tuple[Path, bool, tuple[Path, ...]] | None = None
        self.joint_angle_unit_var = tk.StringVar(value="rad")
        self._joint_angle_unit_last = "rad"
        self.tree_link_by_item: dict[str, str] = {}
        self.tree_item_by_link: dict[str, str] = {}
        self.tree_display_by_item: dict[str, str] = {}
        self.tree_group_items: set[str] = set()
        self.current_link: str | None = None
        self.link_filter_var = tk.StringVar(value="")
        self._link_filter_after_id: str | None = None
        self.link_scan_path: Path | None = None
        self.link_scan_expand_xacro = False
        self.link_scan_summaries: list[object] = []
        self.link_scan_source_summaries: list[object] = []
        self.link_scan_groups: list[object] = []
        self.link_scan_mode = "empty"
        self.joint_by_item: dict[str, str] = {}
        self.joint_item_by_name: dict[str, str] = {}
        self.joint_group_items: set[str] = set()
        self.joint_requires_confirmation: dict[str, bool] = {}
        self.preview_mesh_states: dict[str, bool] = {}
        self.preview_tab: ttk.Frame | None = None
        self.vtk_widget: object | None = None
        self.vtk_renderer: object | None = None
        self.preview_info: tk.Text | None = None
        self._shutting_down = False
        self._signal_pipe: tuple[int, int] | None = None
        self._signal_poll_after_id: str | None = None
        self._layout_after_id: str | None = None
        self._last_paned_layout_key: tuple[int, str, int] | None = None
        self._last_paned_layout_key: tuple[int, str, int] | None = None
        self._configure_fonts()
        self._build_widgets()
        self.protocol("WM_DELETE_WINDOW", self.request_close)
        self.install_signal_handlers()
        if stale_preview_count:
            self.log_line(f"残留プレビューを停止: {stale_preview_count}件")

    def _build_widgets(self) -> None:
        root = ttk.Frame(self, padding=8)
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
        self.notebook.pack(fill=tk.BOTH, expand=True, pady=6)

        inertia_tab = ttk.Frame(self.notebook)
        joint_tab = ttk.Frame(self.notebook)
        self.preview_tab = None
        self.notebook.add(inertia_tab, text="質量/慣性")
        self.notebook.add(joint_tab, text="ジョイント")

        middle = ttk.PanedWindow(inertia_tab, orient=tk.HORIZONTAL)
        self.inertia_middle = middle
        middle.pack(fill=tk.BOTH, expand=True)

        table_frame = ttk.Frame(middle)
        middle.add(table_frame, weight=4)
        filter_row = ttk.Frame(table_frame)
        filter_row.pack(fill=tk.X, pady=(0, 6))
        ttk.Label(filter_row, text="検索").pack(side=tk.LEFT)
        filter_entry = ttk.Entry(filter_row, textvariable=self.link_filter_var)
        filter_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(6, 6))
        filter_entry.bind("<Escape>", lambda _event: self.clear_link_filter())
        ttk.Button(filter_row, text="クリア", command=self.clear_link_filter).pack(side=tk.LEFT)
        self.link_filter_var.trace_add("write", self.on_link_filter_changed)
        columns = ("mass", "existing", "meshes", "status")
        self.tree = FrozenTreeTable(table_frame, "\u30ea\u30f3\u30af", columns, name_width=190, right_width=260)
        self.tree.heading("mass", text="\u5165\u529b\u8cea\u91cf kg")
        self.tree.heading("existing", text="\u65e2\u5b58\u8cea\u91cf kg")
        self.tree.heading("meshes", text="mesh\u6570")
        self.tree.heading("status", text="\u72b6\u614b")
        self.tree.column("mass", width=100, anchor=tk.E, stretch=False)
        self.tree.column("existing", width=100, anchor=tk.E, stretch=False)
        self.tree.column("meshes", width=70, anchor=tk.E, stretch=False)
        self.tree.column("status", width=150, stretch=False)
        self.tree.pack(fill=tk.BOTH, expand=True)
        self.tree.set_selection_callback(self.on_select)
        edit_frame = ttk.Frame(middle, padding=(10, 0, 0, 0))
        middle.add(edit_frame, weight=3)
        ttk.Label(edit_frame, text="選択リンク").pack(anchor=tk.W)
        self.selected_var = tk.StringVar()
        ttk.Entry(edit_frame, textvariable=self.selected_var, state="readonly").pack(fill=tk.X, pady=(2, 10))
        ttk.Label(edit_frame, text="入力質量 kg").pack(anchor=tk.W)
        self.mass_var = tk.StringVar()
        mass_entry = ttk.Entry(edit_frame, textvariable=self.mass_var)
        mass_entry.pack(fill=tk.X, pady=(2, 8))
        mass_entry.bind("<Return>", lambda _event: self.set_mass())
        self.mass_var.trace_add("write", self.on_mass_changed)
        ttk.Button(edit_frame, text="反映", command=self.set_mass).pack(fill=tk.X)
        ttk.Button(edit_frame, text="現在値を使用", command=self.use_current_mass).pack(fill=tk.X, pady=(6, 0))
        ttk.Button(edit_frame, text="クリア", command=self.clear_mass).pack(fill=tk.X, pady=(6, 0))
        self.auto_apply_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(edit_frame, text="自動反映", variable=self.auto_apply_var).pack(
            anchor=tk.W, pady=(8, 0)
        )
        ttk.Separator(edit_frame).pack(fill=tk.X, pady=14)
        ttk.Button(edit_frame, text="一括反映", command=self.apply_update).pack(fill=tk.X)
        ttk.Button(edit_frame, text="3D確認", command=self.preview_selected_link).pack(fill=tk.X, pady=(6, 0))
        self.status_var = tk.StringVar(value="")
        ttk.Label(edit_frame, textvariable=self.status_var, wraplength=180).pack(fill=tk.X, pady=(10, 0))

        preview_frame = ttk.Frame(middle)
        middle.add(preview_frame, weight=5)
        self.build_preview_tab(preview_frame)

        self.build_joint_tab(joint_tab)
        self.after_idle(self.apply_paned_layout)
        self.bind("<Configure>", self.on_root_configure, add="+")
        self.notebook.bind("<<NotebookTabChanged>>", self.on_tab_changed, add="+")
        self.log = tk.Text(root, height=5, wrap=tk.WORD, font=("TkFixedFont", 8))
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
        self.preview_mesh_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(
            controls,
            text="STL表示",
            variable=self.preview_mesh_var,
            command=self.on_preview_mesh_toggle,
        ).pack(side=tk.LEFT, padx=(10, 0))

        middle = ttk.PanedWindow(parent, orient=tk.HORIZONTAL)
        self.preview_middle = middle
        middle.pack(fill=tk.BOTH, expand=True)

        viewer_frame = ttk.Frame(middle)
        middle.add(viewer_frame, weight=4)
        info_frame = ttk.Frame(middle, padding=(10, 0, 0, 0))
        middle.add(info_frame, weight=1)

        self.preview_info = tk.Text(info_frame, width=26, height=13, wrap=tk.NONE, font=("TkFixedFont", 8))
        self.preview_info.pack(fill=tk.BOTH, expand=True)
        self.preview_info.configure(state=tk.DISABLED)

        if not VTK_AVAILABLE:
            reason = f"VTKを読み込めません: {VTK_IMPORT_ERROR}"
            ttk.Label(viewer_frame, text=reason, wraplength=440).pack(fill=tk.BOTH, expand=True)
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

    def preview_mesh_enabled_for_link(self, link: str) -> bool:
        return self.preview_mesh_states.get(link, True)

    def sync_preview_mesh_toggle(self, link: str) -> None:
        if hasattr(self, "preview_mesh_var"):
            self.preview_mesh_var.set(self.preview_mesh_enabled_for_link(link))

    def on_preview_mesh_toggle(self) -> None:
        link = self.active_preview_link
        if link is None:
            return
        show_mesh = bool(self.preview_mesh_var.get())
        previous = self.preview_mesh_states.get(link, True)
        self.preview_mesh_states[link] = show_mesh
        self._preview_payload_cache.clear()
        self.invalidate_preview_model_cache()
        self._preview_requested_link = None
        try:
            target_path = self.apply_target_path()
            backup = target_path not in self.auto_backup_paths
            _, report = self.apply_mesh_visibility_to_target({link: show_mesh}, backup=backup)
            self.invalidate_preview_model_cache()
            if report.updated and backup:
                self.auto_backup_paths.add(target_path)
            self.refresh_preview_for_selection(link, force=True)
            if self.auto_rebuild_var.get():
                self.schedule_workspace_rebuild("STL表示更新")
            self.preview_status_var.set(f"STL表示: {link} -> {'ON' if show_mesh else 'OFF'}")
            self.log_line(f"STL表示: {link} -> {'ON' if show_mesh else 'OFF'}")
        except Exception as exc:  # noqa: BLE001 - preview toggle should not break editing
            self.preview_mesh_states[link] = previous
            self.preview_mesh_var.set(previous)
            self.preview_status_var.set(f"STL表示失敗: {exc}")
            self.log_line(f"STL表示失敗: {exc}")

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

    def invalidate_preview_model_cache(self) -> None:
        self.preview_model_tree = None
        self.preview_model_key = None

    def preview_model_tree_for_current_state(self) -> ET.ElementTree:
        if self.urdf_path is None:
            raise ValueError("URDF/xacro??????????")
        package_roots = tuple(self.package_roots())
        key = (self.urdf_path, self.expanded_mode(), package_roots)
        if self.preview_model_tree is None or self.preview_model_key != key:
            self.preview_model_tree = expand_xacro_to_tree(self.urdf_path, package_roots) if self.expanded_mode() else parse_urdf(self.urdf_path)
            self.preview_model_key = key
        return self.preview_model_tree

    def preview_mass_for_link(self, link: str, link_element: object) -> tuple[float | None, str]:
        value = self.mass_values.get(link, "").strip()
        if value:
            return float(value), "入力質量"
        existing = find_existing_mass(link_element)
        if existing is not None and existing > 0.0:
            return existing, "現在値"
        return None, "未設定"

    def _build_preview_payload(self, link: str) -> dict[str, object]:
        if self.urdf_path is None:
            raise ValueError("URDF/xacro??????????????")
        if not VTK_AVAILABLE:
            raise RuntimeError(f"VTK????????: {VTK_IMPORT_ERROR}")
        if self.vtk_renderer is None:
            raise RuntimeError("3D???????????????")

        package_roots = self.package_roots()
        tree = self.preview_model_tree_for_current_state()
        link_element = find_direct_link(tree, link)
        if link_element is None:
            raise ValueError(f"???????????: {link}")

        mesh_visible = link_mesh_visible(link_element)
        refs = extract_mesh_refs(link_element, self.urdf_path, package_roots)
        if not refs:
            raise ValueError(f"STL mesh??????: {link}")
        meshes = [load_transformed_stl(ref) for ref in refs]

        all_vertices = np.concatenate([np.asarray(mesh.vertices, dtype=float) for mesh in meshes], axis=0)
        bounds_min = all_vertices.min(axis=0)
        bounds_max = all_vertices.max(axis=0)
        diagonal = float(np.linalg.norm(bounds_max - bounds_min))
        if diagonal <= 1.0e-9:
            diagonal = 1.0
        axis_length = diagonal * 0.28
        marker_radius = diagonal * 0.025

        mass, mass_source = self.preview_mass_for_link(link, link_element)
        result = None
        inertia_error = ""
        if mass is not None:
            try:
                result = calculate_link_inertial(link_element, self.urdf_path, mass, package_roots)
            except Exception as exc:  # noqa: BLE001 - mesh display still helps inspection
                inertia_error = str(exc)

        info_lines = [
            f"???: {link}",
            f"mesh?: {len(meshes)}",
            f"STL??: {'ON' if mesh_visible else 'OFF'}",
            "STL??: ??????????????????",
            "STL:",
            *[f"  {ref.filename}" for ref in refs],
            "",
            f"??: {format_float(mass)} kg ({mass_source})" if mass is not None else "??: ???",
        ]
        if result is not None:
            inertia = result.inertia
            moments, _axes = np.linalg.eigh(inertia)
            info_lines.extend(
                [
                    f"??: {' '.join(format_float(float(v)) for v in result.center)}",
                    "??:",
                    f"  ixx {format_float(float(inertia[0, 0]))}",
                    f"  ixy {format_float(float(inertia[0, 1]))}",
                    f"  ixz {format_float(float(inertia[0, 2]))}",
                    f"  iyy {format_float(float(inertia[1, 1]))}",
                    f"  iyz {format_float(float(inertia[1, 2]))}",
                    f"  izz {format_float(float(inertia[2, 2]))}",
                    "???:",
                    f"  {' '.join(format_float(float(v)) for v in moments)}",
                ]
            )
        elif inertia_error:
            info_lines.extend(["", f"??????: {inertia_error}"])

        return {
            "link": link,
            "mesh_visible": mesh_visible,
            "meshes": meshes,
            "axis_length": axis_length,
            "marker_radius": marker_radius,
            "mass": mass,
            "mass_source": mass_source,
            "result": result,
            "info_lines": info_lines,
        }

    def _apply_preview_payload(self, link: str, payload: dict[str, object]) -> None:
        self.preview_mesh_states[link] = bool(payload["mesh_visible"])
        self.sync_preview_mesh_toggle(link)
        self.set_preview_info("\n".join(payload["info_lines"]))
        self.preview_status_var.set(f"3D???: {link}")
        self.log_line(f"3D_PREPARED: {link}")
        self._pending_preview_payload = (link, payload)
        after_id = self._preview_render_after_id
        if after_id is not None:
            try:
                self.after_cancel(after_id)
            except tk.TclError:
                pass
        self._preview_requested_link = None
        self._preview_render_after_id = self.after_idle(lambda: self._finalize_preview_render(link))

    def _finalize_preview_render(self, link: str) -> None:
        pending = self._pending_preview_payload
        if pending is None or pending[0] != link:
            return
        self._preview_render_after_id = None
        self._pending_preview_payload = None
        self._preview_requested_link = None
        _, payload = pending
        try:
            self.clear_preview_scene("")

            meshes = payload["meshes"]
            if self.preview_mesh_var.get():
                for mesh in meshes:
                    self.add_mesh_actor(mesh)

            axis_length = float(payload["axis_length"])
            for direction, color in (
                (np.array([1.0, 0.0, 0.0]), (0.9, 0.2, 0.2)),
                (np.array([0.0, 1.0, 0.0]), (0.2, 0.8, 0.3)),
                (np.array([0.0, 0.0, 1.0]), (0.25, 0.45, 1.0)),
            ):
                self.add_arrow_actor(np.zeros(3), direction, axis_length, color, thickness=0.06)

            result = payload["result"]
            if result is not None:
                marker_radius = float(payload["marker_radius"])
                moments, axes = self.add_inertia_ellipsoid(result.center, result.mass, result.inertia)
                principal_axis_length = max(axis_length, axis_length * 0.64)
                for index, color in enumerate(((1.0, 0.25, 0.25), (0.25, 1.0, 0.35), (0.35, 0.55, 1.0))):
                    self.add_arrow_actor(result.center, axes[:, index], principal_axis_length, color, thickness=0.045)
                self.add_sphere_actor(result.center, marker_radius, (1.0, 0.15, 0.15))

            self._preview_payload_cache[link] = payload
            self.vtk_renderer.ResetCamera()
            self.vtk_widget.GetRenderWindow().Render()
            self.active_preview_link = link
            self.preview_status_var.set(f"3D??: {link}")
            self.log_line(f"3D_RENDER_DONE: {link}")
        except Exception as exc:  # noqa: BLE001 - preview should fail visibly, not stall
            self.active_preview_link = None
            self.preview_status_var.set(f"3D????: {exc}")
            self.log_line(f"3D_RENDER_FAIL: {link}: {exc}")

    def render_link_preview(self, link: str) -> None:
        payload = self._build_preview_payload(link)
        self.log_line(f'3D_WORKER: {link}')
        self._apply_preview_payload(link, payload)

    def build_joint_tab(self, parent: ttk.Frame) -> None:
        middle = ttk.PanedWindow(parent, orient=tk.HORIZONTAL)
        self.joint_middle = middle
        middle.pack(fill=tk.BOTH, expand=True)

        table_frame = ttk.Frame(middle)
        middle.add(table_frame, weight=4)
        columns = ("type", "lower", "upper", "effort", "velocity", "damping", "friction", "status")
        self.joint_tree = FrozenTreeTable(table_frame, "\u30b8\u30e7\u30a4\u30f3\u30c8", columns, name_width=190, right_width=260)
        self.joint_tree.heading("type", text="\u7a2e\u985e")
        self.joint_tree.heading("lower", text="\u4e0b\u9650(rad)")
        self.joint_tree.heading("upper", text="\u4e0a\u9650(rad)")
        self.joint_tree.heading("effort", text="\u30c8\u30eb\u30af/\u529b")
        self.joint_tree.heading("velocity", text="\u901f\u5ea6")
        self.joint_tree.heading("damping", text="\u6e1b\u8870")
        self.joint_tree.heading("friction", text="\u6469\u64e6")
        self.joint_tree.heading("status", text="\u72b6\u614b")
        for column in ("type", "lower", "upper", "effort", "velocity", "damping", "friction"):
            self.joint_tree.column(column, width=82, anchor=tk.E if column != "type" else tk.W, stretch=False)
        self.joint_tree.column("status", width=150, stretch=False)
        self.joint_tree.pack(fill=tk.BOTH, expand=True)
        self.joint_tree.set_selection_callback(self.on_joint_select)
        edit_frame = ttk.Frame(middle, padding=(10, 0, 0, 0))
        middle.add(edit_frame, weight=3)
        ttk.Label(edit_frame, text="選択ジョイント").pack(anchor=tk.W)
        self.selected_joint_var = tk.StringVar()
        ttk.Entry(edit_frame, textvariable=self.selected_joint_var, state="readonly").pack(fill=tk.X, pady=(2, 10))
        joint_type_row = ttk.Frame(edit_frame)
        joint_type_row.pack(fill=tk.X, pady=(0, 10))
        ttk.Label(joint_type_row, text="ジョイント型").pack(side=tk.LEFT)
        self.joint_type_var = tk.StringVar(value="revolute")
        self.joint_type_combo = ttk.Combobox(
            joint_type_row,
            textvariable=self.joint_type_var,
            values=JOINT_TYPE_OPTIONS,
            state="readonly",
            width=12,
        )
        self.joint_type_combo.pack(side=tk.LEFT, padx=(6, 0))
        self.joint_type_combo.bind("<<ComboboxSelected>>", self.on_joint_type_changed)
        unit_row = ttk.Frame(edit_frame)
        unit_row.pack(fill=tk.X, pady=(0, 10))
        ttk.Label(unit_row, text="角度単位").pack(side=tk.LEFT)
        self.joint_angle_unit_combo = ttk.Combobox(
            unit_row,
            textvariable=self.joint_angle_unit_var,
            values=JOINT_ANGLE_UNITS,
            state="readonly",
            width=6,
        )
        self.joint_angle_unit_combo.pack(side=tk.LEFT, padx=(6, 0))
        self.joint_angle_unit_combo.bind("<<ComboboxSelected>>", self.on_joint_angle_unit_changed)
        self.allow_joint_edit_var = tk.BooleanVar(value=False)
        self.allow_joint_edit_check = ttk.Checkbutton(
            edit_frame,
            text="このジョイントの編集を許可",
            variable=self.allow_joint_edit_var,
            command=self.update_joint_edit_state,
        )
        self.allow_joint_edit_check.pack(anchor=tk.W, pady=(0, 10))
        self.reverse_joint_axis_var = tk.BooleanVar(value=False)
        self.reverse_joint_axis_check = ttk.Checkbutton(
            edit_frame,
            text="回転方向を反転",
            variable=self.reverse_joint_axis_var,
            command=self.update_joint_edit_state,
        )
        self.reverse_joint_axis_check.pack(anchor=tk.W, pady=(0, 10))
        self.joint_edit_widgets: list[tk.Widget] = []
        self.joint_edit_widgets.append(self.joint_angle_unit_combo)
        self.joint_edit_widgets.append(self.joint_type_combo)
        self.joint_edit_widgets.append(self.reverse_joint_axis_check)
        self.joint_field_vars: dict[str, tk.StringVar] = {}
        for label, key in (
            ("下限", "lower"),
            ("上限", "upper"),
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
        ttk.Label(edit_frame, textvariable=self.joint_status_var, wraplength=180).pack(fill=tk.X, pady=(10, 0))
        self.update_joint_edit_state()

    def schedule_paned_layout(self) -> None:
        if self._layout_after_id is not None:
            try:
                self.after_cancel(self._layout_after_id)
            except tk.TclError:
                pass
        try:
            self._layout_after_id = self.after(80, self.apply_paned_layout)
        except tk.TclError:
            self._layout_after_id = None

    def on_root_configure(self, _event: object | None = None) -> None:
        self.schedule_paned_layout()

    def on_tab_changed(self, _event: object | None = None) -> None:
        self.schedule_paned_layout()

    def apply_paned_layout(self, attempt: int = 0) -> None:
        self._layout_after_id = None
        width = max(1, self.winfo_width())
        current_tab = str(self.notebook.select()) if hasattr(self, "notebook") else ""
        layout_key = (width, current_tab, self.winfo_height())
        if attempt == 0 and layout_key == self._last_paned_layout_key:
            return
        configs = (
            ("inertia_middle", (int(width * 0.37), int(width * 0.63))),
            ("preview_middle", (int(width * 0.73),)),
            ("joint_middle", (int(width * 0.37), int(width * 0.63))),
        )
        for name, positions in configs:
            pane = getattr(self, name, None)
            if pane is None or not hasattr(pane, "sashpos"):
                continue
            try:
                for index, position in enumerate(positions):
                    pane.sashpos(index, position)
            except tk.TclError:
                pass
        self._last_paned_layout_key = layout_key
        if attempt < 4:
            try:
                self.after(150, lambda: self.apply_paned_layout(attempt + 1))
            except tk.TclError:
                pass

    def log_line(self, message: str) -> None:
        try:
            with open('/tmp/urdf_xacro_tuner.log', 'a', encoding='utf-8') as handle:
                handle.write(message + "\n")
        except OSError:
            pass
        self.log.insert(tk.END, message + "\n")
        self.log.see(tk.END)

    def package_roots(self) -> list[Path]:
        value = self.package_root_var.get().strip()
        return [Path(value).expanduser().resolve()] if value else []

    def package_dir(self) -> Path | None:
        if self.urdf_path is None:
            return None
        for parent in [self.urdf_path.parent, *self.urdf_path.parents]:
            if (parent / "package.xml").exists():
                return parent
        return None

    def package_name(self) -> str | None:
        package_dir = self.package_dir()
        if package_dir is None:
            return None
        package_xml = package_dir / "package.xml"
        try:
            root = ET.parse(package_xml).getroot()
            name = root.findtext("name")
            if name and name.strip():
                return name.strip()
        except Exception:
            pass
        return package_dir.name

    def workspace_root(self) -> Path | None:
        roots = self.package_roots()
        if roots:
            return roots[0].parent
        package_dir = self.package_dir()
        if package_dir is not None:
            return package_dir.parent.parent
        return None

    def workspace_setup_script(self) -> Path | None:
        workspace_root = self.workspace_root()
        if workspace_root is None:
            return None
        setup = workspace_root / "install" / "setup.bash"
        return setup if setup.exists() else None

    def _ui_after(self, func, *args) -> None:
        try:
            if self.winfo_exists():
                self.after(0, lambda: func(*args))
        except tk.TclError:
            pass

    def schedule_workspace_rebuild(self, reason: str) -> None:
        if not self.auto_rebuild_var.get():
            return
        if self.urdf_path is None:
            return
        package_name = self.package_name()
        workspace_root = self.workspace_root()
        if package_name is None or workspace_root is None:
            self.log_line(f"{reason}: 再ビルド先を決められません")
            return
        if self._rebuild_thread is not None and self._rebuild_thread.is_alive():
            self.log_line(f"{reason}: 再ビルド中のため待機")
            return
        self.status_var.set(f"{reason}: {package_name} を再ビルド中")
        self.log_line(f"{reason}: 再ビルド開始 {package_name}")
        thread = threading.Thread(
            target=self._run_workspace_rebuild,
            args=(workspace_root, package_name, reason),
            daemon=True,
        )
        self._rebuild_thread = thread
        thread.start()

    def _run_workspace_rebuild(self, workspace_root: Path, package_name: str, reason: str) -> None:
        cmd = (
            f'source /opt/ros/humble/setup.bash && cd {shlex.quote(str(workspace_root))} '
            f'&& colcon build --packages-select {shlex.quote(package_name)}'
        )
        try:
            proc = subprocess.run(['bash', '-lc', cmd], check=False, capture_output=True, text=True)
            stdout = proc.stdout.strip()
            stderr = proc.stderr.strip()
            success = proc.returncode == 0
        except Exception as exc:
            stdout = ''
            stderr = str(exc)
            success = False

        def finish() -> None:
            if stdout:
                for line in stdout.splitlines():
                    self.log_line(f"[build] {line}")
            if stderr:
                for line in stderr.splitlines():
                    self.log_line(f"[build-err] {line}")
            if success:
                self.status_var.set(f"{reason}: 再ビルド完了 {package_name}")
                self.log_line(f"{reason}: 再ビルド完了 {package_name}")
            else:
                self.status_var.set(f"{reason}: 再ビルド失敗 {package_name}")
                self.log_line(f"{reason}: 再ビルド失敗 {package_name}")

            self._rebuild_thread = None
        self._ui_after(finish)

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
            self.current_link = None
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
        if self._preview_refresh_after_id is not None:
            try:
                self.after_cancel(self._preview_refresh_after_id)
            except tk.TclError:
                pass
            self._preview_refresh_after_id = None
        if self._preview_render_after_id is not None:
            try:
                self.after_cancel(self._preview_render_after_id)
            except tk.TclError:
                pass
            self._preview_render_after_id = None
        self._pending_preview_payload = None
        self._preview_requested_link = None
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
        self.current_link = None
        self.invalidate_preview_model_cache()
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
        if hasattr(self, "joint_angle_unit_var"):
            self.joint_angle_unit_var.set("rad")
            self._joint_angle_unit_last = "rad"
        if hasattr(self, "allow_joint_edit_var"):
            self.joint_type_var.set("revolute")
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
        self.current_link = None

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
        self.link_scan_path = path
        self._preview_payload_cache.clear()
        self.invalidate_preview_model_cache()
        self._preview_requested_link = None
        self.link_scan_expand_xacro = expand_xacro
        self.current_link = None
        if expand_xacro and looks_like_xacro(path):
            self.link_scan_mode = "xacro_source"
            self.link_scan_source_summaries = list(scan_xacro_source_links(path, self.package_roots()))
            self.link_scan_summaries = []
            self.link_scan_groups = []
            for summary in self.link_scan_source_summaries:
                current = "" if summary.existing_mass is None else format_float(summary.existing_mass)
                self.mass_values[summary.representative_name] = current
            self.render_link_tree()
            self.log_line(f"読込完了（xacro元定義）: {path}")
            return

        self.link_scan_mode = "urdf"
        summaries = list(scan_urdf(path, self.package_roots(), expand_xacro=expand_xacro))
        groups = list(fixed_joint_groups(path, self.package_roots(), expand_xacro=expand_xacro))
        self.link_scan_summaries = summaries
        self.link_scan_groups = groups
        self.link_scan_source_summaries = []
        for summary in summaries:
            current = "" if summary.existing_mass is None else format_float(summary.existing_mass)
            self.mass_values[summary.name] = current
        self.render_link_tree()
        mode = "xacro展開" if expand_xacro else "直接"
        self.log_line(f"読込完了（{mode}）: {path}")

    def render_link_tree(self) -> None:
        self.loading_selection = True
        try:
            if self.link_scan_path is None:
                return
            filter_text = self.link_filter_var.get().strip().lower()
            selected_link = self.selected_link()
            self.tree.delete(*self.tree.get_children())
            self.tree_link_by_item.clear()
            self.tree_item_by_link.clear()
            self.tree_display_by_item.clear()
            self.tree_group_items.clear()
            self.selected_var.set("")
            self.mass_var.set("")

            def matches(*parts: str) -> bool:
                if not filter_text:
                    return True
                return any(filter_text in part.lower() for part in parts if part)

            restored = False
            if self.link_scan_mode == "xacro_source":
                source_group_items: dict[Path, str] = {}
                for index, summary in enumerate(self.link_scan_source_summaries):
                    source_file = summary.source_file
                    source_label = source_file.name
                    current = "" if summary.existing_mass is None else format_float(summary.existing_mass)
                    instances = ", ".join(summary.instance_names[:2])
                    if len(summary.instance_names) > 2:
                        instances += f", ... ({len(summary.instance_names)})"
                    status = f"instances: {instances}" if instances else "OK"
                    if summary.warnings:
                        status = f"{status}; {'; '.join(summary.warnings)}"
                    if not matches(summary.representative_name, summary.source_link_name, source_label, status):
                        continue
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
                    display_value = self.mass_values.get(summary.representative_name, current)
                    item_id = f"source::{index}"
                    self.tree_link_by_item[item_id] = summary.representative_name
                    self.tree_item_by_link[summary.representative_name] = item_id
                    self.tree_display_by_item[item_id] = f"{source_label}:{summary.source_link_name}"
                    self.tree.insert(
                        group_item,
                        tk.END,
                        iid=item_id,
                        text=summary.source_link_name,
                        values=(display_value, current, str(summary.mesh_count), status),
                    )
            else:
                summary_by_name = {summary.name: summary for summary in self.link_scan_summaries}
                grouped_links: set[str] = set()
                for group in self.link_scan_groups:
                    if len(group.link_names) <= 1:
                        continue
                    group_status = "; ".join(group.warnings) if group.warnings else "OK"
                    group_matches = matches(group.label, group_status)
                    matched_children: list[str] = []
                    for link_name in group.link_names:
                        summary = summary_by_name.get(link_name)
                        if summary is None:
                            continue
                        child_status = "; ".join(summary.warnings) if summary.warnings else "OK"
                        if group_matches or matches(summary.name, child_status):
                            matched_children.append(link_name)
                    if not matched_children:
                        continue
                    group_mass = "" if group.existing_mass is None else format_float(group.existing_mass)
                    group_item = f"__group__{group.group_id}"
                    self.tree_group_items.add(group_item)
                    self.tree.insert(
                        "",
                        tk.END,
                        iid=group_item,
                        text=group.label,
                        values=("", group_mass, str(group.mesh_count), group_status),
                        open=len(matched_children) <= 8,
                    )
                    for link_name in matched_children:
                        summary = summary_by_name.get(link_name)
                        if summary is None:
                            continue
                        grouped_links.add(link_name)
                        current = "" if summary.existing_mass is None else format_float(summary.existing_mass)
                        status = "; ".join(summary.warnings) if summary.warnings else "OK"
                        display_value = self.mass_values.get(summary.name, current)
                        item_id = f"link::{summary.name}"
                        self.tree_link_by_item[item_id] = summary.name
                        self.tree_item_by_link[summary.name] = item_id
                        self.tree.insert(
                            group_item,
                            tk.END,
                            iid=item_id,
                            text=summary.name,
                            values=(display_value, current, str(summary.mesh_count), status),
                        )
                for summary in self.link_scan_summaries:
                    if summary.name in grouped_links:
                        continue
                    current = "" if summary.existing_mass is None else format_float(summary.existing_mass)
                    status = "; ".join(summary.warnings) if summary.warnings else "OK"
                    if not matches(summary.name, status):
                        continue
                    display_value = self.mass_values.get(summary.name, current)
                    item_id = f"link::{summary.name}"
                    self.tree_link_by_item[item_id] = summary.name
                    self.tree_item_by_link[summary.name] = item_id
                    self.tree.insert(
                        "",
                        tk.END,
                        iid=item_id,
                        text=summary.name,
                        values=(display_value, current, str(summary.mesh_count), status),
                    )

            if selected_link:
                item_id = self.tree_item_by_link.get(selected_link)
                if item_id and self.tree.exists(item_id):
                    self.tree.selection_set(item_id)
                    self.tree.see(item_id)
                    self.on_select()
                    restored = True
            if not restored and self.current_link is not None:
                self.selected_var.set(self.current_link)
                self.loading_selection = True
                self.mass_var.set(self.mass_values.get(self.current_link, ""))
                self.loading_selection = False
                self.status_var.set(f"\u30d5\u30a3\u30eb\u30bf\u5916: {self.current_link}")

        finally:
            self.loading_selection = False
    def on_link_filter_changed(self, *_args: object) -> None:
        if self._link_filter_after_id is not None:
            try:
                self.after_cancel(self._link_filter_after_id)
            except tk.TclError:
                pass
        self._link_filter_after_id = self.after(120, self.refresh_link_tree_from_filter)

    def refresh_link_tree_from_filter(self) -> None:
        self._link_filter_after_id = None
        if self.link_scan_path is None:
            return
        self.render_link_tree()

    def clear_link_filter(self) -> None:
        if self.link_filter_var.get():
            self.link_filter_var.set("")

    def filter_to_selected_link(self) -> None:
        link = self.selected_link()
        if link is None:
            return
        self.link_filter_var.set(link)

    def scan_joint_path(self, path: Path, expand_xacro: bool) -> None:
        self.joint_tree.delete(*self.joint_tree.get_children())
        self.joint_by_item.clear()
        self.joint_item_by_name.clear()
        self.joint_group_items.clear()
        self.joint_requires_confirmation.clear()
        self.selected_joint_var.set("")
        for var in self.joint_field_vars.values():
            var.set("")
            self.joint_type_var.set("revolute")
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
            self.current_link = None
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
        if selection:
            link = self.tree_link_by_item.get(str(selection[0]))
            if link is not None:
                self.current_link = link
                return link
        return self.current_link

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

    def joint_angle_unit(self) -> str:
        value = self.joint_angle_unit_var.get().strip().lower()
        return value if value in JOINT_ANGLE_UNITS else "rad"

    def joint_uses_angle(self, joint_type: str | None) -> bool:
        return (joint_type or "").strip().lower() in ANGLE_JOINT_TYPES

    def convert_joint_angle_value(self, value: str, from_unit: str, to_unit: str) -> str:
        value = value.strip()
        if not value:
            return value
        try:
            numeric = float(value)
        except ValueError:
            return value
        from_unit = from_unit.strip().lower()
        to_unit = to_unit.strip().lower()
        if from_unit == to_unit:
            return format_float(numeric)
        if from_unit == "deg" and to_unit == "rad":
            numeric = float(np.deg2rad(numeric))
        elif from_unit == "rad" and to_unit == "deg":
            numeric = float(np.rad2deg(numeric))
        else:
            return format_float(numeric)
        return format_float(numeric)

    def populate_joint_fields(self, item_id: str) -> None:
        joint_type = self.joint_tree.set(item_id, "type")
        current_unit = self.joint_angle_unit()
        self.joint_type_var.set(joint_type or "revolute")
        self.reverse_joint_axis_var.set(False)
        for key in ("lower", "upper", "effort", "velocity", "damping", "friction"):
            value = self.joint_tree.set(item_id, key)
            if key in ("lower", "upper") and self.joint_uses_angle(joint_type):
                value = self.convert_joint_angle_value(value, "rad", current_unit)
            self.joint_field_vars[key].set(value)
        self._joint_angle_unit_last = current_unit

    def on_joint_angle_unit_changed(self, _event: object | None = None) -> None:
        joint = self.selected_joint()
        current_unit = self.joint_angle_unit()
        if joint is None:
            self._joint_angle_unit_last = current_unit
            return
        item_id = self.joint_item_for_name(joint)
        if item_id is None:
            self._joint_angle_unit_last = current_unit
            return
        joint_type = self.joint_tree.set(item_id, "type")
        previous_unit = self._joint_angle_unit_last or current_unit
        if self.joint_uses_angle(joint_type) and previous_unit != current_unit:
            for key in ("lower", "upper"):
                self.joint_field_vars[key].set(
                    self.convert_joint_angle_value(self.joint_field_vars[key].get(), previous_unit, current_unit)
                )
        self._joint_angle_unit_last = current_unit

    def on_joint_type_changed(self, _event: object | None = None) -> None:
        joint = self.selected_joint()
        if joint is None:
            return
        if not self.joint_uses_angle(self.joint_type_var.get()):
            self.reverse_joint_axis_var.set(False)
        self.update_joint_edit_state()
    def on_joint_select(self, _event: object | None = None) -> None:
        joint = self.selected_joint()
        if joint is None:
            self.selected_joint_var.set("")
            for var in self.joint_field_vars.values():
                var.set("")
            self.joint_type_var.set("revolute")
            self.reverse_joint_axis_var.set(False)
            self.allow_joint_edit_var.set(False)
            self.update_joint_edit_state()
            return
        item_id = self.joint_item_for_name(joint)
        if item_id is None:
            return
        self.selected_joint_var.set(joint)
        self.populate_joint_fields(item_id)
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
        reverse_enabled = enabled and self.joint_uses_angle(self.joint_type_var.get())
        for widget in self.joint_edit_widgets:
            widget.configure(state=tk.NORMAL if enabled else tk.DISABLED)
        if hasattr(self, "joint_type_combo"): 
            self.joint_type_combo.configure(state="readonly" if enabled else tk.DISABLED)
        if hasattr(self, "joint_angle_unit_combo"): 
            self.joint_angle_unit_combo.configure(state="readonly" if enabled else tk.DISABLED)
        if hasattr(self, "reverse_joint_axis_check"):
            self.reverse_joint_axis_check.configure(state=tk.NORMAL if reverse_enabled else tk.DISABLED)
        if not reverse_enabled:
            self.reverse_joint_axis_var.set(False)
        if not hasattr(self, "joint_status_var"):
            return
        if not has_joint:
            self.joint_status_var.set("")
        elif self.joint_requires_confirmation.get(joint, False) and not self.allow_joint_edit_var.get():
            self.joint_status_var.set("mimic/補助ジョイントの可能性があるため、編集許可が必要です。")
        elif not self.allow_joint_edit_var.get():
            self.joint_status_var.set("編集ロック中です。")

    def collect_joint_values(self) -> dict[str, str]:
        values = {"type": self.joint_type_var.get().strip(), **{key: var.get().strip() for key, var in self.joint_field_vars.items()}}
        joint = self.selected_joint()
        if joint is None:
            return values
        item_id = self.joint_item_for_name(joint)
        if item_id is None:
            return values
        joint_type = values.get("type") or self.joint_tree.set(item_id, "type")
        if self.joint_uses_angle(joint_type):
            unit = self.joint_angle_unit()
            for key in ("lower", "upper"):
                values[key] = self.convert_joint_angle_value(values[key], unit, "rad")
            if self.reverse_joint_axis_var.get():
                values["reverse_axis"] = "true"
        return values

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
        self._preview_payload_cache.clear()
        self.schedule_workspace_rebuild("ジョイント反映")
        item_id = self.joint_item_by_name.get(joint)
        if item_id and self.joint_tree.exists(item_id):
            self.joint_tree.selection_set(item_id)
            self.joint_tree.see(item_id)
            self.on_joint_select()

    def clear_joint_fields(self) -> None:
        for var in self.joint_field_vars.values():
            var.set("")
        self.reverse_joint_axis_var.set(False)

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
        selection = self.tree.selection()
        selection_iid = str(selection[0]) if selection else ""
        self.loading_selection = True
        display = self.tree_display_by_item.get(selection_iid, link) if selection else link
        self.selected_var.set(display if display == link else f"{display} -> {link}")
        self.mass_var.set(self.mass_values.get(link, ""))
        self.current_link = link
        self.loading_selection = False
        self.log_line(f'??: {link}')
        self.refresh_preview_for_selection(link)

    def refresh_preview_for_selection(self, link: str, force: bool = False) -> None:
        if not hasattr(self, 'preview_status_var') or not VTK_AVAILABLE:
            return
        after_id = getattr(self, '_preview_refresh_after_id', None)
        if not force and link == self.active_preview_link and after_id is None and self._pending_preview_payload is None and link in self._preview_payload_cache:
            return
        if not force and getattr(self, '_preview_requested_link', None) == link and (after_id is not None or self._pending_preview_payload is not None):
            return
        if after_id is not None:
            try:
                self.after_cancel(after_id)
            except tk.TclError:
                pass
            self._preview_refresh_after_id = None
        self._preview_requested_link = link
        self.active_preview_link = link
        cached = self._preview_payload_cache.get(link)
        if cached is not None:
            self.log_line(f'3D_CACHE_HIT: {link}')
            self.preview_status_var.set(f'3D?????: {link}')
            self.set_preview_info(f'???: {link}\n3D?????...')
            self._pending_preview_payload = (link, cached)
            if self._preview_render_after_id is not None:
                try:
                    self.after_cancel(self._preview_render_after_id)
                except tk.TclError:
                    pass
            self._preview_render_after_id = self.after_idle(lambda: self._finalize_preview_render(link))
            return
        self.preview_status_var.set(f'3D?????: {link}')
        self.log_line(f'3D_APPLY: {link}')
        self.set_preview_info(f'???: {link}\n3D?????...')
        token = self._preview_render_token + 1
        self._preview_render_token = token
        self._preview_refresh_after_id = self.after(30, lambda: self._start_preview_refresh(link, token))

    def _start_preview_refresh(self, link: str, token: int) -> None:
        self._preview_refresh_after_id = None
        if token != self._preview_render_token or self.selected_link() != link:
            return
        worker = threading.Thread(target=self._preview_refresh_worker, args=(link, token), daemon=True)
        self._preview_worker = worker
        self.log_line(f'3D_WORKER: {link}')
        worker.start()

    def _preview_refresh_worker(self, link: str, token: int) -> None:
        try:
            payload = self._build_preview_payload(link)
        except Exception as exc:  # noqa: BLE001 - show in UI instead of freezing
            self._ui_after(self._finish_preview_refresh_error, link, token, exc)
            return
        self._ui_after(self._finish_preview_refresh, link, token, payload)

    def _finish_preview_refresh_error(self, link: str, token: int, exc: Exception) -> None:
        if token != self._preview_render_token or self.selected_link() != link:
            return
        self._preview_requested_link = None
        self.active_preview_link = None
        self.preview_status_var.set(f'3D????: {exc}')
        self.set_preview_info(f'???: {link}\n3D????: {exc}')
        self.log_line(f'3D_WORKER_FAIL: {link}: {exc}')

    def _finish_preview_refresh(self, link: str, token: int, payload: dict[str, object]) -> None:
        if token != self._preview_render_token or self.selected_link() != link:
            return
        self._apply_preview_payload(link, payload)

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
        self._preview_payload_cache.clear()
        self._preview_requested_link = None
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
        self.schedule_workspace_rebuild("質量反映")

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
            self.mass_var.set(self.mass_values.get(link, ""))
            self.set_mass()
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

    def apply_mesh_visibility_to_target(self, visibility_updates: dict[str, bool], backup: bool) -> tuple[Path, object]:
        target_path = self.apply_target_path()
        package_roots = self.package_roots()
        if self.expanded_mode():
            report = apply_mesh_visibility_to_xacro_sources(
                self.urdf_path,
                visibility_updates,
                package_roots,
                backup=backup,
            )
        else:
            report = apply_mesh_visibility_to_urdf(target_path, visibility_updates, backup=backup)
        self.last_applied_path = target_path
        return target_path, report

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
            self.invalidate_preview_model_cache()
            self.log_apply_report(target_path, report, check_messages)
            if self.expanded_mode():
                self.invalidate_preview_model_cache()
                self.scan_path(self.urdf_path, expand_xacro=True)
            else:
                self.invalidate_preview_model_cache()
                self.scan_path(target_path, expand_xacro=False)
            self.schedule_workspace_rebuild("質量反映")
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
            show_mesh=self.preview_mesh_enabled_for_link(link),
        )
        return preview_urdf

    def launch_rviz_preview(self, link: str) -> None:
        preview_urdf = self.write_preview_urdf(link)
        self.stop_rviz_preview()
        setup_script = self.workspace_setup_script()
        command_parts = ["source /opt/ros/humble/setup.bash"]
        if setup_script is not None:
            command_parts.append(f"source {shlex.quote(str(setup_script))}")
        command_parts.append(
            f"ros2 launch urdf_xacro_tuner preview_link.launch.py urdf:={shlex.quote(str(preview_urdf))}"
        )
        command = " && ".join(command_parts)
        self.rviz_process = subprocess.Popen(
            ["bash", "-lc", command],
            start_new_session=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        self.active_preview_urdf = preview_urdf

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
            self.start_or_refresh_preview(link)
        except Exception as exc:  # noqa: BLE001 - shown in GUI
            messagebox.showerror("3D確認失敗", str(exc))
            return

    def start_or_refresh_preview(self, link: str) -> None:
        self.sync_preview_mesh_toggle(link)
        self.refresh_preview_for_selection(link, force=True)
        try:
            self.launch_rviz_preview(link)
        except Exception as exc:  # noqa: BLE001 - keep the editor usable
            self.log_line(f"RViz起動失敗: {exc}")
        self.status_var.set(f"3D確認: {link}")
        self.log_line(f"3D確認: {link}")

    def refresh_active_preview_if_needed(self, link: str) -> None:
        if self.active_preview_link != link:
            return
        try:
            self.sync_preview_mesh_toggle(link)
            self.refresh_preview_for_selection(link, force=True)
            self.launch_rviz_preview(link)
            self.log_line(f"3D確認: {link}")
        except Exception as exc:  # noqa: BLE001 - keep the editor usable
            self.preview_status_var.set(f"3D確認失敗: {exc}")


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('urdf', nargs='?')
    parser.add_argument('--urdf', dest='urdf_option')
    parser.add_argument('--package-root', dest='package_root')
    args, _unknown = parser.parse_known_args(sys.argv[1:] if argv is None else argv)

    initial_path_text = args.urdf_option or args.urdf
    app = InertiaEditor()
    if initial_path_text:
        initial_path = Path(initial_path_text).expanduser().resolve()
        app.urdf_var.set(str(initial_path))
        package_root_text = args.package_root
        if package_root_text:
            app.package_root_var.set(str(Path(package_root_text).expanduser().resolve()))
        else:
            guessed = guess_package_root(initial_path)
            if guessed is not None:
                app.package_root_var.set(str(guessed))
        app.after_idle(app.scan)
    try:
        app.mainloop()
    except KeyboardInterrupt:
        app.request_close("received KeyboardInterrupt")


if __name__ == "__main__":
    main()
