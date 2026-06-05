#!/usr/bin/env python3
"""ROS-free URDF inertial updater based on STL mesh geometry."""

from __future__ import annotations

import argparse
import base64
import ast
import copy
import csv
import json
import math
import re
import shutil
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable
import xml.etree.ElementTree as ET

import numpy as np
import trimesh


EPS = 1.0e-12
XACRO_NS = "http://www.ros.org/wiki/xacro"
MESH_VISIBILITY_COMMENT_PREFIX = "urdf_xacro_tuner_hidden_mesh"


@dataclass
class MeshRef:
    filename: str
    resolved_path: Path | None
    scale: np.ndarray
    xyz: np.ndarray
    rpy: np.ndarray
    source: str


@dataclass
class LinkSummary:
    name: str
    existing_mass: float | None
    mesh_count: int
    mesh_files: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)


@dataclass
class XacroSourceLinkSummary:
    source_file: Path
    source_link_name: str
    representative_name: str
    instance_names: list[str]
    existing_mass: float | None
    mesh_count: int
    warnings: list[str] = field(default_factory=list)


@dataclass
class JointSummary:
    name: str
    joint_type: str
    parent: str
    child: str
    axis: str
    lower: str
    upper: str
    effort: str
    velocity: str
    damping: str
    friction: str
    requires_confirmation: bool = False
    warnings: list[str] = field(default_factory=list)


@dataclass
class XacroSourceJointSummary:
    source_file: Path
    source_joint_name: str
    representative_name: str
    instance_names: list[str]
    joint_type: str
    parent: str
    child: str
    axis: str
    lower: str
    upper: str
    effort: str
    velocity: str
    damping: str
    friction: str
    requires_confirmation: bool = False
    warnings: list[str] = field(default_factory=list)


@dataclass
class JointApplyReport:
    updated: list[str] = field(default_factory=list)
    skipped: dict[str, list[str]] = field(default_factory=dict)
    backup_path: Path | None = None
    backup_paths: list[Path] = field(default_factory=list)
    output_path: Path | None = None


@dataclass
class FixedJointGroup:
    group_id: str
    label: str
    link_names: list[str]
    root_link: str
    existing_mass: float | None
    mesh_count: int
    warnings: list[str] = field(default_factory=list)


@dataclass
class InertialResult:
    link_name: str
    mass: float
    center: np.ndarray
    inertia: np.ndarray
    mesh_count: int
    warnings: list[str] = field(default_factory=list)


@dataclass
class ApplyReport:
    updated: list[InertialResult] = field(default_factory=list)
    skipped: dict[str, list[str]] = field(default_factory=dict)
    backup_path: Path | None = None
    backup_paths: list[Path] = field(default_factory=list)
    output_path: Path | None = None


@dataclass
class MacroDef:
    params: list[str]
    body: list[ET.Element]
    source_path: Path | None = None
    source_tree: ET.ElementTree | None = None


def parse_vector(text: str | None, default: Iterable[float]) -> np.ndarray:
    if not text:
        return np.array(list(default), dtype=float)
    parts = text.split()
    if len(parts) != 3:
        raise ValueError(f"expected 3 values, got: {text!r}")
    return np.array([float(part) for part in parts], dtype=float)


def format_float(value: float) -> str:
    if abs(value) < 5.0e-15:
        value = 0.0
    return f"{value:.9g}"


def render_xacro_element(element: ET.Element, env: dict[str, object] | None) -> ET.Element:
    rendered = copy.deepcopy(element)
    if env is None:
        return rendered

    for node in rendered.iter():
        for key, value in list(node.attrib.items()):
            if isinstance(value, str):
                substituted = substitute_xacro_text(value, env)
                if substituted is not None:
                    node.set(key, substituted)
        if isinstance(node.text, str):
            substituted_text = substitute_xacro_text(node.text, env)
            if substituted_text is not None:
                node.text = substituted_text
    return rendered


def encode_hidden_mesh_comment(source: str, element: ET.Element, env: dict[str, object] | None = None) -> ET.Element:
    payload = base64.b64encode(ET.tostring(render_xacro_element(element, env), encoding="utf-8")).decode("ascii")
    return ET.Comment(f"{MESH_VISIBILITY_COMMENT_PREFIX}|{source}|{payload}")


def decode_hidden_mesh_comment(node: ET.Element) -> tuple[str, ET.Element] | None:
    if isinstance(node.tag, str):
        return None
    text = (node.text or "").strip()
    prefix = f"{MESH_VISIBILITY_COMMENT_PREFIX}|"
    if not text.startswith(prefix):
        return None
    parts = text.split("|", 2)
    if len(parts) != 3:
        return None
    source = parts[1]
    try:
        xml = base64.b64decode(parts[2].encode("ascii")).decode("utf-8")
        element = ET.fromstring(xml)
    except Exception:
        return None
    return source, element


def iter_link_mesh_nodes(link: ET.Element, source: str) -> Iterable[ET.Element]:
    direct_nodes = [node for node in direct_children(link, source)]
    if direct_nodes:
        yield from direct_nodes
        return
    for child in list(link):
        decoded = decode_hidden_mesh_comment(child)
        if decoded is None:
            continue
        comment_source, element = decoded
        if comment_source == source:
            yield element


def set_link_mesh_visibility(link: ET.Element, show_mesh: bool, env: dict[str, object] | None = None) -> None:
    if show_mesh:
        for index, child in enumerate(list(link)):
            decoded = decode_hidden_mesh_comment(child)
            if decoded is None:
                continue
            source, element = decoded
            if local_name(element.tag) not in {"visual", "collision"}:
                continue
            link.remove(child)
            link.insert(index, element)
        return

    for source in ("visual", "collision"):
        for node in list(direct_children(link, source)):
            index = list(link).index(node)
            link.remove(node)
            link.insert(index, encode_hidden_mesh_comment(source, node, env))


def link_mesh_visible(link: ET.Element) -> bool:
    visible = False
    hidden = False
    for child in list(link):
        if local_name(child.tag) in {"visual", "collision"}:
            visible = True
            continue
        decoded = decode_hidden_mesh_comment(child)
        if decoded is None:
            continue
        _source, element = decoded
        if local_name(element.tag) in {"visual", "collision"}:
            hidden = True
    if visible:
        return True
    if hidden:
        return False
    return True


def rotation_matrix_from_rpy(rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def mesh_transform(scale: np.ndarray, xyz: np.ndarray, rpy: np.ndarray) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, :3] = rotation_matrix_from_rpy(rpy) @ np.diag(scale)
    transform[:3, 3] = xyz
    return transform


def local_name(tag: object) -> str:
    if not isinstance(tag, str):
        return ""
    if tag.startswith("{"):
        return tag.rsplit("}", 1)[-1]
    return tag


def namespace_uri(tag: object) -> str:
    if not isinstance(tag, str):
        return ""
    if tag.startswith("{"):
        return tag[1:].split("}", 1)[0]
    return ""


def is_xacro_element(element: ET.Element) -> bool:
    return namespace_uri(element.tag) == XACRO_NS or str(element.tag).startswith("xacro:")


def direct_children(element: ET.Element, name: str) -> list[ET.Element]:
    return [child for child in list(element) if local_name(child.tag) == name]


def first_child(element: ET.Element | None, name: str) -> ET.Element | None:
    if element is None:
        return None
    for child in list(element):
        if local_name(child.tag) == name:
            return child
    return None


def child_attr(element: ET.Element | None, child_name: str, attr_name: str) -> str:
    child = first_child(element, child_name)
    if child is None:
        return ""
    return child.get(attr_name) or ""


def joint_axis_vector(joint: ET.Element) -> np.ndarray:
    axis = child_attr(joint, "axis", "xyz").strip()
    if not axis:
        return np.array([1.0, 0.0, 0.0], dtype=float)
    try:
        return parse_vector(axis, (1.0, 0.0, 0.0))
    except ValueError:
        return np.array([1.0, 0.0, 0.0], dtype=float)


def set_joint_axis_vector(joint: ET.Element, axis: np.ndarray) -> None:
    axis_child = first_child(joint, "axis")
    if axis_child is None:
        axis_child = ET.SubElement(joint, "axis")
    axis_child.set("xyz", " ".join(format_float(float(value)) for value in axis))


def direct_link_elements(root: ET.Element) -> list[ET.Element]:
    return direct_children(root, "link")


def direct_joint_elements(root: ET.Element) -> list[ET.Element]:
    return direct_children(root, "joint")


def register_document_namespaces(path: Path) -> None:
    seen: set[tuple[str, str]] = set()
    for _event, item in ET.iterparse(path, events=("start-ns",)):
        prefix, uri = item
        if (prefix, uri) in seen:
            continue
        seen.add((prefix, uri))
        ET.register_namespace(prefix, uri)


def parse_urdf(path: Path) -> ET.ElementTree:
    register_document_namespaces(path)
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    return ET.parse(path, parser=parser)


def looks_like_xacro(path: Path) -> bool:
    if path.suffix.lower() == ".xacro" or path.name.lower().endswith(".urdf.xacro"):
        return True
    try:
        sample = path.read_text(encoding="utf-8", errors="ignore")[:4096]
    except OSError:
        return False
    return "xacro:" in sample or XACRO_NS in sample


def parse_boolish_number(value: object) -> object:
    if not isinstance(value, str):
        return value
    try:
        return float(value)
    except ValueError:
        return value


def safe_eval_expr(expr: str, env: dict[str, object]) -> object:
    expr = expr.strip()
    if re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", expr):
        return env.get(expr, "")

    converted_env = {name: parse_boolish_number(value) for name, value in env.items()}
    tree = ast.parse(expr, mode="eval")
    allowed_nodes = (
        ast.Expression,
        ast.BinOp,
        ast.UnaryOp,
        ast.Add,
        ast.Sub,
        ast.Mult,
        ast.Div,
        ast.Pow,
        ast.USub,
        ast.UAdd,
        ast.Constant,
        ast.Name,
        ast.Load,
    )
    for node in ast.walk(tree):
        if not isinstance(node, allowed_nodes):
            raise ValueError(f"unsupported xacro expression: {expr}")
        if isinstance(node, ast.Name) and node.id not in converted_env:
            raise ValueError(f"unknown xacro variable: {node.id}")
    return eval(compile(tree, "<xacro-expr>", "eval"), {"__builtins__": {}}, converted_env)


def substitute_xacro_text(value: str | None, env: dict[str, object]) -> str | None:
    if value is None or "${" not in value:
        return value

    def repl(match: re.Match[str]) -> str:
        result = safe_eval_expr(match.group(1), env)
        if isinstance(result, (int, float)):
            return format_float(float(result))
        return str(result)

    return re.sub(r"\$\{([^}]*)\}", repl, value)


def resolve_xacro_include(
    filename: str,
    xacro_path: Path,
    package_roots: Iterable[Path] | None = None,
) -> Path | None:
    substituted = filename
    if substituted.startswith("package://") or substituted.startswith("$(find "):
        return resolve_mesh_filename(substituted, xacro_path, package_roots)
    path = Path(substituted)
    if path.is_absolute():
        return path if path.exists() else None
    candidate = (xacro_path.parent / path).resolve()
    return candidate if candidate.exists() else None


def parse_macro_params(params: str | None) -> list[str]:
    if not params:
        return []
    parsed: list[str] = []
    for token in params.split():
        name = token.split(":=", 1)[0].strip("*")
        if name:
            parsed.append(name)
    return parsed


def expand_xacro_element(
    element: ET.Element,
    env: dict[str, object],
    macros: dict[str, MacroDef],
    package_roots: Iterable[Path] | None,
) -> list[ET.Element]:
    name = local_name(element.tag)
    if is_xacro_element(element):
        if name in {"include", "macro", "property"}:
            return []
        if name in macros:
            macro = macros[name]
            child_env = dict(env)
            for param in macro.params:
                raw_value = element.get(param)
                child_env[param] = substitute_xacro_text(raw_value, env) if raw_value is not None else ""
            expanded: list[ET.Element] = []
            for child in macro.body:
                expanded.extend(expand_xacro_element(copy.deepcopy(child), child_env, macros, package_roots))
            return expanded
        return []

    new_element = ET.Element(
        element.tag,
        {key: substitute_xacro_text(value, env) or "" for key, value in element.attrib.items()},
    )
    new_element.text = substitute_xacro_text(element.text, env)
    new_element.tail = substitute_xacro_text(element.tail, env)
    for child in list(element):
        for expanded_child in expand_xacro_element(child, env, macros, package_roots):
            new_element.append(expanded_child)
    return [new_element]


def collect_xacro_definitions(
    path: Path,
    macros: dict[str, MacroDef],
    properties: dict[str, object],
    package_roots: Iterable[Path] | None,
    visited: set[Path],
) -> list[ET.Element]:
    resolved = path.resolve()
    if resolved in visited:
        return []
    visited.add(resolved)
    tree = parse_urdf(resolved)
    root = tree.getroot()
    passthrough: list[ET.Element] = []

    for child in list(root):
        name = local_name(child.tag)
        if is_xacro_element(child) and name == "include":
            include_name = child.get("filename")
            if include_name:
                include_path = resolve_xacro_include(include_name, resolved, package_roots)
                if include_path is not None:
                    passthrough.extend(
                        collect_xacro_definitions(include_path, macros, properties, package_roots, visited)
                    )
            continue
        if is_xacro_element(child) and name == "property":
            prop_name = child.get("name")
            prop_value = child.get("value")
            if prop_name:
                properties[prop_name] = substitute_xacro_text(prop_value, properties) or ""
            continue
        if is_xacro_element(child) and name == "macro":
            macro_name = child.get("name")
            if macro_name:
                macros[macro_name] = MacroDef(
                    parse_macro_params(child.get("params")),
                    list(child),
                    resolved,
                    tree,
                )
            continue
        passthrough.append(child)
    return passthrough


def expand_xacro_to_tree(
    xacro_path: Path,
    package_roots: Iterable[Path] | None = None,
) -> ET.ElementTree:
    macros: dict[str, MacroDef] = {}
    properties: dict[str, object] = {}
    top_level = collect_xacro_definitions(xacro_path, macros, properties, package_roots, set())
    original = parse_urdf(xacro_path).getroot()
    robot = ET.Element("robot", {key: value for key, value in original.attrib.items() if not key.startswith("xmlns")})
    if "name" not in robot.attrib:
        robot.set("name", xacro_path.stem)

    for child in top_level:
        for expanded in expand_xacro_element(child, properties, macros, package_roots):
            robot.append(expanded)
    return ET.ElementTree(robot)


def write_expanded_xacro(
    xacro_path: Path,
    output_path: Path,
    package_roots: Iterable[Path] | None = None,
) -> None:
    tree = expand_xacro_to_tree(xacro_path, package_roots)
    ET.indent(tree, space="  ")
    tree.write(output_path, encoding="utf-8", xml_declaration=True)


def find_existing_mass(link: ET.Element) -> float | None:
    mass = first_child(first_child(link, "inertial"), "mass")
    if mass is None:
        return None
    value = mass.get("value")
    if value is None:
        return None
    try:
        return float(value)
    except ValueError:
        return None


def guess_package_root(urdf_path: Path) -> Path | None:
    for parent in [urdf_path.parent, *urdf_path.parents]:
        if (parent / "package.xml").exists():
            return parent.parent
    parts = list(urdf_path.resolve().parts)
    if "src" in parts:
        idx = len(parts) - 1 - parts[::-1].index("src")
        return Path(*parts[: idx + 1])
    return None


def resolve_mesh_filename(
    filename: str,
    urdf_path: Path,
    package_roots: Iterable[Path] | None = None,
) -> Path | None:
    if filename.startswith("file://"):
        filename = filename[len("file://") :]

    if filename.startswith("$(find "):
        end = filename.find(")")
        if end > 0:
            package = filename[len("$(find ") : end].strip()
            rel = filename[end + 1 :].lstrip("/")
            roots = list(package_roots or [])
            guessed = guess_package_root(urdf_path)
            if guessed is not None:
                roots.append(guessed)
            for root in roots:
                candidate = root / package / rel
                if candidate.exists():
                    return candidate
            return None

    if filename.startswith("package://"):
        rest = filename[len("package://") :]
        pieces = rest.split("/", 1)
        if len(pieces) != 2:
            return None
        package, rel = pieces
        roots = list(package_roots or [])
        guessed = guess_package_root(urdf_path)
        if guessed is not None:
            roots.append(guessed)
        for root in roots:
            candidate = root / package / rel
            if candidate.exists():
                return candidate
        return None

    path = Path(filename)
    if path.is_absolute():
        return path if path.exists() else None

    candidate = (urdf_path.parent / path).resolve()
    if candidate.exists():
        return candidate
    return None


def extract_mesh_refs(
    link: ET.Element,
    urdf_path: Path,
    package_roots: Iterable[Path] | None = None,
) -> list[MeshRef]:
    refs: list[MeshRef] = []
    for source in ("collision", "visual"):
        for node in iter_link_mesh_nodes(link, source):
            mesh = first_child(first_child(node, "geometry"), "mesh")
            if mesh is None:
                continue
            filename = mesh.get("filename")
            if not filename:
                continue
            origin = first_child(node, "origin")
            xyz = parse_vector(origin.get("xyz") if origin is not None else None, (0.0, 0.0, 0.0))
            rpy = parse_vector(origin.get("rpy") if origin is not None else None, (0.0, 0.0, 0.0))
            scale = parse_vector(mesh.get("scale"), (1.0, 1.0, 1.0))
            refs.append(
                MeshRef(
                    filename=filename,
                    resolved_path=resolve_mesh_filename(filename, urdf_path, package_roots),
                    scale=scale,
                    xyz=xyz,
                    rpy=rpy,
                    source=source,
                )
            )
        if refs:
            break
    return refs


def scan_urdf(
    urdf_path: Path,
    package_roots: Iterable[Path] | None = None,
    expand_xacro: bool = False,
) -> list[LinkSummary]:
    tree = expand_xacro_to_tree(urdf_path, package_roots) if expand_xacro else parse_urdf(urdf_path)
    summaries: list[LinkSummary] = []
    for link in direct_link_elements(tree.getroot()):
        name = link.get("name") or ""
        refs = extract_mesh_refs(link, urdf_path, package_roots)
        warnings = []
        for ref in refs:
            if ref.resolved_path is None:
                warnings.append(f"mesh not found: {ref.filename}")
            elif ref.resolved_path.suffix.lower() != ".stl":
                warnings.append(f"not stl: {ref.filename}")
        summaries.append(
            LinkSummary(
                name=name,
                existing_mass=find_existing_mass(link),
                mesh_count=len(refs),
                mesh_files=[ref.filename for ref in refs],
                warnings=warnings,
            )
        )
    return summaries


def source_mesh_count(link: ET.Element) -> int:
    for source in ("collision", "visual"):
        count = 0
        for node in iter_link_mesh_nodes(link, source):
            mesh = first_child(first_child(node, "geometry"), "mesh")
            if mesh is not None and mesh.get("filename"):
                count += 1
        if count:
            return count
    return 0


def scan_xacro_source_links(
    xacro_path: Path,
    package_roots: Iterable[Path] | None = None,
) -> list[XacroSourceLinkSummary]:
    macros: dict[str, MacroDef] = {}
    properties: dict[str, object] = {}
    top_level = collect_xacro_definitions(xacro_path, macros, properties, package_roots, set())
    entries: dict[tuple[Path, int], XacroSourceLinkSummary] = {}

    for element in top_level:
        if is_xacro_element(element):
            for expanded_name, source_link, _env, macro in iter_xacro_macro_link_targets(element, properties, macros):
                source_path = macro.source_path or xacro_path
                key = (source_path, id(source_link))
                source_link_name = source_link.get("name") or expanded_name
                entry = entries.get(key)
                if entry is None:
                    entry = XacroSourceLinkSummary(
                        source_file=source_path,
                        source_link_name=source_link_name,
                        representative_name=expanded_name,
                        instance_names=[],
                        existing_mass=find_existing_mass(source_link),
                        mesh_count=source_mesh_count(source_link),
                    )
                    entries[key] = entry
                if expanded_name not in entry.instance_names:
                    entry.instance_names.append(expanded_name)
            continue

        for link in iter_link_elements(element):
            expanded_name = substitute_xacro_text(link.get("name"), properties) or link.get("name") or ""
            if not expanded_name:
                continue
            key = (xacro_path, id(link))
            if key in entries:
                continue
            entries[key] = XacroSourceLinkSummary(
                source_file=xacro_path,
                source_link_name=link.get("name") or expanded_name,
                representative_name=expanded_name,
                instance_names=[expanded_name],
                existing_mass=find_existing_mass(link),
                mesh_count=source_mesh_count(link),
            )

    return list(entries.values())


def joint_summary_from_element(name: str, joint: ET.Element) -> JointSummary:
    joint_type = joint.get("type") or ""
    parent = child_attr(joint, "parent", "link")
    child = child_attr(joint, "child", "link")
    warnings: list[str] = []
    if not joint_type:
        warnings.append("missing type")
    mimic = first_child(joint, "mimic")
    if mimic is not None:
        target = mimic.get("joint") or ""
        warnings.append(f"mimic: {target}" if target else "mimic")
    requires_confirmation = joint_requires_edit_confirmation(name, joint)
    if requires_confirmation:
        warnings.append("編集要確認")
    return JointSummary(
        name=name,
        joint_type=joint_type,
        parent=parent,
        child=child,
        axis=" ".join(format_float(float(value)) for value in joint_axis_vector(joint)),
        lower=child_attr(joint, "limit", "lower"),
        upper=child_attr(joint, "limit", "upper"),
        effort=child_attr(joint, "limit", "effort"),
        velocity=child_attr(joint, "limit", "velocity"),
        damping=child_attr(joint, "dynamics", "damping"),
        friction=child_attr(joint, "dynamics", "friction"),
        requires_confirmation=requires_confirmation,
        warnings=warnings,
    )


def is_movable_joint(joint: ET.Element) -> bool:
    joint_type = joint.get("type") or ""
    return bool(joint_type and joint_type != "fixed")


def joint_requires_edit_confirmation(name: str, joint: ET.Element) -> bool:
    if first_child(joint, "mimic") is not None:
        return True
    expanded_name = name.replace("${prefix}", "")
    return "gripper_" in expanded_name or "counterweight" in expanded_name


def scan_urdf_joints(
    urdf_path: Path,
    package_roots: Iterable[Path] | None = None,
    expand_xacro: bool = False,
    movable_only: bool = True,
) -> list[JointSummary]:
    tree = expand_xacro_to_tree(urdf_path, package_roots) if expand_xacro else parse_urdf(urdf_path)
    summaries: list[JointSummary] = []
    for joint in direct_joint_elements(tree.getroot()):
        name = joint.get("name") or ""
        if movable_only and not is_movable_joint(joint):
            continue
        summaries.append(joint_summary_from_element(name, joint))
    return summaries


def iter_joint_elements(element: ET.Element) -> Iterable[ET.Element]:
    if local_name(element.tag) == "joint":
        yield element
    for child in list(element):
        yield from iter_joint_elements(child)


def iter_xacro_macro_joint_targets(
    element: ET.Element,
    env: dict[str, object],
    macros: dict[str, MacroDef],
) -> Iterable[tuple[str, ET.Element, dict[str, object], MacroDef]]:
    name = local_name(element.tag)
    if not is_xacro_element(element) or name not in macros:
        return

    macro = macros[name]
    child_env = dict(env)
    for param in macro.params:
        raw_value = element.get(param)
        child_env[param] = substitute_xacro_text(raw_value, env) if raw_value is not None else ""

    for child in macro.body:
        if is_xacro_element(child) and local_name(child.tag) in macros:
            yield from iter_xacro_macro_joint_targets(child, child_env, macros)
            continue
        for joint in iter_joint_elements(child):
            expanded_name = substitute_xacro_text(joint.get("name"), child_env)
            if expanded_name:
                yield expanded_name, joint, child_env, macro


def scan_xacro_source_joints(
    xacro_path: Path,
    package_roots: Iterable[Path] | None = None,
    movable_only: bool = True,
) -> list[XacroSourceJointSummary]:
    macros: dict[str, MacroDef] = {}
    properties: dict[str, object] = {}
    top_level = collect_xacro_definitions(xacro_path, macros, properties, package_roots, set())
    entries: dict[tuple[Path, int], XacroSourceJointSummary] = {}

    for element in top_level:
        if is_xacro_element(element):
            for expanded_name, source_joint, _env, macro in iter_xacro_macro_joint_targets(
                element, properties, macros
            ):
                if movable_only and not is_movable_joint(source_joint):
                    continue
                source_path = macro.source_path or xacro_path
                key = (source_path, id(source_joint))
                source_joint_name = source_joint.get("name") or expanded_name
                entry = entries.get(key)
                if entry is None:
                    summary = joint_summary_from_element(source_joint_name, source_joint)
                    entry = XacroSourceJointSummary(
                        source_file=source_path,
                        source_joint_name=source_joint_name,
                        representative_name=expanded_name,
                        instance_names=[],
                        joint_type=summary.joint_type,
                        parent=summary.parent,
                        child=summary.child,
                        axis=summary.axis,
                        lower=summary.lower,
                        upper=summary.upper,
                        effort=summary.effort,
                        velocity=summary.velocity,
                        damping=summary.damping,
                        friction=summary.friction,
                        requires_confirmation=summary.requires_confirmation,
                        warnings=summary.warnings,
                    )
                    entries[key] = entry
                if expanded_name not in entry.instance_names:
                    entry.instance_names.append(expanded_name)
            continue

        for joint in iter_joint_elements(element):
            joint_type = joint.get("type") or ""
            expanded_name = substitute_xacro_text(joint.get("name"), properties) or joint.get("name") or ""
            if movable_only and not is_movable_joint(joint):
                continue
            if not expanded_name:
                continue
            key = (xacro_path, id(joint))
            if key in entries:
                continue
            summary = joint_summary_from_element(expanded_name, joint)
            entries[key] = XacroSourceJointSummary(
                source_file=xacro_path,
                source_joint_name=joint.get("name") or expanded_name,
                representative_name=expanded_name,
                instance_names=[expanded_name],
                joint_type=summary.joint_type,
                parent=summary.parent,
                child=summary.child,
                axis=summary.axis,
                lower=summary.lower,
                upper=summary.upper,
                effort=summary.effort,
                velocity=summary.velocity,
                damping=summary.damping,
                friction=summary.friction,
                requires_confirmation=summary.requires_confirmation,
                warnings=summary.warnings,
            )

    return list(entries.values())


def model_tree(
    urdf_path: Path,
    package_roots: Iterable[Path] | None = None,
    expand_xacro: bool = False,
) -> ET.ElementTree:
    return expand_xacro_to_tree(urdf_path, package_roots) if expand_xacro else parse_urdf(urdf_path)


def fixed_joint_groups(
    urdf_path: Path,
    package_roots: Iterable[Path] | None = None,
    expand_xacro: bool = False,
) -> list[FixedJointGroup]:
    tree = model_tree(urdf_path, package_roots, expand_xacro)
    root = tree.getroot()
    links = [link.get("name") or "" for link in direct_link_elements(root)]
    links = [name for name in links if name]
    order = {name: index for index, name in enumerate(links)}
    parent = {name: name for name in links}
    fixed_children: set[str] = set()

    def find(name: str) -> str:
        while parent[name] != name:
            parent[name] = parent[parent[name]]
            name = parent[name]
        return name

    def union(a: str, b: str) -> None:
        ra = find(a)
        rb = find(b)
        if ra == rb:
            return
        if order[ra] <= order[rb]:
            parent[rb] = ra
        else:
            parent[ra] = rb

    for joint in direct_children(root, "joint"):
        if joint.get("type") != "fixed":
            continue
        parent_node = first_child(joint, "parent")
        child_node = first_child(joint, "child")
        parent_link = parent_node.get("link") if parent_node is not None else None
        child_link = child_node.get("link") if child_node is not None else None
        if not parent_link or not child_link or parent_link not in parent or child_link not in parent:
            continue
        fixed_children.add(child_link)
        union(parent_link, child_link)

    summaries = {summary.name: summary for summary in scan_urdf(urdf_path, package_roots, expand_xacro)}
    components: dict[str, list[str]] = {}
    for link in links:
        components.setdefault(find(link), []).append(link)

    groups: list[FixedJointGroup] = []
    for index, names in enumerate(sorted(components.values(), key=lambda item: min(order[name] for name in item))):
        names = sorted(names, key=lambda name: order[name])
        root_candidates = [name for name in names if name not in fixed_children]
        root_link = root_candidates[0] if root_candidates else names[0]
        masses = [summaries[name].existing_mass for name in names if summaries.get(name) and summaries[name].existing_mass is not None]
        existing_mass = sum(masses) if masses else None
        mesh_count = sum(summaries[name].mesh_count for name in names if name in summaries)
        warnings: list[str] = []
        for name in names:
            if name in summaries:
                warnings.extend(summaries[name].warnings)
        label = root_link if len(names) == 1 else f"{root_link} ({len(names)} fixed links)"
        groups.append(
            FixedJointGroup(
                group_id=f"fixed_group_{index + 1}",
                label=label,
                link_names=names,
                root_link=root_link,
                existing_mass=existing_mass,
                mesh_count=mesh_count,
                warnings=warnings,
            )
        )
    return groups


def load_transformed_stl(ref: MeshRef) -> trimesh.Trimesh:
    if ref.resolved_path is None:
        raise FileNotFoundError(ref.filename)
    if ref.resolved_path.suffix.lower() != ".stl":
        raise ValueError(f"unsupported mesh type: {ref.filename}")
    mesh = trimesh.load_mesh(str(ref.resolved_path), force="mesh", process=True)
    if not isinstance(mesh, trimesh.Trimesh):
        raise ValueError(f"not a mesh: {ref.filename}")
    mesh = mesh.copy()
    mesh.apply_transform(mesh_transform(ref.scale, ref.xyz, ref.rpy))
    if mesh.volume < 0:
        mesh.invert()
    return mesh


def calculate_link_inertial(
    link: ET.Element,
    urdf_path: Path,
    mass_kg: float,
    package_roots: Iterable[Path] | None = None,
) -> InertialResult:
    link_name = link.get("name") or ""
    if not math.isfinite(mass_kg) or mass_kg <= 0.0:
        raise ValueError("mass must be positive")

    refs = extract_mesh_refs(link, urdf_path, package_roots)
    if not refs:
        raise ValueError("no STL mesh in collision or visual")

    meshes: list[trimesh.Trimesh] = []
    warnings: list[str] = []
    for ref in refs:
        mesh = load_transformed_stl(ref)
        if not mesh.is_watertight:
            raise ValueError(f"STL mesh is not watertight: {ref.filename}")
        volume = abs(float(mesh.volume))
        if not math.isfinite(volume) or volume <= EPS:
            raise ValueError(f"STL mesh volume is zero: {ref.filename}")
        meshes.append(mesh)

    volumes = np.array([abs(float(mesh.volume)) for mesh in meshes], dtype=float)
    total_volume = float(volumes.sum())
    if total_volume <= EPS:
        raise ValueError("total STL volume is zero")

    masses = mass_kg * volumes / total_volume
    centers = np.array([mesh.center_mass for mesh in meshes], dtype=float)
    center = (centers * masses[:, None]).sum(axis=0) / mass_kg
    inertia = np.zeros((3, 3), dtype=float)

    for mesh, mesh_mass, mesh_center in zip(meshes, masses, centers):
        mesh.density = float(mesh_mass / abs(float(mesh.volume)))
        mesh_inertia = np.array(mesh.moment_inertia, dtype=float)
        d = np.array(mesh_center - center, dtype=float)
        inertia += mesh_inertia + mesh_mass * ((np.dot(d, d) * np.eye(3)) - np.outer(d, d))

    validate_inertia_matrix(inertia)
    return InertialResult(
        link_name=link_name,
        mass=mass_kg,
        center=center,
        inertia=inertia,
        mesh_count=len(meshes),
        warnings=warnings,
    )


def validate_inertia_matrix(inertia: np.ndarray) -> None:
    if inertia.shape != (3, 3):
        raise ValueError("inertia matrix must be 3x3")
    if not np.all(np.isfinite(inertia)):
        raise ValueError("inertia contains non-finite values")
    if not np.allclose(inertia, inertia.T, atol=1.0e-9):
        raise ValueError("inertia matrix is not symmetric")
    eigenvalues = np.linalg.eigvalsh(inertia)
    if np.any(eigenvalues <= 0.0):
        raise ValueError(f"inertia matrix is not positive definite: {eigenvalues}")
    principal = np.sort(eigenvalues)
    if principal[0] + principal[1] + 1.0e-10 < principal[2]:
        raise ValueError(f"inertia principal moments fail triangle inequality: {principal}")


def set_link_inertial(link: ET.Element, result: InertialResult) -> None:
    for inertial in direct_children(link, "inertial"):
        link.remove(inertial)

    inertial = ET.Element("inertial")
    ET.SubElement(
        inertial,
        "origin",
        {
            "xyz": " ".join(format_float(float(v)) for v in result.center),
            "rpy": "0 0 0",
        },
    )
    ET.SubElement(inertial, "mass", {"value": format_float(result.mass)})
    inertia = result.inertia
    ET.SubElement(
        inertial,
        "inertia",
        {
            "ixx": format_float(float(inertia[0, 0])),
            "ixy": format_float(float(inertia[0, 1])),
            "ixz": format_float(float(inertia[0, 2])),
            "iyy": format_float(float(inertia[1, 1])),
            "iyz": format_float(float(inertia[1, 2])),
            "izz": format_float(float(inertia[2, 2])),
        },
    )
    link.insert(0, inertial)


def xacro_signed_value(value: float, env: dict[str, object], variable: str = "side_y") -> str:
    side_value = env.get(variable)
    try:
        side = float(side_value) if side_value is not None else 0.0
    except (TypeError, ValueError):
        side = 0.0
    if abs(side) <= EPS:
        return format_float(value)
    base = format_float(value / side)
    return f"${{{variable} * {base}}}"


def set_link_inertial_xacro(link: ET.Element, result: InertialResult, env: dict[str, object]) -> None:
    for inertial in direct_children(link, "inertial"):
        link.remove(inertial)

    inertial = ET.Element("inertial")
    center = result.center
    ET.SubElement(
        inertial,
        "origin",
        {
            "xyz": " ".join(
                [
                    format_float(float(center[0])),
                    xacro_signed_value(float(center[1]), env),
                    format_float(float(center[2])),
                ]
            ),
            "rpy": "0 0 0",
        },
    )
    ET.SubElement(inertial, "mass", {"value": format_float(result.mass)})
    inertia = result.inertia
    ET.SubElement(
        inertial,
        "inertia",
        {
            "ixx": format_float(float(inertia[0, 0])),
            "ixy": xacro_signed_value(float(inertia[0, 1]), env),
            "ixz": format_float(float(inertia[0, 2])),
            "iyy": format_float(float(inertia[1, 1])),
            "iyz": xacro_signed_value(float(inertia[1, 2]), env),
            "izz": format_float(float(inertia[2, 2])),
        },
    )
    link.insert(0, inertial)


def iter_link_elements(element: ET.Element) -> Iterable[ET.Element]:
    if local_name(element.tag) == "link":
        yield element
    for child in list(element):
        yield from iter_link_elements(child)


def iter_xacro_macro_link_targets(
    element: ET.Element,
    env: dict[str, object],
    macros: dict[str, MacroDef],
) -> Iterable[tuple[str, ET.Element, dict[str, object], MacroDef]]:
    name = local_name(element.tag)
    if not is_xacro_element(element) or name not in macros:
        return

    macro = macros[name]
    child_env = dict(env)
    for param in macro.params:
        raw_value = element.get(param)
        child_env[param] = substitute_xacro_text(raw_value, env) if raw_value is not None else ""

    for child in macro.body:
        if is_xacro_element(child) and local_name(child.tag) in macros:
            yield from iter_xacro_macro_link_targets(child, child_env, macros)
            continue
        for link in iter_link_elements(child):
            expanded_name = substitute_xacro_text(link.get("name"), child_env)
            if expanded_name:
                yield expanded_name, link, child_env, macro


def apply_inertial_results_to_xacro_sources(
    xacro_path: Path,
    results: Iterable[InertialResult],
    package_roots: Iterable[Path] | None = None,
    backup: bool = True,
) -> ApplyReport:
    result_by_link = {result.link_name: result for result in results}
    report = ApplyReport(output_path=xacro_path)
    if not result_by_link:
        return report

    macros: dict[str, MacroDef] = {}
    properties: dict[str, object] = {}
    top_level = collect_xacro_definitions(xacro_path, macros, properties, package_roots, set())
    source_trees: dict[Path, ET.ElementTree] = {}
    source_link_owner: dict[tuple[Path, int], str] = {}
    seen_results: set[str] = set()

    for element in top_level:
        for expanded_name, source_link, env, macro in iter_xacro_macro_link_targets(element, properties, macros):
            result = result_by_link.get(expanded_name)
            if result is None:
                continue
            if expanded_name in seen_results:
                continue
            if macro.source_path is None or macro.source_tree is None:
                report.skipped[expanded_name] = ["xacro macro source file is unknown"]
                continue
            source_key = (macro.source_path, id(source_link))
            if source_key in source_link_owner:
                report.skipped[expanded_name] = [
                    f"same xacro macro link already updated from {source_link_owner[source_key]}"
                ]
                continue
            set_link_inertial_xacro(source_link, result, env)
            source_link_owner[source_key] = expanded_name
            source_trees[macro.source_path] = macro.source_tree
            seen_results.add(expanded_name)
            report.updated.append(result)

    direct_tree = parse_urdf(xacro_path)
    for source_link in direct_link_elements(direct_tree.getroot()):
        expanded_name = substitute_xacro_text(source_link.get("name"), properties) or source_link.get("name") or ""
        result = result_by_link.get(expanded_name)
        if result is None or expanded_name in seen_results:
            continue
        set_link_inertial_xacro(source_link, result, properties)
        source_trees[xacro_path] = direct_tree
        seen_results.add(expanded_name)
        report.updated.append(result)

    for name in result_by_link:
        if name not in seen_results and name not in report.skipped:
            report.skipped[name] = ["matching xacro source link was not found"]

    if not report.updated:
        return report

    for source_path, tree in source_trees.items():
        if backup:
            backup_path = source_path.with_suffix(source_path.suffix + ".bak")
            shutil.copy2(source_path, backup_path)
            report.backup_paths.append(backup_path)
            if report.backup_path is None:
                report.backup_path = backup_path
        ET.indent(tree, space="  ")
        tree.write(source_path, encoding="utf-8", xml_declaration=True)

    return report


def apply_mesh_visibility_to_urdf(
    urdf_path: Path,
    visibility_updates: dict[str, bool],
    backup: bool = True,
) -> JointApplyReport:
    tree = parse_urdf(urdf_path)
    report = JointApplyReport(output_path=urdf_path)
    seen: set[str] = set()
    for link in direct_link_elements(tree.getroot()):
        name = link.get("name") or ""
        seen.add(name)
        if name not in visibility_updates:
            continue
        set_link_mesh_visibility(link, visibility_updates[name], None)
        report.updated.append(name)
    for name in visibility_updates:
        if name not in seen:
            report.skipped[name] = ["link not found"]
    if not report.updated:
        return report
    if backup:
        backup_path = urdf_path.with_suffix(urdf_path.suffix + ".bak")
        shutil.copy2(urdf_path, backup_path)
        report.backup_path = backup_path
        report.backup_paths.append(backup_path)
    ET.indent(tree, space="  ")
    tree.write(urdf_path, encoding="utf-8", xml_declaration=True)
    return report


def apply_mesh_visibility_to_xacro_sources(
    xacro_path: Path,
    visibility_updates: dict[str, bool],
    package_roots: Iterable[Path] | None = None,
    backup: bool = True,
) -> JointApplyReport:
    report = JointApplyReport(output_path=xacro_path)
    if not visibility_updates:
        return report

    macros: dict[str, MacroDef] = {}
    properties: dict[str, object] = {}
    top_level = collect_xacro_definitions(xacro_path, macros, properties, package_roots, set())
    source_trees: dict[Path, ET.ElementTree] = {}
    source_link_owner: dict[tuple[Path, int], str] = {}
    seen: set[str] = set()
    direct_tree = parse_urdf(xacro_path)

    for element in top_level:
        if is_xacro_element(element):
            for expanded_name, source_link, _env, macro in iter_xacro_macro_link_targets(
                element, properties, macros
            ):
                show_mesh = visibility_updates.get(expanded_name)
                if show_mesh is None:
                    continue
                if expanded_name in seen:
                    continue
                if macro.source_path is None or macro.source_tree is None:
                    report.skipped[expanded_name] = ["xacro macro source file is unknown"]
                    continue
                source_key = (macro.source_path, id(source_link))
                if source_key in source_link_owner:
                    report.skipped[expanded_name] = [
                        f"same xacro macro link already updated from {source_link_owner[source_key]}"
                    ]
                    continue
                set_link_mesh_visibility(source_link, show_mesh, _env)
                source_link_owner[source_key] = expanded_name
                source_trees[macro.source_path] = macro.source_tree
                seen.add(expanded_name)
                report.updated.append(expanded_name)
            continue

        for link in iter_link_elements(element):
            expanded_name = substitute_xacro_text(link.get("name"), properties) or link.get("name") or ""
            show_mesh = visibility_updates.get(expanded_name)
            if show_mesh is None or expanded_name in seen:
                continue
            set_link_mesh_visibility(link, show_mesh, properties)
            source_trees[xacro_path] = direct_tree
            seen.add(expanded_name)
            report.updated.append(expanded_name)

    for name in visibility_updates:
        if name not in seen and name not in report.skipped:
            report.skipped[name] = ["matching xacro source link was not found"]

    if not report.updated:
        return report

    for source_path, tree in source_trees.items():
        if backup:
            backup_path = source_path.with_suffix(source_path.suffix + ".bak")
            shutil.copy2(source_path, backup_path)
            report.backup_paths.append(backup_path)
            if report.backup_path is None:
                report.backup_path = backup_path
        ET.indent(tree, space="  ")
        tree.write(source_path, encoding="utf-8", xml_declaration=True)

    return report


def find_direct_link(tree: ET.ElementTree, link_name: str) -> ET.Element | None:
    for link in direct_link_elements(tree.getroot()):
        if link.get("name") == link_name:
            return link
    return None


def rewrite_meshes_to_file_uri(
    link: ET.Element,
    source_path: Path,
    package_roots: Iterable[Path] | None = None,
) -> None:
    for source in ("visual", "collision"):
        for node in direct_children(link, source):
            mesh = first_child(first_child(node, "geometry"), "mesh")
            if mesh is None:
                continue
            filename = mesh.get("filename")
            if not filename:
                continue
            resolved = resolve_mesh_filename(filename, source_path, package_roots)
            if resolved is not None:
                mesh.set("filename", resolved.resolve().as_uri())


def write_single_link_preview_urdf(
    source_path: Path,
    link_name: str,
    output_path: Path,
    package_roots: Iterable[Path] | None = None,
    expand_xacro: bool = False,
    show_mesh: bool = True,
) -> None:
    tree = expand_xacro_to_tree(source_path, package_roots) if expand_xacro else parse_urdf(source_path)
    source_link = find_direct_link(tree, link_name)
    if source_link is None:
        raise ValueError(f"link not found: {link_name}")

    preview_link = copy.deepcopy(source_link)
    preview_link.set("name", "base_link")
    if show_mesh:
        rewrite_meshes_to_file_uri(preview_link, source_path, package_roots)
    else:
        for source in ("visual", "collision"):
            for node in list(direct_children(preview_link, source)):
                preview_link.remove(node)

    robot = ET.Element("robot", {"name": f"inertia_preview_{link_name}"})
    robot.append(preview_link)
    output_tree = ET.ElementTree(robot)
    ET.indent(output_tree, space="  ")
    output_tree.write(output_path, encoding="utf-8", xml_declaration=True)


def apply_masses_to_urdf(
    urdf_path: Path,
    masses: dict[str, float],
    package_roots: Iterable[Path] | None = None,
    backup: bool = True,
) -> ApplyReport:
    tree = parse_urdf(urdf_path)
    root = tree.getroot()
    report = ApplyReport(output_path=urdf_path)
    seen_links: set[str] = set()

    for link in direct_link_elements(root):
        name = link.get("name") or ""
        seen_links.add(name)
        if name not in masses:
            continue
        try:
            result = calculate_link_inertial(link, urdf_path, masses[name], package_roots)
            set_link_inertial(link, result)
            report.updated.append(result)
        except Exception as exc:  # noqa: BLE001 - collected for GUI display
            report.skipped[name] = [str(exc)]

    for name in masses:
        if name not in seen_links:
            if not seen_links and looks_like_xacro(urdf_path):
                report.skipped[name] = [
                    "link not found in direct XML; xacro macro/include expansion is not performed"
                ]
            else:
                report.skipped[name] = ["link not found"]

    if not report.updated:
        return report

    if backup:
        backup_path = urdf_path.with_suffix(urdf_path.suffix + ".bak")
        shutil.copy2(urdf_path, backup_path)
        report.backup_path = backup_path

    ET.indent(tree, space="  ")
    tree.write(urdf_path, encoding="utf-8", xml_declaration=True)
    return report


def validate_urdf_inertials(urdf_path: Path, link_names: Iterable[str] | None = None) -> list[str]:
    tree = parse_urdf(urdf_path)
    return validate_inertials_in_tree(tree, link_names)


def validate_xacro_inertials(
    xacro_path: Path,
    package_roots: Iterable[Path] | None = None,
    link_names: Iterable[str] | None = None,
) -> list[str]:
    tree = expand_xacro_to_tree(xacro_path, package_roots)
    return validate_inertials_in_tree(tree, link_names)


def validate_inertials_in_tree(tree: ET.ElementTree, link_names: Iterable[str] | None = None) -> list[str]:
    messages: list[str] = []
    target_links = set(link_names or [])
    for link in direct_link_elements(tree.getroot()):
        name = link.get("name") or ""
        if target_links and name not in target_links:
            continue
        inertial = first_child(link, "inertial")
        if inertial is None:
            continue
        mass_node = first_child(inertial, "mass")
        inertia_node = first_child(inertial, "inertia")
        if mass_node is None or inertia_node is None:
            messages.append(f"{name}: missing mass or inertia")
            continue
        try:
            mass = float(mass_node.get("value", "nan"))
            if not math.isfinite(mass) or mass <= 0.0:
                messages.append(f"{name}: invalid mass {mass_node.get('value')}")
            inertia = np.array(
                [
                    [
                        float(inertia_node.get("ixx", "nan")),
                        float(inertia_node.get("ixy", "nan")),
                        float(inertia_node.get("ixz", "nan")),
                    ],
                    [
                        float(inertia_node.get("ixy", "nan")),
                        float(inertia_node.get("iyy", "nan")),
                        float(inertia_node.get("iyz", "nan")),
                    ],
                    [
                        float(inertia_node.get("ixz", "nan")),
                        float(inertia_node.get("iyz", "nan")),
                        float(inertia_node.get("izz", "nan")),
                    ],
                ],
                dtype=float,
            )
            validate_inertia_matrix(inertia)
        except Exception as exc:  # noqa: BLE001 - user-facing validation
            messages.append(f"{name}: {exc}")
    if not messages:
        messages.append("OK")
    return messages


def apply_masses_to_xacro_sources(
    xacro_path: Path,
    masses: dict[str, float],
    package_roots: Iterable[Path] | None = None,
    backup: bool = True,
) -> ApplyReport:
    tree = expand_xacro_to_tree(xacro_path, package_roots)
    report = ApplyReport(output_path=xacro_path)
    seen_links: set[str] = set()

    for link in direct_link_elements(tree.getroot()):
        name = link.get("name") or ""
        seen_links.add(name)
        if name not in masses:
            continue
        try:
            result = calculate_link_inertial(link, xacro_path, masses[name], package_roots)
            report.updated.append(result)
        except Exception as exc:  # noqa: BLE001 - collected for GUI display
            report.skipped[name] = [str(exc)]

    for name in masses:
        if name not in seen_links:
            report.skipped[name] = ["link not found in expanded xacro"]

    if not report.updated:
        return report

    xacro_report = apply_inertial_results_to_xacro_sources(
        xacro_path,
        report.updated,
        package_roots,
        backup=backup,
    )
    report.backup_paths.extend(xacro_report.backup_paths)
    report.backup_path = xacro_report.backup_path
    report.skipped.update(xacro_report.skipped)
    return report


JOINT_LIMIT_FIELDS = ("lower", "upper", "effort", "velocity")
JOINT_DYNAMICS_FIELDS = ("damping", "friction")
JOINT_TYPE_VALUES = ("fixed", "revolute", "continuous", "prismatic", "floating", "planar")


def set_or_clear_attrs(parent: ET.Element, child_name: str, values: dict[str, str], fields: tuple[str, ...]) -> None:
    child = first_child(parent, child_name)
    has_value = any(values.get(field, "").strip() for field in fields)
    if not has_value:
        if child is not None:
            parent.remove(child)
        return
    if child is None:
        child = ET.SubElement(parent, child_name)
    for field in fields:
        value = values.get(field, "").strip()
        if value:
            child.set(field, value)
        elif field in child.attrib:
            del child.attrib[field]


def set_joint_properties(joint: ET.Element, values: dict[str, str]) -> None:
    joint_type = values.get("type", "").strip()
    if joint_type:
        joint.set("type", joint_type)
    set_or_clear_attrs(joint, "limit", values, JOINT_LIMIT_FIELDS)
    set_or_clear_attrs(joint, "dynamics", values, JOINT_DYNAMICS_FIELDS)


def prepare_joint_update_values(joint: ET.Element, values: dict[str, str]) -> tuple[dict[str, str], bool]:
    prepared = dict(values)
    reverse_axis = prepared.pop("reverse_axis", "").strip().lower() in {"1", "true", "yes", "on"}
    effective_type = (prepared.get("type", "").strip() or (joint.get("type") or "")).strip().lower()
    if reverse_axis and effective_type in {"revolute", "continuous"}:
        lower = prepared.get("lower", "").strip()
        upper = prepared.get("upper", "").strip()
        if lower and upper:
            prepared["lower"] = format_float(-float(upper))
            prepared["upper"] = format_float(-float(lower))
        else:
            if lower:
                prepared["lower"] = format_float(-float(lower))
            if upper:
                prepared["upper"] = format_float(-float(upper))
    return prepared, reverse_axis and effective_type in {"revolute", "continuous"}


def validate_joint_update_values(values: dict[str, str]) -> list[str]:
    messages: list[str] = []
    joint_type = values.get("type", "").strip().lower()
    if joint_type and joint_type not in JOINT_TYPE_VALUES:
        messages.append(f"type must be one of {', '.join(JOINT_TYPE_VALUES)}")
    for key in (*JOINT_LIMIT_FIELDS, *JOINT_DYNAMICS_FIELDS):
        value = values.get(key, "").strip()
        if not value:
            continue
        try:
            float(value)
        except ValueError:
            messages.append(f"{key} must be a number")
    lower = values.get("lower", "").strip()
    upper = values.get("upper", "").strip()
    if lower and upper:
        try:
            if float(lower) > float(upper):
                messages.append("lower must be <= upper")
        except ValueError:
            pass
    return messages


def apply_joint_properties_to_urdf(
    urdf_path: Path,
    updates: dict[str, dict[str, str]],
    backup: bool = True,
) -> JointApplyReport:
    tree = parse_urdf(urdf_path)
    report = JointApplyReport(output_path=urdf_path)
    seen: set[str] = set()
    for joint in direct_joint_elements(tree.getroot()):
        name = joint.get("name") or ""
        seen.add(name)
        if name not in updates:
            continue
        values, reverse_axis = prepare_joint_update_values(joint, updates[name])
        errors = validate_joint_update_values(values)
        if errors:
            report.skipped[name] = errors
            continue
        if reverse_axis:
            set_joint_axis_vector(joint, -joint_axis_vector(joint))
        set_joint_properties(joint, values)
        report.updated.append(name)
    for name in updates:
        if name not in seen:
            report.skipped[name] = ["joint not found"]
    if not report.updated:
        return report
    if backup:
        backup_path = urdf_path.with_suffix(urdf_path.suffix + ".bak")
        shutil.copy2(urdf_path, backup_path)
        report.backup_path = backup_path
        report.backup_paths.append(backup_path)
    ET.indent(tree, space="  ")
    tree.write(urdf_path, encoding="utf-8", xml_declaration=True)
    return report


def apply_joint_properties_to_xacro_sources(
    xacro_path: Path,
    updates: dict[str, dict[str, str]],
    package_roots: Iterable[Path] | None = None,
    backup: bool = True,
) -> JointApplyReport:
    report = JointApplyReport(output_path=xacro_path)
    if not updates:
        return report

    for name, values in updates.items():
        errors = validate_joint_update_values(values)
        if errors:
            report.skipped[name] = errors
    valid_updates = {name: values for name, values in updates.items() if name not in report.skipped}
    if not valid_updates:
        return report

    macros: dict[str, MacroDef] = {}
    properties: dict[str, object] = {}
    top_level = collect_xacro_definitions(xacro_path, macros, properties, package_roots, set())
    source_trees: dict[Path, ET.ElementTree] = {}
    source_joint_owner: dict[tuple[Path, int], str] = {}
    seen: set[str] = set()

    for element in top_level:
        for expanded_name, source_joint, _env, macro in iter_xacro_macro_joint_targets(
            element, properties, macros
        ):
            values = valid_updates.get(expanded_name)
            if values is None:
                continue
            if expanded_name in seen:
                continue
            if macro.source_path is None or macro.source_tree is None:
                report.skipped[expanded_name] = ["xacro macro source file is unknown"]
                continue
            source_key = (macro.source_path, id(source_joint))
            if source_key in source_joint_owner:
                report.skipped[expanded_name] = [
                    f"same xacro macro joint already updated from {source_joint_owner[source_key]}"
                ]
                continue
            prepared_values, reverse_axis = prepare_joint_update_values(source_joint, values)
            errors = validate_joint_update_values(prepared_values)
            if errors:
                report.skipped[expanded_name] = errors
                continue
            if reverse_axis:
                set_joint_axis_vector(source_joint, -joint_axis_vector(source_joint))
            set_joint_properties(source_joint, prepared_values)
            source_joint_owner[source_key] = expanded_name
            source_trees[macro.source_path] = macro.source_tree
            seen.add(expanded_name)
            report.updated.append(expanded_name)

    direct_tree = parse_urdf(xacro_path)
    for source_joint in direct_joint_elements(direct_tree.getroot()):
        expanded_name = substitute_xacro_text(source_joint.get("name"), properties) or source_joint.get("name") or ""
        values = valid_updates.get(expanded_name)
        if values is None or expanded_name in seen:
            continue
        prepared_values, reverse_axis = prepare_joint_update_values(source_joint, values)
        errors = validate_joint_update_values(prepared_values)
        if errors:
            report.skipped[expanded_name] = errors
            continue
        if reverse_axis:
            set_joint_axis_vector(source_joint, -joint_axis_vector(source_joint))
        set_joint_properties(source_joint, prepared_values)
        source_trees[xacro_path] = direct_tree
        seen.add(expanded_name)
        report.updated.append(expanded_name)

    for name in valid_updates:
        if name not in seen and name not in report.skipped:
            report.skipped[name] = ["matching xacro source joint was not found"]

    if not report.updated:
        return report

    for source_path, tree in source_trees.items():
        if backup:
            backup_path = source_path.with_suffix(source_path.suffix + ".bak")
            shutil.copy2(source_path, backup_path)
            report.backup_paths.append(backup_path)
            if report.backup_path is None:
                report.backup_path = backup_path
        ET.indent(tree, space="  ")
        tree.write(source_path, encoding="utf-8", xml_declaration=True)

    return report


def read_mass_file(path: Path) -> dict[str, float]:
    if path.suffix.lower() == ".json":
        data = json.loads(path.read_text(encoding="utf-8"))
        return {str(key): float(value) for key, value in data.items()}

    masses: dict[str, float] = {}
    with path.open("r", encoding="utf-8", newline="") as handle:
        sample = handle.read(4096)
        handle.seek(0)
        dialect = csv.Sniffer().sniff(sample) if "," in sample else csv.excel
        reader = csv.reader(handle, dialect)
        for row in reader:
            if not row or row[0].strip().startswith("#"):
                continue
            if len(row) < 2:
                continue
            if row[0].strip().lower() in {"link", "link_name", "name"}:
                continue
            masses[row[0].strip()] = float(row[1])
    return masses


def parse_mass_pairs(pairs: Iterable[str]) -> dict[str, float]:
    masses: dict[str, float] = {}
    for pair in pairs:
        if "=" not in pair:
            raise ValueError(f"mass must be LINK=KG: {pair}")
        link, value = pair.split("=", 1)
        masses[link.strip()] = float(value)
    return masses


def package_roots_from_args(values: Iterable[str] | None) -> list[Path]:
    return [Path(value).expanduser().resolve() for value in values or []]


def cmd_scan(args: argparse.Namespace) -> int:
    urdf_path = Path(args.urdf).expanduser().resolve()
    package_roots = package_roots_from_args(args.package_root)
    summaries = scan_urdf(urdf_path, package_roots, expand_xacro=args.expand_xacro)
    if not summaries and looks_like_xacro(urdf_path):
        print("# no direct <link> elements found; use --expand-xacro for macro/include expansion")
    for summary in summaries:
        existing = "" if summary.existing_mass is None else format_float(summary.existing_mass)
        status = "; ".join(summary.warnings) if summary.warnings else "OK"
        print(f"{summary.name},{existing},{summary.mesh_count},{status}")
    return 0


def cmd_apply(args: argparse.Namespace) -> int:
    urdf_path = Path(args.urdf).expanduser().resolve()
    package_roots = package_roots_from_args(args.package_root)
    if args.expanded_output:
        output_path = Path(args.expanded_output).expanduser().resolve()
        write_expanded_xacro(urdf_path, output_path, package_roots)
        urdf_path = output_path
    masses: dict[str, float] = {}
    if args.mass_file:
        masses.update(read_mass_file(Path(args.mass_file).expanduser().resolve()))
    masses.update(parse_mass_pairs(args.mass or []))
    if not masses:
        print("No masses specified.", file=sys.stderr)
        return 2
    report = apply_masses_to_urdf(
        urdf_path,
        masses,
        package_roots,
        backup=not args.no_backup,
    )
    print(f"updated: {len(report.updated)}")
    for result in report.updated:
        print(
            f"  {result.link_name}: mass={format_float(result.mass)} "
            f"com={' '.join(format_float(float(v)) for v in result.center)}"
        )
    if report.skipped:
        print(f"skipped: {len(report.skipped)}")
        for link, warnings in report.skipped.items():
            print(f"  {link}: {'; '.join(warnings)}")
    if report.backup_path:
        print(f"backup: {report.backup_path}")
    for message in validate_urdf_inertials(urdf_path):
        print(f"check: {message}")
    return 1 if report.skipped else 0


def cmd_check(args: argparse.Namespace) -> int:
    urdf_path = Path(args.urdf).expanduser().resolve()
    messages = validate_urdf_inertials(urdf_path)
    for message in messages:
        print(message)
    return 0 if messages == ["OK"] else 1


def cmd_gui(_args: argparse.Namespace) -> int:
    from urdf_xacro_tuner.gui import main as gui_main

    gui_main()
    return 0


def cmd_expand(args: argparse.Namespace) -> int:
    xacro_path = Path(args.xacro).expanduser().resolve()
    output_path = Path(args.output).expanduser().resolve()
    write_expanded_xacro(xacro_path, output_path, package_roots_from_args(args.package_root))
    print(f"expanded: {output_path}")
    return 0


def cmd_preview_link(args: argparse.Namespace) -> int:
    source_path = Path(args.source).expanduser().resolve()
    output_path = Path(args.output).expanduser().resolve()
    write_single_link_preview_urdf(
        source_path,
        args.link,
        output_path,
        package_roots_from_args(args.package_root),
        expand_xacro=args.expand_xacro,
    )
    print(f"preview: {output_path}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Update URDF inertial tags from STL meshes.")
    subparsers = parser.add_subparsers(required=True)

    scan = subparsers.add_parser("scan", help="list links and mesh status")
    scan.add_argument("urdf")
    scan.add_argument("--package-root", action="append", help="ROS workspace src path for package:// meshes")
    scan.add_argument("--expand-xacro", action="store_true", help="expand xacro includes and macros for scanning")
    scan.set_defaults(func=cmd_scan)

    apply = subparsers.add_parser("apply", help="overwrite URDF inertial tags")
    apply.add_argument("urdf")
    apply.add_argument("--mass", action="append", help="link mass as LINK=KG")
    apply.add_argument("--mass-file", help="CSV or JSON mass file")
    apply.add_argument("--package-root", action="append", help="ROS workspace src path for package:// meshes")
    apply.add_argument("--expanded-output", help="write expanded xacro to this URDF and apply masses there")
    apply.add_argument("--no-backup", action="store_true", help="do not create URDF .bak file")
    apply.set_defaults(func=cmd_apply)

    check = subparsers.add_parser("check", help="validate existing inertial tags")
    check.add_argument("urdf")
    check.set_defaults(func=cmd_check)

    gui = subparsers.add_parser("gui", help="open the mass-entry GUI")
    gui.set_defaults(func=cmd_gui)

    expand = subparsers.add_parser("expand", help="write a ROS-free expanded URDF from xacro")
    expand.add_argument("xacro")
    expand.add_argument("output")
    expand.add_argument("--package-root", action="append", help="ROS workspace src path for package:// meshes")
    expand.set_defaults(func=cmd_expand)

    preview = subparsers.add_parser("preview-link", help="write a selected-link-only URDF for RViz")
    preview.add_argument("source")
    preview.add_argument("link")
    preview.add_argument("output")
    preview.add_argument("--package-root", action="append", help="ROS workspace src path for package:// meshes")
    preview.add_argument("--expand-xacro", action="store_true", help="expand xacro includes and macros first")
    preview.set_defaults(func=cmd_preview_link)
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
