from __future__ import annotations

import argparse
import json
import math
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple


def _to_float(text: str) -> float:
    token = text.strip().replace("D", "E").replace("d", "e")
    if token.endswith("E") or token.endswith("e"):
        token = token + "0"
    if token in {"+", "-", ""}:
        raise ValueError(f"Invalid numeric token: '{text}'")
    return float(token)


def _is_entity_start(line: str) -> bool:
    return (not line.startswith(",")) and bool(re.match(r"^[A-Z_]+\s*/", line.strip()))


def _split_csv_payload(line: str) -> List[str]:
    payload = line.split("!")[0].strip()
    if not payload:
        return []
    if payload.startswith(","):
        payload = payload[1:].strip()
    if "=" in payload and not re.match(r"^[+-]?\d", payload):
        return []
    return [p.strip() for p in payload.split(",") if p.strip()]


def _parse_key_value(line: str, key: str) -> Optional[str]:
    m = re.search(rf"{re.escape(key)}\s*=\s*(.+)", line)
    if not m:
        return None
    return m.group(1).strip()


@dataclass
class Part:
    part_id: int
    name: str = ""
    mass: float = 1.0
    cm_marker: Optional[int] = None
    ip_diag: Tuple[float, float, float] = (1.0, 1.0, 1.0)
    ip_products: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # Ixy, Ixz, Iyz
    qg: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    reuler: Tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass
class Marker:
    marker_id: int
    name: str = ""
    part_id: int = -1
    qp: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    reuler: Tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass
class JointRevolute:
    joint_id: int
    i_marker: int
    j_marker: int


@dataclass
class JointFixed:
    i_marker: int
    j_marker: int


@dataclass
class PatchSurface:
    csurface_id: int
    name: str = ""
    rm_marker: Optional[int] = None
    patchtype: str = ""
    patches_raw: List[List[float]] = field(default_factory=list)
    nodes: List[Tuple[float, float, float]] = field(default_factory=list)


def _resolve_reuler_unit(parts: Dict[int, Part], markers: Dict[int, Marker], requested_unit: str) -> str:
    requested = requested_unit.lower().strip()
    if requested in {"deg", "rad"}:
        return requested

    max_abs = 0.0
    for p in parts.values():
        max_abs = max(max_abs, abs(p.reuler[0]), abs(p.reuler[1]), abs(p.reuler[2]))
    for m in markers.values():
        max_abs = max(max_abs, abs(m.reuler[0]), abs(m.reuler[1]), abs(m.reuler[2]))
    return "rad" if max_abs <= (2.0 * math.pi + 1e-6) else "deg"


def _convert_reulers_to_degrees(parts: Dict[int, Part], markers: Dict[int, Marker], source_unit: str) -> None:
    if source_unit == "deg":
        return
    for p in parts.values():
        p.reuler = (math.degrees(p.reuler[0]), math.degrees(p.reuler[1]), math.degrees(p.reuler[2]))
    for m in markers.values():
        m.reuler = (math.degrees(m.reuler[0]), math.degrees(m.reuler[1]), math.degrees(m.reuler[2]))


def _rotate_xyz_deg(v: Tuple[float, float, float], rdeg: Tuple[float, float, float]) -> Tuple[float, float, float]:
    x, y, z = v
    rx, ry, rz = math.radians(rdeg[0]), math.radians(rdeg[1]), math.radians(rdeg[2])

    cx, sx = math.cos(rx), math.sin(rx)
    y, z = y * cx - z * sx, y * sx + z * cx

    cy, sy = math.cos(ry), math.sin(ry)
    x, z = x * cy + z * sy, -x * sy + z * cy

    cz, sz = math.cos(rz), math.sin(rz)
    x, y = x * cz - y * sz, x * sz + y * cz
    return x, y, z


def _inverse_rotate_xyz_deg(v: Tuple[float, float, float], rdeg: Tuple[float, float, float]) -> Tuple[float, float, float]:
    x, y, z = v
    rx, ry, rz = math.radians(rdeg[0]), math.radians(rdeg[1]), math.radians(rdeg[2])

    cz, sz = math.cos(-rz), math.sin(-rz)
    x, y = x * cz - y * sz, x * sz + y * cz

    cy, sy = math.cos(-ry), math.sin(-ry)
    x, z = x * cy + z * sy, -x * sy + z * cy

    cx, sx = math.cos(-rx), math.sin(-rx)
    y, z = y * cx - z * sx, y * sx + z * cx
    return x, y, z


def _vadd(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> Tuple[float, float, float]:
    return a[0] + b[0], a[1] + b[1], a[2] + b[2]


def _vsub(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> Tuple[float, float, float]:
    return a[0] - b[0], a[1] - b[1], a[2] - b[2]


def _normalized(v: Tuple[float, float, float]) -> Tuple[float, float, float]:
    n = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    if n <= 1e-12:
        return 1.0, 0.0, 0.0
    return v[0] / n, v[1] / n, v[2] / n


def _surface_body_name(surface_name: str) -> Optional[str]:
    m = re.search(r"##(?:SOLID|GSURFACE)##_([^\.]+)\.", surface_name)
    return m.group(1) if m else None


def _marker_by_name(markers: Dict[int, Marker], name: str) -> Optional[Marker]:
    for marker in markers.values():
        if marker.name == name:
            return marker
    return None


def _parse_ip_values(line: str) -> Optional[List[float]]:
    m = re.search(r"IP\s*=\s*(.+)$", line)
    if not m:
        return None
    chunks = [x.strip() for x in m.group(1).split(",") if x.strip()]
    values: List[float] = []
    for token in chunks:
        try:
            values.append(_to_float(token))
        except ValueError:
            break
    return values if values else None


def parse_rmd(rmd_path: Path, reuler_unit: str = "auto") -> tuple[
    Dict[int, Part],
    Dict[int, Marker],
    Dict[int, PatchSurface],
    List[JointFixed],
    List[JointRevolute],
]:
    parts: Dict[int, Part] = {}
    markers: Dict[int, Marker] = {}
    surfaces: Dict[int, PatchSurface] = {}
    fixed_joints: List[JointFixed] = []
    revolute_joints: List[JointRevolute] = []

    current_entity: Optional[str] = None
    current_id: Optional[int] = None

    in_surface = False
    surface_id: Optional[int] = None
    reading_patch = False
    reading_node = False

    joint_is_fixed = False
    joint_is_revolute = False
    joint_current_id: Optional[int] = None
    joint_i: Optional[int] = None
    joint_j: Optional[int] = None

    pending_ip_products_part: Optional[int] = None

    def flush_joint_if_needed() -> None:
        nonlocal joint_is_fixed, joint_is_revolute, joint_current_id, joint_i, joint_j
        if joint_is_fixed and joint_i is not None and joint_j is not None:
            fixed_joints.append(JointFixed(i_marker=joint_i, j_marker=joint_j))
        if joint_is_revolute and joint_current_id is not None and joint_i is not None and joint_j is not None:
            revolute_joints.append(JointRevolute(joint_id=joint_current_id, i_marker=joint_i, j_marker=joint_j))
        joint_is_fixed = False
        joint_is_revolute = False
        joint_current_id = None
        joint_i = None
        joint_j = None

    with rmd_path.open("r", encoding="utf-8", errors="ignore") as f:
        for raw in f:
            line = raw.lstrip("\ufeff")
            stripped = line.strip()

            if _is_entity_start(line):
                if current_entity == "JOINT":
                    flush_joint_if_needed()

                reading_patch = False
                reading_node = False

                m = re.match(r"^([A-Z_]+)\s*/\s*(.*)$", stripped)
                current_entity = m.group(1) if m else None
                tail = m.group(2) if m else ""

                in_surface = current_entity in {"CSURFACE", "GGEOM"}
                if in_surface:
                    m2 = re.match(r"([0-9]+)", tail.strip())
                    if m2:
                        surface_id = int(m2.group(1))
                        surfaces[surface_id] = PatchSurface(csurface_id=surface_id)
                else:
                    surface_id = None

                if current_entity in {"PART", "MARKER", "JOINT"}:
                    m2 = re.match(r"([0-9]+)", tail.strip())
                    current_id = int(m2.group(1)) if m2 else None
                    if current_entity == "PART" and current_id is not None:
                        parts[current_id] = Part(part_id=current_id)
                    elif current_entity == "MARKER" and current_id is not None:
                        markers[current_id] = Marker(marker_id=current_id)
                    elif current_entity == "JOINT":
                        joint_current_id = current_id
                        joint_i = None
                        joint_j = None
                        joint_is_fixed = False
                        joint_is_revolute = False
                else:
                    current_id = None
                continue

            if in_surface and surface_id is not None:
                surf = surfaces[surface_id]
                if ", PATCH" in line and "=" in line:
                    reading_patch = True
                    reading_node = False
                    continue
                if ", NODE" in line and "=" in line:
                    reading_node = True
                    reading_patch = False
                    continue

                if reading_patch:
                    vals = _split_csv_payload(line)
                    if vals:
                        surf.patches_raw.append([_to_float(v) for v in vals])
                    continue

                if reading_node:
                    vals = _split_csv_payload(line)
                    if len(vals) >= 3:
                        surf.nodes.append((_to_float(vals[0]), _to_float(vals[1]), _to_float(vals[2])))
                    continue

                if "NAME" in line:
                    m = re.search(r"NAME\s*=\s*'([^']+)'", line)
                    if m:
                        surf.name = m.group(1)
                if "RM" in line:
                    v = _parse_key_value(line, "RM")
                    if v:
                        surf.rm_marker = int(v.split(",")[0].strip())
                if "PATCHTYPE" in line:
                    v = _parse_key_value(line, "PATCHTYPE")
                    if v:
                        surf.patchtype = v.split(",")[0].strip()
                continue

            if current_entity == "PART" and current_id in parts:
                part = parts[current_id]
                if "NAME" in line:
                    m = re.search(r"NAME\s*=\s*'([^']+)'", line)
                    if m:
                        part.name = m.group(1)
                elif re.search(r"\bCM\s*=", line):
                    v = _parse_key_value(line, "CM")
                    if v:
                        try:
                            part.cm_marker = int(v.split(",")[0].strip())
                        except ValueError:
                            part.cm_marker = None
                elif "MASS" in line:
                    v = _parse_key_value(line, "MASS")
                    if v:
                        part.mass = _to_float(v.split(",")[0])
                elif re.search(r"\bIP\s*=", line):
                    vals = _parse_ip_values(line) or []
                    if len(vals) >= 3:
                        part.ip_diag = (vals[0], vals[1], vals[2])
                    if len(vals) >= 6:
                        part.ip_products = (vals[3], vals[4], vals[5])
                        pending_ip_products_part = None
                    elif len(vals) == 3:
                        pending_ip_products_part = current_id
                elif "QG" in line:
                    m = re.search(r"QG\s*=\s*([^,]+),\s*([^,]+),\s*([^,\n]+)", line)
                    if m:
                        part.qg = (_to_float(m.group(1)), _to_float(m.group(2)), _to_float(m.group(3)))
                elif "REULER" in line:
                    m = re.search(r"REULER\s*=\s*([^,]+),\s*([^,]+),\s*([^,\n]+)", line)
                    if m:
                        part.reuler = (_to_float(m.group(1)), _to_float(m.group(2)), _to_float(m.group(3)))
                elif pending_ip_products_part == current_id and stripped.startswith(",") and "=" not in stripped:
                    vals = _split_csv_payload(line)
                    if len(vals) >= 3:
                        try:
                            part.ip_products = (_to_float(vals[0]), _to_float(vals[1]), _to_float(vals[2]))
                            pending_ip_products_part = None
                        except ValueError:
                            pass

            elif current_entity == "MARKER" and current_id in markers:
                marker = markers[current_id]
                if "NAME" in line:
                    m = re.search(r"NAME\s*=\s*'([^']+)'", line)
                    if m:
                        marker.name = m.group(1)
                elif re.search(r"\bPART\s*=", line):
                    v = _parse_key_value(line, "PART")
                    if v:
                        marker.part_id = int(v.split(",")[0].strip())
                elif "QP" in line:
                    m = re.search(r"QP\s*=\s*([^,]+),\s*([^,]+),\s*([^,\n]+)", line)
                    if m:
                        marker.qp = (_to_float(m.group(1)), _to_float(m.group(2)), _to_float(m.group(3)))
                elif "REULER" in line:
                    m = re.search(r"REULER\s*=\s*([^,]+),\s*([^,]+),\s*([^,\n]+)", line)
                    if m:
                        marker.reuler = (_to_float(m.group(1)), _to_float(m.group(2)), _to_float(m.group(3)))

            elif current_entity == "JOINT":
                if stripped.startswith(",") and "=" not in stripped:
                    joint_type = stripped[1:].strip().lower()
                    if joint_type.startswith("fixed"):
                        joint_is_fixed = True
                    elif joint_type.startswith("revolute"):
                        joint_is_revolute = True
                if re.search(r"\bI\s*=", line):
                    v = _parse_key_value(line, "I")
                    if v:
                        joint_i = int(v.split(",")[0].strip())
                if re.search(r"\bJ\s*=", line):
                    v = _parse_key_value(line, "J")
                    if v:
                        joint_j = int(v.split(",")[0].strip())

    if current_entity == "JOINT":
        flush_joint_if_needed()

    source_unit = _resolve_reuler_unit(parts, markers, reuler_unit)
    _convert_reulers_to_degrees(parts, markers, source_unit)

    return parts, markers, surfaces, fixed_joints, revolute_joints


def _triangles_from_surface(surface: PatchSurface) -> List[Tuple[int, int, int]]:
    triangles: List[Tuple[int, int, int]] = []
    for p in surface.patches_raw:
        if len(p) < 3:
            continue
        if int(round(p[0])) == 3 and len(p) >= 4:
            i, j, k = int(p[1]) - 1, int(p[2]) - 1, int(p[3]) - 1
        else:
            i, j, k = int(p[0]) - 1, int(p[1]) - 1, int(p[2]) - 1
        if i < 0 or j < 0 or k < 0:
            continue
        if i >= len(surface.nodes) or j >= len(surface.nodes) or k >= len(surface.nodes):
            continue
        if i == j or j == k or i == k:
            continue
        triangles.append((i, j, k))
    return triangles


def _export_body_obj(
    out_obj: Path,
    surfaces: List[PatchSurface],
    markers: Dict[int, Marker],
    body_translation: Tuple[float, float, float],
    body_rotation_deg: Tuple[float, float, float],
) -> tuple[int, int]:
    out_obj.parent.mkdir(parents=True, exist_ok=True)

    vertices: List[Tuple[float, float, float]] = []
    faces: List[Tuple[int, int, int]] = []

    for surface in surfaces:
        rm_translation = (0.0, 0.0, 0.0)
        rm_rotation = (0.0, 0.0, 0.0)
        if surface.rm_marker is not None and surface.rm_marker in markers:
            rm = markers[surface.rm_marker]
            rm_translation = rm.qp
            rm_rotation = rm.reuler

        base_index = len(vertices)
        transformed_nodes: List[Tuple[float, float, float]] = []
        for v in surface.nodes:
            # RM-local -> world -> body-local(CM)
            world_v = _vadd(_rotate_xyz_deg(v, rm_rotation), rm_translation)
            body_v = _inverse_rotate_xyz_deg(_vsub(world_v, body_translation), body_rotation_deg)
            transformed_nodes.append(body_v)

        vertices.extend(transformed_nodes)
        for (i, j, k) in _triangles_from_surface(surface):
            faces.append((base_index + i + 1, base_index + j + 1, base_index + k + 1))

    with out_obj.open("w", encoding="utf-8") as f:
        f.write("# Generated from RMD\n")
        for v in vertices:
            f.write(f"v {v[0]:.17g} {v[1]:.17g} {v[2]:.17g}\n")
        for (i, j, k) in faces:
            f.write(f"f {i} {j} {k}\n")

    return len(vertices), len(faces)


def _part_pose(part: Part, markers: Dict[int, Marker]) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
    cm_marker = markers.get(part.cm_marker) if part.cm_marker is not None else None
    if cm_marker is not None:
        return cm_marker.qp, cm_marker.reuler

    by_name = _marker_by_name(markers, f"{part.name}.CM")
    if by_name is not None:
        return by_name.qp, by_name.reuler

    return part.qg, part.reuler


def build_project_json(
    rmd_path: Path,
    parts: Dict[int, Part],
    markers: Dict[int, Marker],
    fixed_joints: List[JointFixed],
    revolute_joints: List[JointRevolute],
    mesh_rel_by_part_id: Dict[int, str],
) -> dict:
    bodies: List[dict] = []

    for part in sorted(parts.values(), key=lambda p: p.part_id):
        if part.name.upper() == "GROUND":
            continue

        position, _ = _part_pose(part, markers)
        ixx, iyy, izz = part.ip_diag
        ixy, ixz, iyz = part.ip_products

        body_entry = {
            "id": part.part_id,
            "name": part.name or f"Part{part.part_id}",
            "mass": part.mass,
            "position": [position[0], position[1], position[2]],
            "inertia": [
                [ixx, ixy, ixz],
                [ixy, iyy, iyz],
                [ixz, iyz, izz],
            ],
        }
        mesh_rel = mesh_rel_by_part_id.get(part.part_id)
        if mesh_rel is not None:
            body_entry["mesh"] = mesh_rel
        bodies.append(body_entry)

    joints: List[dict] = []

    def part_from_marker(marker_id: int) -> Optional[Part]:
        mk = markers.get(marker_id)
        if mk is None:
            return None
        return parts.get(mk.part_id)

    for joint in fixed_joints:
        pi = part_from_marker(joint.i_marker)
        pj = part_from_marker(joint.j_marker)
        if pi is None or pj is None:
            continue

        if pi.name.upper() == "GROUND" and pj.name.upper() != "GROUND":
            target = pj
        elif pj.name.upper() == "GROUND" and pi.name.upper() != "GROUND":
            target = pi
        else:
            continue

        pos, _ = _part_pose(target, markers)
        joints.append(
            {
                "type": "fixed",
                "body": target.part_id,
                "position": [pos[0], pos[1], pos[2]],
            }
        )

    for joint in revolute_joints:
        mi = markers.get(joint.i_marker)
        mj = markers.get(joint.j_marker)
        if mi is None or mj is None:
            continue

        pi = parts.get(mi.part_id)
        pj = parts.get(mj.part_id)
        if pi is None or pj is None:
            continue
        if pi.name.upper() == "GROUND" or pj.name.upper() == "GROUND":
            continue

        pos_i, _ = _part_pose(pi, markers)
        pos_j, _ = _part_pose(pj, markers)

        anchor_a = _vsub(mi.qp, pos_i)
        anchor_b = _vsub(mj.qp, pos_j)

        axis_i = _normalized(_rotate_xyz_deg((1.0, 0.0, 0.0), mi.reuler))
        axis_j = _normalized(_rotate_xyz_deg((1.0, 0.0, 0.0), mj.reuler))

        joints.append(
            {
                "type": "hinge",
                "body_a": pi.part_id,
                "body_b": pj.part_id,
                "anchor_a": [anchor_a[0], anchor_a[1], anchor_a[2]],
                "anchor_b": [anchor_b[0], anchor_b[1], anchor_b[2]],
                "axis_a": [axis_i[0], axis_i[1], axis_i[2]],
                "axis_b": [axis_j[0], axis_j[1], axis_j[2]],
            }
        )

    return {
        "case_name": rmd_path.stem,
        "description": f"Generated from {rmd_path.name}",
        "settings": {
            "dt": 0.001,
            "max_time": 1.0,
            "integrator_type": "ImplicitNewmark",
            "newmark_beta": 0.25,
            "newmark_gamma": 0.5,
            "joint_stiffness": 1.0e7,
            "output_dir": f"output/{rmd_path.stem}",
            "output_name": rmd_path.stem,
        },
        "bodies": bodies,
        "joints": joints,
        "constraints": [],
    }


def convert(
    rmd_path: Path,
    repo_root: Path,
    json_output: Optional[Path],
    reuler_unit: str = "auto",
) -> tuple[Path, Path]:
    parts, markers, surfaces, fixed_joints, revolute_joints = parse_rmd(rmd_path, reuler_unit=reuler_unit)

    mesh_dir = repo_root / "models" / "obj" / rmd_path.stem
    mesh_dir.mkdir(parents=True, exist_ok=True)

    surfaces_by_body: Dict[str, List[PatchSurface]] = {}
    for surface in surfaces.values():
        body_name = _surface_body_name(surface.name or "")
        if body_name is None:
            continue
        surfaces_by_body.setdefault(body_name, []).append(surface)

    parts_by_name = {p.name: p for p in parts.values()}
    mesh_rel_by_part_id: Dict[int, str] = {}
    mesh_manifest: List[dict] = []

    for body_name, body_surfaces in surfaces_by_body.items():
        part = parts_by_name.get(body_name)
        if part is None:
            continue

        body_translation, body_rotation = _part_pose(part, markers)
        obj_path = mesh_dir / f"{body_name}.obj"
        v_count, f_count = _export_body_obj(
            obj_path,
            body_surfaces,
            markers,
            body_translation,
            body_rotation,
        )
        mesh_rel = obj_path.relative_to(repo_root).as_posix()
        mesh_rel_by_part_id[part.part_id] = mesh_rel
        mesh_manifest.append(
            {
                "part_id": part.part_id,
                "part_name": body_name,
                "obj": mesh_rel,
                "vertex_count": v_count,
                "face_count": f_count,
            }
        )

    scene = build_project_json(
        rmd_path=rmd_path,
        parts=parts,
        markers=markers,
        fixed_joints=fixed_joints,
        revolute_joints=revolute_joints,
        mesh_rel_by_part_id=mesh_rel_by_part_id,
    )

    out_json = json_output if json_output is not None else rmd_path.with_suffix(".json")
    out_json.parent.mkdir(parents=True, exist_ok=True)
    out_json.write_text(json.dumps(scene, indent=2, ensure_ascii=False), encoding="utf-8")

    manifest = {
        "rmd": rmd_path.as_posix(),
        "json": out_json.as_posix(),
        "mesh_dir": mesh_dir.as_posix(),
        "mesh_count": len(mesh_manifest),
        "meshes": mesh_manifest,
        "reuler_unit": reuler_unit,
    }
    manifest_path = out_json.with_name(f"{rmd_path.stem}_project_convert_manifest.json")
    manifest_path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False), encoding="utf-8")

    return out_json, mesh_dir


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Export OBJ meshes and generate project-compatible JSON from RecurDyn RMD."
    )
    parser.add_argument("rmd", type=str, help="Path to input .rmd file")
    parser.add_argument(
        "--repo-root",
        type=str,
        default=None,
        help="Repository root (default: auto from script location)",
    )
    parser.add_argument(
        "--json-output",
        type=str,
        default=None,
        help="Output JSON path (default: same folder as rmd, same stem)",
    )
    parser.add_argument(
        "--reuler-unit",
        choices=["auto", "deg", "rad"],
        default="auto",
        help="Unit of REULER values in RMD",
    )
    args = parser.parse_args()

    rmd_path = Path(args.rmd).resolve()
    if not rmd_path.exists():
        raise FileNotFoundError(rmd_path)

    if args.repo_root is not None:
        repo_root = Path(args.repo_root).resolve()
    else:
        repo_root = Path(__file__).resolve().parents[1]

    json_output = Path(args.json_output).resolve() if args.json_output else None
    out_json, mesh_dir = convert(
        rmd_path=rmd_path,
        repo_root=repo_root,
        json_output=json_output,
        reuler_unit=args.reuler_unit,
    )

    print(f"RMD:      {rmd_path}")
    print(f"JSON:     {out_json}")
    print(f"OBJ dir:  {mesh_dir}")


if __name__ == "__main__":
    main()
