from __future__ import annotations

import json
import hashlib
import math
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
import zipfile
from pathlib import Path
from typing import Any

import numpy as np
import trimesh


ROOT = Path(__file__).resolve().parent
BED = (220.0, 220.0)
SLICER_BLOCKING_MARKERS = (
    "Floating bridge anchors",
    "Loose extrusions",
    "Long bridging extrusions",
    "Collapsing overhang",
)
SLICER_ADHESION_MARKER = "Low bed adhesion"
VARIANTS = {
    "A": {
        "folder": ROOT / "A_верхнее_коромысло",
        "manifest": "manifest_A.json",
        "expected_print_stl": 4,
        "expected_3mf_plates": 1,
        "state_globs": [
            "STL_СБОРКА_НЕ_ПЕЧАТАТЬ/*HOME*",
            "STL_СБОРКА_НЕ_ПЕЧАТАТЬ/*MID*",
            "STL_СБОРКА_НЕ_ПЕЧАТАТЬ/*EJECT*",
        ],
    },
    "B": {
        "folder": ROOT / "B_роторная_кассета",
        "manifest": "manifest_V11_B.json",
        "expected_print_stl": 10,  # 9 рабочих + 1 одноразовый купон
        "expected_3mf_plates": 5,
        "state_globs": [
            "STL_СБОРКА_НЕ_ПЕЧАТАТЬ/*HOME*",
            "STL_СБОРКА_НЕ_ПЕЧАТАТЬ/*MID*",
            "STL_СБОРКА_НЕ_ПЕЧАТАТЬ/*EJECT*",
        ],
    },
    "C": {
        "folder": ROOT / "C_верхние_линейные_ножи",
        "manifest": "MANIFEST_C2.json",
        "expected_print_stl": 6,  # 5 рабочих + 1 одноразовый купон
        "expected_3mf_plates": 1,
        "state_globs": [
            "STL_СБОРКА_НЕ_ПЕЧАТАТЬ/*HOME*",
            "STL_СБОРКА_НЕ_ПЕЧАТАТЬ/*MID*",
            "STL_СБОРКА_НЕ_ПЕЧАТАТЬ/*EJECT*",
        ],
        "required_manifest_true": [
            "collision_checks.pass_at_0_01_mm3",
            "print.ready_3mf_plate.bed_220x220_pass",
            "print.ready_3mf_plate.overlap_pass",
        ],
    },
}


def load_json_strict(path: Path) -> Any:
    def reject_constant(value: str) -> None:
        raise ValueError(f"недопустимое JSON-значение {value}")

    return json.loads(path.read_text(encoding="utf-8"), parse_constant=reject_constant)


def finite_tree(value: Any) -> bool:
    if isinstance(value, float):
        return math.isfinite(value)
    if isinstance(value, dict):
        return all(finite_tree(v) for v in value.values())
    if isinstance(value, (list, tuple)):
        return all(finite_tree(v) for v in value)
    return True


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def iter_hashed_artifacts(value: Any) -> list[tuple[str, str]]:
    """Return every manifest record which binds a relative path to a SHA-256."""
    records: list[tuple[str, str]] = []
    if isinstance(value, dict):
        path = value.get("path")
        digest = value.get("sha256")
        if isinstance(path, str) and isinstance(digest, str):
            records.append((path, digest))
        for child in value.values():
            records.extend(iter_hashed_artifacts(child))
    elif isinstance(value, list):
        for child in value:
            records.extend(iter_hashed_artifacts(child))
    return records


def inspect_stl(path: Path) -> dict[str, Any]:
    # Processing merges the duplicated per-triangle STL vertices. Without it,
    # every triangle of an otherwise valid binary STL looks like a separate body.
    loaded = trimesh.load(path, force="mesh", process=True)
    if not isinstance(loaded, trimesh.Trimesh):
        raise TypeError(f"{path.name}: загружен не единый Trimesh")
    ext = [round(float(v), 3) for v in loaded.extents]
    bounds = [[round(float(v), 3) for v in row] for row in loaded.bounds]
    components = len(loaded.split(only_watertight=False))
    return {
        "file": str(path.relative_to(ROOT)),
        "extents_mm": ext,
        "bounds_mm": bounds,
        "watertight": bool(loaded.is_watertight),
        "components": int(components),
        "fits_220x220_xy": ext[0] <= BED[0] + 1e-6 and ext[1] <= BED[1] + 1e-6,
        "min_z_mm": bounds[0][2],
        "triangles": int(len(loaded.faces)),
    }


def inspect_3mf(path: Path) -> dict[str, Any]:
    with zipfile.ZipFile(path) as archive:
        bad_member = archive.testzip()
        if bad_member:
            raise ValueError(f"повреждён ZIP member {bad_member}")
        model_names = [name for name in archive.namelist() if name.lower().endswith(".model")]
        if not model_names:
            raise ValueError("нет 3D/*.model")
        root = ET.fromstring(archive.read(model_names[0]))
        objects = root.findall(".//{*}resources/{*}object")
        build_items = root.findall(".//{*}build/{*}item")
        mesh_objects = [obj for obj in objects if obj.find("{*}mesh") is not None]
        if not mesh_objects:
            raise ValueError("нет mesh-объектов")
        if not build_items:
            raise ValueError("build пуст")
        xml_result = {
            "file": str(path.relative_to(ROOT)),
            "unit": root.attrib.get("unit", "millimeter"),
            "objects": len(objects),
            "mesh_objects": len(mesh_objects),
            "build_items": len(build_items),
            "zip_member_count": len(archive.namelist()),
            "zip_integrity": True,
        }

    # Повторно открываем именно готовую плиту, а не исходные STL: так ловятся
    # ошибки раскладки, трансформаций и случайно слипшиеся детали внутри 3MF.
    loaded = trimesh.load(path, force="scene", process=True)
    if isinstance(loaded, trimesh.Trimesh):
        plate_meshes = [loaded]
    elif isinstance(loaded, trimesh.Scene):
        plate_meshes = list(loaded.dump(concatenate=False))
    else:
        raise TypeError("3MF не загрузился как mesh/scene")
    if len(plate_meshes) != len(build_items):
        raise ValueError(
            f"после загрузки найдено {len(plate_meshes)} тел при {len(build_items)} build items"
        )
    for index, mesh in enumerate(plate_meshes, start=1):
        if not mesh.is_watertight:
            raise ValueError(f"объект {index} в 3MF не герметичен")
        components = len(mesh.split(only_watertight=False))
        if components != 1:
            raise ValueError(f"объект {index} в 3MF содержит {components} несвязанных тел")

    plate_bounds = trimesh.util.concatenate(plate_meshes).bounds
    plate_extents = plate_bounds[1] - plate_bounds[0]
    if (
        plate_bounds[0][0] < -0.05
        or plate_bounds[0][1] < -0.05
        or plate_bounds[0][2] < -0.05
        or plate_bounds[1][0] > BED[0] + 0.05
        or plate_bounds[1][1] > BED[1] + 0.05
    ):
        raise ValueError(
            "раскладка выходит за координаты стола 0…220 мм: "
            f"bounds={plate_bounds.tolist()}"
        )

    intersections: list[dict[str, Any]] = []
    for right in range(len(plate_meshes)):
        for left in range(right):
            a, b = plate_meshes[left], plate_meshes[right]
            overlap = np.minimum(a.bounds[1], b.bounds[1]) - np.maximum(a.bounds[0], b.bounds[0])
            if np.all(overlap > 1e-5):
                common = trimesh.boolean.intersection([a, b], engine="manifold")
                volume = 0.0 if common is None else abs(float(common.volume))
                if volume > 0.05:
                    intersections.append(
                        {"objects": [left + 1, right + 1], "volume_mm3": round(volume, 4)}
                    )
    if intersections:
        raise ValueError(f"детали на плите пересекаются: {intersections}")

    xml_result.update(
        {
            "plate_bounds_mm": [[round(float(v), 3) for v in row] for row in plate_bounds],
            "plate_extents_mm": [round(float(v), 3) for v in plate_extents],
            "all_objects_watertight_one_body": True,
            "object_intersections": [],
            "inside_220x220": True,
            "minimum_xy_edge_margin_mm": round(
                float(
                    min(
                        plate_bounds[0][0],
                        plate_bounds[0][1],
                        BED[0] - plate_bounds[1][0],
                        BED[1] - plate_bounds[1][1],
                    )
                ),
                3,
            ),
        }
    )
    return xml_result


def find_prusa_slicer() -> Path | None:
    discovered = shutil.which("prusa-slicer-console") or shutil.which("prusa-slicer")
    candidates = [
        Path(discovered) if discovered else None,
        Path(r"C:\Program Files\Prusa3D\PrusaSlicer\prusa-slicer-console.exe"),
    ]
    return next((path for path in candidates if path and path.is_file()), None)


PRUSA_SLICER = find_prusa_slicer()


def inspect_slicer(path: Path) -> dict[str, Any]:
    """Slice the actual ready plate and reject unsupported/floating toolpaths.

    Watertight meshes and an in-bed AABB do not prove FDM printability.  This
    check deliberately uses the locally installed production slicer with
    supports disabled (its CLI default) and treats structural bridge warnings
    as hard failures.  Low adhesion remains a warning because a documented
    brim may be the correct remedy.
    """
    if PRUSA_SLICER is None:
        return {
            "available": False,
            "status": "SKIP",
            "blocking_warnings": [],
            "adhesion_warning": False,
            "note": "PrusaSlicer CLI не найден; проверка траекторий не выполнена",
        }

    with tempfile.TemporaryDirectory(prefix="v11_slicer_audit_") as tmp:
        output = Path(tmp) / "audit.gcode"
        brim_mm = 0.0
        perimeters = 4
        infill_percent = 30
        normalized = path.as_posix()
        if "/A_верхнее_коромысло/" in normalized:
            brim_mm = 4.0
            infill_percent = 40
        if "/B_роторная_кассета/" in normalized and "plate_02" in path.name:
            brim_mm = 4.0
        if "/00_ОБЩАЯ_КОМПОНОВКА/" in normalized:
            perimeters = 5
            infill_percent = 40
            brim_mm = 5.0
        command = [
            str(PRUSA_SLICER),
            "--slice",
            "--no-support-material",
            "--bed-shape",
            "0x0,220x0,220x220,0x220",
            "--layer-height",
            "0.20",
            "--perimeters",
            str(perimeters),
            "--fill-density",
            f"{infill_percent}%",
            "--top-solid-layers",
            "5",
            "--bottom-solid-layers",
            "5",
        ]
        if brim_mm:
            command.extend(["--brim-width", str(brim_mm)])
        command.extend(["--output", str(output), str(path)])
        completed = subprocess.run(
            command,
            cwd=ROOT,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=180,
            check=False,
        )
        log = completed.stdout or ""
        blocking = [marker for marker in SLICER_BLOCKING_MARKERS if marker in log]
        adhesion = SLICER_ADHESION_MARKER in log
        gcode_created = output.is_file() and output.stat().st_size > 0
        return {
            "available": True,
            "executable": str(PRUSA_SLICER),
            "exit_code": int(completed.returncode),
            "gcode_created": gcode_created,
            "status": (
                "PASS"
                if completed.returncode == 0 and gcode_created and not blocking
                else "FAIL"
            ),
            "blocking_warnings": blocking,
            "adhesion_warning": adhesion,
            "profile": {
                "bed_mm": [220, 220],
                "layer_height_mm": 0.20,
                "perimeters": perimeters,
                "infill_percent": infill_percent,
                "top_solid_layers": 5,
                "bottom_solid_layers": 5,
                "supports": False,
                "brim_mm": brim_mm,
            },
        }


def find_hard_failures(value: Any, trail: str = "") -> list[str]:
    found: list[str] = []
    if isinstance(value, dict):
        for key, child in value.items():
            here = f"{trail}.{key}" if trail else key
            normalized = key.lower().replace(" ", "_")
            if normalized in {"hard_failures", "failures", "errors"} and child:
                found.append(f"{here}: {child}")
            found.extend(find_hard_failures(child, here))
    elif isinstance(value, list):
        for i, child in enumerate(value):
            found.extend(find_hard_failures(child, f"{trail}[{i}]"))
    return found


def dotted_get(value: Any, path: str) -> Any:
    current = value
    for part in path.split("."):
        if not isinstance(current, dict) or part not in current:
            raise KeyError(path)
        current = current[part]
    return current


def inspect_common_cassette() -> dict[str, Any]:
    """Validate the *active* six-sheet cassette and its eight final fixators.

    This deliberately does more than opening meshes: the manifest is treated as
    a signed bill of materials and every safety-critical geometric proof must be
    present.  Legacy prototypes may remain in LEGACY_DO_NOT_USE, but cannot be
    referenced by the active package.
    """
    base = ROOT / "00_ОБЩАЯ_КОМПОНОВКА"
    errors: list[str] = []
    warnings: list[str] = []

    def require(condition: bool, message: str) -> None:
        if not condition:
            errors.append(message)

    canonical_dir = base / "STL_КАССЕТА_СБОРКА"
    expected_canonical = {
        "cassette_upper_sheet.stl",
        "cassette_lower_sheet.stl",
        "cassette_divider.stl",
    }
    actual_canonical = {path.name for path in canonical_dir.glob("*.stl")}
    require(
        actual_canonical == expected_canonical,
        "активные STL кассеты не равны трём каноническим файлам: "
        f"ожидалось {sorted(expected_canonical)}, найдено {sorted(actual_canonical)}",
    )

    parts: list[dict[str, Any]] = []
    for name in sorted(expected_canonical & actual_canonical):
        path = canonical_dir / name
        try:
            item = inspect_stl(path)
            parts.append(item)
            require(item["watertight"], f"{name}: сетка не герметична")
            require(item["components"] == 1, f"{name}: не одно связное тело")
        except Exception as exc:
            errors.append(f"{name}: {exc}")

    print_dir = base / "STL_ПЕЧАТЬ"
    expected_print = "фиксатор_ската_универсальный_V11_PRINT.stl"
    active_print = sorted(path.name for path in print_dir.glob("*.stl"))
    require(active_print == [expected_print],
            f"активный набор печати фиксатора неверен: {active_print}")
    print_part: dict[str, Any] | None = None
    if (print_dir / expected_print).is_file():
        try:
            print_part = inspect_stl(print_dir / expected_print)
            require(print_part["watertight"], "универсальный фиксатор не герметичен")
            require(print_part["components"] == 1,
                    "универсальный фиксатор содержит не одно связное тело")
            require(print_part["fits_220x220_xy"],
                    "универсальный фиксатор не помещается на 220×220")
            require(abs(print_part["min_z_mm"]) <= 0.05,
                    "универсальный фиксатор не лежит на столе в сохранённой ориентации")
        except Exception as exc:
            errors.append(f"{expected_print}: {exc}")

    plate_path = base / "PLATES_3MF" / "КАССЕТА_8_ВНУТРЕННИХ_ФИКСАТОРОВ_220x220.3mf"
    active_plates = sorted(path.name for path in (base / "PLATES_3MF").glob("*.3mf"))
    require(active_plates == [plate_path.name],
            f"активная 3MF-плита фиксаторов неверна: {active_plates}")
    plate: dict[str, Any] | None = None
    if plate_path.is_file():
        try:
            plate = inspect_3mf(plate_path)
            require(plate["build_items"] == 8,
                    f"в плите фиксаторов {plate['build_items']} объектов вместо 8")
            plate["prusa_slicer_audit"] = inspect_slicer(plate_path)
            slicer = plate["prusa_slicer_audit"]
            require(slicer["status"] == "PASS",
                    "плита фиксаторов не прошла свежий PrusaSlicer-аудит: "
                    f"{slicer.get('blocking_warnings', [])}")
            if slicer.get("adhesion_warning"):
                warnings.append("для плиты восьми фиксаторов обязателен brim 5 мм")
        except Exception as exc:
            errors.append(f"плита восьми фиксаторов: {exc}")
    else:
        errors.append(f"нет {plate_path.name}")

    # Nothing carrying the rejected blind-clip/X-lock identity may leak back
    # into an active manufacturing folder.
    for folder in (canonical_dir, print_dir, base / "PLATES_3MF", base / "STL_ЗАМКИ_СБОРКА"):
        for path in folder.glob("*"):
            lowered = path.name.lower()
            if "cassette_lock_" in lowered or "x_зам" in lowered or "x-lock" in lowered:
                errors.append(f"устаревший X-lock остался активным: {path.relative_to(ROOT)}")

    manifest_path = base / "ФИКСАТОРЫ_V11_MANIFEST.json"
    manifest: dict[str, Any] | None = None
    if not manifest_path.is_file():
        errors.append(f"нет {manifest_path.name}")
    else:
        try:
            raw = load_json_strict(manifest_path)
            require(isinstance(raw, dict), "манифест фиксаторов не является объектом JSON")
            if isinstance(raw, dict):
                manifest = raw
                require(finite_tree(manifest), "в манифесте фиксаторов есть NaN/Infinity")
                errors.extend(find_hard_failures(manifest))

                # Every artifact tied to a digest must still be exactly the one
                # which underwent the recorded checks.
                seen_hash_paths: set[str] = set()
                for relative, expected_hash in iter_hashed_artifacts(manifest):
                    if relative in seen_hash_paths:
                        continue
                    seen_hash_paths.add(relative)
                    artifact = base / Path(relative)
                    require(artifact.is_file(), f"манифест ссылается на отсутствующий файл: {relative}")
                    if artifact.is_file():
                        require(
                            sha256_file(artifact).lower() == expected_hash.lower(),
                            f"SHA-256 не совпадает: {relative}",
                        )
                for relative, expected_hash in manifest.get("cut_file_sha256", {}).items():
                    artifact = base / Path(relative)
                    require(artifact.is_file(), f"нет файла раскроя из манифеста: {relative}")
                    if artifact.is_file():
                        require(sha256_file(artifact).lower() == str(expected_hash).lower(),
                                f"SHA-256 раскроя не совпадает: {relative}")

                mechanical = manifest.get("mechanical_connection", {})
                require(mechanical.get("mandatory_fixator_count") == 8,
                        "манифест не требует ровно 8 фиксаторов")
                require(mechanical.get("upper_count") == 4 and mechanical.get("lower_count") == 4,
                        "разбиение фиксаторов должно быть 4 верхних + 4 нижних")
                require(mechanical.get("peg_engagement_mm", 0) >= 3.0,
                        "номинальное зацепление штифта меньше 3 мм")
                require(mechanical.get("bank_surface_recess_mm", 0) >= 1.0,
                        "штифт утоплен от поверхности банки менее чем на 1 мм")
                require(abs(float(mechanical.get("shoe_gap_mm", 999))) <= 1e-9,
                        "опорная пятка не имеет касательного нулевого базирования")
                require("outward-open" in str(mechanical.get("printed_bore", "")),
                        "печатная люлька болта не подтверждена как открытая наружу")
                stack = mechanical.get("clearance_stack", {})
                require(stack.get("worst_remaining_peg_engagement_mm", 0) > 2.0,
                        "при худшем люфте зацепление штифта не превышает 2 мм")

                washer = manifest.get("washer_bearing", {})
                require(washer.get("result") == "PASS",
                        "широкая шайба не прошла проверку опорной площади PETG")
                require(washer.get("outer_diameter_mm", 0) >= 12.0,
                        "для открытого седла требуется широкая шайба M4 Ø12 мм")
                require(washer.get("printed_face_bearing_area_mm2", 0) >= 60.0,
                        "опорная площадь шайбы на PETG меньше 60 мм²")

                collisions = manifest.get("collision_checks", [])
                require(len(collisions) == 8,
                        f"должно быть 8 отчётов коллизий, найдено {len(collisions)}")
                for index, check in enumerate(collisions, start=1):
                    for key in ("sheet_intersection_mm3", "divider_intersection_mm3",
                                "opposite_sheet_intersection_mm3"):
                        require(abs(float(check.get(key, 999))) <= 0.001,
                                f"фиксатор {index}: ненулевая коллизия {key}")
                    for key in ("peg_axis_alignment_error_mm", "bolt_axis_alignment_error_mm"):
                        require(abs(float(check.get(key, 999))) <= 1e-6,
                                f"фиксатор {index}: ошибка соосности {key}")

                hardware = manifest.get("hardware_collision_checks", {})
                for key in ("hardware_vs_upper_sheet_mm3", "hardware_vs_lower_sheet_mm3",
                            "hardware_vs_4_exact_dividers_mm3"):
                    require(abs(float(hardware.get(key, 999))) <= 0.001,
                            f"крепёж пересекает кассету: {key}")
                require(hardware.get("can_half_space_result") == "PASS",
                        "крепёж не прошёл проверку пространства банки")

                anti = manifest.get("anti_rotation", {})
                for group_name in ("nominal", "worst_clearance_stack"):
                    group = anti.get(group_name, {})
                    for kind in ("upper", "lower"):
                        sweeps = group.get(kind, [])
                        require(len(sweeps) == 2,
                                f"anti_rotation.{group_name}.{kind}: нужны два направления")
                        for sweep in sweeps:
                            require(sweep.get("contact_before_release") is True,
                                    f"anti_rotation.{group_name}.{kind}: возможен выход до упора")
                worst = anti.get("worst_clearance_stack", {})
                require(worst.get("remaining_peg_engagement_mm", 0) > 2.0,
                        "anti_rotation: остаточное зацепление в худшем случае ≤2 мм")

                can_check = manifest.get("can_swept_envelope_check", {})
                require(str(can_check.get("result", "")).startswith("PASS"),
                        "фиксатор не прошёл консервативную проверку пространства банки")
                require(can_check.get("upper_clearance_mm", 0) > 0
                        and can_check.get("lower_clearance_mm", 0) > 0,
                        "нет положительного зазора до пространства банки")

                box = manifest.get("box_clearance", {})
                require(box.get("complete_cassette_inside_nominal_box") is True,
                        "полная кассета с крепежом не подтверждена внутри коробки")
                require(abs(float(box.get("installation_translation_x_mm", 999)) - 0.5) <= 1e-9,
                        "не зафиксирован монтажный сдвиг кассеты X=+0,5 мм")
                nominal = box.get("nominal_inside_mm", [])
                installed = box.get("complete_cassette_installed_bounds_mm", [])
                if len(nominal) == 3 and len(installed) == 2:
                    require(all(float(installed[0][axis]) >= -1e-6 for axis in range(3))
                            and all(float(installed[1][axis]) <= float(nominal[axis]) + 1e-6
                                    for axis in range(3)),
                            "числовые границы полной кассеты выходят за 595×395×395")
                else:
                    errors.append("в манифесте нет полных числовых границ кассеты")

                require(manifest.get("validation_limits", {}).get("physical_validation_required") is True,
                        "манифест обязан явно требовать физические испытания")
                require(manifest.get("slice_validation", {}).get("status") == "PASS",
                        "зафиксированный слайсинг фиксаторов не PASS")
                require(manifest.get("print_profile", {}).get("supports") is False,
                        "для фиксаторов должны быть отключены поддержки")
        except Exception as exc:
            errors.append(f"{manifest_path.name}: {exc}")

    return {
        "status": "PASS" if not errors else "FAIL",
        "parts": parts,
        "print_part": print_part,
        "ready_fixator_plate": plate,
        "manifest": str(manifest_path.relative_to(ROOT)),
        "errors": errors,
        "warnings": warnings,
    }


def inspect_variant(code: str, cfg: dict[str, Any]) -> dict[str, Any]:
    folder: Path = cfg["folder"]
    errors: list[str] = []
    warnings: list[str] = []
    print_dir = folder / "STL_ПЕЧАТЬ"
    print_files = sorted(print_dir.glob("*.stl"))
    if len(print_files) != cfg["expected_print_stl"]:
        errors.append(
            f"ожидалось {cfg['expected_print_stl']} STL для печати, найдено {len(print_files)}"
        )

    parts: list[dict[str, Any]] = []
    for path in print_files:
        try:
            item = inspect_stl(path)
            parts.append(item)
            if not item["watertight"]:
                errors.append(f"{path.name}: сетка не герметична")
            if item["components"] != 1:
                errors.append(f"{path.name}: {item['components']} несвязанных тел вместо одного")
            if not item["fits_220x220_xy"]:
                errors.append(f"{path.name}: не помещается на стол 220×220 в сохранённой ориентации")
            if abs(item["min_z_mm"]) > 0.05:
                warnings.append(f"{path.name}: min Z = {item['min_z_mm']} мм, слайсер должен положить на стол")
        except Exception as exc:
            errors.append(f"{path.name}: {exc}")

    manifest_path = folder / cfg["manifest"]
    manifest: Any = None
    if not manifest_path.exists():
        errors.append(f"нет манифеста {manifest_path.name}")
    else:
        try:
            manifest = load_json_strict(manifest_path)
            if not finite_tree(manifest):
                errors.append(f"{manifest_path.name}: найдено NaN/Infinity")
            errors.extend(find_hard_failures(manifest))
            for required_path in cfg.get("required_manifest_true", []):
                try:
                    if dotted_get(manifest, required_path) is not True:
                        errors.append(
                            f"{manifest_path.name}: обязательная проверка "
                            f"{required_path} не равна true"
                        )
                except KeyError:
                    errors.append(
                        f"{manifest_path.name}: нет обязательной проверки {required_path}"
                    )
        except Exception as exc:
            errors.append(f"{manifest_path.name}: {exc}")

    state_counts: dict[str, int] = {}
    for pattern in cfg["state_globs"]:
        label = pattern.split("*")[1]
        count = len(list(folder.glob(pattern)))
        state_counts[label] = count
        if count == 0:
            errors.append(f"не найдены сборочные STL состояния {label}")

    plates: list[dict[str, Any]] = []
    # Проверяем только активные пластины варианта. Архивные/отбракованные
    # концепции могут лежать глубже в REJECTED_* и не должны давать ложный PASS.
    plates_dir = folder / "PLATES_3MF"
    plate_files = sorted(
        set(folder.glob("*.3mf"))
        | set(print_dir.glob("*.3mf"))
        | (set(plates_dir.glob("*.3mf")) if plates_dir.exists() else set())
    )
    if not plate_files:
        errors.append("нет готовой 3MF-плиты для открытия в слайсере")
    expected_plates = cfg.get("expected_3mf_plates")
    if expected_plates is not None and len(plate_files) != expected_plates:
        errors.append(
            f"ожидалось {expected_plates} готовых 3MF-плит, найдено {len(plate_files)}"
        )
    for path in plate_files:
        try:
            plate = inspect_3mf(path)
            slicer = inspect_slicer(path)
            plate["prusa_slicer_audit"] = slicer
            if slicer["status"] == "FAIL":
                reason = ", ".join(slicer["blocking_warnings"])
                if not slicer.get("gcode_created"):
                    reason = (reason + "; " if reason else "") + "G-code не создан"
                errors.append(
                    f"{path.name}: PrusaSlicer обнаружил непечатаемую без поддержек геометрию: "
                    + reason
                )
            elif slicer["status"] == "SKIP":
                warnings.append(f"{path.name}: {slicer['note']}")
            if slicer["adhesion_warning"]:
                warnings.append(
                    f"{path.name}: PrusaSlicer предупреждает о слабой адгезии; "
                    "использовать указанный в инструкции brim"
                )
            plates.append(plate)
            if plate["minimum_xy_edge_margin_mm"] < 3.0:
                warnings.append(
                    f"{path.name}: край детали всего в "
                    f"{plate['minimum_xy_edge_margin_mm']} мм от границы стола; brim нужно учитывать отдельно"
                )
        except Exception as exc:
            errors.append(f"{path.name}: {exc}")

    packaged_objects = sum(int(plate.get("build_items", 0)) for plate in plates)
    if plates and packaged_objects != len(print_files):
        errors.append(
            "суммарно в готовых 3MF находится "
            f"{packaged_objects} объектов при {len(print_files)} печатных STL"
        )

    return {
        "status": "PASS" if not errors else "FAIL",
        "print_stl_count": len(print_files),
        "print_parts": parts,
        "state_asset_counts": state_counts,
        "ready_3mf_plates": plates,
        "manifest": str(manifest_path.relative_to(ROOT)),
        "errors": errors,
        "warnings": warnings,
    }


def main() -> int:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    common = inspect_common_cassette()
    results = {code: inspect_variant(code, cfg) for code, cfg in VARIANTS.items()}
    viewer = ROOT / "ОТКРОЙ_V11_ТРИ_ВАРИАНТА.html"
    viewer_errors: list[str] = []
    if not viewer.exists():
        viewer_errors.append("нет итогового HTML-просмотрщика")
    else:
        text = viewer.read_text(encoding="utf-8")
        if "C_опрокидывающаяся_люлька" in text:
            viewer_errors.append("в просмотрщик случайно включена отбракованная люлька")
        forbidden_active_refs = (
            "КАССЕТА_8_X_ЗАМКОВ",
            "cassette_lock_",
            "X-lock",
            "X‑зам",
        )
        for marker in forbidden_active_refs:
            if marker in text:
                viewer_errors.append(f"просмотрщик содержит устаревшую ссылку/термин: {marker}")
        required_viewer_refs = (
            "КАССЕТА_8_ВНУТРЕННИХ_ФИКСАТОРОВ_220x220.3mf",
            "фиксатор_верхнего_ската_world_y79_V11.stl",
            "фиксатор_нижнего_ската_world_y79_V11.stl",
            "M4x25_болты_8шт_world_REFERENCE.stl",
        )
        for marker in required_viewer_refs:
            if marker not in text:
                viewer_errors.append(f"просмотрщик не содержит обязательный новый ресурс: {marker}")

    overall_errors = list(common["errors"]) + list(viewer_errors)
    for code, result in results.items():
        overall_errors.extend(f"{code}: {message}" for message in result["errors"])

    report = {
        "project": "V11 — три варианта для одной SG90/MG90S на канал",
        "status": "PASS" if not overall_errors else "FAIL",
        "scope": "геометрия, восемь внутренних фиксаторов, хеши раскроя/STL/3MF, коллизии, удержание при худших люфтах, границы полной кассеты, 3MF вариантов и свежий PrusaSlicer-аудит; физические испытания всё равно обязательны",
        "bed_mm": list(BED),
        "common_cassette": common,
        "variants": results,
        "viewer": {
            "file": viewer.name,
            "status": "PASS" if not viewer_errors else "FAIL",
            "errors": viewer_errors,
        },
        "overall_errors": overall_errors,
    }
    out = ROOT / "ОТЧЕТ_V11_ОБЩИЙ.json"
    out.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    print(f"{report['status']}: {out}")
    for message in overall_errors:
        print(f"  - {message}")
    return 0 if not overall_errors else 1


if __name__ == "__main__":
    raise SystemExit(main())
