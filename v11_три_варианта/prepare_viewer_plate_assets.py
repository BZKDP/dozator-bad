from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import trimesh


ROOT = Path(__file__).resolve().parent
OUT = ROOT / "viewer_plate_assets"


def active_plates() -> dict[str, list[Path]]:
    a = ROOT / "A_верхнее_коромысло"
    b = ROOT / "B_роторная_кассета" / "PLATES_3MF"
    c = ROOT / "C_верхние_линейные_ножи" / "STL_ПЕЧАТЬ"
    return {
        "A": sorted(a.glob("*.3mf")),
        "B": sorted(b.glob("*.3mf")),
        "C": sorted(c.glob("*.3mf")),
    }


def transformed_scene_meshes(path: Path) -> list[tuple[str, trimesh.Trimesh]]:
    loaded = trimesh.load(path, force="scene", process=True)
    if isinstance(loaded, trimesh.Trimesh):
        return [("object_01", loaded)]
    if not isinstance(loaded, trimesh.Scene):
        raise TypeError(f"{path}: expected Trimesh or Scene")

    result: list[tuple[str, trimesh.Trimesh]] = []
    for index, node in enumerate(loaded.graph.nodes_geometry, start=1):
        transform, geometry_name = loaded.graph.get(node)
        mesh = loaded.geometry[geometry_name].copy()
        mesh.apply_transform(transform)
        result.append((str(geometry_name or f"object_{index:02d}"), mesh))
    return result


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    manifest: dict[str, object] = {
        "note": "Только облегчённые STL для интерактивного просмотра готовых 3MF-плит; печатать исходные 3MF.",
        "variants": {},
    }

    for variant, plate_paths in active_plates().items():
        if not plate_paths:
            raise FileNotFoundError(f"Не найдена активная 3MF-плита для {variant}")
        variant_dir = OUT / variant
        variant_dir.mkdir(parents=True, exist_ok=True)
        plate_entries: list[dict[str, object]] = []

        for plate_index, plate_path in enumerate(plate_paths, start=1):
            items: list[dict[str, object]] = []
            meshes = transformed_scene_meshes(plate_path)
            if not meshes:
                raise ValueError(f"Пустая 3MF-плита: {plate_path}")

            for item_index, (name, mesh) in enumerate(meshes, start=1):
                if not mesh.is_watertight or len(mesh.split(only_watertight=False)) != 1:
                    raise ValueError(
                        f"{plate_path.name}, object {item_index}: нужен один watertight body"
                    )
                filename = f"plate_{plate_index:02d}_part_{item_index:02d}.stl"
                target = variant_dir / filename
                mesh.export(target)
                items.append(
                    {
                        "file": target.relative_to(ROOT).as_posix(),
                        "name": name,
                        "bounds_mm": np.round(mesh.bounds, 3).tolist(),
                    }
                )

            combined = trimesh.util.concatenate([mesh for _, mesh in meshes])
            plate_entries.append(
                {
                    "source_3mf": plate_path.relative_to(ROOT).as_posix(),
                    "plate_number": plate_index,
                    "bounds_mm": np.round(combined.bounds, 3).tolist(),
                    "extents_mm": np.round(combined.extents, 3).tolist(),
                    "items": items,
                }
            )

        manifest["variants"][variant] = {  # type: ignore[index]
            "plate_count": len(plate_entries),
            "plates": plate_entries,
        }

    (OUT / "layout.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(OUT / "layout.json")


if __name__ == "__main__":
    main()
