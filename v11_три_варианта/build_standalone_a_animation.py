from __future__ import annotations

import base64
import json
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parent
SOURCE = ROOT / "DEV_ИСТОЧНИК_АНИМАЦИИ_A_НЕ_ОТКРЫВАТЬ.html"
TARGET = ROOT / "АНИМАЦИЯ_A_ПЛАВНАЯ.html"
LIBS = ROOT.parent / "libs"
ASSEMBLY = ROOT / "A_верхнее_коромысло" / "STL_СБОРКА_НЕ_ПЕЧАТАТЬ"

STL_FILES = (
    "A_VIEW_HOME_frame.stl",
    "A_VIEW_HOME_clamp.stl",
    "A_VIEW_HOME_rocker.stl",
    "A_VIEW_HOME_link.stl",
    "A_VIEW_HOME_servo.stl",
    "A_VIEW_HOME_horn.stl",
    "A_VIEW_HOME_lower_sheet.stl",
    "A_VIEW_HOME_can_1.stl",
    "A_VIEW_HOME_can_2.stl",
    "A_VIEW_HOME_can_3.stl",
)


def b64(path: Path) -> str:
    return base64.b64encode(path.read_bytes()).decode("ascii")


def build() -> None:
    source = SOURCE.read_text(encoding="utf-8")
    three_b64 = b64(LIBS / "three.module.js")
    orbit_b64 = b64(LIBS / "OrbitControls.js")
    stl_b64 = {name: b64(ASSEMBLY / name) for name in STL_FILES}

    source = re.sub(
        r"\s*<script type=\"importmap\">.*?</script>\s*",
        "\n",
        source,
        count=1,
        flags=re.DOTALL,
    )

    imports = """import * as THREE from 'three';
import {OrbitControls} from '../libs/OrbitControls.js';"""
    bootstrap = f"""const decodeEmbeddedText=value=>new TextDecoder().decode(Uint8Array.from(atob(value),character=>character.charCodeAt(0)));
const threeModuleUrl=URL.createObjectURL(new Blob([decodeEmbeddedText('{three_b64}')],{{type:'text/javascript'}}));
const THREE=await import(threeModuleUrl);
const orbitSource=decodeEmbeddedText('{orbit_b64}').replace(\"from 'three';\",\"from '\"+threeModuleUrl+\"';\");
const orbitModuleUrl=URL.createObjectURL(new Blob([orbitSource],{{type:'text/javascript'}}));
const {{OrbitControls}}=await import(orbitModuleUrl);"""
    if imports not in source:
        raise RuntimeError("Import block was not found in the development source")
    source = source.replace(imports, bootstrap, 1)

    marker = "const FILES={"
    embedded = "const EMBEDDED_STL=" + json.dumps(
        stl_b64, ensure_ascii=False, separators=(",", ":")
    ) + ";\n"
    if marker not in source:
        raise RuntimeError("FILES marker was not found in the development source")
    source = source.replace(marker, embedded + marker, 1)

    old_loader = """async function loadGeometry(file){
  const response=await fetch(ASSET+file);if(!response.ok)throw new Error(response.status+' · '+file);
  return parseSTL(await response.arrayBuffer());
}"""
    new_loader = """function embeddedBuffer(value){
  const raw=atob(value),bytes=new Uint8Array(raw.length);
  for(let i=0;i<raw.length;i++)bytes[i]=raw.charCodeAt(i);
  return bytes.buffer;
}
async function loadGeometry(file){
  const embedded=EMBEDDED_STL[file];
  if(embedded)return parseSTL(embeddedBuffer(embedded));
  const response=await fetch(ASSET+file);if(!response.ok)throw new Error(response.status+' · '+file);
  return parseSTL(await response.arrayBuffer());
}"""
    if old_loader not in source:
        raise RuntimeError("STL loader block was not found in the development source")
    source = source.replace(old_loader, new_loader, 1)

    source = source.replace(
        "<!doctype html>",
        "<!doctype html>\n<!-- Автономная сборка: Three.js, OrbitControls и демонстрационные STL встроены внутрь. -->",
        1,
    )
    TARGET.write_text(source, encoding="utf-8", newline="\n")


if __name__ == "__main__":
    build()
    print(f"PASS: {TARGET} ({TARGET.stat().st_size} bytes)")
