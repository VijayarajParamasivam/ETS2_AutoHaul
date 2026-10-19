"""Generate the shared-memory layout for C++ and Python from native/spec/{layout,inputs}.toml.

    python tools/gen_layout.py          # write both outputs
    python tools/gen_layout.py --check  # exit 1 if the outputs are stale (used by tests)

Offsets are computed once with ctypes (MSVC-compatible natural alignment) and emitted into the
C header as static_asserts, so the two sides cannot silently disagree.
"""

from __future__ import annotations

import argparse
import ctypes
import hashlib
import re
import sys
import tomllib
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SPEC = ROOT / "native/spec/layout.toml"
INPUTS = ROOT / "native/spec/inputs.toml"
OUT_H = ROOT / "native/bridge/generated/autohaul_layout.h"
OUT_PY = ROOT / "autohaul/io/_layout.py"

PRIM = {
    "u8": (ctypes.c_uint8, "uint8_t", "ctypes.c_uint8"),
    "u16": (ctypes.c_uint16, "uint16_t", "ctypes.c_uint16"),
    "u32": (ctypes.c_uint32, "uint32_t", "ctypes.c_uint32"),
    "u64": (ctypes.c_uint64, "uint64_t", "ctypes.c_uint64"),
    "s32": (ctypes.c_int32, "int32_t", "ctypes.c_int32"),
    "s64": (ctypes.c_int64, "int64_t", "ctypes.c_int64"),
    "f32": (ctypes.c_float, "float", "ctypes.c_float"),
    "f64": (ctypes.c_double, "double", "ctypes.c_double"),
    "char": (ctypes.c_char, "char", "ctypes.c_char"),
}

# SDK value type -> (SCS constant, expected destination type name)
SDK_TYPES = {
    "bool": ("SCS_VALUE_TYPE_bool", "u8"),
    "s32": ("SCS_VALUE_TYPE_s32", "s32"),
    "u32": ("SCS_VALUE_TYPE_u32", "u32"),
    "u64": ("SCS_VALUE_TYPE_u64", "u64"),
    "float": ("SCS_VALUE_TYPE_float", "f32"),
    "double": ("SCS_VALUE_TYPE_double", "f64"),
    "fvector": ("SCS_VALUE_TYPE_fvector", "Vec3"),
    "fplacement": ("SCS_VALUE_TYPE_fplacement", "FPlacement"),
    "dplacement": ("SCS_VALUE_TYPE_dplacement", "DPlacement"),
    "string": ("SCS_VALUE_TYPE_string", "char"),
}

TYPE_RE = re.compile(r"^(\w+)(?:\[(\w+)\])?$")


@dataclass
class FieldInfo:
    offset: int
    size: int
    type_name: str      # primitive or struct name of the (element) type
    count: int          # array length, 1 for scalars


def load() -> tuple[dict, dict, bytes]:
    raw_spec = SPEC.read_bytes()
    raw_inputs = INPUTS.read_bytes()
    spec = tomllib.loads(raw_spec.decode("utf-8"))
    inputs = tomllib.loads(raw_inputs.decode("utf-8"))
    # Hash normalized content (line endings differ between checkouts).
    digest = hashlib.sha256(raw_spec.replace(b"\r\n", b"\n") + b"\0" + raw_inputs.replace(b"\r\n", b"\n")).digest()
    return spec, inputs, digest


class Layout:
    def __init__(self, spec: dict, inputs: dict, digest: bytes):
        self.spec = spec
        self.inputs = inputs
        self.version = spec["version"]
        self.layout_hash = int.from_bytes(digest[:8], "little")
        self.consts: dict[str, int] = dict(spec["constants"])
        self.axes = inputs["axes"]
        self.buttons = inputs["buttons"]
        self.consts["NUM_AXES"] = len(self.axes)
        self.consts["NUM_BUTTONS"] = len(self.buttons)
        self.consts["NUM_INPUTS"] = len(self.axes) + len(self.buttons)
        self.structs: dict[str, type[ctypes.Structure]] = {}
        self.struct_fields: dict[str, list[tuple[str, str]]] = {}
        for s in spec["struct"]:
            self.add_struct(s["name"], [tuple(f) for f in s["fields"]])
        self.add_struct("ControlsBlock", [
            ("header", "ControlsHeader"),
            ("axes", "f32[NUM_AXES]"),
            ("holds", "u8[NUM_BUTTONS]"),
            ("pulses", "u32[NUM_BUTTONS]"),
        ])
        self.compute_mapping()
        self.channels = self.expand_channels()
        self.sections, self.attrs = self.expand_config()

    # ------------------------------------------------------------- types
    def dim(self, token: str) -> int:
        return int(token) if token.isdigit() else self.consts[token]

    def parse_type(self, t: str) -> tuple[str, int]:
        m = TYPE_RE.match(t)
        if not m:
            raise ValueError(f"bad type {t!r}")
        base, n = m.group(1), m.group(2)
        return base, (self.dim(n) if n else 1)

    def ctype(self, t: str):
        base, n = self.parse_type(t)
        elem = PRIM[base][0] if base in PRIM else self.structs[base]
        return elem * n if (n > 1 or base == "char" or "[" in t) else elem

    def add_struct(self, name: str, fields: list[tuple[str, str]]) -> None:
        cls = type(name, (ctypes.Structure,), {"_fields_": [(f, self.ctype(t)) for f, t in fields]})
        self.structs[name] = cls
        self.struct_fields[name] = fields

    def resolve(self, struct: str, path: str) -> FieldInfo:
        """Offset/type of a dotted path like 'trailers[2].placement' inside `struct`."""
        offset = 0
        cur = struct
        parts = path.split(".")
        for i, part in enumerate(parts):
            m = re.match(r"^(\w+)(?:\[(\d+)\])?$", part)
            if not m:
                raise ValueError(f"bad path {path!r}")
            name, idx = m.group(1), m.group(2)
            fields = dict(self.struct_fields[cur])
            if name not in fields:
                raise KeyError(f"{cur} has no field {name!r} (path {path!r})")
            offset += getattr(self.structs[cur], name).offset
            base, n = self.parse_type(fields[name])
            elem_size = ctypes.sizeof(PRIM[base][0] if base in PRIM else self.structs[base])
            if idx is not None:
                if int(idx) >= n:
                    raise IndexError(f"{path!r}: index {idx} >= {n}")
                offset += int(idx) * elem_size
                n = 1
            if i < len(parts) - 1:
                if base in PRIM:
                    raise ValueError(f"{path!r}: {name} is not a struct")
                cur = base
            else:
                return FieldInfo(offset, elem_size * n, base, n)
        raise AssertionError

    # ------------------------------------------------------------- mapping
    @staticmethod
    def align(n: int, a: int = 64) -> int:
        return (n + a - 1) // a * a

    def compute_mapping(self) -> None:
        s = self.structs
        self.header_size = ctypes.sizeof(s["TelemetryHeader"])
        self.frame_size = ctypes.sizeof(s["TelemetryFrame"])
        self.config_size = ctypes.sizeof(s["ConfigBlock"])
        self.event_size = ctypes.sizeof(s["EventSlot"])
        self.frames_offset = self.align(self.header_size)
        self.config_offset = self.align(self.frames_offset + self.frame_size * self.consts["RING_SLOTS"])
        self.events_offset = self.align(self.config_offset + self.config_size)
        self.telemetry_size = self.align(self.events_offset + self.event_size * self.consts["EVENT_SLOTS"], 4096)
        self.controls_size = self.align(ctypes.sizeof(s["ControlsBlock"]), 4096)

    # ------------------------------------------------------------- channels / config
    def expand_channels(self) -> list[dict]:
        out = []
        for ch in self.spec["channel"]:
            trailers = range(self.consts["MAX_TRAILERS"]) if "{t}" in ch["sdk"] else [None]
            indices = range(self.consts[ch["indexed"]]) if "indexed" in ch else [None]
            scs_const, want = SDK_TYPES[ch["type"]]
            for t in trailers:
                for i in indices:
                    sdk = ch["sdk"].replace("{t}", str(t)) if t is not None else ch["sdk"]
                    path = ch["field"].replace("{t}", str(t) if t is not None else "").replace("{i}", str(i) if i is not None else "")
                    info = self.resolve("TelemetryFrame", path)
                    if info.type_name != want:
                        raise TypeError(f"channel {sdk}: field {path} is {info.type_name}, expected {want}")
                    out.append({"id": len(out), "sdk": sdk, "index": i, "type": ch["type"], "scs": scs_const,
                                "offset": info.offset, "size": info.size, "field": path})
        if len(out) > 64 * self.consts["CHANNEL_SEEN_WORDS"]:
            raise ValueError(f"{len(out)} channels exceed CHANNEL_SEEN_WORDS")
        return out

    def expand_config(self) -> tuple[list[dict], list[dict]]:
        sections = []
        for sec in self.spec["config_section"]:
            trailers = range(self.consts["MAX_TRAILERS"]) if "{t}" in sec["id"] else [None]
            for t in trailers:
                sid = sec["id"].replace("{t}", str(t)) if t is not None else sec["id"]
                path = sec["field"].replace("{t}", str(t)) if t is not None else sec["field"]
                info = self.resolve("ConfigBlock", path)
                sections.append({"id": sid, "template": sec["id"], "trailer": t, "offset": info.offset,
                                 "size": info.size, "field": path})
        attrs = []
        for cfg in self.spec["config"]:
            for si, sec in enumerate(sections):
                if sec["template"] != cfg["section"]:
                    continue
                for a in cfg["attrs"]:
                    name, typ, field = a[0], a[1], a[2]
                    indices = range(self.consts[a[3]]) if len(a) > 3 else [None]
                    scs_const, want = SDK_TYPES[typ]
                    for i in indices:
                        path = f"{sec['field']}.{field.replace('{i}', str(i) if i is not None else '')}"
                        info = self.resolve("ConfigBlock", path)
                        if info.type_name != want:
                            raise TypeError(f"config {sec['id']}/{name}: {path} is {info.type_name}, expected {want}")
                        attrs.append({"section": si, "name": name, "index": i, "type": typ, "scs": scs_const,
                                      "offset": info.offset, "size": info.size})
        return sections, attrs


# ----------------------------------------------------------------- emitters

def c_field_decl(layout: Layout, name: str, t: str) -> str:
    base, n = layout.parse_type(t)
    ctype = PRIM[base][1] if base in PRIM else base
    return f"    {ctype} {name}[{n}];" if "[" in t else f"    {ctype} {name};"


def c_str(s: str) -> str:
    return '"' + s.replace("\\", "\\\\").replace('"', '\\"') + '"'


def emit_header(layout: Layout) -> str:
    L: list[str] = []
    w = L.append
    w("// GENERATED by tools/gen_layout.py from native/spec/layout.toml + inputs.toml. Do not edit.")
    w("#pragma once")
    w("#include <cstddef>")
    w("#include <cstdint>")
    w("")
    w("namespace autohaul {")
    w("")
    w(f"constexpr uint32_t kLayoutVersion = {layout.version};")
    w(f"constexpr uint64_t kLayoutHash = 0x{layout.layout_hash:016x}ull;")
    for k, v in layout.consts.items():
        w(f"constexpr uint32_t {k} = {v};")
    names = layout.spec["names"]
    w(f"constexpr uint32_t kTelemetryMagic = 0x{names['telemetry_magic']:08x}u;")
    w(f"constexpr uint32_t kControlsMagic = 0x{names['controls_magic']:08x}u;")
    w(f"constexpr const wchar_t* kTelemetryMapping = L{c_str(names['telemetry_mapping'])};")
    w(f"constexpr const wchar_t* kControlsMapping = L{c_str(names['controls_mapping'])};")
    w(f"constexpr const wchar_t* kFrameEvent = L{c_str(names['frame_event'])};")
    for k, v in layout.spec["bridge_flags"].items():
        w(f"constexpr uint32_t BRIDGE_FLAG_{k} = {v}u;")
    w("")
    w(f"constexpr uint32_t kFramesOffset = {layout.frames_offset};")
    w(f"constexpr uint32_t kConfigOffset = {layout.config_offset};")
    w(f"constexpr uint32_t kEventsOffset = {layout.events_offset};")
    w(f"constexpr uint32_t kTelemetrySize = {layout.telemetry_size};")
    w(f"constexpr uint32_t kControlsSize = {layout.controls_size};")
    w("")
    for name, fields in layout.struct_fields.items():
        w(f"struct {name} {{")
        for f, t in fields:
            w(c_field_decl(layout, f, t))
        w("};")
        cls = layout.structs[name]
        w(f"static_assert(sizeof({name}) == {ctypes.sizeof(cls)}, \"{name} size\");")
        for f, _ in fields:
            w(f"static_assert(offsetof({name}, {f}) == {getattr(cls, f).offset}, \"{name}.{f}\");")
        w("")
    w("enum class ValueType : uint32_t { Bool, S32, U32, U64, Float, Double, FVector, FPlacement, DPlacement, String };")
    w("")
    w("struct ChannelDef { const char* name; uint32_t index; uint32_t scs_type; ValueType type; uint32_t offset; uint32_t size; };")
    w("struct ConfigSectionDef { const char* id; uint32_t offset; uint32_t size; };")
    w("struct ConfigAttrDef { uint32_t section; const char* name; uint32_t index; uint32_t scs_type; ValueType type; uint32_t offset; uint32_t size; };")
    w("struct InputDef { const char* name; const char* display; bool is_axis; };")
    w("")
    vt = {"bool": "Bool", "s32": "S32", "u32": "U32", "u64": "U64", "float": "Float", "double": "Double",
          "fvector": "FVector", "fplacement": "FPlacement", "dplacement": "DPlacement", "string": "String"}
    NIL = "0xFFFFFFFFu"
    w(f"constexpr uint32_t kChannelCount = {len(layout.channels)};")
    w("// scs_type values match SCS_VALUE_TYPE_* in scssdk_value.h")
    scs_num = {"bool": 1, "s32": 2, "u32": 3, "u64": 4, "float": 5, "double": 6, "fvector": 7, "fplacement": 10,
               "dplacement": 11, "string": 12}
    w("inline const ChannelDef kChannels[kChannelCount] = {")
    for c in layout.channels:
        idx = NIL if c["index"] is None else str(c["index"])
        w(f"    {{{c_str(c['sdk'])}, {idx}, {scs_num[c['type']]}, ValueType::{vt[c['type']]}, {c['offset']}, {c['size']}}},  // {c['id']}: {c['field']}")
    w("};")
    w("")
    w(f"constexpr uint32_t kConfigSectionCount = {len(layout.sections)};")
    w("inline const ConfigSectionDef kConfigSections[kConfigSectionCount] = {")
    for s in layout.sections:
        w(f"    {{{c_str(s['id'])}, {s['offset']}, {s['size']}}},  // {s['field']}")
    w("};")
    w(f"constexpr uint32_t kConfigAttrCount = {len(layout.attrs)};")
    w("inline const ConfigAttrDef kConfigAttrs[kConfigAttrCount] = {")
    for a in layout.attrs:
        idx = NIL if a["index"] is None else str(a["index"])
        w(f"    {{{a['section']}, {c_str(a['name'])}, {idx}, {scs_num[a['type']]}, ValueType::{vt[a['type']]}, {a['offset']}, {a['size']}}},")
    w("};")
    w("")
    w("inline const InputDef kInputs[NUM_INPUTS] = {")
    for mix, display, _desc in layout.axes:
        w(f"    {{{c_str(mix)}, {c_str(display)}, true}},")
    for mix, _kind, _group, desc in layout.buttons:
        display = re.sub(r"[^A-Za-z0-9 _.]", " ", desc).strip()[:40] or mix
        w(f"    {{{c_str(mix)}, {c_str(display)}, false}},")
    w("};")
    w("")
    w("}  // namespace autohaul")
    w("")
    return "\n".join(L)


def emit_python(layout: Layout) -> str:
    L: list[str] = []
    w = L.append
    w('"""GENERATED by tools/gen_layout.py from native/spec/layout.toml + inputs.toml. Do not edit."""')
    w("")
    w("# ruff: noqa")
    w("import ctypes")
    w("")
    w(f"LAYOUT_VERSION = {layout.version}")
    w(f"LAYOUT_HASH = 0x{layout.layout_hash:016x}")
    for k, v in layout.consts.items():
        w(f"{k} = {v}")
    names = layout.spec["names"]
    w(f"TELEMETRY_MAGIC = 0x{names['telemetry_magic']:08x}")
    w(f"CONTROLS_MAGIC = 0x{names['controls_magic']:08x}")
    w(f"TELEMETRY_MAPPING = {names['telemetry_mapping']!r}")
    w(f"CONTROLS_MAPPING = {names['controls_mapping']!r}")
    w(f"FRAME_EVENT = {names['frame_event']!r}")
    for k, v in layout.spec["bridge_flags"].items():
        w(f"BRIDGE_FLAG_{k} = {v}")
    w("")
    w(f"FRAMES_OFFSET = {layout.frames_offset}")
    w(f"CONFIG_OFFSET = {layout.config_offset}")
    w(f"EVENTS_OFFSET = {layout.events_offset}")
    w(f"TELEMETRY_SIZE = {layout.telemetry_size}")
    w(f"CONTROLS_SIZE = {layout.controls_size}")
    w("")
    for name, fields in layout.struct_fields.items():
        w("")
        w(f"class {name}(ctypes.Structure):")
        w("    _fields_ = [")
        for f, t in fields:
            base, n = layout.parse_type(t)
            elem = PRIM[base][2] if base in PRIM else base
            w(f"        ({f!r}, {elem} * {n})," if "[" in t else f"        ({f!r}, {elem}),")
        w("    ]")
        w("")
        w(f"assert ctypes.sizeof({name}) == {ctypes.sizeof(layout.structs[name])}")
    w("")
    w("")
    w("# (channel id, sdk name, index or None, sdk type, frame field path)")
    w("CHANNELS = [")
    for c in layout.channels:
        w(f"    ({c['id']}, {c['sdk']!r}, {c['index']!r}, {c['type']!r}, {c['field']!r}),")
    w("]")
    w("CHANNEL_IDS = {(c[1], c[2]): c[0] for c in CHANNELS}")
    w("")
    w("# (mix, display, description)")
    w(f"AXES = {[tuple(a) for a in layout.axes]!r}")
    w("# (mix, kind, group, description)")
    w(f"BUTTONS = {[tuple(b) for b in layout.buttons]!r}")
    w("AXIS_INDEX = {a[0]: i for i, a in enumerate(AXES)}")
    w("BUTTON_INDEX = {b[0]: i for i, b in enumerate(BUTTONS)}")
    w("")
    return "\n".join(L)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--check", action="store_true", help="fail if generated files are out of date")
    args = parser.parse_args()

    layout = Layout(*load())
    outputs = {OUT_H: emit_header(layout), OUT_PY: emit_python(layout)}
    stale = [p for p, text in outputs.items() if not p.exists() or p.read_text(encoding="utf-8") != text]
    if args.check:
        for p in stale:
            print(f"stale: {p.relative_to(ROOT)}")
        return 1 if stale else 0
    for p, text in outputs.items():
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(text, encoding="utf-8", newline="\n")
        print(f"wrote {p.relative_to(ROOT)}")
    print(f"layout v{layout.version} hash=0x{layout.layout_hash:016x}: {len(layout.channels)} channels, "
          f"{len(layout.attrs)} config attrs, {layout.consts['NUM_AXES']} axes, {layout.consts['NUM_BUTTONS']} buttons; "
          f"telemetry mapping {layout.telemetry_size} B, controls {layout.controls_size} B")
    return 0


if __name__ == "__main__":
    sys.exit(main())
