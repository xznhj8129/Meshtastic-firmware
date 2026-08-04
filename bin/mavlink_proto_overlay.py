#!/usr/bin/env python3
"""Build-time overlay for the MAVLink SerialConfig additions.

The canonical schema lives in the pinned meshtastic-protobufs submodule. Until the
normal nanopb regeneration is run locally, this creates an equivalent generated
header in the PlatformIO build directory without modifying the source tree.
"""

from pathlib import Path


def _replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"MAVLink protobuf overlay expected one {label}, found {count}")
    return text.replace(old, new, 1)


def install_mavlink_proto_overlay(env) -> None:
    project_dir = Path(env["PROJECT_DIR"])
    source = project_dir / "src/mesh/generated/meshtastic/module_config.pb.h"
    text = source.read_text(encoding="utf-8")

    if "meshtastic_ModuleConfig_SerialConfig_Serial_Mode_MAVLINK" not in text:
        text = _replace_once(
            text,
            "    meshtastic_ModuleConfig_SerialConfig_Serial_Mode_LOGTEXT = 10 /* only text (channel & DM) */",
            "    meshtastic_ModuleConfig_SerialConfig_Serial_Mode_LOGTEXT = 10, /* only text (channel & DM) */\n"
            "    /* Transparent low-throughput MAVLink byte-stream bridge over SERIAL_APP. */\n"
            "    meshtastic_ModuleConfig_SerialConfig_Serial_Mode_MAVLINK = 11",
            "Serial_Mode enum insertion point",
        )
        text = _replace_once(
            text,
            "    bool override_console_serial_port;\n} meshtastic_ModuleConfig_SerialConfig;",
            "    bool override_console_serial_port;\n"
            "    /* Fixed Meshtastic peer for MAVLINK mode. Zero keeps first-sender discovery. */\n"
            "    uint32_t peer_node;\n"
            "} meshtastic_ModuleConfig_SerialConfig;",
            "SerialConfig struct insertion point",
        )
        text = _replace_once(
            text,
            "#define _meshtastic_ModuleConfig_SerialConfig_Serial_Mode_MAX meshtastic_ModuleConfig_SerialConfig_Serial_Mode_LOGTEXT\n"
            "#define _meshtastic_ModuleConfig_SerialConfig_Serial_Mode_ARRAYSIZE ((meshtastic_ModuleConfig_SerialConfig_Serial_Mode)(meshtastic_ModuleConfig_SerialConfig_Serial_Mode_LOGTEXT+1))",
            "#define _meshtastic_ModuleConfig_SerialConfig_Serial_Mode_MAX meshtastic_ModuleConfig_SerialConfig_Serial_Mode_MAVLINK\n"
            "#define _meshtastic_ModuleConfig_SerialConfig_Serial_Mode_ARRAYSIZE ((meshtastic_ModuleConfig_SerialConfig_Serial_Mode)(meshtastic_ModuleConfig_SerialConfig_Serial_Mode_MAVLINK+1))",
            "Serial_Mode bounds",
        )
        old_init = (
            "{0, 0, 0, 0, _meshtastic_ModuleConfig_SerialConfig_Serial_Baud_MIN, 0, "
            "_meshtastic_ModuleConfig_SerialConfig_Serial_Mode_MIN, 0}"
        )
        new_init = (
            "{0, 0, 0, 0, _meshtastic_ModuleConfig_SerialConfig_Serial_Baud_MIN, 0, "
            "_meshtastic_ModuleConfig_SerialConfig_Serial_Mode_MIN, 0, 0}"
        )
        if text.count(old_init) != 2:
            raise RuntimeError("MAVLink protobuf overlay expected default and zero SerialConfig initializers")
        text = text.replace(old_init, new_init)
        text = _replace_once(
            text,
            "#define meshtastic_ModuleConfig_SerialConfig_override_console_serial_port_tag 8",
            "#define meshtastic_ModuleConfig_SerialConfig_override_console_serial_port_tag 8\n"
            "#define meshtastic_ModuleConfig_SerialConfig_peer_node_tag 9",
            "peer_node tag",
        )
        text = _replace_once(
            text,
            "X(a, STATIC,   SINGULAR, BOOL,     override_console_serial_port,   8)",
            "X(a, STATIC,   SINGULAR, BOOL,     override_console_serial_port,   8) \\\n"
            "X(a, STATIC,   SINGULAR, UINT32,   peer_node,         9)",
            "peer_node field descriptor",
        )
        text = _replace_once(
            text,
            "#define meshtastic_ModuleConfig_SerialConfig_size 28",
            "#define meshtastic_ModuleConfig_SerialConfig_size 34",
            "SerialConfig encoded size",
        )

    required = (
        "meshtastic_ModuleConfig_SerialConfig_Serial_Mode_MAVLINK",
        "uint32_t peer_node;",
        "meshtastic_ModuleConfig_SerialConfig_peer_node_tag 9",
        "UINT32,   peer_node,         9",
    )
    missing = [needle for needle in required if needle not in text]
    if missing:
        raise RuntimeError(f"MAVLink protobuf overlay incomplete: {missing}")

    overlay_root = Path(env.subst("$BUILD_DIR")) / "mavlink-protobuf-overlay"
    header = overlay_root / "mesh/generated/meshtastic/module_config.pb.h"
    header.parent.mkdir(parents=True, exist_ok=True)
    header.write_text(text, encoding="utf-8")

    # Support both include forms used by the firmware:
    #   mesh/generated/meshtastic/module_config.pb.h
    #   meshtastic/module_config.pb.h
    env.Prepend(CPPPATH=[str(overlay_root), str(overlay_root / "mesh/generated")])
