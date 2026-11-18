"""PDS3 label parsing on a few dozen bytes, not a downloaded LROC strip."""

import importlib.util
from pathlib import Path

_SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "convert_nasa_images.py"
_spec = importlib.util.spec_from_file_location("convert_nasa_images", _SCRIPT)
_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_mod)


def _label_image(record_bytes, image_record, pixels: bytes, extra_lines: str) -> bytes:
    label = (
        "PDS_VERSION_ID = PDS3\r\n"
        f"RECORD_BYTES = {record_bytes}\r\n"
        f"^IMAGE = {image_record}\r\n"
        f"{extra_lines}"
        "END\r\n"
    ).encode("ascii")
    offset = (image_record - 1) * record_bytes
    if len(label) > offset:
        raise RuntimeError("label does not fit in the records before ^IMAGE")
    return label.ljust(offset, b" ") + pixels


def test_label_offset_and_8bit_geometry():
    pixels = bytes([0, 10, 20, 255, 1, 2, 3, 4])
    raw = _label_image(
        256, 2, pixels,
        "LINES = 2\r\nLINE_SAMPLES = 4\r\nSAMPLE_BITS = 8\r\n"
        "SAMPLE_TYPE = UNSIGNED_INTEGER\r\n",
    )
    params = _mod.parse_pds3_label(raw)
    assert params["LINES"] == "2"
    assert params["LINE_SAMPLES"] == "4"
    assert _mod.get_label_size(raw, 256) == 256
    assert raw[256:264] == pixels


def test_16bit_lsb_samples_are_little_endian():
    pixels = (1).to_bytes(2, "little") + (256).to_bytes(2, "little")
    raw = _label_image(
        256, 2, pixels,
        "LINES = 1\r\nLINE_SAMPLES = 2\r\nSAMPLE_BITS = 16\r\n"
        "SAMPLE_TYPE = LSB_INTEGER\r\n",
    )
    params = _mod.parse_pds3_label(raw)
    assert params["SAMPLE_BITS"] == "16"
    assert "LSB" in params["SAMPLE_TYPE"]
    assert _mod.get_label_size(raw, 256) == 256
    assert raw[256:260] == pixels
