"""
NASA PDS .IMG to .PNG converter for LROC NAC images.

LROC NAC EDR files use PDS3 format: an ASCII label block padded to a
fixed record length, followed by raw 8-bit or 16-bit DN (digital number)
image data. Standard tools reject the label; this script strips it,
reads the raw pixels, normalises contrast, and saves PNG files.
"""

import os
import re
import struct
import numpy as np
from PIL import Image

IMG_DIR = "./lunar_images"
PNG_DIR = "./lunar_images"   # PNGs land alongside the originals
DELETE_RAW = True            # set False to keep the .IMG files


def parse_pds3_label(data: bytes) -> dict:
    """
    Extract key=value pairs from the ASCII PDS3 label at the start of
    the file. Returns a dict of uppercase keys -> stripped string values.
    """
    params = {}
    # Labels end with 'END\r\n' or 'END\n'
    end_match = re.search(rb'\bEND\b', data)
    label_bytes = data[:end_match.start()] if end_match else data[:4096]
    text = label_bytes.decode('ascii', errors='replace')
    for line in text.splitlines():
        if '=' in line:
            k, _, v = line.partition('=')
            params[k.strip().upper()] = v.strip().strip('"').strip()
    return params


def get_label_size(data: bytes, record_bytes: int) -> int:
    """
    PDS3 labels are padded to a multiple of RECORD_BYTES.
    ^IMAGE is the keyword that tells us at which record the image starts.
    """
    params = parse_pds3_label(data)
    # ^IMAGE gives the record number (1-based) where image data starts
    image_ptr = params.get('^IMAGE', '2')
    # May contain units like '2 <BYTES>' – extract the leading integer
    image_record = int(re.match(r'\d+', image_ptr).group())
    return (image_record - 1) * record_bytes


def convert(img_path: str) -> str | None:
    """Convert a single .IMG file to .PNG. Returns output path or None on error."""
    with open(img_path, 'rb') as f:
        raw = f.read()

    params = parse_pds3_label(raw)

    # ---- geometry -------------------------------------------------------
    try:
        lines      = int(params.get('LINES',      params.get('IMAGE_LINES', '0')))
        line_samps = int(params.get('LINE_SAMPLES', params.get('SAMPLES',  '0')))
        record_bytes = int(params.get('RECORD_BYTES', str(line_samps)))
        sample_bits  = int(params.get('SAMPLE_BITS', '8'))
    except (ValueError, KeyError) as e:
        print(f"  [WARN] Could not parse geometry from {img_path}: {e}")
        return None

    if lines == 0 or line_samps == 0:
        print(f"  [WARN] Zero dimensions in {img_path}, skipping")
        return None

    # ---- locate image data ----------------------------------------------
    offset = get_label_size(raw, record_bytes)
    dtype  = np.uint16 if sample_bits == 16 else np.uint8

    # Byte order: LROC NAC EDRs are LSB_INTEGER for 16-bit, unsigned for 8-bit
    byte_order = params.get('SAMPLE_TYPE', 'LSB_UNSIGNED_INTEGER')
    np_dtype   = np.dtype(dtype)
    if '16' in str(sample_bits):
        np_dtype = np.dtype('<u2') if 'LSB' in byte_order else np.dtype('>u2')

    expected_bytes = lines * line_samps * np_dtype.itemsize
    image_bytes    = raw[offset: offset + expected_bytes]

    if len(image_bytes) < expected_bytes:
        # Tolerate truncated files – pad with zeros
        image_bytes = image_bytes.ljust(expected_bytes, b'\x00')

    arr = np.frombuffer(image_bytes, dtype=np_dtype).reshape((lines, line_samps))

    # ---- contrast normalisation -----------------------------------------
    # Use 2nd/98th percentile to clip cosmic rays and saturated pixels
    p2,  p98 = np.percentile(arr, (2, 98))
    if p98 <= p2:
        p2, p98 = float(arr.min()), float(arr.max())
    if p98 == p2:
        p98 = p2 + 1  # degenerate flat image

    arr_f   = arr.astype(np.float32)
    arr_f   = np.clip(arr_f, p2, p98)
    arr_u8  = ((arr_f - p2) / (p98 - p2) * 255.0).astype(np.uint8)

    # ---- save PNG -------------------------------------------------------
    stem     = os.path.splitext(os.path.basename(img_path))[0]
    out_path = os.path.join(PNG_DIR, stem + '.png')
    Image.fromarray(arr_u8, mode='L').save(out_path, optimize=True)
    return out_path


def main():
    img_files = sorted(
        f for f in os.listdir(IMG_DIR) if f.upper().endswith('.IMG')
    )

    if not img_files:
        print(f"No .IMG files found in {IMG_DIR}")
        return

    print(f"Found {len(img_files)} .IMG file(s) to convert.\n")
    converted, errors = [], []

    for fname in img_files:
        img_path = os.path.join(IMG_DIR, fname)
        print(f"Converting: {fname}")
        try:
            out = convert(img_path)
            if out:
                size_kb = os.path.getsize(out) // 1024
                print(f"  -> {os.path.basename(out)} ({size_kb} KB)")
                converted.append(img_path)
            else:
                errors.append(fname)
        except Exception as e:
            print(f"  [ERROR] {e}")
            errors.append(fname)

    print(f"\n{len(converted)} converted, {len(errors)} failed.")

    if DELETE_RAW and converted:
        print("\nDeleting source .IMG files...")
        for p in converted:
            os.remove(p)
            print(f"  deleted {os.path.basename(p)}")

    if errors:
        print("\nFailed files:")
        for e in errors:
            print(f"  {e}")


if __name__ == '__main__':
    main()
