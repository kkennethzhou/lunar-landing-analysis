# COLMAP Post-Mortem

Structure-from-Motion on LROC orbital imagery failed completely. This documents why, because the failure is more instructive than a success would have been.

## What was run

```bash
colmap automatic_reconstructor \
  --image_path ./lunar_images \
  --workspace_path ./colmap_workspace \
  --data_type video --quality low
```

## What happened

- 17+ min stalled downloading a 141 MB vocabulary-tree matching file
- ~33 min total for feature extraction and matching
- **3 of 38 images registered. 1 point in the resulting 3D model.**

Verified by parsing `colmap_workspace/sparse/0/`: `points3D.bin` contains exactly one entry.

A usable terrain model needs millions of points. This failed fundamentally, not marginally — no amount of parameter tuning was going to help.

## Why

### 1. Pushbroom geometry breaks the camera model

LROC NAC is a **line scanner**. It captures one 2,532-pixel line at a time while the orbiter moves, assembling up to 52,224 lines into a strip. Every scanline has a **different camera centre**.

COLMAP assumes a pinhole camera: one centre of projection per image. That assumption is not approximately violated here, it's structurally wrong. A 52,224-line strip is effectively 52,224 cameras, and no perspective camera model can describe it.

Correct tools model the sensor explicitly — USGS ISIS3 (`jigsaw`), or the Ames Stereo Pipeline, both of which support pushbroom sensors.

### 2. The images don't overlap

The 38 strips span **2011, 2012, 2015, 2019, and 2020** across different orbits. Different ground footprints, different sun angles, different shadow directions.

SfM recovers geometry by triangulating the *same* features from *different* viewpoints. These images largely don't show the same ground, and where they do, a 2011 image at one sun angle and a 2019 image at another produce shadows that feature detectors treat as different terrain.

### 3. `--data_type video` was the wrong flag

It assumes sequential frames with short baselines and high overlap, restricting matching to a temporal window. Orbital strips from different years have no such structure. `--data_type individual` with exhaustive matching would have been the right call — though it wouldn't have saved the run given causes 1 and 2.

### 4. Only left-camera products were downloaded

Every product in `download_lunar.py` is an `LE` file. LROC carries **two** NACs, left and right, with overlapping fields of view — the natural stereo pair, and the geometry the instrument was designed to exploit for topography.

Downloading only `LE` removed the one configuration with a realistic chance of working.

## What was done instead

Pivoted to procedural terrain. The physics work doesn't require real terrain — it requires *plausible* terrain with known ground truth, which a generator provides and a photogrammetric reconstruction wouldn't have.

## What should be done for real terrain

**Don't reconstruct it — download it.** NASA publishes ready-made lunar DEMs:

- **LOLA** gridded topography (laser altimetry, global)
- **SLDEM2015** — LOLA merged with SELENE Terrain Camera stereo, 60 m/px
- **NAC DTMs** — ~2 m/px stereo-derived DEMs for selected sites

Any of these drops straight into `generate_world.py` in place of the procedural heightfield, and would turn a methodology demonstration into an actual site study.

## Takeaway

The instructive part isn't that SfM failed — it's that it failed *quietly*. COLMAP ran for 33 minutes, exited without error, and produced a valid-looking sparse model directory. Only counting the points revealed that the output was empty of content.
