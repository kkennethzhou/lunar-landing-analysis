# COLMAP on the LROC strips

Structure-from-motion on these images did not produce a terrain model. This file is what was run and why it came out empty.

## Command

```bash
colmap automatic_reconstructor \
  --image_path ./lunar_images \
  --workspace_path ./colmap_workspace \
  --data_type video --quality low
```

## Output

- More than 17 minutes stalled on a 141 MB vocabulary-tree file.
- About 33 minutes for feature extraction and matching.
- 3 of 38 images registered. 1 point in the sparse model.

`colmap_workspace/sparse/0/points3D.bin` has one entry. A terrain mesh needs a lot more than that. Tuning the matcher was not going to fix it.

## Causes

### 1. The camera is a line scanner

LROC NAC takes one 2,532-pixel line at a time while the orbiter moves, up to 52,224 lines. Each line has a different camera centre.

COLMAP wants a pinhole camera: one centre per image. A 52,224-line strip is not one camera. ISIS3 (`jigsaw`) and the Ames Stereo Pipeline both have pushbroom models. COLMAP does not.

### 2. The strips do not show the same ground

The 38 files are from 2011, 2012, 2015, 2019, and 2020, different orbits. Different footprints, different sun angles.

SfM needs the same features from different viewpoints. These images mostly do not overlap. Where they do, shadows from different years get detected as different terrain.

### 3. `--data_type video` was the wrong mode

That mode expects sequential frames with short baselines and only matches inside a time window. Strips from different years do not have that structure. `--data_type individual` with exhaustive matching would have been the right flag. It would not have fixed causes 1 and 2.

### 4. Only the left camera was downloaded

Every product in `download_lunar.py` is an `LE` file. LROC has two NACs, left and right, with overlapping fields of view. That pair is what the instrument uses for topography. Downloading only `LE` skipped it.

## What the simulation uses instead

Procedural terrain. The landing trials need a surface with a known height, which the generator has. A photogrammetric mesh from this image set would not have.

## If you want real topography

NASA already publishes DEMs. Reconstructing them again from raw EDRs is the long way around.

- LOLA gridded topography (laser altimetry, global)
- SLDEM2015, LOLA merged with SELENE Terrain Camera stereo, 60 m/px
- NAC DTMs, about 2 m/px, for selected sites

Any of those can replace the procedural grid in `generate_world.py`.

## What the log looked like

COLMAP ran for 33 minutes, exited without an error, and wrote a normal-looking sparse directory. The model was empty. The only check that showed it was counting the points.
