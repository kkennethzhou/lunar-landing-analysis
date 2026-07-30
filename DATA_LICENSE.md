# Data Licensing & Provenance

## Code

MIT — see [LICENSE](LICENSE).

## NASA LROC imagery

The imagery this project references comes from the **Lunar Reconnaissance Orbiter Camera (LROC) Narrow Angle Camera**, distributed through the NASA Planetary Data System.

**These files are not redistributed in this repository.** They total ~1.9 GB and are excluded via `.gitignore`. Regenerate them:

```bash
python scripts/download_lunar.py
python scripts/convert_nasa_images.py
```

**Status.** NASA data is generally not subject to copyright in the United States and may be used without restriction. NASA nonetheless requests acknowledgement, and the LROC team asks that publications cite the instrument paper.

**Attribution:**

> NASA/GSFC/Arizona State University

**Instrument citation:**

> Robinson, M. S., et al. (2010). Lunar Reconnaissance Orbiter Camera (LROC) Instrument Overview. *Space Science Reviews*, 150(1–4), 81–124.

**Archive:** [PDS Imaging Node](https://pds-imaging.jpl.nasa.gov/) · dataset `LRO-L-LROC-2-EDR-V1.0`

Products used are Left NAC (`LE`) EDR strips spanning 2011–2020; the manifest is in `scripts/download_lunar.py`.

## Generated artifacts

Procedural terrain, simulation results, and figures are produced by this repository's code and fall under the MIT license. All are regenerable — `make all` reproduces every figure deterministically from `seed = 42`.

## Third-party software

- **MuJoCo** — Apache 2.0, DeepMind
- **COLMAP** — BSD, Johannes Schönberger et al.
- **NumPy / SciPy / Matplotlib** — BSD-style
