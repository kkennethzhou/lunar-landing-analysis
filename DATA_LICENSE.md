# Data licensing and provenance

## Code

MIT. See [LICENSE](LICENSE).

## NASA LROC imagery

The images referenced here are from the Lunar Reconnaissance Orbiter Camera (LROC) Narrow Angle Camera, distributed by the NASA Planetary Data System.

They are not in this repository. The set is about 1.9 GB and is listed in `.gitignore`. To fetch them:

```bash
python scripts/download_lunar.py
python scripts/convert_nasa_images.py
```

NASA data is generally not copyrighted in the United States and can be used freely. NASA asks for an acknowledgement, and the LROC team asks publications to cite the instrument paper.

Attribution: NASA/GSFC/Arizona State University.

Instrument paper: Robinson, M. S., et al. (2010). Lunar Reconnaissance Orbiter Camera (LROC) Instrument Overview. Space Science Reviews, 150(1-4), 81-124.

Archive: [PDS Imaging Node](https://pds-imaging.jpl.nasa.gov/), dataset `LRO-L-LROC-2-EDR-V1.0`.

Products used are Left NAC (`LE`) EDR strips from 2011-2020. The list is in `scripts/download_lunar.py`.

## Generated files

Procedural terrain, simulation results, and figures are produced by this code and are under the MIT license. `make all` regenerates the figures from `seed = 42`.

## Other software

- MuJoCo, Apache 2.0, DeepMind
- COLMAP, BSD, Johannes Schönberger et al.
- NumPy, SciPy, and Matplotlib, BSD-style licenses
