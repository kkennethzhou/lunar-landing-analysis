---
title: 'Lunar landing analysis: a MuJoCo Monte Carlo pipeline that tests its own site recommendation'
tags:
  - Python
  - MuJoCo
  - lunar landing
  - Monte Carlo
  - planetary science
  - simulation
authors:
  - name: Kenneth Zhou
    corresponding: true
    affiliation: 1
affiliations:
  - name: Independent researcher, Hong Kong
    index: 1
date: 18 November 2026
bibliography: paper.bib
---

# Summary

Lunar landing analysis drops a 1,000 kg rigid-body lander onto a 127 m patch of cratered terrain in MuJoCo, estimates a safest touchdown coordinate from 500 randomized descents, and then tests whether that coordinate is a property of the terrain. On the campaign in the repository, it is not. Peak landing-gear load is uncorrelated with local slope, including after controlling for horizontal drift, and bootstrap resampling moves the recommended point by tens of metres. The package also measures a collision failure that would have made the null result uninterpretable: MuJoCo mesh geoms collide against the convex hull, which on this terrain sits up to 4.10 m above a crater floor. The corrected model uses a heightfield. The software is a methods tool for Monte Carlo contact studies. It does not ingest a lunar digital elevation model.

# Statement of need

A smoothed map of simulated landings produces a single safest coordinate. That coordinate is an engineering answer only if the measured loads depend on where the vehicle touched down, and only if another draw of the same experiment would pick a nearby point. Simulators used for this kind of study, including MuJoCo, will integrate whatever model they are given and will not apply those checks [@todorov2012].

The pipeline is built around that gap. Terrain, a powered-descent controller, the trial loop, a kernel-regression site estimate, and a validation harness that is allowed to fail all live in one installable package. Every quoted campaign number is written to JSON by the script that computed it. The target user is someone who needs to know whether a safety map survived its own trials.

Two limits are part of the design. A photogrammetry attempt on Lunar Reconnaissance Orbiter Camera Narrow Angle Camera strips did not produce a terrain model. The camera is a pushbroom line scanner [@robinson2010], the 38 downloaded strips are not a stereo pair of one site, and COLMAP, which assumes one pinhole centre per image [@schoenberger2016], registered 3 of 38 images and kept a single 3D point. Real topography for engineering use already exists, notably Lunar Orbiter Laser Altimeter products [@smith2010] and stereo terrain models from the Ames Stereo Pipeline [@beyer2018]. This package uses a procedural heightfield so the statistical test can be rerun without a multi-gigabyte archive.

# State of the field

MuJoCo and PyBullet both host rigid-body contact from Python [@todorov2012; @coumans2016]. Neither ships a landing-site estimator, and neither reports when a preferred coordinate is a draw of the noise. Mission-oriented lunar descent tools usually start from a real elevation grid and a guidance stack. They answer a question about a named site. This software answers a prior question: given a terrain model and a contact simulator, does a Monte Carlo safety map contain a terrain signal at all? The procedural surface is a limitation relative to those elevation products, and it is also why a reviewer can regenerate the experiment from a seed.

Mesh collision in MuJoCo is documented engine behaviour: contacts use the convex hull. On cratered terrain the hull is a lid. The render still shows the bowl. Publishing the size of that lid, and shipping the heightfield alternative, is the practical contribution for anyone importing non-convex terrain as a mesh.

# Software design

The generator writes a 128 by 128 grid at 1 m spacing from summed sinusoids, seven Gaussian craters with raised rims, and four boulder bumps. Importing the module does not write files. Nominal relief is 4.44 m. Per-cell slopes, from the gradient rather than from relief divided by width, have mean 6.46°, 95th percentile 12.73°, and maximum 27.77°, with 63% of cells steeper than 5°. The validator calls the same function, so the slope at a landing coordinate is the slope the generator defined.

Collision uses the heightfield. An OBJ is still written and is not the colliding geom. `lunar_lander.convex_gap` compares the two. Across the seven crater samples the convex lid averages 3.08 m above the floor. At the worst sample the geometric gap is 4.10 m, and a sphere dropped onto the mesh geom comes to rest 4.10 m above that floor. The same sphere on the heightfield stops 2.6 cm below the floor, inside the 8-bit PNG quantization (relief/255, 1.7 cm on this grid) plus contact compliance. The heightfield is the collision surface used for the campaign. It is not a centimetre-accurate elevation model.

The lander masses sum to 1,000 kg, and MuJoCo derives inertia from the geoms. An earlier explicit inertial element had silently produced 960 kg. Gravity is 1.62 m/s² and the timestep is 2 ms. Each leg is a vertical slide with 0.8 m of stroke, a 3,000 N/m spring, and 1,200 N·s/m of damping. Static lunar weight is about 405 N per leg, which at 3,000 N/m compresses the spring 0.135 m. A previous 320 N/m spring needed 1.27 m and could not hold the vehicle inside the stroke. One actuator applies up to 5,000 N. The descent law tracks a profile from 10 m/s down to a 1.5 m/s touchdown. Once three feet are in contact, thrust ramps to zero over 0.2 s. Leaving the hover feed-forward on had let the settle detector fire while the engine still carried the weight.

Each of 500 trials, drawn with seed 42, randomises position inside a 15 m margin (a 97 m window), horizontal velocity, a small initial tilt and spin, foot friction, and the touchdown-speed target. A trial ends as landed, tipped past 15°, failed on load, stroke, or a 10 g windowed acceleration, crashed, or timed out at 24 s. Strut load is read from the joint. The site estimate is Nadaraya–Watson regression of peak strut force on successful landings [@nadaraya1964; @watson1964], masked below five effective samples. The recommended site is the minimum of that field.

The validator then writes a verdict. It checks bottom-out, the correlation of force with slope and the partial correlation controlling for horizontal speed, whether failures track slope, whether replacing forces by a constant moves the site, and the 95% radius of 1,000 bootstrap sites [@efron1979]. The process exits non-zero when the overall verdict fails.

A second preset deepens the craters by three and the medium ridges by two. `compare_roughness` reruns a shorter seeded campaign on both presets without replacing the nominal world. On 40 trials the harsher grid has 8.77 m of relief and a 37.5% failure rate, against 4.44 m and 12.5% on the nominal preset. The force–slope correlation is r = 0.039 (p = 0.85) on the harsher grid. The extra failures are real. A terrain signal in the loads is not, at this sample size.

# Research impact statement

The repository has no external citations and no downstream packages, and this paper does not claim use on a flight programme. The result in the archive is specific.

On the 500-trial run, 422 trials landed, 43 tipped, 16 exceeded the acceleration limit, and 19 timed out (15.6% not landed). No leg bottomed out (maximum stroke 0.144 m). Correlation of peak strut force with local slope is r = 0.025 (p = 0.61). Controlling for horizontal drift leaves r = 0.005 (p = 0.92), while force against drift itself is r = 0.525. Bootstrap resampling spreads the recommended point over a 58.4 m radius inside the 97 m window. The point estimate is reported together with a failed verdict. The 40-trial roughness probe, above, is the check that a steeper map might have created the missing signal. It raised the failure rate and did not create the correlation. Both JSON files are in the repository.

# AI usage disclosure

Generative AI assisted this project. During development, Anthropic Claude (model version not recorded in the repository) reviewed simulation outputs against first-principles checks, including static leg load versus stroke and contact-solver force versus strut force, and helped revise validation logic and documentation. The author set the terrain, the controller, the failure criteria, and the decision to keep a failing validation result. In November 2026, Grok, through the Cursor editor, drafted this manuscript and `paper.bib` from the source tree and the committed statistics, and made the packaging and test changes described above. The author must check every number against `output/summary_stats.json`, `docs/convex_gap.json`, and `docs/roughness_comparison.json` before submission. AI tools were not used to turn a failing check into a passing one.

# Acknowledgements

NASA Goddard Space Flight Center, Arizona State University, and the PDS Imaging Node provide the LROC Narrow Angle Camera strips used in the unsuccessful reconstruction attempt. The physics engine is MuJoCo. Development began during the Tsinghua University AI and Robotic Design Programme.

# References
