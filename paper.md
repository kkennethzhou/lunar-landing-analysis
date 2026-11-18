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

Lunar landing analysis drops a 1,000 kg rigid-body lander on a 127 m patch of cratered terrain in MuJoCo, picks a touchdown coordinate from 500 randomized descents, and checks whether that coordinate is a property of the terrain. On the campaign shipped with the code, it is not. Peak landing-gear load does not correlate with local slope, including after controlling for horizontal drift, and bootstrap resampling moves the recommended point by tens of metres. The package also records a collision error that would have made that null result hard to read: MuJoCo mesh geoms collide on the convex hull, which here sits up to 4.10 m above a crater floor. The campaign uses a heightfield. The software does not build a lunar digital elevation model from images.

# Statement of need

A smoothed map of simulated landings always has a lowest cell. That cell is an engineering answer only if the loads depend on where the vehicle touched down, and if another draw of the same experiment would pick a nearby point. MuJoCo will step whatever model it is given and will not apply those checks [@todorov2012].

The package is built around that check. The terrain, a powered-descent controller, the trial loop, a kernel-regression site estimate, and a validator that is allowed to fail are in one installable module. Campaign numbers quoted in the docs are written to JSON by the script that computed them.

A photogrammetry attempt on Lunar Reconnaissance Orbiter Camera Narrow Angle Camera strips did not produce a terrain model. The camera is a pushbroom line scanner [@robinson2010]. The 38 downloaded strips are not a stereo pair of one site. COLMAP assumes one pinhole centre per image [@schoenberger2016], registered 3 of 38 images, and kept a single 3D point. Real topography for this kind of work already exists, including Lunar Orbiter Laser Altimeter products [@smith2010] and stereo models from the Ames Stereo Pipeline [@beyer2018]. This package uses a procedural heightfield so the same statistical test can be rerun from a seed, without a multi-gigabyte archive.

# State of the field

MuJoCo and PyBullet both run rigid-body contact from Python [@todorov2012; @coumans2016]. Neither ships a landing-site estimator, and neither reports when a preferred coordinate is a draw of the noise. Mission lunar-descent tools usually start from a real elevation grid and a guidance stack, and they answer a question about a named site. This software asks an earlier question: given a terrain model and a contact simulator, does a Monte Carlo safety map contain a terrain signal? The procedural surface is a limitation next to those elevation products. It is also why the experiment can be regenerated from a seed.

Mesh collision in MuJoCo uses the convex hull. On cratered terrain the hull is a lid over the bowl, and the render still shows the bowl. The package measures that lid and ships the heightfield used for the campaign.

# Software design

The generator writes a 128 by 128 grid at 1 m spacing from summed sinusoids, seven Gaussian craters with raised rims, and four boulder bumps. Importing the module does not write files. Nominal relief is 4.44 m. Per-cell slopes come from the gradient, not from relief divided by width: mean 6.46°, 95th percentile 12.73°, maximum 27.77°, and 63% of cells steeper than 5°. The validator calls the same function, so the slope at a landing coordinate is the slope the generator defined.

Collision uses the heightfield. An OBJ is still written and is not the colliding geom. `lunar_lander.convex_gap` compares the two. Across the seven crater samples the convex lid averages 3.08 m above the floor. At the worst sample the geometric gap is 4.10 m, and a sphere dropped onto the mesh geom stops 4.10 m above that floor. The same sphere on the heightfield stops 2.6 cm below the floor, within the 8-bit PNG step (relief/255, 1.7 cm on this grid) plus contact compliance. The heightfield is the collision surface for the campaign. It is not a centimetre-accurate elevation model.

Lander geom masses sum to 1,000 kg, and MuJoCo derives inertia from the geoms. An earlier explicit inertial element had produced 960 kg without a warning. Gravity is 1.62 m/s² and the timestep is 2 ms. Each leg is a vertical slide with 0.8 m of stroke, a 3,000 N/m spring, and 1,200 N·s/m of damping. Static lunar weight is about 405 N per leg, so at 3,000 N/m the spring compresses 0.135 m. A previous 320 N/m spring needed 1.27 m and could not hold the vehicle inside the stroke. One actuator applies up to 5,000 N. The descent law tracks a profile from 10 m/s down to a 1.5 m/s touchdown. Once three feet are in contact, thrust ramps to zero over 0.2 s. Leaving the hover feed-forward on had let the settle check fire while the engine still carried the weight.

Each of 500 trials, drawn with seed 42, randomizes position inside a 15 m margin (a 97 m window), horizontal velocity, a small initial tilt and spin, foot friction, and the touchdown-speed target. A trial ends as landed, tipped past 15°, failed on load, stroke, or a 10 g windowed acceleration, crashed, or timed out at 24 s. Strut load is read from the joint. The site estimate is Nadaraya-Watson regression of peak strut force on successful landings [@nadaraya1964; @watson1964], with cells below five effective samples masked out. The recommended site is the minimum of that field.

The validator then writes a verdict. It checks bottom-out, the correlation of force with slope, the partial correlation controlling for horizontal speed, whether failures track slope, whether replacing forces by a constant moves the site, and the 95% radius of 1,000 bootstrap sites [@efron1979]. The process exits non-zero when the overall verdict fails.

A second preset deepens the craters by three and the medium ridges by two. `compare_roughness` reruns a shorter seeded campaign on both presets and does not overwrite the nominal world. On 40 trials the rougher grid has 8.77 m of relief and a 37.5% failure rate, against 4.44 m and 12.5% on the nominal preset. The force-slope correlation on the rougher grid is r = 0.039 (p = 0.85). Failures went up. A slope signal in the loads did not, at this sample size.

# Research impact statement

Nothing outside this repository depends on the package, and it has not been used on a flight programme. The numbers below are for this terrain and this controller.

On the 500-trial run, 422 trials landed, 43 tipped, 16 exceeded the acceleration limit, and 19 timed out (15.6% not landed). No leg bottomed out (maximum stroke 0.144 m). Correlation of peak strut force with local slope is r = 0.025 (p = 0.61). Controlling for horizontal drift leaves r = 0.005 (p = 0.92). Force against drift is r = 0.525. Bootstrap resampling spreads the recommended point over a 58.4 m radius inside the 97 m window. The point is reported with a failed verdict. The 40-trial roughness probe is the check on whether a steeper map would have created the missing correlation. It raised the failure rate and did not. Both JSON files are in the repository.

# AI usage disclosure

Generative AI assisted this project. During development, Anthropic Claude (model version not recorded in the repository) compared simulation outputs with hand calculations, including static leg load against stroke and contact-solver force against strut force, and helped revise the validation logic and the docs. The author chose the terrain, the controller, the failure criteria, and the decision to keep a failing validation result. In November 2026, Grok, through the Cursor editor, drafted this manuscript and `paper.bib` from the source tree and the committed statistics, and made the packaging and test changes described above. The quoted figures were taken from `output/summary_stats.json`, `docs/convex_gap.json`, and `docs/roughness_comparison.json`. AI tools were not used to turn a failing check into a passing one.

# Acknowledgements

NASA Goddard Space Flight Center, Arizona State University, and the PDS Imaging Node provide the LROC Narrow Angle Camera strips used in the failed reconstruction. The physics engine is MuJoCo. Development started during the Tsinghua University AI and Robotic Design Programme.

# References
