"""
Cross-check the presentation deck against summary_stats.json.

The rest of this repo already applies one rule everywhere: every number
that gets reported is written to summary_stats.json and read back from
there (see lunar_lander/summary_stats.py). A slide deck is exactly the
kind of place that rule quietly stops applying -- numbers get typed in by
hand once and then never revisited when the pipeline changes underneath
them. This script extracts the numeric claims and terminology from the
deck's text frames and checks them against the current summary_stats.json,
so a stale deck fails loudly instead of just looking plausible forever.

Usage:
    python scripts/verify_deck.py path/to/deck.pptx
"""

import argparse
import json
import re
import sys

from pptx import Presentation

from lunar_lander.paths import SUMMARY_STATS

NUM = r"[-+]?\d+(?:\.\d+)?"


def load_stats(path=SUMMARY_STATS):
    with open(path) as f:
        return json.load(f)


def slide_texts(pptx_path):
    """Return {slide_number: full_text} for every slide with any text,
    shapes joined in document order so nearby claims stay nearby."""
    prs = Presentation(pptx_path)
    out = {}
    for i, slide in enumerate(prs.slides, 1):
        parts = [
            shape.text_frame.text.strip()
            for shape in slide.shapes
            if shape.has_text_frame and shape.text_frame.text.strip()
        ]
        if parts:
            out[i] = "\n".join(parts)
    return out


def check_numeric_checks(mismatches, slide_no, text, stats):
    # Signal test: "corr(force, slope) = +R, p=P"
    m = re.search(rf"corr\(force,\s*slope\)\s*=\s*({NUM}),\s*p\s*=\s*({NUM})", text)
    if m:
        r_found, p_found = float(m.group(1)), float(m.group(2))
        if abs(r_found - stats["corr_force_slope_r"]) > 0.005:
            mismatches.append(
                f"slide {slide_no}: signal-test r={r_found:+.3f} vs "
                f"summary_stats.json corr_force_slope_r={stats['corr_force_slope_r']:+.4f}"
            )
        if abs(p_found - stats["corr_force_slope_p"]) > 0.02:
            mismatches.append(
                f"slide {slide_no}: signal-test p={p_found} vs "
                f"summary_stats.json corr_force_slope_p={stats['corr_force_slope_p']:.4f}"
            )

    # Bootstrap spread: "95% spread = N m"
    m = re.search(rf"95%\s*spread\s*=\s*({NUM})\s*m", text)
    if m:
        found = float(m.group(1))
        expected = stats["bootstrap_spread_95pct_m"]
        if expected is not None and abs(found - expected) > 1.0:
            mismatches.append(
                f"slide {slide_no}: bootstrap spread {found} m vs "
                f"summary_stats.json bootstrap_spread_95pct_m={expected:.1f} m"
            )

    # Density-confound distance + verdict: "site moves N m ... PASS/FAIL/INCONCLUSIVE"
    m = re.search(rf"site moves\s*({NUM})\s*m.{{0,80}}", text)
    if m:
        found = float(m.group(1))
        expected = stats["density_confound_distance_m"]
        if expected is not None and abs(found - expected) > 1.0:
            mismatches.append(
                f"slide {slide_no}: density-confound distance {found} m vs "
                f"summary_stats.json density_confound_distance_m={expected:.1f} m"
            )
        window = m.group(0)
        if "PASS" in window and not stats["signal_ok"]:
            mismatches.append(
                f"slide {slide_no}: density-confound marked PASS, but "
                f"summary_stats.json signal_ok is false -- should read "
                f"INCONCLUSIVE (a confound test only means something once "
                f"the signal test it's confounding actually passes)"
            )

    # Force range: "Force range: MIN-MAX kN"
    m = re.search(rf"Force range:\s*({NUM})-({NUM})\s*kN", text)
    if m:
        fmin, fmax = float(m.group(1)), float(m.group(2))
        exp_min, exp_max = stats["force_min_N"] / 1000, stats["force_max_N"] / 1000
        if abs(fmin - exp_min) > 0.2:
            mismatches.append(
                f"slide {slide_no}: force range min {fmin} kN vs "
                f"summary_stats.json force_min_N={exp_min:.1f} kN"
            )
        if abs(fmax - exp_max) > 0.2:
            mismatches.append(
                f"slide {slide_no}: force range max {fmax} kN vs "
                f"summary_stats.json force_max_N={exp_max:.1f} kN"
            )

    # Max stroke: "Max stroke: N m"
    m = re.search(rf"Max stroke:\s*({NUM})\s*m", text)
    if m:
        found = float(m.group(1))
        expected = stats["stroke_max_m"]
        if abs(found - expected) > 0.02:
            mismatches.append(
                f"slide {slide_no}: max stroke {found} m vs "
                f"summary_stats.json stroke_max_m={expected:.3f} m"
            )

    # Outcome counts: "Landed: N ... Tipped: N ... Failed: N ... Timeout: N"
    m = re.search(
        r"Landed:\s*(\d+).*?Tipped:\s*(\d+).*?Failed:\s*(\d+).*?Timeout:\s*(\d+)",
        text, re.S,
    )
    if m:
        oc = stats["outcome_counts"]
        for label, found in zip(
            ("landed", "tipped", "failed", "timeout"),
            (int(g) for g in m.groups()),
            strict=True,
        ):
            if found != oc.get(label):
                mismatches.append(
                    f"slide {slide_no}: outcome count {label}={found} vs "
                    f"summary_stats.json outcome_counts[{label!r}]={oc.get(label)}"
                )


def check_terminology(mismatches, slide_no, text):
    # Collision terminology: any "mesh" mention near collision/triangle
    # language must be paired with an hfield/visual-only qualifier -- MuJoCo
    # collides mesh geoms against their convex hull, so an unqualified claim
    # that the terrain mesh drives collision is the exact bug this repo
    # spent a round finding and fixing (docs/known-issues.md).
    for m in re.finditer(r".{0,200}mesh.{0,200}", text, re.IGNORECASE):
        window = m.group(0)
        low = window.lower()
        if "hfield" in low or "visual" in low:
            continue
        if "collis" in low or "triangle" in low:
            mismatches.append(
                f"slide {slide_no}: unqualified 'mesh' near collision/triangle "
                f"language -- should note collision uses an hfield: "
                f"{window.strip()!r}"
            )

    # Any displayed landing coordinate needs a disclaimer -- either that it's
    # a stale pre-fix run (slides 15/16: "v1"/"illustrative"), or that it's
    # not a validated recommendation (slide 11: "not a certified
    # recommendation" / "did not pass"). Bare, undisclaimed coordinates are
    # exactly how the original deck ended up presenting a null result as a
    # confident answer.
    disclaimer_phrases = ("illustrative", "v1", "not a certified", "did not pass",
                           "not certified", "not statistically significant")
    if re.search(r"X\s*=\s*" + NUM, text):
        low = text.lower()
        if not any(phrase in low for phrase in disclaimer_phrases):
            mismatches.append(
                f"slide {slide_no}: shows a landing coordinate (X = ...) with "
                f"no disclaimer that it isn't a validated recommendation"
            )


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("pptx_path", help="Path to the .pptx deck to check")
    args = ap.parse_args()

    stats = load_stats()
    texts = slide_texts(args.pptx_path)

    mismatches = []
    for slide_no, text in texts.items():
        check_numeric_checks(mismatches, slide_no, text, stats)
        check_terminology(mismatches, slide_no, text)

    print(f"Checked {len(texts)} slides against {SUMMARY_STATS}")
    if mismatches:
        print(f"\n{len(mismatches)} MISMATCH(ES):")
        for m in mismatches:
            print(f"  - {m}")
        return 1

    print("\nNo mismatches found.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
