# 20. A real camera and a real detector

## In plain words

Up to here, the "camera" was maths: I knew exactly where every marker was,
because I computed it. Real cameras don't work like that. They render pixels,
and a detector has to find the markers in those pixels, with blur, lighting
and rounding errors.

So the next step was to replace the perfect camera with a rendered one
(MuJoCo, a physics and graphics engine) showing real ArUco markers — the black
and white squares you've seen on robots — and let OpenCV's real detector find
them. Then rerun the conclusions and see which survive.

Three of the four verdicts reproduced. The fourth turned out to be
**structurally untestable** with markers, for a reason worth understanding.

## The gate

Before trusting any conclusion from rendered images, one number had to be
established: does the rendered-and-detected feature position agree with the
exact projection?

**0.095 px mean.** That is `mj_validate.py`, and it is a gate, not a result:
if this had been 2 px, nothing downstream would have meant anything.

## Five bugs the rendered port surfaced

Every one was found by measurement, not by reading code. Two of them produced
confident wrong conclusions that were written down before being caught.

### 1. The half-pixel principal point, and the refinement that hid it

Detected positions carried a systematic bias of exactly **(−0.4904, −0.5010)
px**. Not noise — a bias, the same in both axes, suspiciously close to half a
pixel.

It is half a pixel. OpenGL maps its normalised coordinates onto a
**continuous** pixel range, so the optical axis lands at `W/2`. OpenCV indexes
pixel **centres**, which sit half a pixel lower, so the principal point is
`(W−1)/2 = 639.5`, not `640`. Fixing it dropped the bias to (+0.0096,
−0.0010) px.

The part worth remembering: before the fix, three corner-refinement methods
were compared, and **APRILTAG looked by far the best** (0.127 px against 0.72).
It was not better. Its corner convention is offset by about half a pixel in
the *opposite* direction, so it was silently cancelling the bug:

| refinement | before fix | after fix |
|---|---|---|
| NONE | 0.7208 px | 0.1990 px |
| SUBPIX | 0.7145 px | **0.0836 px** |
| APRILTAG | **0.1266 px** | 0.5972 px |

Two errors of opposite sign, each hiding the other. Picking the method by
measured accuracy alone would have locked in **both**. The lesson generalises:
a systematic offset is evidence about a *convention*; a mean error is not.

### 2. Mirrored markers

The camera views the target plane from its back side, and a marker seen from
behind is mirrored. ArUco dictionaries are not mirror-invariant: **0 of 6
detected as rendered, 6 of 6 after mirroring the texture.**

Fixed by mirroring the texture, not by flipping the captured image — flipping
the frame would have hidden a genuine geometry error instead of fixing one.

### 3. MuJoCo's auto-computed `extent` clipped the scene

Near and far clipping planes are derived from a model "extent" that MuJoCo
computes from the geometry. Shrinking the markers therefore moved the near
plane and clipped everything: **image mean fell from 254 to 13 with no error
raised.** Rendering must not depend on the geometry under test, so `extent` is
now pinned.

### 4. Live renderers degrade silently, and faked a physical limit

Open renderer instances accumulate GPU resources. Past a handful, frames come
back **visually plausible but undetectable**. A sweep that built ~30 renderers
in one process reported 0 of 6 detections for every scene after the first.

That looked exactly like a physical limit of the geometry, and **was reported
as one**: "collapse is unrenderable". It was not. Closing each renderer showed
collapse renders cleanly down to c = 0.06. The real limit had to be
established a second time, by arithmetic on both constraints rather than by a
sweep.

### 5. Ghosting in the motion-blur model

Accumulation blur with a fixed, small number of sub-frames places samples
further apart than a marker, producing a row of ghost copies instead of a
smear. The detector locked onto a ghost and reported a feature **156 px** from
the truth — and detection *improved* with more blur, which is the signature of
an artifact, since blur cannot help. The sub-frame count now adapts so
successive samples land under a pixel apart.

## Correlated degradation: the honest hard case

Independent random noise is the easy case, and the guard shrugs it off
(d′ = 454 at 0.1–0.5 px; still 7.1 at a catastrophic 8 px; 15 with 20% gross
outliers). The hard case is noise that **co-varies with the failure**: a
control lurch causes motion blur on the same step, which corrupts the very
measurement that would have detected the lurch.

That needs a rendered camera to study at all. `mj_degrade.py` drives motion
blur from the camera's own measured inter-frame motion and adds a lagged
auto-exposure chasing image brightness. Measured at 10 fps with λ = 1.5:

| blur across exposure | N visible | operating margin | detector failure | run |
|---|---|---|---|---|
| 0 px | 3 | 1.62× | 0.00% | converges |
| 5.1 px | 3 | 1.62× | 0.00% | converges |
| 7.6 px | 3 | 1.63× | 0.42% | converges |
| 12.7 px | 0 | — | 50% | **target lost** |

Markers are ~41 px, so the cliff sits near a quarter of a marker width. The
finding: the guard's margin is **flat right up to the cliff and then its input
vanishes**. Correlated degradation does not *bias* the guard, it *removes* its
input — so a guard failure and a detection failure stay cleanly
distinguishable, which is the property you want when something goes wrong in a
real cell.

## The case markers cannot test

The collapsed target is a degeneracy *of 2D extent*, and an area-based marker
needs 2D extent to decode. Reproducing the numpy verdict needs collapse
c ≤ 0.05; the smallest collapse leaving six decodable markers at this working
distance is c = 0.35, even at 2560 px wide. A factor of seven, and **not a
resolution limit** — the guard's trigger condition and the detector's failure
coincide exactly in that regime.

That is why the Gazebo work later used plain dark dots rather than ArUco
markers, and why the README says real deployments need corner or edge features
there.

## Say it like this

> I replaced the perfect camera with a rendered one and OpenCV's real ArUco
> detector, gated on agreeing with exact projection to 0.095 pixels. That port
> surfaced five bugs, and two of them had already produced confident wrong
> conclusions — including one where a corner-refinement method looked best
> only because its convention was cancelling a half-pixel error of mine.
> Three of the four verdicts reproduced; the fourth is untestable with markers
> for a structural reason, which is itself a result.
