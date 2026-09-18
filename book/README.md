# The Blind Spot — the book

The complete account of the project in `..`, built from the ground up: it
assumes no robotics and no linear algebra, explains each term as it arrives,
and derives every idea before using it.

**Read the PDF**: [`docs/the-blind-spot.pdf`](../docs/the-blind-spot.pdf),
built from these files with `.venv/bin/python book/build_pdf.py` (needs
`reportlab`, which the study itself does not).

Every chapter opens **in plain words** with no jargon, gives the precise
version with the real numbers, and closes with **"Say it like this"** — a
spoken version for an interview.

## Contents

**Part 0 — Foundations**

| | |
|---|---|
| 1 | [What this is about](01-what-this-is-about.md) |
| 2 | [Where things are: frames and transforms](02-frames-and-transforms.md) |
| 3 | [How a camera turns the world into numbers](03-the-camera.md) |
| 4 | [Matrices, least squares, and the one idea behind every failure](04-matrices-and-svd.md) |
| 5 | [Feedback control in one chapter](05-feedback-control.md) |

**Part 1 — Visual servoing, derived**

| | |
|---|---|
| 6 | [The servo loop](06-the-servo-loop.md) |
| 7 | [Deriving the interaction matrix](07-interaction-matrix.md) |
| 8 | [The control law, and turning it into motion](08-control-law-and-motion.md) |
| 9 | [The harness: what is actually run](09-the-harness.md) |

**Part 2 — Four ways to go blind**

| | |
|---|---|
| 10 | [Camera retreat](10-camera-retreat.md) |
| 11 | [Feature dropout](11-feature-dropout.md) |
| 12 | [Two features left](12-two-features.md) |
| 13 | [The collapsed target](13-collapsed-target.md) |

**Part 3 — Fixes, and what they cost**

| | |
|---|---|
| 14 | [The 2001 partition](14-the-partition.md) |
| 15 | [Truncation: declining to move blind](15-truncation.md) |
| 16 | [The switch: deciding per step](16-the-switch.md) |
| 17 | [The guard that shipped](17-the-guard.md) |

**Part 4 — Making sure it is true**

| | |
|---|---|
| 18 | [How correctness was established](18-verification.md) |
| 19 | [Dead claims](19-dead-claims.md) |
| 20 | [A real camera and a real detector](20-rendered-camera.md) |

**Part 5 — On a robot**

| | |
|---|---|
| 21 | [ROS 2, from zero](21-ros2-from-zero.md) |
| 22 | [The guard as a ROS 2 node](22-the-guard-node.md) |
| 23 | [Building the Gazebo cell](23-the-gazebo-cell.md) |
| 24 | [Kinematics and perception on the arm](24-kinematics-and-perception.md) |
| 25 | [What the arm found](25-what-the-arm-found.md) |
| 26 | [The folding part, and the clip](26-the-folding-part.md) |

**Part 6 — Using it**

| | |
|---|---|
| 27 | [Limits, collected](27-limits.md) |
| 28 | [Telling the story](28-telling-the-story.md) |
| 29 | [Glossary, and every number in one place](29-glossary.md) |

## Reading paths

- **Never seen robotics**: 1 → 5 in order, then 10, 14, 17, then 26.
- **Know control, new to visual servoing**: 6 → 9, then part 2, then 17.
- **Preparing for an interview tomorrow**: 28, then 19, then 27, then 23.
- **Reviewing the engineering**: 9, 18, 19, 27.
