# 11. Failure 2: feature dropout

## In plain words

Half the markers disappear for a couple of seconds — an arm passes in front, a
fixture blocks them, the lighting changes. Three of six are gone, then they
come back.

The controller does not pause or slow down. It commands a motion **28.7 times**
larger than it was moving before.

## Why it happens

Six markers give twelve equations for six unknowns: comfortably redundant.
Three markers give six equations for six unknowns: exactly enough, in
principle, but with no slack.

More importantly, the three that remain are on one side of the target, so the
*shape* they make is thinner. In the language of chapter 4, one of the six
singular values of the interaction matrix has collapsed toward zero: there is
now a way the camera can move that these three markers can barely see.

And the controller inverts that matrix. Any component of the requested image
change that points along the weak direction gets multiplied by 1/σ, which is
now a big number. The command jumps.

This is the textbook **conditioning** failure, and unlike retreat it is a
numerical problem with a numerical answer.

## The fix that works

Truncation (chapter 15): look at the singular values, and refuse to move along
any direction weaker than a fraction τ of the strongest.

| controller | spike |
|---|---|
| classic IBVS | 28.7× |
| adaptive-rank truncation | 0.9× |
| partitioned (2001) | 1.1× |
| switched, area guard | 0.5× |

A spike **below 1** means the controller moved more slowly during the
disruption than before it. That is the behaviour you want: information went
missing, so do less. 28.7× is the opposite — information went missing, so
sprint.

## What this case teaches about thresholds

This scenario is also where a subtle calibration problem shows up, and it is
worth understanding because it looks like a healthy result until you know why.

If the guard's threshold is calibrated with all six markers visible, then
during dropout — three visible — the statistic is compared against a threshold
built for a different situation. The margin comes out at **1.19×**:
technically above the line, but only just, and for a structural reason rather
than a physical one. A subset of points always has a smaller polygon than the
full set, so the comparison is unfair by construction (the mathematical name
is singular-value **interlacing**).

Calibrating **per visible count** — a separate threshold for 6, 5, 4, 3
markers — restores the same operating point to **45.9×** of its threshold.
Same run, same physics, honest comparison. Chapter 17 has the procedure.

That fix is in the shipped library, and the README labels it plainly as "a fix
for a defect, not a free property".

## A note on what "the same disruption" means

Every controller in the table above meets the identical occlusion: the same
three markers, hidden from step 30 to step 90, scripted by step number rather
than by time or by pose. Without that, a slower controller would meet the
occlusion at a different place on its trajectory and the comparison would be
meaningless.

The same principle drives the two-arm Gazebo demo later: both cells live in
**one** physics world so they see the same part fold at the same instant
(chapter 23).

## Say it like this

> Occlude three of six markers and classic IBVS spikes 28.7× — that's the
> textbook conditioning failure, where one direction of the interaction matrix
> goes blind and the inverse amplifies it. Truncation fixes it properly, down
> to 0.9×. This case is also where I found that a guard threshold calibrated
> at six markers is structurally unfair at three: the margin reads 1.19×, and
> conditioning the calibration on the visible count makes the same instant
> read 45.9×.
