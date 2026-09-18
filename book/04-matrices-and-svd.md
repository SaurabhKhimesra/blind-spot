# 4. Matrices, least squares, and the one idea that explains every failure

This chapter is the mathematical spine of the book. It builds up to the
**singular value decomposition**, because "a small singular value" is the
answer to almost every "why did the robot do that?" in later chapters.

## A matrix is a machine that turns one list of numbers into another

If `v` is six numbers (a camera twist) and `L` is a table with 12 rows and 6
columns, then `L v` is 12 numbers (how fast each of six markers moves in the
image, x and y each). That is all a matrix-vector product is: each output is a
weighted sum of the inputs, and the weights are one row of the table.

The reverse question is the interesting one. **Given the output I want, what
input produces it?** That is solving `L v = b` for `v`, and three cases exist.

## Three cases of solving

**Square and healthy.** As many equations as unknowns, all independent. One
exact answer, `v = L⁻¹ b`.

**More equations than unknowns** (12 rows, 6 unknowns — six markers). Usually
no exact answer exists, because real measurements disagree slightly. The
sensible answer is the one that misses by the least, measured as the sum of
squared errors: **least squares**. Redundancy is protective here, because no
single noisy measurement can drag the answer far.

**Fewer equations than unknowns** (4 rows, 6 unknowns — two markers). Now
infinitely many answers fit exactly, and you must pick one. The standard
choice is the **smallest** one, the minimum-norm solution.

The **pseudo-inverse**, written `L⁺` (`numpy.linalg.pinv`), does the right
thing in all three cases: exact when it can, least squares when
over-determined, minimum-norm when under-determined.

That last property matters more than it sounds. In chapter 12, classic IBVS
survives having only two markers **precisely because** it is under-determined
and the pseudo-inverse hands it the smallest solution, which is gentle. The
2001 partition, in the same situation, has an exactly-determined 4×4 with no
slack, and it explodes.

## The singular value decomposition

Any matrix `L` can be written

```
L = U Σ Vᵀ
```

Read it right to left as three simple operations:

1. `Vᵀ` — rotate the input space so its axes line up with the matrix's
   natural "input directions".
2. `Σ` — stretch each of those axes by a number: σ₁ ≥ σ₂ ≥ … ≥ σ₆ ≥ 0. These
   are the **singular values**.
3. `U` — rotate into the output space.

So a matrix does nothing more exotic than: rotate, stretch each axis, rotate.
The singular values say **how strongly the matrix responds** along each of its
six input directions.

For our `L`, the six input directions are combinations of camera motions, and
the stretch says how much the image changes when the camera moves that way.

- A **large** singular value: move this way and the image changes a lot. Easy
  to see, easy to control.
- A **small** singular value: move this way and the image barely changes. The
  camera can move a long way with almost no evidence in the picture.

## Why small singular values are dangerous

Inverting a matrix inverts the stretches: 1/σ. So a direction with σ = 0.004
gets multiplied by 250 on the way back.

Now put the control law next to it. The controller says "I want the image to
change like this", inverts, and commands the motion. If the requested change
has even a *tiny* component along a weak direction — from noise, or from an
asymmetry, or from a modelling error — the inverse amplifies it by 1/σ and the
robot makes a large, confident, wrong movement.

**That single sentence explains the velocity spikes in chapters 11, 12 and 13,
the orbit in chapter 25, and the creep in chapter 26.**

Vocabulary that follows from this:

- **Rank**: how many singular values are meaningfully non-zero, i.e. how many
  directions the matrix can actually see.
- **Null space**: directions with σ = 0. The matrix is completely blind to
  them; motion along them changes the image not at all.
- **Condition number**: σ₁ / σ_last. How lopsided the matrix is. Large means
  trouble.
- **Ill-conditioned**: a polite way of saying "some direction is nearly
  invisible and the inverse is about to shout".

## Two ways to respond

**Truncation.** Look at the singular values, and any direction weaker than
some fraction τ of the strongest, simply refuse to move along it. You give up
control of a direction you could not see anyway, in exchange for not being
amplified by 1/σ. Chapter 15.

**Watching the numbers.** Compute a singular value each step and treat it as a
health signal. σ₆ — the sixth, padded so that a matrix with too few rows reads
exactly zero — is the project's spectral health metric.

Padding matters. An earlier version used "the smallest singular value of
whatever shape the matrix happens to be". With two markers, `L` is 4×6, its
smallest singular value is a healthy-looking **9.97e-2**, and the true sixth
one is **exactly 0** because two whole directions are invisible. The metric
could not see a null space; padding to a fixed length fixes it.

## The ambiguity that shows up everywhere here

Look at two columns of the interaction matrix (chapter 7): the one for sliding
sideways (`vx`) contains `−1/Z`, and the one for rotating about the vertical
axis (`ωy`) contains `−(1 + x²)`. For a small target that is far away, `x` is
small and both columns are nearly constant — so **sliding right and rotating
right look almost the same in the image**.

"Almost the same" means: the *difference* between them is a direction with a
very small singular value. In this project's benchmark that pair sits at
**0.8% of the strongest** direction, an amplification of about **130×**, and
it corresponds to the camera *orbiting* the target: `vx = −Z·ωy`. Chapter 25
is what happens when a controller's command lands there.

## Say it like this

> An SVD says a matrix rotates, stretches along six axes, and rotates again.
> The stretches are the singular values, and inverting the matrix inverts
> them, so a direction the image can barely see gets amplified by one over a
> very small number. Every lurch in this project is that amplification, and
> the two responses are to truncate those directions or to watch them and
> change control law.
