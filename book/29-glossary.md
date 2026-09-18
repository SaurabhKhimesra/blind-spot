# 29. Glossary, and every number in one place

## Glossary

| term | meaning |
|---|---|
| **IBVS** | image-based visual servoing: control computed directly from image error |
| **PBVS** | position-based: estimate the 3D pose first, then control it |
| **feature** | a point the detector finds; here, a marker centre |
| **normalised image coordinates** | pixels with the intrinsics divided out: x = (u−cx)/fx = X/Z |
| **intrinsics** | focal lengths and principal point; arrive as `camera_info` in ROS |
| **principal point** | where the optical axis meets the sensor; `(W−1)/2`, not `W/2` |
| **frame** | an origin plus three axes; every coordinate is relative to one |
| **pose** | position and orientation together, kept as a 4×4 matrix |
| **twist** | six numbers: three linear and three angular velocities |
| **se3_exp / se3_log** | apply a twist to a pose / recover a twist from a pose difference |
| **interaction matrix `L`** | maps camera twist to feature velocity in the image, 2N×6 |
| **pseudo-inverse `L⁺`** | least-squares inverse; minimum-norm when under-determined |
| **SVD** | rotate, stretch along six axes, rotate: `L = U Σ Vᵀ` |
| **singular value** | how strongly the matrix responds along one direction |
| **σ₆** | the sixth singular value, padded — exactly 0 if `L` has a null space |
| **null space** | directions the matrix cannot see at all |
| **ill-conditioned** | some direction is nearly invisible, so the inverse amplifies |
| **rank** | how many directions the matrix can actually see |
| **truncation (τ)** | skipping directions weaker than τ × the strongest |
| **the partition** | the 2001 scheme: `vz` from polygon area, `ωz` from a line angle |
| **coupling term** | `L_z v_z` added so the other four DOF cancel what the partitioned two cause |
| **the guard** | the runtime check on the partition's own area feature |
| **margin** | signal ÷ threshold; below 1 means the guard fires |
| **calibration** | the healthy pose distribution that sets the threshold |
| **spike** | peak commanded ‖v‖ during a disruption ÷ the same controller's ‖v‖ before it |
| **convergence** | final image error below 1e-4 |
| **clean cost** | extra steps on an undisturbed run, at a stated convergence threshold |
| **node / topic / message** | a ROS program / a named channel / the typed thing on it |
| **ros2_control** | the ROS 2 joint-level control framework |
| **TF** | the live tree of transforms between frames |
| **sim time** | the simulator's clock, published on `/clock` |
| **danger cylinder** | the surface through three points where the geometry is singular |

## Every number, and where it comes from

**The four failures**

| number | meaning |
|---|---|
| 68.19 m | how far classic IBVS retreats on a 180° rotation |
| −1.0000 | command/error alignment during that retreat — perfect, while failing |
| 13.77 m | truncation on the same case: still diverges |
| 0.80 m | every partition-based controller: never backed up |
| 28.7× | classic IBVS spike on 3-of-6 dropout |
| 0.9× | truncation on the same dropout |
| 288.4× | partitioned spike with two features |
| 11.6× | the combined controller on two features — the gap to close |
| 0.5× | every switched rule on two features — the gap closed |
| 7.3e-3 / 2.0e-3 | partitioned / combined floors on the collapsed target |
| 2.4e-5 | truncation and the geometric guards: converged |
| [3.68, 3.66, 0.0124, 0.0037] | the reduced matrix's singular values when collapsed |

**The guard**

| number | meaning |
|---|---|
| 1st percentile, 4000 poses, seed 0 | the calibration procedure |
| 3.87e-3 / 1.74e-2 | healthy threshold for the ring / the square, 4.5× apart |
| 1.19× → 45.9× | dropout margin, single threshold vs conditioned on visible count |
| 7.96e-2 → 1.13e-2 | known-good vs in-situ threshold: 7× lower, guard disabled |
| 0.33% / 0.76% | how far 2 px of noise moves the area statistic (square / ring) |
| 1.03% / 2.35% | the same for σ₆: about 3× more |
| 0.5× → 3.0× → 3.7× → 5.7× | two-feature spike with hysteresis of 0, 2, 3, 5 steps |
| +13% | clean cost of the partition at a 1% convergence threshold |

**Dead claims**

| number | what it killed |
|---|---|
| 0.5× vs a 2× bar | the learned-policy plan |
| rank 3 in a 4×4 | the rank-margin rule |
| 1.65–3.73× overlap band | a single global spectral threshold |
| 27.01× vs 0.99× | claim 4's revival: σ₆ silent where area fires |
| 98.78% of σ_min | the collapsed-target spike, as τ artifact |
| 231 ms / 0 ms / 0 ms | forward prediction: warning only where it is not needed |
| 15 of 16 identical | my retracted claim that switching caused an arm spike |
| 9.97e-2 vs exactly 0 | why the health metric had to be padded σ₆ |

**The rendered camera**

| number | meaning |
|---|---|
| 0.095 px | rendered-and-detected vs exact projection: the gate |
| (−0.4904, −0.5010) px | the half-pixel principal-point bias |
| 0.1266 → 0.5972 px | APRILTAG refinement before / after fixing that bias |
| 0 of 6 → 6 of 6 | marker detection before / after mirroring the texture |
| 254 → 13 | image mean when the auto-computed extent clipped the scene |
| 156 px | how far a ghost from the blur model displaced a feature |
| d′ = 454 | separation of healthy and degenerate under realistic noise |
| 1.62× flat, then 50% detector failure at 12.7 px | correlated degradation: flat, then cliff |
| c ≤ 0.05 needed vs c = 0.35 achievable | why markers cannot test the collapsed case |

**The arm**

| number | meaning |
|---|---|
| 1e-6 m | hand-written forward kinematics vs TF |
| 2.1e-15 | the partition's sideways command at the exactly symmetric retreat start |
| ~130× | amplification of the weak orbit directions of `L_xy` |
| 97 / 3 of 100 | partition and σ₆ at 180° from imperfect starts: converged / lost target |
| 100 / 0 of 100 | area guard on the same starts: converged / lost target |
| 2.02–2.88 m | how far the area guard's false positive backs the camera away |
| 123–125 steps, 1.80–2.03 m | the same triggered by 0.5 px of noise, five seeds |
| 26.3 → 16.2 cm, 1.46 s | the unguarded arm's lunge into the protective stop, folding part |
| 0.96 s, 0.90× | when the guard fired, and its margin |
| 22–24 cm | where the guarded arm held |
| 6.8 cm / 18 cm / 22.5 cm | the same act in numpy: partition / guarded / plain IBVS |
| 1.29× | σ₆'s minimum during the fold: never fired |
| 1.16× at 85° over 3 s | the masked slow fold: guard never fires |

**Test counts**

| number | meaning |
|---|---|
| 177 | regression checks in `tests.py` |
| 16 | finite-difference checks in `fd_check.py` |
| 34 | API and ROS-bridge checks in `test_blindspot.py` |

## References

- Chaumette & Hutchinson, *Visual servo control part I: basic approaches*, 2006
- Corke & Hutchinson, *A new partitioned approach to image-based visual servo
  control*, IEEE T-RA, 2001
- Malis, Chaumette & Boudet, *2½D visual servoing*, 1999
- Michel & Rives, *Singularities in the determination of the situation of a
  robot effector from the perspective view of three points*, INRIA, 1993
