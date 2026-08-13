# Orientation formats

How a placement's orientation is shown and typed. `Common/OrientationFormat.h`.

The stored orientation is always the CadNode's rotation matrix. Nothing in the format table is
persisted and nothing in it is authoritative: every format reads that matrix and writes it back. So
switching format cannot move a robot, and a station file means the same thing whatever the panel was
set to when it was written.

The formats exist because controllers do not agree. The same three numbers are a different
orientation on a KUKA and a Fanuc, and an operator transcribing a placement off a teach pendant needs
to say whose numbers they are copying.

The header is header-only so `tools/orientation_format_check.cpp` exercises the same code the UI runs
rather than a transcription of it.

## What the round-trip check cannot tell you

It proves each format is self-consistent — read a rotation, type nothing, write it back, get the same
rotation. That is a different claim from "this is what a KUKA means by A, B, C".

**A misattributed convention round-trips perfectly and is still wrong.**

So attributions carry their own evidence. Most rows are pinned to reference values in
`orientation_format_check.cpp`, read by an independent conversion tool on two rotations. The AR4 rows
come from the vendor's own kinematics source, quoted below. Omron TM is the one row with neither.

Confirming an attribution on hardware means typing a known pose into a real pendant and into this
panel and comparing. It has to be a pose with three distinct non-zero angles: anything with two zeros
reads the same in most of these, and a symmetric rotation makes several coincide outright.

## Grouping

Rows are grouped by who uses each one rather than by axis order, because "whose pendant am I copying
off" is a question an operator can answer and "which axis order is this" is not. Manufacturers
sharing a row genuinely share a representation: ABB's EulerZYX is KUKA's A, B, C, and Motoman's
numbers are Fanuc's.

Getting the choice wrong is silent. Every one of these produces plausible-looking numbers, and only
the arm ends up rotated.

An extrinsic convention — rotations about the fixed frame, which is what Fanuc and Yaskawa mean by
W, P, R — is the same rotation as the reversed intrinsic one, so it is stored as its intrinsic
equivalent with the displayed fields run backwards rather than as a second code path.

## The AR4, which has two rows

They are not the same representation. The arm's **tool frame** is XYZ intrinsic Euler. The pose its
**HMI** shows and accepts is a rotation vector in degrees, the same representation Universal Robots
uses. Which row a number wants depends on which box it was copied out of.

The rotation-vector attribution is established from the Annin Robotics ar4-hmi source. Three separate
things in that source point the wrong way:

- `ARrobots/src/bindings.cpp` exposes only `xyzuvw` to Python. `forward_kinematics` returns it and
  `inverse_kinematics` takes it. No pose-to-Euler conversion is exposed at all, so whatever the GUI
  shows has to come from there.
- `kinematics.cpp`'s `pose_2_xyzuvw` ends with `out[i] = vector[i] * angle` — the degree and radian
  factors either side cancel — so it is axis times angle, in radians. `AR4.py` then applies
  `math.degrees()` before display.
- There **is** an Euler routine in `kinematics.cpp`, `xyzwpr_2_pose`, and it builds `Rx*Ry*Rz`. But
  the bindings use it for `set_robot_tool_frame`, not for the arm's pose. Reading that function and
  stopping there gives XYZ intrinsic, which is the wrong answer for the position readout.

One more trap in the HMI itself: its three orientation boxes are named Rz, Ry, Rx, but they are
filled from `fk_xyzuvw[3]`, `[4]` and `[5]` in that order, so the box labelled Rz holds the rotation
vector's X component. The names are legacy from the AR2/AR3 software, which did report Euler angles.
**Read those boxes positionally, not by name.**

## The default

XYZ intrinsic, chosen from this project's own code rather than from a vendor. The tool target
editor's `composeLocalWpr` builds `basePose * Rx(w) * Ry(p) * Rz(r)` and `localWprDegrees` reads it
back the same way, so a base frame and a tool target describe orientation identically instead of by
two different rules in one application. It is also what the AR4's own `set_robot_tool_frame` takes,
and a base frame is closer in kind to a tool frame than to a pose readout.

Deliberately **not** the AR4 HMI's pose convention. That attribution is read off transpiled source
and is not hardware-confirmed, and defaulting to something uncertain is worse than defaulting to
something this codebase already does.

## Sanity checks

The three numbers of a rotation vector always satisfy `sqrt(a^2+b^2+c^2) <= 180 degrees`, because
every rotation is at most a half turn about some axis. An Euler triple has no such bound, so three
numbers that break it did not come from a rotation vector.

Epson is the Staubli composition listed in the opposite order, so an Epson reading is a Staubli
reading backwards. Identified by fitting the reference values on two rotations at once; one rotation
alone left it tied with ZXY intrinsic, which agrees on symmetric cases and not otherwise.

ZXZ is the sibling of the Adept ZYZ row: since `Ry = Rz(90) * Rx * Rz(-90)`, a ZYZ triple
`(a, b, c)` is the ZXZ triple `(a+90, b, c-90)`, which is exactly the offset between those rows in
the reference values.

## Pole handling

`kPoleEpsilon` is how close to a pole an Euler decomposition switches to its degenerate branch.

Both branches get worse as the pole is approached, in opposite directions, so this is a floor on the
worse of the two rather than a preference. Off the pole the formulas take `atan2` of two matrix
entries that both shrink with `cos(second)`; once they reach the `1e-16` noise floor of the matrix
the answer is noise, and at the pole exactly they vanish. On the pole the branch forces the third
angle to zero, misstating the rotation by about `cos(second)`. The two are equal at about `1e-8`.

It has to be this loose. A half turn about a slanted axis composes to a matrix whose pole term is
`2e-8` off, not `1e-16`, so a threshold of `1e-9` misses it, takes the off-pole branch, and reads an
orientation nearly two units of matrix wrong.

At a pole only the sum or the difference of the first and third angles is determined, so the third is
pinned to zero and everything goes into the first. With the third angle zero the rotation is just
`Ri(first) * Rj(second)`, so dividing `Rj(second)` back out leaves a plain rotation about axis i.
One derivation covers all twelve axis orders; a per-convention branch was wrong for ZXZ in a way no
ZYZ round-trip could show.

At a half turn the skew part vanishes, so the axis comes from the symmetric part. `(R + I)/2` is the
outer product of the axis with itself, so its diagonal is the axis components squared — take the
largest to divide by, because a small one amplifies the error in the rest. The off-diagonals are
averaged across the two mirrored entries and then halved, hence the quarter: halving alone
double-counts, because R is symmetric at a half turn. That mistake reads a half turn about
`(1, 0, 1)` as one about `(2, 0, 1)`, which is only visibly wrong when the axis is slanted.
