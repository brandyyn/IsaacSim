# Photo/video fold pattern used by the joint workshop

The supplied stills and motion references show one open-ended square joint,
not a stack of independent hinges. The two square perimeter frames stay rigid;
the four triangular web sectors on each side of the waist fold through the
thin PET/PLA seams. At the waist the seams form the repeated diagonal/X
pattern visible in the close-up photos. The black and red crosses are cable
routing, not extra fold faces.

The workshop therefore keeps the original `input.json` 28-vertex / 50-panel /
76-line registry and uses the following photo-aligned actuator groups:

| Family | Cable indices | Observed action |
|---|---:|---|
| Compression | 0–3 | Four same-corner axial pulls shorten the joint symmetrically. |
| Bend X+ / X− | 0–1 / 2–3 | One axial corner pair pulls one side of the waist. |
| Bend Y+ / Y− | 1–2 / 0–3 | The adjacent axial pair pulls the orthogonal side. |
| Twist CW / CCW | 4–7 / 8–11 | One diagonal crossing direction is pulled at a time. |

`exact_joint.mechanics.cable_anchors()` orders the same-corner strands first,
then the two diagonal crossing directions. `tension_pattern()` is the single
source of truth used by both the FEM and the UI, so selecting a family cannot
silently switch to a prescribed lower-frame displacement.

The visible white fold lines are the original source crease registry. The
photo/open preset removes only the two JSON end-cap faces and leaves the square
perimeters as the rigid interfaces; the side-panel interiors and PET fold strips
remain deformable. A loaded cable is highlighted gold, while black/red routing
remains visible for comparison with the physical joint.

This is a photo-informed structural mapping, not a claim that the videos supply
measured cable tension, guide friction, pre-crease angles, or material failure
data. The FEM still reports small, uncalibrated quasistatic motion at the
reference 0.4 mm PLA / 80 um PET material values. Increase tension or adjust
the fold-compliance control only as an experiment and retain the guard/mesh
checks when comparing against the physical videos.
