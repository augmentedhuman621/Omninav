OmniNav simulation meshes (CAD-derived, decimated)
==================================================

Source: the SolidWorks CAD set of the robot (DI31013 battery-box / 4WD mecanum chassis
project) exported to binary STL and decimated with a pure-numpy vertex-clustering
script (grid 2.0 mm, wheels 1.0 mm), then recentred so that each mesh is centred on its
bounding-box centre. Every file is in millimetres and is used with
<mesh ... scale="0.001 0.001 0.001"/>.

The wheels are deliberately kept below 21,845 triangles (= 65,536 vertices): Ogre, the
renderer behind Gazebo classic, silently fails to draw meshes whose vertex count exceeds
the 16-bit index limit on GPUs without 32-bit index buffer support (e.g. a VirtualBox VM
with 3D acceleration disabled). The first cut at 24,915 triangles hit exactly that limit
and the wheels were invisible in gzclient while RViz2 showed them fine.

Files (triangles, size):
  base_plate.stl        5,244 tris  256 KB   top chassis deck 230 x 304.88 x 3 mm
  wheel_left.stl       17,114 tris  836 KB   left mecanum wheel, 41 x 80.3 x 80.6 mm
  wheel_right.stl      16,692 tris  815 KB   right mecanum wheel (mirrored hub)
  motor.stl             1,900 tris   93 KB   JGB37 encoder gear motor 37 x 36.9 x 74.5 mm
  coupling.stl            826 tris   40 KB   6 mm to 8 mm shaft coupling 12 x 12 x 26 mm
  battery_box.stl       2,712 tris  133 KB   sheet-metal battery tray 68.6 x 120.4 x 36.2 mm
  side_panel_left.stl    468 tris   23 KB   U-shaped side panel hanging from the deck, 182 x 51.3 x 40 mm
  side_panel_right.stl   468 tris   23 KB   same part, right-hand side
  gimbal_yoke.stl       2,328 tris  114 KB   2-DOF camera gimbal lower/upper yoke 45.5 x 39 x 46.6 mm
  camera_body.stl       2,652 tris  130 KB   camera body 31.6 x 35.5 x 27.2 mm
  camera_lens.stl         592 tris   29 KB   camera lens barrel 10.4 x 10.0 x 10.3 mm

Not included on purpose:
  * upper plate (158 x 260 mm): its SLDPRT exports as a zero-thickness surface body, so it
    cannot be meshed; the real assembly uses the 304.88 x 230 plate as the top deck anyway.
  * motor L-brackets: the CAD part's own frame could not be resolved, so the URDF models
    the mounts with two small boxes instead of the mesh.
  * screws, washers, hidden driver boards: sub-millimetre detail, invisible at robot scale.

Collision geometry is intentionally kept as simple boxes/cylinders; only the visuals
use these meshes. Mass and inertia stay engineering estimates (the CAD model has no
material assigned).
