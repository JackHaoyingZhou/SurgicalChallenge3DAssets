# Phantom 1.4 notes
SRC assets in SI units with new PSM instruments.

## Changelog

- Checkpoint 15: Reduce radius of the new tool pitch link's collision shape. Set restitution value to zero for `tool_roll_link`, `tool_pitch_link` and `fingers` in new tool.
- Checkpoint 14: Rename old instruments. Blender does not allow having two objects with the same name.
- Checkpoint 13: Fix collision margins for instruments. Collision margins 0.0001m for all instruments.
- Checkpoint 12: Apply decimation modifiers.
- Checkpoint 10: Add old instruments to the blender file.
- Checkpoint 11: Add decimate modifier, but not applied. Kept  30% of vertices in instrument tip and 10% of enclosure box
- Checkpoint 09: Improve textures
- Checkpoint 08: Full resolution assets fixed by Jack. Fixs included changing orientation of joint between gripper and tool yaw link and fix collision bodies for the grippers.
    - This assets have the original white textures
    - They have not been decimated.
- Checkpoint 04: Decimate grippers and remove white textures. (kept only 15% of vertices)
- Checkpoint 03: Fix collision of grippers. Remove second collision primitive of each finger in the gripper.
- Checkpoint 01: Add new instruments in the correct orientation

## Running instructions.

Download the `high_res` and `low_res` folders 
[here](https://livejohnshopkins.sharepoint.com/:f:/r/sites/Surgicalroboticschallenge/Shared%20Documents/General/AMBF_assets/3d_med?csf=1&web=1&e=UfgpFX) and add them to this directory.

Then, the run simulation with the `./run_env.sh` script. Blender files are included in onedrive link.

## Missing features.

* Teleoperation scripts are still not working
