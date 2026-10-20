# Auckland University Electrical Engineering Robotics Association - Undergrad Balloon Popper Competition. Use with X-UAV Mini Talon (V-tail). Aiming for first place this year !!!
"""export_paths - an OPTIONAL fixed override for WHERE the export buttons copy to.

By default the export buttons copy into a "Downloaded Files" folder beside
main.py.  main.py records its own location at launch (from __file__, so it is
where main.py lives regardless of the working directory the program was started
from) and hands it to file_export.set_destination().  The folder itself is
created on the first Copy that has a file to write, not at launch, so a run that
never exports leaves no empty folder behind.  The destination therefore follows
the code and needs no per-machine setting.

Set EXPORT_DEST_WINDOWS to force a fixed destination instead - e.g. a folder on
the Windows side of a WSL install (a /mnt/<drive>/... path, NOT a native
C:\\Users\\... path).  Leave it None to use the launch-relative default; a hand
override wins over the default.

Kept out of constants.py deliberately: constants.py is regenerated wholesale by
the plant-ID / sweep suggest writers and copied between machines and between sim
and the real aircraft - a personal desktop path would ride along every time.

WSL notes (only when a /mnt override is set):
  - /mnt/<drive> is the DrvFs mount of a Windows drive.  A path with spaces
    needs no quoting or escaping here - it is passed to shutil as a plain Python
    string, never to a shell.
  - The folder is created on first use if it does not exist.
  - If Windows drives are not mounted (automount disabled, or the path names a
    drive letter that does not exist), the export button reports the failure and
    copies nothing; nothing raises into the GUI or the control loop.
"""

# Optional fixed override.  None = use the launch-relative "Downloaded Files"
# folder beside main.py (see the module docstring).  Set a WSL-visible
# /mnt/<drive>/... path to force a fixed destination.
EXPORT_DEST_WINDOWS = None
