"""Helpers shared by the launch files."""
import os

_SNAP_SUFFIX = "_VSCODE_SNAP_ORIG"


def clean_gui_env():
    """Environment for GUI processes without VS Code snap leakage.

    A terminal inside VS Code (snap) exports GTK_PATH, LOCPATH, GIO_MODULE_DIR, ... pointing into
    /snap, which makes rviz2 load snap libraries and die (libpthread symbol lookup error). VS Code
    keeps the originals in <VAR>_VSCODE_SNAP_ORIG: restore them (unset when empty).
    """
    env = dict(os.environ)
    for key in [k for k in env if k.endswith(_SNAP_SUFFIX)]:
        var, orig = key[: -len(_SNAP_SUFFIX)], env.pop(key)
        if orig:
            env[var] = orig
        else:
            env.pop(var, None)
    return env
