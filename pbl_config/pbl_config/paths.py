from pathlib import Path

from ament_index_python.packages import get_package_share_directory


def get_stack_master_path(subpath: str = '') -> Path:
    """Resolves a path relative to the stack_master package source tree.

    Goes via the installed share directory to find the workspace root, since
    stack_master's config folder is not fully mirrored into the install space.
    """
    return Path(get_package_share_directory('stack_master')).parents[3] / 'src/race_stack/stack_master' / subpath
