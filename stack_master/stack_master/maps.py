import os
from pathlib import Path


def maps_dir() -> Path:
    """Directory holding all maps, set by MAPS_DIR (see docker-compose.yaml)."""
    path = os.environ.get('MAPS_DIR')
    if not path:
        raise RuntimeError('MAPS_DIR is not set; it should point to race_stack/maps')
    return Path(path)


def map_dir(map_name: str) -> Path:
    """Directory of a single map, e.g. $MAPS_DIR/<map_name>."""
    return maps_dir() / map_name
