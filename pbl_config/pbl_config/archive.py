import logging
from datetime import datetime
from pathlib import Path
from typing import Callable


def write_archived(folder: Path, filename: str, write: Callable[[Path], None], update_latest: bool = True) -> Path:
    """Writes `folder/archive/<YYYYMMDD>_<filename>` with `write(path)`.

    If `update_latest`, `folder/default.<ext>` becomes a relative symlink to the new file.
    """
    archive = Path(folder) / 'archive'
    archive.mkdir(parents=True, exist_ok=True)
    path = archive / f'{datetime.now():%Y%m%d}_{filename}'
    if path.exists():
        logging.warning("%s already exists and will be overwritten", path)
    write(path)
    logging.info("Saved %s", path)

    if update_latest:
        latest = Path(folder) / ('default' + path.suffix)
        if latest.is_symlink() or latest.exists():
            latest.unlink()
        latest.symlink_to(Path('archive') / path.name)
        logging.info("Updated %s to point to %s", latest, path)
    return path
