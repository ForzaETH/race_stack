import sys
from pathlib import Path


def main():
    """Print the first unused <folder>/output-<N> path, without newline so $(command) can use it."""
    folder = Path(sys.argv[1])
    n = 1
    while (folder / f'output-{n}').exists():
        n += 1
    sys.stdout.write(str(folder / f'output-{n}'))


if __name__ == '__main__':
    main()
