"""Keep the project's outputs directory exclusively for detection images."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def _run_name(output):
    try:
        relative = Path(output).resolve().relative_to(ROOT/'outputs')
    except ValueError:
        return None
    return relative if relative.parts else Path('latest')


def detail_directory(output):
    name = _run_name(output)
    return ROOT/'reports'/'runs'/name if name is not None else Path(output)


def raw_directory(output):
    name = _run_name(output)
    return ROOT/'data'/'captures'/name if name is not None else Path(output)
