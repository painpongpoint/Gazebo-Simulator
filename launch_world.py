"""Launch the saved Gazebo world from any checkout directory."""
import os
from pathlib import Path


def main():
    root = Path(__file__).resolve().parent
    paths = [root, root / 'models', root / 'models/hatchback_drivable']
    existing = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    os.environ['GZ_SIM_RESOURCE_PATH'] = os.pathsep.join(
        [str(path) for path in paths] + ([existing] if existing else [])
    )
    os.execvp('gz', ['gz', 'sim', '-r', str(root / 'world_drivable.sdf')])


if __name__ == '__main__':
    main()
