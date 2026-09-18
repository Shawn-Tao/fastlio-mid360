"""Read-only environment/PCD smoke check, not a map-quality evaluator."""
import argparse
import json
from pathlib import Path
import site
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--maps', type=Path, help='Optional existing directory of PCDs; never modified')
    args = parser.parse_args()
    eval_dir = Path(__file__).resolve().parent
    workspace = eval_dir.parents[1]
    assert Path(sys.prefix).resolve() == (eval_dir/'.venv').resolve(), sys.prefix
    assert Path(sys.base_prefix).resolve().is_relative_to(workspace/'.local_tools/python'), sys.base_prefix
    assert site.ENABLE_USER_SITE is False, 'User site-packages must stay isolated'
    assert sys.version_info[:2] == (3, 12), sys.version
    import numpy as np
    import scipy
    from scipy.spatial import cKDTree
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import open3d as o3d
    assert o3d.__version__ == '0.19.0', o3d.__version__
    sample = np.array([[0., 0., 0.], [1., 0., 0.], [0., 1., 0.]])
    distance, _ = cKDTree(sample).query([[0., 0., 0.]])
    assert distance[0] == 0
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(sample))
    assert len(cloud.points) == 3
    figure = plt.figure(); figure.canvas.draw(); plt.close(figure)
    print(f'Python {sys.version.split()[0]}: {sys.executable}')
    print(f'Base Python: {sys.base_prefix}')
    print(f'NumPy {np.__version__}; SciPy {scipy.__version__}; Matplotlib {matplotlib.__version__}; Open3D {o3d.__version__}')
    if args.maps is not None:
        directory = args.maps.expanduser().resolve(strict=True)
        paths = sorted(directory.glob('*.pcd'))
        if not paths:
            raise ValueError(f'No PCD files in {directory}')
        for path in paths:
            loaded = o3d.io.read_point_cloud(str(path))
            xyz = np.asarray(loaded.points)
            assert len(xyz) > 0 and np.isfinite(xyz).all(), path
            metadata = path.with_suffix('.pcd.json')
            if metadata.is_file():
                assert len(xyz) == json.loads(metadata.read_text())['points'], path
            print(f'PCD read OK: {path.name} ({len(xyz)} points)')
    print('PASS: isolated managed Python, imports, CPU geometry/KDTree and headless plotting')


if __name__ == '__main__':
    main()
