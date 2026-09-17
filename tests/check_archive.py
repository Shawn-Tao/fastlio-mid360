#!/usr/bin/env python3
"""Read-only checks of a portable deployment ZIP, including CRC and SDK ELF."""
from pathlib import Path, PurePosixPath
import stat
import sys
import zipfile


def check_archive(path):
    with zipfile.ZipFile(path) as archive:
        infos = archive.infolist()
        roots = {PurePosixPath(item.filename).parts[0] for item in infos}
        if len(roots) != 1:
            raise ValueError('Expected exactly one workspace directory')
        root = next(iter(roots))
        names = {item.filename for item in infos}
        if len(names) != len(infos):
            raise ValueError('Duplicate archive entries')
        excluded = {'build', 'install', 'log', 'bags', 'records', 'Record_Path', 'reference', 'maps', 'secrets',
                    '.git', '.agents', '.codex', '__pycache__'}
        for item in infos:
            parts = PurePosixPath(item.filename).parts
            if item.filename.startswith('/') or '..' in parts:
                raise ValueError(f'Unsafe archive path: {item.filename}')
            # Exclusion applies to workspace-level artifacts, not C++ source
            # directories such as src/.../comm or source cmake modules.
            if len(parts) > 1 and parts[1] in excluded:
                raise ValueError(f'Unexpected deployment artifact: {item.filename}')
            if len(parts) > 2 and parts[1:3] == ('scripts', 'backup'):
                raise ValueError(f'Archived scripts must not ship: {item.filename}')
            if any(p in {'.git', '.agents', '.codex', '__pycache__'} for p in parts):
                raise ValueError(f'Unexpected metadata/cache: {item.filename}')
            local_suffixes = ('.local.json', '.local.yaml', '.local.yml', '.local.xml',
                              '.local.sh', '.local.bash', '.local.zsh')
            if ((len(parts) > 2 and parts[1:3] == ('config', 'local')) or
                    parts[-1].endswith(local_suffixes) or parts[-1].startswith('.env')):
                raise ValueError(f'Machine-specific configuration must not ship: {item.filename}')
            if stat.S_ISLNK(item.external_attr >> 16):
                target = archive.read(item).decode()
                if target.startswith('/') or '..' in PurePosixPath(target).parts:
                    raise ValueError(f'Nonportable symlink: {item.filename} -> {target}')
        required = [
            'README.md', 'VERIFICATION.md', '.dockerignore', '.gitignore', '.gitattributes', 'colcon.meta',
            'config/README.md', 'config/COLCON_IGNORE',
            'scripts/README.md', 'scripts/rviz.sh', 'scripts/setenv.bash', 'scripts/setenv.zsh',
            'scripts/init_local_config.sh', 'scripts/lib/init_local_config.py',
            'scripts/lib/common.sh', 'scripts/lib/dds_env.sh', 'scripts/lib/in_container.sh',
            'scripts/lib/check_network.py',
            'src/FAST_LIO/rviz/mapping.rviz', 'src/FAST_LIO/rviz/localization.rviz',
            'doc/JETSON_DEPLOY.md', 'doc/GT_SYSTEM_README.md',
            'dds_config/cyclonedds.xml', 'dds_config/cyclonedds_pc.xml',
            'docker/Dockerfile', 'docker/Dockerfile_jetson', 'docker/build.sh', 'docker/run.sh',
            'pcd_map/test.pcd', 'pcd_map/COLCON_IGNORE',
            'src/FAST_LIO/package.xml', 'src/FAST_LIO/include/map_storage.hpp',
            'src/FAST_LIO/include/bounded_relocalization.hpp',
            'src/FAST_LIO/include/startup_relocalization.hpp',
            'src/FAST_LIO/include/workspace_runtime.hpp', 'src/FAST_LIO/test/runtime_test.cpp',
            'src/FAST_LIO/scripts/trajectory_csv.py', 'tests/test_runtime_config.py',
            'doc/RUNTIME_SAFETY.md',
            'src/FAST_LIO/test/relocalization_test.cpp', 'doc/STARTUP_RELOCALIZATION.md',
            'src/FAST_LIO/include/ikd-Tree/ikd_Tree.cpp',
            'src/FAST_LIO/include/IKFoM_toolkit/esekfom/esekfom.hpp',
            'src/livox_ros_driver2/package.xml', 'src/livox_ros_driver2/cmake/livox_sdk.cmake',
        ]
        for name in required:
            if f'{root}/{name}' not in names:
                raise ValueError(f'Missing required file: {name}')
        for name in ('scripts/build.sh', 'scripts/test.sh', 'scripts/run.sh', 'scripts/rviz.sh',
                     'scripts/package.sh', 'scripts/init_local_config.sh', 'src/livox_ros_driver2/build.sh'):
            info = archive.getinfo(f'{root}/{name}')
            if not info.external_attr >> 16 & stat.S_IXUSR:
                raise ValueError(f'Executable permission lost: {name}')
        for arch, machine in (('x86', 62), ('arm', 183)):
            base = f'{root}/livox-sdk-{arch}'
            for name in ('include/livox_lidar_api.h', 'lib/liblivox_lidar_sdk_static.a'):
                if f'{base}/{name}' not in names:
                    raise ValueError(f'Missing SDK file: {base}/{name}')
            header = archive.read(f'{base}/lib/liblivox_lidar_sdk_shared.so')[:20]
            if header[:6] != b'\x7fELF\x02\x01' or int.from_bytes(header[18:20], 'little') != machine:
                raise ValueError(f'Incorrect bundled SDK ELF: {arch}')
        bad = archive.testzip()
        if bad:
            raise ValueError(f'CRC failure: {bad}')
        print(f'[PASS] ZIP: {len(infos)} entries; complete sources, SDKs, portable paths and script permissions')


if __name__ == '__main__':
    check_archive(Path(sys.argv[1]))
