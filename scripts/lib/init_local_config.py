#!/usr/bin/env python3
"""Create a NEW, ignored MID360 local JSON; never change NICs or a template."""
import argparse
import copy
import ipaddress
import json
from pathlib import Path
import re
import shlex

ROOT = Path(__file__).resolve().parents[2]
HOST_KEYS = ('cmd_data_ip', 'push_msg_ip', 'point_data_ip', 'imu_data_ip', 'log_data_ip')


def ipv4(value):
    try:
        address = ipaddress.IPv4Address(value)
    except (ValueError, TypeError) as error:
        raise ValueError(f'Expected IPv4 address, got {value!r}') from error
    if address.is_unspecified or address.is_multicast or str(address) == '255.255.255.255':
        raise ValueError(f'Expected a usable unicast IPv4 address, got {value!r}')
    return str(address)


def prepare_configuration(template, host_ip=None, lidar_ip=None):
    config = copy.deepcopy(template)
    try:
        network = config['MID360']['host_net_info']
        lidars = config['lidar_configs']
        if not isinstance(lidars, list) or not lidars or any(not isinstance(item, dict) for item in lidars):
            raise ValueError('lidar_configs must be a nonempty list of LiDAR entries')
        old_lidars = [item['ip'] for item in lidars]
        if lidar_ip is not None:
            if len(lidars) != 1:
                raise ValueError('--lidar-ip supports one LiDAR only; edit multi-LiDAR local copies manually')
            lidar_ip = ipv4(lidar_ip)
            lidars[0]['ip'] = lidar_ip
        if host_ip is not None:
            host_ip = ipv4(host_ip)
        if isinstance(network, list):
            if not network:
                raise ValueError('MID360.host_net_info must not be empty')
            for entry in network:
                if not isinstance(entry, dict) or not isinstance(entry.get('lidar_ip'), list):
                    raise ValueError('Each host_net_info entry needs host_ip and a lidar_ip list')
                if host_ip is not None:
                    entry['host_ip'] = host_ip
                if lidar_ip is not None:
                    entry['lidar_ip'] = [lidar_ip if value == old_lidars[0] else value for value in entry['lidar_ip']]
                entry['host_ip'] = ipv4(entry['host_ip'])
                if not entry['lidar_ip']:
                    raise ValueError('host_net_info lidar_ip must not be empty')
                entry['lidar_ip'] = [ipv4(value) for value in entry['lidar_ip']]
                if set(entry['lidar_ip']) - {item['ip'] for item in lidars}:
                    raise ValueError('host_net_info lidar_ip must match lidar_configs IPs')
            hosts = {entry['host_ip'] for entry in network}
        elif isinstance(network, dict):
            keys = [key for key in HOST_KEYS if key in network]
            if not all(key in network for key in HOST_KEYS[:4]):
                raise ValueError('Object host_net_info needs command/push/point/IMU IP fields')
            for key in keys:
                network[key] = ipv4(host_ip if host_ip is not None else network[key])
            hosts = {network[key] for key in keys}
        else:
            raise ValueError('MID360.host_net_info must be a list or object')
        for entry in lidars:
            entry['ip'] = ipv4(entry['ip'])
        if hosts & {entry['ip'] for entry in lidars}:
            raise ValueError('Host receiver IP must not equal a LiDAR IP')
    except (KeyError, TypeError) as error:
        raise ValueError('Expected a MID360 SDK JSON with host_net_info and lidar_configs IPs') from error
    return config


def create_local_config(template_path, destination, host_ip=None, lidar_ip=None):
    destination = Path(destination).expanduser().absolute()
    if not destination.name.endswith('.local.json'):
        raise ValueError('Local output must end in .local.json so it stays out of Git and deployment ZIPs')
    config = prepare_configuration(json.loads(Path(template_path).expanduser().read_text(encoding='utf-8')),
                                   host_ip, lidar_ip)
    content = json.dumps(config, indent=2, ensure_ascii=False) + '\n'
    destination.parent.mkdir(parents=True, exist_ok=True)
    # Exclusive creation: never truncate or overwrite an existing user config,
    # including a concurrent initializer's file or a dangling symlink.
    with destination.open('x', encoding='utf-8') as stream:
        stream.write(content)
    return destination


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host-ip', help='Real local IPv4 receiver address; default copies the template')
    parser.add_argument('--lidar-ip', help='Real MID360 IPv4 address; default copies the template')
    parser.add_argument('--name', default='', help='Optional machine label, e.g. jetson or pc')
    parser.add_argument('--template', type=Path, default=ROOT / 'src/livox_ros_driver2/config/MID360.json')
    parser.add_argument('--output', type=Path, help='Optional NEW output ending in .local.json')
    args = parser.parse_args()
    if args.name and not re.fullmatch(r'[\w-]+', args.name):
        parser.error('--name must use letters/digits/underscores/hyphens, never path separators')
    if args.name and args.output:
        parser.error('Use either --name or --output, not both')
    name = f'MID360.{args.name}.local.json' if args.name else 'MID360.local.json'
    destination = args.output or ROOT / 'config/local' / name
    try:
        path = create_local_config(args.template, destination, args.host_ip, args.lidar_ip)
    except FileExistsError:
        parser.error(f'Local config already exists; refusing to overwrite: {destination}')
    except (OSError, ValueError) as error:
        parser.error(str(error))
    try:
        runtime_path = str(path.relative_to(ROOT))
    except ValueError:
        runtime_path = str(path)
    print(f'Created local config: {path}')
    print('Factory template was not modified. Review the actual NIC/IPs; no networking changes were made.')
    print(f'Check: bash scripts/check_network.sh {shlex.quote(runtime_path)}')
    print(f'Mapping: bash scripts/run.sh mapping {shlex.quote("lidar_config:=" + runtime_path)}')
    print(f'Localization: bash scripts/run.sh localization {shlex.quote("lidar_config:=" + runtime_path)} map_path:=pcd_map/YOUR_SCENE.pcd')


if __name__ == '__main__':
    main()
