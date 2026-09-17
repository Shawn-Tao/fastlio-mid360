#!/usr/bin/env python3
"""Read-only check: configured Livox host addresses must exist in this namespace."""
import ipaddress
import json
import subprocess
import sys
from pathlib import Path


def main():
    path = Path(sys.argv[1]).expanduser()
    config = json.loads(path.read_text())
    network = config['MID360']['host_net_info']
    if isinstance(network, list):
        hosts = {entry['host_ip'] for entry in network}
    else:
        hosts = {network[key] for key in ('cmd_data_ip', 'push_msg_ip', 'point_data_ip', 'imu_data_ip')}
    interfaces = json.loads(subprocess.check_output(['ip', '-j', '-4', 'addr'], text=True))
    local = {address['local'] for interface in interfaces for address in interface['addr_info']}
    lidars = [entry['ip'] for entry in config['lidar_configs']]
    for address in hosts | set(lidars):
        ipaddress.IPv4Address(address)
    print(f'Config: {path}\nConfigured host IPs: {sorted(hosts)}\nLiDAR IPs: {lidars}')
    missing = hosts - local
    if missing:
        print(f'[FAIL] Configured host IPs are absent in this network namespace: {sorted(missing)}', file=sys.stderr)
        print('Configure the physical NIC/container networking and edit a local JSON copy; this check changes nothing.', file=sys.stderr)
        return 1
    for address in lidars:
        print(subprocess.check_output(['ip', 'route', 'get', address], text=True).strip())
    print('[PASS] Host addresses exist. This does not verify LiDAR power, UDP reception or firewall rules.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
