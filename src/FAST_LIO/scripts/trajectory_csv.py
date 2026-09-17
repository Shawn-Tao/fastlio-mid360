"""Dependency-free reader for legacy and durable streaming trajectory CSVs."""
import math
import json
import warnings


def load_csv(path):
    meta, header, rows = {}, None, []
    with open(path, encoding='utf-8') as stream:
        for line in stream:
            line = line.strip()
            if not line:
                continue
            if line.startswith('#'):
                if ':' in line:
                    key, value = line[1:].split(':', 1)
                    meta[key.strip()] = value.strip()
                continue
            if header is None:
                header = [field.strip() for field in line.split(',')]
                continue
            try:
                row = [float(value) for value in line.split(',')]
                if len(row) != len(header) or not all(math.isfinite(value) for value in row):
                    raise ValueError('Malformed/nonfinite CSV row')
                rows.append(row)
            except ValueError as error:
                remaining = [tail.strip() for tail in stream if tail.strip()]
                if any(not tail.startswith('#') for tail in remaining):
                    raise ValueError(f'{path}: invalid trajectory row before end of file') from error
                warnings.warn(f'{path}: ignoring one damaged final CSV row', RuntimeWarning)
                meta['truncated_final_row'] = 'true'
                break
    required = ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw', 'stamp_sec', 'stamp_nanosec')
    if header is None or any(field not in header for field in required):
        raise ValueError(f'{path}: missing trajectory CSV header/columns')
    if not rows:
        raise ValueError(f'{path}: no complete pose rows found')
    if meta.get('fastlio_gt_recording_version') == '3' and 'interrupted' not in meta:
        meta['interrupted'] = 'true'
    if meta.get('fastlio_gt_recording_version') == '3':
        for key in ('reference_map', 'control_source', 'instruction_id', 'note'):
            if key in meta:
                value = json.loads(meta[key])
                if not isinstance(value, str):
                    raise ValueError(f'{path}: invalid string metadata: {key}')
                meta[key] = value
    return meta, header, rows
