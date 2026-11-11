#!/usr/bin/env python3
"""Copy the active presets with a local bridge endpoint; never overwrite sources."""
import argparse
import json
from pathlib import Path

import yaml


def rewrite_endpoint(value, host, port):
    if isinstance(value, dict):
        if 'RosBridgePort' in value:
            value.update(RosBridgeIP=host, RosBridgePort=port,
                         RosBridgeServerUrl='ws://{}:{}'.format(host, port))
        for child in value.values():
            rewrite_endpoint(child, host, port)
    elif isinstance(value, list):
        for child in value:
            rewrite_endpoint(child, host, port)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=9090, help='Team default: 9090')
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error('port must be 1..65535')
    root = Path(__file__).resolve().parents[1]
    contract = yaml.safe_load((root / 'config/ros/topics.yaml').read_text())
    args.output_dir.mkdir(parents=True, exist_ok=True)
    for key in ('sensors', 'network'):
        source = root / contract['presets'][key]
        destination = args.output_dir / (source.stem + '_local' + str(args.port) + '.json')
        data = json.loads(source.read_text())
        rewrite_endpoint(data, args.host, args.port)
        destination.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n')
        print(destination)


if __name__ == '__main__':
    main()
