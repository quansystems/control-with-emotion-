"""Draw named regions interactively, or run an explicit automatic/seeds mode."""

import argparse
from pathlib import Path

import yaml

from robot_node.partition_io import write_partition
from robot_node.semantic_map import OccupancyMap, partition


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', required=True, help='ROS map YAML (trinary)')
    parser.add_argument('--output', required=True, help='Output directory')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--gui', action='store_true', help='Open the region editor (default)')
    mode.add_argument('--auto', action='store_true', help='Generate regions automatically')
    mode.add_argument('--seeds', help='Generate regions from a seeds YAML')
    parser.add_argument('--clearance', type=float, default=0.3)
    parser.add_argument('--opening-radius', type=float, default=0.4)
    parser.add_argument('--min-area', type=float, default=0.5)
    options = parser.parse_args(args)
    if not options.auto and not options.seeds:
        from robot_node.map_region_editor import main as editor_main
        return editor_main(['--map', options.map, '--output', options.output,
                            '--clearance', str(options.clearance)])
    grid = OccupancyMap(options.map)
    seeds = yaml.safe_load(Path(options.seeds).read_text())['regions'] if options.seeds else None
    labels, seeds = partition(grid, seeds, options.opening_radius, options.min_area)
    regions = write_partition(grid, labels, seeds, options.clearance, options.output,
                              source='seeds' if options.seeds else 'automatic')
    filename = Path(options.output).resolve() / 'regions.yaml'
    unassigned = int((grid.free & (labels == 0)).sum())
    print(f'{len(regions)} regions -> {filename}; unassigned free pixels: {unassigned}')
    for region in regions:
        print(f'{region["id"]}: {region["name"]} -> {region["goal"]}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
