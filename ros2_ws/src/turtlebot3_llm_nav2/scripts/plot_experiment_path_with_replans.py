#!/usr/bin/env python3

import argparse
import csv
import json
import os

import matplotlib.image as mpimg
import matplotlib.pyplot as plt


def load_map_yaml(path):
    data = {}
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#') or ':' not in line:
                continue
            key, value = line.split(':', 1)
            data[key.strip()] = value.strip().strip('"').strip("'")
    image_path = data['image']
    if not os.path.isabs(image_path):
        image_path = os.path.join(os.path.dirname(path), image_path)
    origin = [float(v) for v in data.get('origin', '0, 0, 0').strip('[]').split(',')]
    return {
        'image': image_path,
        'resolution': float(data['resolution']),
        'origin': origin,
    }


def load_xy_csv(path):
    points = []
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            points.append((float(row['x']), float(row['y'])))
    return points


def load_waypoints(path):
    if not path:
        return []
    with open(path) as f:
        data = json.load(f)
    return [(float(x), float(y)) for x, y in data.get('waypoints', [])]


def load_replan_events(path):
    if not path:
        return []
    events = []
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            events.append({
                'x': float(row['x']),
                'y': float(row['y']),
                'reason': row.get('reason', 'replan'),
            })
    return events


def downsample(points, step):
    if step <= 1:
        return points
    return points[::step]


def plot_path(args):
    map_info = load_map_yaml(args.map_yaml)
    image = mpimg.imread(map_info['image'])
    height, width = image.shape[:2]
    resolution = map_info['resolution']
    origin_x, origin_y, _ = map_info['origin']
    extent = [
        origin_x,
        origin_x + width * resolution,
        origin_y,
        origin_y + height * resolution,
    ]

    traj = downsample(load_xy_csv(args.trajectory), args.downsample)
    waypoints = load_waypoints(args.waypoints)
    replans = load_replan_events(args.replan_events)

    fig, ax = plt.subplots(figsize=(6, 5), dpi=220)
    ax.imshow(image, cmap='gray', origin='lower', extent=extent)

    if traj:
        xs, ys = zip(*traj)
        ax.plot(xs, ys, color='#f28e2b', linewidth=2.2, label=args.path_label)
        ax.scatter(xs[0], ys[0], color='#1f77b4', s=34, marker='o', label='Path start', zorder=5)
        ax.scatter(xs[-1], ys[-1], color='#d62728', s=44, marker='*', label='Path end', zorder=5)

    if waypoints:
        wx, wy = zip(*waypoints)
        ax.plot(wx, wy, color='#4c78a8', linestyle='--', linewidth=1.2, label='Current waypoints')
        ax.scatter(wx, wy, color='#4c78a8', s=18, zorder=4)

    if replans:
        rx = [event['x'] for event in replans]
        ry = [event['y'] for event in replans]
        ax.scatter(
            rx,
            ry,
            color='#d62728',
            edgecolors='white',
            linewidths=0.8,
            s=95,
            marker='*',
            label='LLM replanning',
            zorder=7,
        )
        for index, event in enumerate(replans, start=1):
            ax.annotate(
                f'R{index}',
                (event['x'], event['y']),
                textcoords='offset points',
                xytext=(5, 5),
                fontsize=7,
                color='#8b0000',
                weight='bold',
            )

    if args.start:
        ax.scatter(args.start[0], args.start[1], color='blue', s=42, marker='s', label='Start target', zorder=6)
    if args.goal:
        ax.scatter(args.goal[0], args.goal[1], color='red', s=54, marker='x', label='Goal', zorder=6)

    ax.set_title(args.title)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_aspect('equal', adjustable='box')
    ax.grid(True, linewidth=0.3, alpha=0.35)
    ax.legend(loc='upper right', fontsize=7, framealpha=0.9)
    fig.tight_layout()
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    fig.savefig(args.out)
    print(f'Saved {args.out}')


def parse_args():
    parser = argparse.ArgumentParser(description='Plot robot path and LLM replanning events on a Nav2 map.')
    parser.add_argument('--map-yaml', required=True)
    parser.add_argument('--trajectory', required=True)
    parser.add_argument('--out', required=True)
    parser.add_argument('--waypoints', default=None)
    parser.add_argument('--replan-events', default=None)
    parser.add_argument('--title', default='Navigation result')
    parser.add_argument('--path-label', default='Robot path')
    parser.add_argument('--start', nargs=2, type=float)
    parser.add_argument('--goal', nargs=2, type=float)
    parser.add_argument('--downsample', type=int, default=1)
    return parser.parse_args()


if __name__ == '__main__':
    plot_path(parse_args())
