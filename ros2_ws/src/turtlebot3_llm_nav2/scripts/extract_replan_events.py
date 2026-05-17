#!/usr/bin/env python3

import argparse
import csv
import os
import re


CALL_RE = re.compile(
    r'Calling LLM for waypoints\. reason=([^,]+), start=\[([^\]]+)\], goal=\[([^\]]+)\]'
)


def parse_xy(text):
    values = [float(part.strip()) for part in text.split(',')[:2]]
    return values[0], values[1]


def extract_events(args):
    rows = []
    with open(args.log, errors='replace') as f:
        for line in f:
            match = CALL_RE.search(line)
            if not match:
                continue
            reason = match.group(1).strip()
            x, y = parse_xy(match.group(2))
            goal_x, goal_y = parse_xy(match.group(3))
            rows.append({
                'scenario': args.scenario,
                'mode': args.mode,
                'trial_id': args.trial_id,
                'reason': reason,
                'x': round(x, 5),
                'y': round(y, 5),
                'goal_x': round(goal_x, 5),
                'goal_y': round(goal_y, 5),
            })

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out, 'w', newline='') as f:
        writer = csv.DictWriter(
            f,
            fieldnames=['scenario', 'mode', 'trial_id', 'reason', 'x', 'y', 'goal_x', 'goal_y'],
        )
        writer.writeheader()
        writer.writerows(rows)
    print(f'Saved {len(rows)} replan events to {args.out}')


def parse_args():
    parser = argparse.ArgumentParser(description='Extract LLM replanning event positions from experiment logs.')
    parser.add_argument('--log', required=True)
    parser.add_argument('--out', required=True)
    parser.add_argument('--scenario', default='unknown')
    parser.add_argument('--mode', default='unknown')
    parser.add_argument('--trial-id', default='0')
    return parser.parse_args()


if __name__ == '__main__':
    extract_events(parse_args())
