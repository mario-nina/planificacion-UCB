"""Run two synthetic trajectories with independent time axes and export evidence."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
from uuid import uuid4

from imt342_irb120_planning import plan
from imt342_irb120_planning.smooth import plan_quintic
from .simulation.follower import Config, simulate, summarize


def compare(start, goal, config=Config()):
    records = {}
    for profile, planner in (("lspb_ideal", plan), ("quintic", plan_quintic)):
        trajectory = planner(start, goal)
        rows = simulate(trajectory, config)
        records[profile] = dict(duration_s=trajectory.duration, analysis=summarize(rows), samples=rows)
    return dict(trial_id=uuid4().hex, provenance="offline_synthetic", clock="Tiempo relativo por ensayo",
                configuration=asdict(config), start_rad=start, goal_rad=goal, profiles=records)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--start", type=float, nargs=6, required=True)
    parser.add_argument("--goal", type=float, nargs=6, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = compare(args.start, args.goal)
    with args.output.open("x", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, ensure_ascii=False, allow_nan=False)


if __name__ == "__main__":
    main()
