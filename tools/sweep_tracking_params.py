"""Batch parameter sweep for the AUV magnetic cable tracking demo."""

import argparse
import copy
import itertools
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple

from tqdm.auto import tqdm


WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = WORKSPACE_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from auv_mag_tracking.config import ScenarioConfig, build_default_scenarios
from auv_mag_tracking.main_viz import AuvCableTrackingSimulation


MODE_SCORE = {
    "HOLD": 3.0,
    "APPROACH": 2.0,
    "TURN": 1.5,
    "SEARCH": 1.0,
    "LOST": 0.0,
}


@dataclass
class SweepResult:
    case_name: str
    score: float
    peak_count: int
    final_confidence: float
    final_mode: str
    tracked_distance_m: float
    parameters: Dict[str, float]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Sweep tracking parameters for selected scenarios")
    parser.add_argument("--cases", nargs="+", default=["case3", "case4"], help="Scenario names to evaluate")
    parser.add_argument("--top-k", type=int, default=8, help="Number of best results to print and save per case")
    parser.add_argument(
        "--method",
        choices=["coordinate", "grid"],
        default="coordinate",
        help="Search strategy: coordinate is much faster and is the default.",
    )
    parser.add_argument(
        "--output",
        default=str(WORKSPACE_ROOT / "扫参结果.md"),
        help="Markdown report output path",
    )
    return parser.parse_args()


def build_parameter_grid(base_scenario: ScenarioConfig) -> Dict[str, Sequence[float]]:
    tracking = base_scenario.tracking
    return {
        "turn_trigger_ratio": [
            round(max(0.70, tracking.turn_trigger_ratio - 0.02), 3),
            round(tracking.turn_trigger_ratio, 3),
            round(min(0.95, tracking.turn_trigger_ratio + 0.02), 3),
        ],
        "peak_cooldown_s": [
            round(max(0.50, tracking.peak_cooldown_s - 0.20), 3),
            round(tracking.peak_cooldown_s, 3),
            round(tracking.peak_cooldown_s + 0.20, 3),
        ],
        "min_peak_strength_nt": [
            round(max(40.0, tracking.min_peak_strength_nt - 20.0), 3),
            round(tracking.min_peak_strength_nt, 3),
            round(tracking.min_peak_strength_nt + 20.0, 3),
        ],
        "envelope_time_constant_s": [
            round(max(0.10, tracking.envelope_time_constant_s * 0.80), 3),
            round(tracking.envelope_time_constant_s, 3),
            round(tracking.envelope_time_constant_s * 1.20, 3),
        ],
    }


def iterate_parameter_sets(parameter_grid: Dict[str, Sequence[float]]) -> Iterable[Dict[str, float]]:
    parameter_names = list(parameter_grid)
    parameter_values = [parameter_grid[name] for name in parameter_names]
    for combination in itertools.product(*parameter_values):
        yield dict(zip(parameter_names, combination))


def clone_with_parameters(base_scenario: ScenarioConfig, parameters: Dict[str, float]) -> ScenarioConfig:
    scenario = copy.deepcopy(base_scenario)
    for name, value in parameters.items():
        setattr(scenario.tracking, name, value)
    return scenario


def score_report(report) -> float:
    peak_score = 1.35 * report.peak_count
    confidence_score = 8.0 * report.final_confidence
    distance_score = min(report.tracked_distance_m / 100.0, 1.0)
    mode_score = MODE_SCORE.get(report.final_mode, 0.0)
    return peak_score + confidence_score + distance_score + mode_score


def evaluate_single(case_name: str, base_scenario: ScenarioConfig, parameters: Dict[str, float]) -> SweepResult:
    scenario = clone_with_parameters(base_scenario, parameters)
    report = AuvCableTrackingSimulation(scenario).run(enable_visualization=False)
    return SweepResult(
        case_name=case_name,
        score=score_report(report),
        peak_count=report.peak_count,
        final_confidence=report.final_confidence,
        final_mode=report.final_mode,
        tracked_distance_m=report.tracked_distance_m,
        parameters=parameters.copy(),
    )


def parameter_signature(parameters: Dict[str, float]) -> Tuple[Tuple[str, float], ...]:
    return tuple(sorted((name, float(value)) for name, value in parameters.items()))


def evaluate_case(case_name: str, base_scenario: ScenarioConfig, method: str) -> Tuple[List[SweepResult], Dict[str, Sequence[float]]]:
    parameter_grid = build_parameter_grid(base_scenario)
    results: List[SweepResult] = []
    seen = set()

    if method == "grid":
        total_candidates = 1
        for values in parameter_grid.values():
            total_candidates *= len(values)
    else:
        total_candidates = 1 + sum(len(values) for values in parameter_grid.values()) * 2
    progress = tqdm(total=total_candidates, desc=f"{case_name} sweep", unit="run", dynamic_ncols=True, leave=False)

    if method == "grid":
        for parameters in iterate_parameter_sets(parameter_grid):
            results.append(evaluate_single(case_name, base_scenario, parameters))
            progress.update(1)
    else:
        best_parameters = {
            name: float(getattr(base_scenario.tracking, name))
            for name in parameter_grid
        }
        baseline_result = evaluate_single(case_name, base_scenario, best_parameters)
        results.append(baseline_result)
        progress.update(1)
        seen.add(parameter_signature(best_parameters))
        best_result = baseline_result

        for _ in range(2):
            improved = False
            for parameter_name, candidate_values in parameter_grid.items():
                local_best = best_result
                local_parameters = best_parameters.copy()
                for candidate_value in candidate_values:
                    candidate_parameters = best_parameters.copy()
                    candidate_parameters[parameter_name] = float(candidate_value)
                    signature = parameter_signature(candidate_parameters)
                    if signature in seen:
                        continue
                    seen.add(signature)
                    candidate_result = evaluate_single(case_name, base_scenario, candidate_parameters)
                    results.append(candidate_result)
                    progress.update(1)
                    if candidate_result.score > local_best.score:
                        local_best = candidate_result
                        local_parameters = candidate_parameters
                if local_best.score > best_result.score:
                    best_result = local_best
                    best_parameters = local_parameters
                    improved = True
            if not improved:
                break

    results.sort(key=lambda item: (item.score, item.peak_count, item.final_confidence), reverse=True)
    progress.close()
    return results, parameter_grid


def format_parameter_grid(parameter_grid: Dict[str, Sequence[float]]) -> str:
    lines = []
    for name, values in parameter_grid.items():
        value_text = ", ".join(str(value) for value in values)
        lines.append(f"- {name}: {value_text}")
    return "\n".join(lines)


def format_markdown_report(
    results_by_case: Dict[str, List[SweepResult]],
    grids_by_case: Dict[str, Dict[str, Sequence[float]]],
    top_k: int,
    method: str,
) -> str:
    lines = ["# 批量扫参结果", "", "本报告由 tools/sweep_tracking_params.py 自动生成。", "", f"搜索方法: {method}", ""]
    for case_name, results in results_by_case.items():
        lines.append(f"## {case_name}")
        lines.append("")
        lines.append("### 搜索空间")
        lines.append("")
        lines.append(format_parameter_grid(grids_by_case[case_name]))
        lines.append("")
        lines.append(f"### Top Results (共评估 {len(results)} 组)")
        lines.append("")
        lines.append("| rank | score | peaks | confidence | mode | distance_m | turn_ratio | cooldown_s | min_peak_nt | envelope_s |")
        lines.append("| --- | ---: | ---: | ---: | --- | ---: | ---: | ---: | ---: | ---: |")
        for rank, result in enumerate(results[:top_k], start=1):
            lines.append(
                "| {rank} | {score:.2f} | {peak_count} | {final_confidence:.2f} | {final_mode} | {tracked_distance_m:.1f} | {turn_trigger_ratio:.2f} | {peak_cooldown_s:.2f} | {min_peak_strength_nt:.1f} | {envelope_time_constant_s:.2f} |".format(
                    rank=rank,
                    score=result.score,
                    peak_count=result.peak_count,
                    final_confidence=result.final_confidence,
                    final_mode=result.final_mode,
                    tracked_distance_m=result.tracked_distance_m,
                    **result.parameters,
                )
            )
        lines.append("")
    return "\n".join(lines)


def main() -> int:
    args = parse_args()
    scenarios = build_default_scenarios()
    results_by_case: Dict[str, List[SweepResult]] = {}
    grids_by_case: Dict[str, Dict[str, Sequence[float]]] = {}

    for case_name in args.cases:
        scenario = scenarios.get(case_name)
        if scenario is None:
            print(f"Unknown case: {case_name}")
            return 2
        results, parameter_grid = evaluate_case(case_name, scenario, args.method)
        results_by_case[case_name] = results
        grids_by_case[case_name] = parameter_grid
        best = results[0]
        print(
            "{case_name}: best score={score:.2f}, peaks={peak_count}, confidence={confidence:.2f}, mode={mode}, params={params}".format(
                case_name=case_name,
                score=best.score,
                peak_count=best.peak_count,
                confidence=best.final_confidence,
                mode=best.final_mode,
                params=best.parameters,
            )
        )

    report_text = format_markdown_report(results_by_case, grids_by_case, args.top_k, args.method)
    output_path = Path(args.output)
    output_path.write_text(report_text, encoding="utf-8")
    print(f"Saved report to {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())