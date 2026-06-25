# Lookahead Feed Diagnostics

## Scope

This note records the current diagnostics added for `magnetic_lookahead -> local_path`
feed gating in `case_maze_sonar_dropout`.  It is a companion note to
`docs/17_zigzag纯磁探针方案.md`.

## Per-frame Instrumentation

New recorded channels:

- `magnetic_lookahead_feed_allowed`
- `magnetic_lookahead_feed_reason_code`
- `magnetic_lookahead_feed_phase_age_s`
- `magnetic_lookahead_feed_innovation_m`
- `magnetic_lookahead_feed_axis_delta_deg`
- `magnetic_lookahead_feed_local_residual_m`

Reason code mapping:

| code | meaning |
| --- | --- |
| `1` | allowed |
| `2` | no lookahead target |
| `3` | feed disabled |
| `4` | confidence too low |
| `5` | lookahead age too large |
| `6` | phase age too large |
| `7` | local residual too large |
| `8` | heading delta too large |
| `9` | innovation too large |

## Metrics and Report Fields

`compute_health_metrics()` and reports now include:

- `lookahead_feed_pct`
- `feed_reject_age`
- `feed_reject_phase_age`
- `feed_reject_residual`
- `feed_reject_heading`
- `feed_reject_innovation`
- `feed_phase_age`
- `feed_innovation`
- `feed_axis_delta`
- `feed_local_residual`

`render_detail()` now includes:

- Lookahead feed gate reason over time.
- Lookahead feed gate margins over time: axis delta, innovation, phase age.

## Key Long-run Results

Commands:

```bash
/Users/bytedance/miniconda3/bin/python tools/evaluate_dropout_variants.py --phase probe --name p25_probe10_feedlocal_gate60 --name p28_probe10_gate45_conservative --max-steps 24000
/Users/bytedance/miniconda3/bin/python tools/evaluate_dropout_variants.py --phase probe --name p31_probe10_gate60_heading30 --name p32_probe10_gate60_heading40 --max-steps 24000
```

| variant | max steps | health | TRACK XT | TRACK vehicle err | route | final dist | feed allowed | reject heading | conclusion |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `p25_probe10_feedlocal_gate60` | 24000 | 14.5 | 27.0m | 138.8deg | 37.6% | 16.2m | 25.0% | 60.1% | Maintains route, but heading is poor. |
| `p28_probe10_gate45_conservative` | 24000 | 43.9 | 8.2m | 63.9deg | 9.0% | 0.9m | 34.3% | 47.1% | Cleaner estimate, but too little useful supply. |
| `p31_probe10_gate60_heading30` | 24000 | 17.0 | 7.7m | 41.2deg | 0.0% | 58.7m | 32.7% | 52.1% | 30deg heading gate is not a useful compromise. |
| `p32_probe10_gate60_heading40` | 24000 | 14.5 | 27.0m | 138.8deg | 37.6% | 16.2m | 25.0% | 60.1% | Same behavior as p25; not a simple threshold issue. |

## Current Interpretation

The dominant rejection reason is `heading delta too large`.  Innovation and local
residual are not the main blockers in these representative runs.  Simple heading
threshold changes do not provide a stable tradeoff: `30deg` blocks useful progress,
while `35-40deg` preserves progress but allows heading instability.

## Next Plan

1. Split lookahead feed into high-confidence phase anchors and low-weight extrapolated
   points, instead of feeding both with the same weight.
2. Add synchronized analysis plots:
   route progress vs feed allowed, heading error vs reject heading, and lookahead
   position error vs local residual.
3. If the strategy stabilizes, use the new channels as paper-facing figures for
   observability, gate rejection causes, estimator readiness, and control benefit.
