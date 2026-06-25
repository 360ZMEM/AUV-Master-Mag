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

## Tiered-feed Follow-up

Commands:

```bash
/Users/bytedance/miniconda3/bin/python tools/evaluate_dropout_variants.py --phase probe --name p33_probe10_tiered_anchor_low --name p34_probe10_anchor_only --name p35_probe10_tiered_anchor_mid --max-steps 24000
/Users/bytedance/miniconda3/bin/python tools/evaluate_dropout_variants.py --phase probe --name p36_probe10_extrapolated_low --name p37_probe10_extrapolated_mid --max-steps 24000
/Users/bytedance/miniconda3/bin/python tools/evaluate_dropout_variants.py --phase probe --name p36_probe10_extrapolated_low
/Users/bytedance/miniconda3/bin/python tools/evaluate_dropout_variants.py --phase probe --name p39_probe10_extrapolated_low_pursuit
```

| variant | run | health | TRACK XT | TRACK vehicle err | route | final dist | lookahead pos err | feed allowed | reject heading | conclusion |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `p33_probe10_tiered_anchor_low` | 24000 | 7.2 | 23.0m | 70.5deg | 0.0% | 40.5m | 17.3m | 84.8% | 0.0% | High-confidence phase anchors destabilize progress. |
| `p34_probe10_anchor_only` | 24000 | 16.7 | 7.7m | 41.2deg | 0.0% | 49.0m | 11.0m | 67.1% | 14.9% | Anchors alone are too sparse for route progress. |
| `p35_probe10_tiered_anchor_mid` | 24000 | 4.1 | 21.8m | 63.2deg | 0.0% | 46.0m | 18.1m | 81.4% | 5.0% | Anchor + mid extrapolation still fails. |
| `p36_probe10_extrapolated_low` | 24000 | 23.8 | 26.9m | 140.7deg | 40.0% | 0.2m | 13.1m | 25.0% | 56.9% | Low-weight extrapolation improves route without anchors. |
| `p37_probe10_extrapolated_mid` | 24000 | 14.5 | 27.0m | 138.8deg | 37.6% | 16.2m | 14.1m | 25.0% | 60.1% | Mid weight behaves like p25. |
| `p36_probe10_extrapolated_low` | full | 15.3 | 30.1m | 115.1deg | 58.9% | 53.8m | 7.4m | 7.5% | 69.4% | Full duration confirms route gain, but heading remains poor. |
| `p39_probe10_extrapolated_low_pursuit` | full | 15.3 | 30.1m | 115.1deg | 58.9% | 53.8m | 7.4m | 7.5% | 69.4% | Pure-pursuit still has no measurable effect. |

Interpretation:

- High-confidence phase anchors are not a free improvement. They can dominate the
  local estimator with sparse points and collapse route progress.
- The most useful direction is low-weight extrapolated lookahead feed without
  phase anchors.  It improves route completion, but does not yet solve high
  vehicle heading error.
- `lookahead_pursuit` does not change the outcome, so the next issue is still
  estimator-side heading consistency rather than controller pursuit strength.

## Next Plan

1. Keep low-weight extrapolated feed as the current best candidate.
2. Add synchronized analysis plots:
   route progress vs feed allowed, heading error vs reject heading, and lookahead
   position error vs local residual.
3. If the strategy stabilizes, use the new channels as paper-facing figures for
   observability, gate rejection causes, estimator readiness, and control benefit.

## Heading-smoothing Follow-up

Implementation:

- Added default-off feed-heading smoothing:
  - `magnetic_lookahead_feed_heading_smoothing_enabled`
  - `magnetic_lookahead_feed_heading_max_step_deg`
- The smoother is applied only to the heading written into `local_path`.
  Raw `MagneticLookaheadTarget` diagnostics are unchanged.
- Representative variants:
  - `p40_probe10_extrapolated_low_smooth12`
  - `p41_probe10_extrapolated_low_smooth6`
  - `p42_probe10_extrapolated_low_smooth9`

Commands:

```bash
/Users/bytedance/miniconda3/bin/python tools/evaluate_dropout_variants.py --phase probe --name p40_probe10_extrapolated_low_smooth12 --name p41_probe10_extrapolated_low_smooth6 --max-steps 24000
/Users/bytedance/miniconda3/bin/python tools/evaluate_dropout_variants.py --phase probe --name p42_probe10_extrapolated_low_smooth9 --max-steps 24000
/Users/bytedance/miniconda3/bin/python tools/evaluate_dropout_variants.py --phase probe --name p40_probe10_extrapolated_low_smooth12
```

| variant | run | health | TRACK XT | TRACK vehicle err | route | final dist | lookahead pos err | feed allowed | reject heading | conclusion |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `p40_probe10_extrapolated_low_smooth12` | 24000 | 23.8 | 26.9m | 140.7deg | 40.0% | 0.2m | 13.1m | 25.0% | 56.9% | Same as p36; 12deg/step is too loose to affect behavior. |
| `p41_probe10_extrapolated_low_smooth6` | 24000 | 17.2 | 18.5m | 49.7deg | 3.5% | 52.3m | 26.7m | 45.0% | 36.0% | Strong smoothing lowers heading error but removes useful turn information. |
| `p42_probe10_extrapolated_low_smooth9` | 24000 | 26.3 | 10.0m | 81.2deg | 0.0% | 5.6m | 15.0m | 49.3% | 39.3% | Intermediate smoothing still collapses route progress. |
| `p40_probe10_extrapolated_low_smooth12` | full | 15.3 | 30.1m | 115.1deg | 58.9% | 53.8m | 7.4m | 7.5% | 69.4% | Full run matches p36: safe but not corrective. |

Interpretation:

- Simple per-feed heading step limiting is not the main missing mechanism.
- The lookahead heading contains both harmful alias jumps and useful turn updates.
  Over-smoothing blocks the latter and collapses route progress.
- Next algorithmic direction should move from scalar heading smoothing to
  stateful axis selection / multi-hypothesis disambiguation, preferably keyed by
  phase events and route-progress consistency.
