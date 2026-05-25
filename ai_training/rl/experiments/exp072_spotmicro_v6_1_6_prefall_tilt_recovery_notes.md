# exp072 spotmicro_v6_1_6_prefall_tilt_recovery_v613_base

## Purpose

- Base policy: stable real-robot v6.1.3 checkpoint.
- Scope: normal locomotion plus pre-fall tilt recovery in one locomotion policy.
- Excluded: full get-up after body contact or complete fall.
- Real target: keep walking at about `vx=0.05~0.10 m/s`, tolerate large roll/pitch before full fall, and recover toward normal gait.
- Priority: do not sacrifice the stable v6.1.3 gait.

## Minimal Training Plan

Time is limited, so use only two practical stages:

1. Train this config as-is:
   - resume from `May20_14-29-52_spotmicro_v6_1_3_stronger_transition_push`, checkpoint `4100`
   - `recovery_roll_pitch_range_deg = 18.0`
   - recovery gating begins at `14 deg`, reaches full effect near `25 deg`
2. If normal gait diagnostics still pass, raise only `recovery_roll_pitch_range_deg` to `28.0~30.0` and run a shorter follow-up fine-tune.

Do not add full get-up randomization or body-contact recovery in these stages.

## Code Changes

- Added tilt-based recovery gating using `norm(projected_gravity[:, :2])`.
- Kept observation size unchanged at 47.
- Added `effective_commands`:
  - normal tilt: unchanged commands
  - large tilt: commands ramp down to `25%`
  - used consistently for phase, IK reference, observation, and velocity tracking rewards
- Added recovery phase slowdown:
  - normal tilt: normal phase progression
  - large tilt: phase progression ramps down to `20%`
  - freeze option exists but defaults to `False`
- Added recovery-only residual action authority:
  - normal action scale: `0.25`
  - large tilt action scale: ramps toward `0.35`
- Changed gait/IK relief from hard early relief to conservative ramp:
  - starts at `14 deg`
  - full near `25 deg`
  - IK/gait reward influence only drops to `65%`, not `25~35%`
- Restored v6.1.3-like reward/gait priorities:
  - `tracking_ik = 0.6`
  - `trot_contact = 0.35`
  - `swing_contact = -0.45`
  - `orientation = -10.0`
  - `feet_clearance = 0.03`
  - `no_stuck_feet = -0.2`
- Replaced v6.1.5 aggressive randomization with mild DR:
  - friction `[0.35, 1.10]`
  - base mass `[-0.05, 0.15] kg`
  - base COM x/y/z `[-0.020, 0.010] / [-0.008, 0.008] / [-0.008, 0.015] m`
  - motor strength `[0.85, 1.10]`
  - PD gain and joint-observation randomization still disabled
  - push/action-delay settings returned close to v6.1.3 baseline

## Pass Criteria

- Normal slow walking still stable at `vx=0.05~0.10 m/s`.
- Normal gait metrics do not regress materially from v6.1.3.
- Transition recovery success remains above about `85%`.
- Horizon-end roll/pitch returns near the stable threshold.
- Torque saturation, action rate, dof acceleration, and real actuator tracking error do not spike.

## Diagnostic / Report Update

- `play_diagnostic.py` now reports tilt recovery by bands:
  - `12-18 deg`
  - `18-25 deg`
  - `25-30 deg`
  - `30+ deg`
  - `18+ deg overall`
- `experiment_report.py` includes those band tables in both Recovery and Transition Recovery sections.
- The headline diagnostic now warns when there are no `18+ deg` transition/pre-fall trials, because then the 30 deg objective is not actually tested.
- To stress-test the current checkpoint near the final objective, run diagnostic with:

```bash
python legged_gym/legged_gym/scripts/play_diagnostic.py --task=spotmicro_test --with_dr --prefall-eval
```

Equivalent explicit override:

```bash
python legged_gym/legged_gym/scripts/play_diagnostic.py --task=spotmicro_test --with_dr --recovery-range-deg 30
```

## Next Stage After exp073

exp073 preserved normal gait and learned the `12-18 deg` band well, but `18+ deg` transition/pre-fall recovery was still weak and sparsely sampled. The next stage should keep all other axes fixed and raise only the reset recovery tilt range:

- run: `spotmicro_v6_2_1_prefall_tilt_recovery_25deg`
- resume: `May25_11-03-49_spotmicro_v6_2_prefall_tilt_recovery`, checkpoint `4900`
- `recovery_roll_pitch_range_deg = 25.0`
- `max_iterations = 600`

Do not expand DR, command range, yaw range, action scale, or relief settings in this stage. Those should wait until `18-25 deg` recovery is reliable and normal gait still passes.

## Next Stage After exp074

exp074 improved reset recovery in the `18-25 deg` band but transition/pre-fall recovery is not reliable enough for 30 deg:

- reset `18-25 deg`: `84.1%`
- transition `18-25 deg`: `56.3%`
- transition `25-30 deg`: `40.0%`
- normal gait still acceptable: timeout `97.0%`, early death `0.95%`, torque saturation `4.1%`

Continue the 25 deg stage instead of reverting or moving to 30 deg:

- run: `spotmicro_v6_2_2_prefall_tilt_recovery_25deg_continue`
- resume: `May25_11-31-19_spotmicro_v6_2_1_prefall_tilt_recovery_25deg`, checkpoint `5500`
- keep `recovery_roll_pitch_range_deg = 25.0`
- `max_iterations = 500`

Still do not expand DR, command range, action scale, reward relief, or push strength in this run.

## Next Stage After exp075

exp075 improved the continued 25 deg run enough to start 30 deg exposure:

- reset `18-25 deg`: `86.8%`
- transition `18-25 deg`: `63.4%`
- transition `18+ overall`: `65.0%`
- transition `25-30 deg`: `80.0%`, but only 5 trials
- normal gait remained stable: timeout `98.8%`, early death `0.19%`, torque saturation `4.0%`

Move to 30 deg reset exposure while keeping other settings fixed:

- run: `spotmicro_v6_2_3_prefall_tilt_recovery_30deg`
- resume: `May25_11-58-44_spotmicro_v6_2_2_prefall_tilt_recovery_25deg_continue`, checkpoint `6000`
- `recovery_roll_pitch_range_deg = 30.0`
- `max_iterations = 600`

Next pass criteria:

- normal gait: timeout `95%+`, early death `<2-3%`, torque saturation `<6-8%`
- reset `25-30 deg`: `75%+`
- transition `18+ overall`: `70%+`
- transition `25-30 deg`: enough trials and `60%+`

## Assessment After exp076

exp076 exposed the policy to the full 30 deg reset range, but it is not ready to expand beyond 30 deg yet:

- normal gait remained usable but regressed: timeout `92.3%`, early death `6.1%`, torque saturation `4.0%`
- reset recovery:
  - `18-25 deg`: `91.1%`
  - `25-30 deg`: `67.1%`, horizon-end tilt `9.0 deg`
- transition recovery:
  - `18-25 deg`: `73.0%`
  - `25-30 deg`: `52.5%` over 40 trials
  - `30+ deg`: `25.0%` over 32 stress trials, with high early failure
  - `18+ overall`: `60.8%`

The report contains the right banded data, but the automatic PASS/FAIL criterion is too loose for the current 30 deg pre-fall objective because warnings do not fail the overall result and the `25-30 deg` transition band is not a hard target.

Recommended next step:

- do not train beyond 30 deg yet
- keep `recovery_roll_pitch_range_deg = 30.0`
- either continue from exp076 or make one small recovery-only adjustment, not a broad expansion
- treat `30+ deg` as diagnostic stress only until `25-30 deg` transition recovery is reliable

Applied next config:

- run: `spotmicro_v6_2_4_prefall_tilt_recovery_30deg_continue`
- resume: `May25_13-32-12_spotmicro_v6_2_3_prefall_tilt_recovery_30deg`, checkpoint `6600`
- `max_iterations = 500`
- keep `recovery_roll_pitch_range_deg = 30.0`
- reduce recovery-only internal command scale from `0.25` to `0.15`
- reduce recovery-only gait phase progression from `0.2` to `0.1`
- keep residual recovery action scale at `0.35`

Report auto-judge was tightened for this pre-fall objective:

- normal gait timeout must be near `95%+`
- warning-level pre-fall metrics now fail the headline PASS in pre-fall runs
- reset `25-30 deg` and transition `25-30 deg` bands are checked explicitly
- transition `25-30 deg` requires at least 20 trials and about `65%+` success for PASS

Suggested readiness criteria before real testing or 30+ expansion:

- normal gait timeout `95%+` and early death `<3-5%`
- reset `25-30 deg` recovery `75-80%+`
- transition `25-30 deg` recovery `65-70%+`
- transition `18+ overall` `70-75%+`
- no rise in torque saturation/action rate

## Assessment After exp077

exp077 improved the stability side of the 30 deg stage but did not solve the main `25-30 deg` transition recovery bottleneck:

- normal gait improved versus exp076:
  - timeout `92.3% -> 94.8%`
  - early death `6.1% -> 3.3%`
  - torque saturation `4.0% -> 4.4%`
- reset recovery improved slightly:
  - `25-30 deg`: `67.1% -> 69.0%`
  - `18+ overall`: `79.8% -> 80.3%`
- transition recovery improved in the easier band:
  - `18-25 deg`: `73.0% -> 78.2%`
  - `18+ overall`: `60.8% -> 65.2%`
- but the target band is nearly flat:
  - transition `25-30 deg`: `52.5% -> 54.2%`

Conclusion: the command/phase slowdown was useful and should stay. Do not expand past 30 deg. For the next run, give the policy a small amount of extra high-tilt residual authority and slightly more relief from IK/gait constraints, but keep normal gait settings unchanged.

Applied next config:

- run: `spotmicro_v6_2_5_prefall_tilt_recovery_30deg_authority`
- resume: `May25_14-02-38_spotmicro_v6_2_4_prefall_tilt_recovery_30deg_continue`, checkpoint `7100`
- keep `recovery_roll_pitch_range_deg = 30.0`
- keep recovery command/phase slowdown: `command_scale = 0.15`, `phase_scale = 0.1`
- increase recovery-only residual action scale from `0.35` to `0.40`
- relax high-tilt gait/IK reward influence from `0.65` to `0.60`
- keep `max_iterations = 500`

Watch for torque/action-rate regression. If transition `25-30 deg` does not move above about `60%`, the next issue is likely not just more training; consider changing recovery sampling or reward shape rather than continuing indefinitely.

## Assessment After exp078

exp078 shows that the extra high-tilt authority was not a good next direction:

- reset recovery improved, but that is not the main real-robot target:
  - reset `25-30 deg`: `69.0% -> 74.4%`
  - reset `18+ overall`: `80.3% -> 84.3%`
- normal gait did not materially improve:
  - timeout `94.8% -> 94.1%`
  - early death `3.3% -> 3.7%`
- the key target regressed:
  - transition `25-30 deg`: `54.2% -> 44.4%`
  - transition `18+ overall`: `65.2% -> 63.5%`
- action rate rose (`0.0066 -> 0.0071`), which is not ideal for sim-to-real actuator tracking.

Decision: roll back the v6.2.5 authority change. The likely issue is not simply residual authority; the policy can recover from static/reset tilt, but walking-transition cases are not being sampled or shaped in a way that produces a visible bracing/stop behavior.

Applied rollback config:

- run: `spotmicro_v6_2_6_prefall_tilt_recovery_30deg_rollback`
- resume from exp077, not exp078: `May25_14-02-38_spotmicro_v6_2_4_prefall_tilt_recovery_30deg_continue`, checkpoint `7100`
- restore `recovery_action_scale = 0.35`
- restore `recovery_gait_relief_scale = 0.65`
- restore `recovery_ik_relief_scale = 0.65`
- keep recovery command/phase slowdown: `command_scale = 0.15`, `phase_scale = 0.1`
- keep `recovery_roll_pitch_range_deg = 30.0`
- set `max_iterations = 400`

Also added a visual inspection script:

```bash
python legged_gym/legged_gym/scripts/play_prefall_visual.py --task=spotmicro_test --tilt-deg 28 --axis pitch --cmd-x 0.08
```

This runs a single robot, walks at slow `vx`, injects a 28 deg pre-fall tilt, and prints the recovery blend and effective command scaling. Use this to visually confirm whether recovery mode looks like bracing/settling or just continuing gait. Add `--record-frames` to save viewer frames under `logs/spotmicro_test/exported/prefall_visual_frames`.
