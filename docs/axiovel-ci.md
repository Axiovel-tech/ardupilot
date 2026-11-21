# Axiovel CI

This fork does not run upstream ArduPilot's CI. Every inherited workflow
(including `pre-commit` and `test_branch_conventions`) runs only by manual
`workflow_dispatch`; re-enable a trigger only when the fork starts depending on
that target again. Two Axiovel workflows run on their own:

| Workflow | Runs on | Jobs |
|---|---|---|
| `axiovel_copter_autotest.yml` (*Axiovel Copter autotest*) | a pull request whose **base** branch matches `AVCopter*` or `AVCopter/**`, from a branch of this repository (fork PRs are skipped) | `Copter SITL smoke`: `build.Copter` and six autotests (`ArmFeatures`, `SetHome`, `InitialMode`, `LoiterToAlt`, `VisionPosition`, `MAV_CMD_CONDITION_YAW`). `AxioLight Copter build`: compiles AXIOLIGHT-REVA, REVB and REVB-NOBARO. |
| `axiovel_sitl_artifact.yml` (*Axiovel SITL artifact*) | a push to a branch matching `AVCopter*` or `AVCopter/**`, so every merge | Publishes the Copter SITL binary as `sitl-copter`, and the REVB and REVB-NOBARO application (`.apj`), the REVB bootloader and the combined provisioning `.hex` as `axiolight-revb-*` artifacts, each with a `manifest.json` (git SHA, board, SHA-256), kept 30 days. |

AxioStack's `tooling/stack artifact fetch` installs `sitl-copter` from the run
of *Axiovel SITL artifact* at the pinned SHA, and finds it by that workflow
name (`ci_workflow` in AxioStack's `repos.yaml`): renaming the workflow is a
coordinated change with AxioStack.

## What CI does not test

- No SHOW-mode test: not `DroneShowTakeoffPadSink`, the show-slot and
  schedule scripts, or the `.gdmb` SITL comparison.
- No unit tests: `test_gdmb`, `test_sdcard_ota`, `test_jitter_correction` and
  `test_baro_bmp581` are not built or run.
- No other Axiovel autotest (`ExtNavClockReset`, `VirtualCompass*`), and no
  bootloader or SD-card update test; the bootloader is only compiled, on push.

Run these yourself; [AXIOVEL.md](../AXIOVEL.md) lists them.

## Runner

Both workflows run on the self-hosted runner labelled `self-hosted`, `Linux`,
`X64`, `axio-sitl`, in the `ardupilot/ardupilot-dev-base` and
`ardupilot-dev-chibios` `v0.1.3` images, capped at 2 CPUs and 6 GB per job.
