# Axiovel CI

This fork intentionally does not run the full upstream ArduPilot CI matrix on
pull requests. Automatic PR validation for branches whose name starts with
`AVCopter` is limited to the Axiovel Copter workflow:

- `Copter SITL smoke`
- `AxioLight Copter build`

The self-hosted runner that executes these jobs is registered at Axiovel
organization scope. The dedicated runner is currently named
`axiovel-workstation-ardupilot` and must expose these labels:

- `self-hosted`
- `Linux`
- `X64`
- `axio-sitl`

The workflows select `axio-sitl`, so other organization runners cannot pick
up ArduPilot jobs using only the generic platform labels. The runner host must
provide Docker Engine, and the runner service account must have access to the
Docker daemon because all Axiovel build and test jobs run in containers.

All inherited upstream workflows are kept available through manual
`workflow_dispatch` only. Re-enable automatic triggers only when the fork starts
depending on those targets again.
