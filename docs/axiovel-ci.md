# Axiovel CI

This fork intentionally does not run the full upstream ArduPilot CI matrix on
pull requests. Automatic PR validation for `AVCopter-4.6` is limited to the
Axiovel Copter workflow:

- `Copter SITL smoke`
- `AxioLight Copter build`

The self-hosted runner that executes these jobs must be registered to the
repository with these labels:

- `self-hosted`
- `Linux`
- `X64`
- `axio-sitl`

All inherited upstream workflows are kept available through manual
`workflow_dispatch` only. Re-enable automatic triggers only when the fork starts
depending on those targets again.
