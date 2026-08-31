# Fusion upstream information

- Project: xioTechnologies/Fusion
- Repository: https://github.com/xioTechnologies/Fusion
- Version: v1.3.2
- Commit: 015d68494274b479b5996bff2530ecbcfdc266f2
- License: MIT (`LICENSE.md`)

Imported without modification:

- `inc/FusionAhrs.h`
- `inc/FusionBias.h`
- `inc/FusionConvention.h`
- `inc/FusionMath.h`
- `src/FusionAhrs.c`
- `src/FusionBias.c`

The `fusion_imu.*` and `fusion_filter.*` files are local application
integration code.  The firmware uses
`FusionAhrsUpdateNoMagnetometer` for six-axis operation and `FusionBias` for
run-time gyroscope bias estimation.  No separate input low-pass filter is
applied by this integration.
