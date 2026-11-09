# `regionGrowCfg` for IWR1843 TLV20

The navigation region-growing stage has no firmware defaults. Before `sensorStart`,
each subframe with `guiMonitor ... customObstacles=1` must receive this command.
`-1` configures every subframe. Send it only while the sensor is stopped.

```text
regionGrowCfg <subFrameIdx|-1> <enterDb> <exitDb> <promDb> <riseDb> <auditCasoDb> <auditAnchorDb> <maxSpanBins> <maxBacktrackBins> <farOuter> <farInner> <nearOuter> <nearInner> <maxDopplerSpanBins> <sidelobeGuardDb>
```

The first six parameters are differences in detection-matrix magnitude expressed
in dB, converted to Q8 log2 units using `round(dB * 256 / 6)`. Valid range is
0–30 dB. `maxSpanBins` is 1–64 and `maxBacktrackBins` is 0–16. The four anchor
offsets are range bins from the candidate, each 1–16, ordered as
`nearInner <= nearOuter < farInner <= farOuter`. The far window is used on the
opposite side when there are no samples to the left. `maxDopplerSpanBins` is
0–8 circular Doppler bins from seed (`0` disables 2D Doppler expansion).
`sidelobeGuardDb` is 0–30 dB converted to Q8 log2 units using
`round(dB * 256 / 6)` and rejects Range FFT sidelobes at `|r - rPeak| >= 2`
below `peak - sidelobeGuardDb`.

The profile used during previous hardware validation is expressed as:

```text
regionGrowCfg -1 7.734375 2.25 7.96875 6 7.96875 9.84375 15 3 8 3 2 1 2 13.5
```

All arguments are validated before any subframe is updated. A rejected command
does not change the active configuration. Configuration persists across
`sensorStop`/`sensorStart 0` until reset or another `regionGrowCfg` command.
The TLV20 UART format is unchanged. Flash the newly built MSS/DSS image before
using this command; an older image rejects it.
