/*
 * Target fixtures. Included only with MMWDEMO_SURFACE_SELFTEST.
 * The startup hook runs fixtures in DPM task context before RF starts.
 * Uses the existing candidate/output buffers; it must not run during DPC.
 */
#pragma DATA_SECTION(gSurfaceReferenceParent, ".l3ram")
static uint16_t gSurfaceReferenceParent[MMWDEMO_DENSE_ELEV_MAX_OUTPUT_SAMPLES];
volatile uint32_t gSurfaceSelfTestFailures;
volatile uint32_t gSurfaceSelfTestMaxClusterCycles;
#pragma RETAIN(MmwDemo_SurfaceGen_selfTest)
#pragma CODE_SECTION(MmwDemo_SurfaceGen_selfTest, ".surfaceSelfTest")
#pragma CODE_SECTION(Fixture_root, ".surfaceSelfTest")
#pragma CODE_SECTION(Fixture_sample, ".surfaceSelfTest")
#pragma CODE_SECTION(Fixture_hash, ".surfaceSelfTest")
#pragma CODE_SECTION(Fixture_run, ".surfaceSelfTest")
extern far uint8_t gNavCandidateScratch[];
extern MmwDemo_HSRAM gHSRAM;

static uint16_t Fixture_root(uint16_t i)
{
    while (gSurfaceReferenceParent[i] != i) i = gSurfaceReferenceParent[i];
    return i;
}

static void Fixture_sample(MmwDemo_DenseElevation_OutputSample *s,
                           uint16_t r, int16_t u)
{
    memset(s, 0, sizeof(*s));
    s->rangeIdx = r; s->azimuthUQ15 = u; s->snrDb10 = 250U;
}

static uint32_t Fixture_hash(const ObstacleBox *out, uint32_t count)
{
    const uint8_t *bytes = (const uint8_t *)out;
    uint32_t i, hash = 2166136261UL;
    for (i = 0U; i < count * sizeof(*out); i++) hash = (hash ^ bytes[i]) * 16777619UL;
    return hash;
}

static int32_t Fixture_run(MmwDemo_DenseElevation_OutputSample *s, uint16_t n,
                          MmwDemo_SurfaceGen_StaticCfg *cfg, uint32_t *count)
{
    uint32_t start, elapsed;
    memset(cfg->status, 0, sizeof(*cfg->status));
    start = Cycleprofiler_getTimeStamp();
    {
        int32_t ret = MmwDemo_SurfaceGen_cluster(s, n, cfg,
            gHSRAM.customObstacles, MMWDEMO_SURFACE_MAX_BOXES, count);
        elapsed = Cycleprofiler_getTimeStamp() - start;
        if (elapsed > gSurfaceSelfTestMaxClusterCycles) gSurfaceSelfTestMaxClusterCycles = elapsed;
        return ret;
    }
}

uint32_t MmwDemo_SurfaceGen_selfTest(void)
{
    MmwDemo_DenseElevation_OutputSample *s =
        (MmwDemo_DenseElevation_OutputSample *)gNavCandidateScratch;
    MmwDemo_SurfaceGen_StaticCfg cfg;
    ObstacleBox *out = gHSRAM.customObstacles;
    uint32_t count = 0U, fail = 0U, i, j, hash;
    memset(&cfg, 0, sizeof(cfg));
    cfg.sourceStaticCfg.numRangeBins = 256U;
    cfg.sourceStaticCfg.numDopplerBins = 16U;
    cfg.rangeStepMeters = 0.1f; cfg.dopplerStepMps = 0.25f;
    cfg.minRangeMeters = 0.15f; cfg.maxRangeMeters = 8.64f;
    cfg.elapsedUsec = 50000U; cfg.status = &gHSRAM.surfaceStatus;
    gSurfaceSelfTestMaxClusterCycles = 0U;
    MmwDemo_SurfaceGen_reset();

    /* Bit 0: calibrated range and conservative enclosure of an extended blob. */
    cfg.rangeBiasMeters = 0.1f;
    Fixture_sample(&s[0], 21U, 0);
    if (Fixture_run(s, 1U, &cfg, &count) || count != 1U ||
        fabsf(out[0].y - 2.0f) > 0.00001f) fail |= 1UL;
    Fixture_sample(&s[1], 23U, 0); s[1].snrDb10 = 3000U;
    if (Fixture_run(s, 2U, &cfg, &count) || count != 1U ||
        out[0].y - 0.5f*out[0].depth_y > 1.95f + 0.00001f ||
        out[0].y + 0.5f*out[0].depth_y < 2.25f - 0.00001f) fail |= 1UL;
    cfg.rangeBiasMeters = 0.0f;

    /* Bit 1: compare production range-window union with an all-pairs oracle. */
    for (i = 0U; i < 400U; i++) {
        Fixture_sample(&s[i], (uint16_t)(2U + i/8U),
            (int16_t)(((int32_t)(i%8U)*2 - 7)*2900));
        gSurfaceReferenceParent[i] = (uint16_t)i;
    }
    for (i = 0U; i < 400U; i++) for (j = i+1U; j < 400U; j++) {
        int32_t dr = (int32_t)s[i].rangeIdx - s[j].rangeIdx;
        int32_t dd = (int32_t)s[i].dopplerIdx - s[j].dopplerIdx;
        float du = fabsf((float)s[i].azimuthUQ15-s[j].azimuthUQ15);
        float dv = fabsf((float)s[i].sourceVQ15-s[j].sourceVQ15);
        if (dr < 0) dr = -dr;
        if (dd < 0) dd = -dd;
        if (dr <= 2 && dd <= 3 &&
            du <= 0.08f*32767.0f + 0.5f*(s[i].azimuthWidthUQ15+s[j].azimuthWidthUQ15) &&
            dv <= 0.20f*32767.0f + 0.5f*(s[i].elevationWidthUQ15+s[j].elevationWidthUQ15))
            gSurfaceReferenceParent[Fixture_root((uint16_t)i)] = Fixture_root((uint16_t)j);
    }
    if (Fixture_run(s, 400U, &cfg, &count) || cfg.status->denseClustered != 400U)
        fail |= 2UL;
    for (i = 0U; i < 400U; i++) for (j = i+1U; j < 400U; j++)
        if ((Surface_find((int16_t)i) == Surface_find((int16_t)j)) !=
            (Fixture_root(gSurfaceOrder[i]) == Fixture_root(gSurfaceOrder[j]))) fail |= 2UL;
    hash = Fixture_hash(out, count);
    /* Bit 2: arbitrary input permutation must preserve every output box. */
    for (i = 0U; i < 400U; i++) {
        uint32_t at = (i*73U+19U)%400U;
        MmwDemo_DenseElevation_OutputSample tmp = s[i]; s[i] = s[at]; s[at] = tmp;
    }
    if (Fixture_run(s, 400U, &cfg, &count) || Fixture_hash(out, count) != hash) fail |= 4UL;

    /* Bit 3: the two endfire directions remain separate and lateral. */
    Fixture_sample(&s[0], 20U, 32767); Fixture_sample(&s[1], 20U, -32767);
    if (Fixture_run(s, 2U, &cfg, &count) || count != 2U ||
        fabsf(out[0].x) < 1.9f || fabsf(out[1].x) < 1.9f ||
        out[0].y > 0.01f || out[1].y > 0.01f) fail |= 8UL;

    /* Bit 4: mobile never publishes old coordinates after a missed detection. */
    if (Fixture_run(s, 0U, &cfg, &count) || count != 0U) fail |= 16UL;

    /* Bit 5: fixed hold is time-based, capped at 100 ms, without extrapolation. */
    cfg.motionMode = MMWDEMO_SURFACE_MOTION_FIXED;
    Fixture_sample(&s[0], 20U, 0); s[0].dopplerIdx = 7;
    Fixture_run(s, 1U, &cfg, &count);
    if (Fixture_run(s, 0U, &cfg, &count) || count != 1U ||
        fabsf(out[0].y - 2.0f) > 0.00001f || cfg.status->heldCount != 1U) fail |= 32UL;
    if (Fixture_run(s, 0U, &cfg, &count) || count != 1U) fail |= 32UL;
    cfg.elapsedUsec = 1U;
    if (Fixture_run(s, 0U, &cfg, &count) || count != 0U) fail |= 32UL;
    cfg.elapsedUsec = 50000U;

    /* Bit 6: subframes and mode transitions cannot share held coordinates. */
    Fixture_run(s, 1U, &cfg, &count);
    cfg.sourceStaticCfg.subFrameNum = 1U;
    if (Fixture_run(s, 0U, &cfg, &count) || count != 0U) fail |= 64UL;
    cfg.sourceStaticCfg.subFrameNum = 0U;
    if (Fixture_run(s, 0U, &cfg, &count) || count != 1U) fail |= 64UL;
    cfg.motionMode = MMWDEMO_SURFACE_MOTION_MOBILE;
    if (Fixture_run(s, 0U, &cfg, &count) || count != 0U) fail |= 64UL;

    /* Bit 7: a full history cannot suppress 64 completely new measurements. */
    cfg.motionMode = MMWDEMO_SURFACE_MOTION_FIXED;
    for (i = 0U; i < 64U; i++)
        Fixture_sample(&s[i], (uint16_t)(5U + (i/8U)*3U),
            (int16_t)(((int32_t)(i%8U)*2-7)*2900));
    Fixture_run(s, 64U, &cfg, &count);
    for (i = 0U; i < 64U; i++) s[i].rangeIdx += 50U;
    if (Fixture_run(s, 64U, &cfg, &count) || cfg.status->clusterCount != 64U ||
        count < 64U || cfg.status->heldCount != 64U) fail |= 128UL;

    /* Bit 8: maximum sample count, reported truncation, unique 8-bit ranks. */
    cfg.motionMode = MMWDEMO_SURFACE_MOTION_MOBILE;
    for (i = 0U; i < 3072U; i++) {
        Fixture_sample(&s[i], (uint16_t)(2U+(i%85U)),
            (int16_t)(((int32_t)((i/85U)%16U)*2-15)*1400));
        s[i].dopplerIdx = (int16_t)((int32_t)(i/1360U)*7-7);
    }
    if (Fixture_run(s, 3072U, &cfg, &count) ||
        cfg.status->denseClustered != 3072U || count > 255U) fail |= 256UL;
    for (i = 0U; i < count; i++)
        if (((uint16_t)out[i].boxId & 255U) != i+1U ||
            !isfinite(out[i].x) || !isfinite(out[i].y) || !isfinite(out[i].z)) fail |= 256UL;
    /* Explicitly generate 300 disconnected components and verify Q7 conservative enclosure. */
    for (i = 0U; i < 300U; i++)
        Fixture_sample(&s[i], (uint16_t)(2U+(i/16U)*3U),
            (int16_t)(((int32_t)(i%16U)*2-15)*1500));
    if (Fixture_run(s, 300U, &cfg, &count) || count != 255U ||
        cfg.status->clusterCount != 300U ||
        !(cfg.status->flags & MMWDEMO_SURFACE_BOX_LIMIT)) fail |= 256UL;
    for (i = 0U; i < 300U; i++) {
        SurfaceBounds sb;
        float cx, cy, cz;
        uint8_t covered = 0U;
        if (Surface_bounds(&s[i], &cfg, &sb) != 1U) { fail |= 256UL; break; }
        cx = 0.5f * (sb.loX + sb.hiX);
        cy = 0.5f * (sb.loY + sb.hiY);
        cz = 0.5f * (sb.loZ + sb.hiZ);
        for (j = 0U; j < count; j++) {
            if (cx >= out[j].x - 0.5f * out[j].width_x - 1e-4f &&
                cx <= out[j].x + 0.5f * out[j].width_x + 1e-4f &&
                cy >= out[j].y - 0.5f * out[j].depth_y - 1e-4f &&
                cy <= out[j].y + 0.5f * out[j].depth_y + 1e-4f &&
                cz >= out[j].z - 0.5f * out[j].height_z - 1e-4f &&
                cz <= out[j].z + 0.5f * out[j].height_z + 1e-4f) {
                covered = 1U; break;
            }
        }
        if (!covered) { fail |= 256UL; break; }
    }
    /* Bit 9: a valid angular footprint must survive a nonphysical independently
     * weighted center, while a footprint wholly outside the disk is rejected. */
    Fixture_sample(&s[0], 20U, 28507); s[0].sourceVQ15 = 19660;
    s[0].azimuthWidthUQ15 = 8192U;
    if (Fixture_run(s, 1U, &cfg, &count) || count != 1U ||
        out[0].x-0.5f*out[0].width_x > 1.5f ||
        out[0].z+0.5f*out[0].height_z < 1.2f) fail |= 512UL;
    Fixture_sample(&s[0], 20U, 31128); s[0].sourceVQ15 = 31128;
    if (Fixture_run(s, 1U, &cfg, &count) || count != 0U ||
        !(cfg.status->flags & MMWDEMO_SURFACE_INVALID_GEOMETRY)) fail |= 512UL;
    MmwDemo_SurfaceGen_reset();
    gSurfaceSelfTestFailures = fail;
    return fail;
}
