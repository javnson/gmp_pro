#ifndef _FILE_CURRENT_DISTRIBUTOR_LUT_H_
#define _FILE_CURRENT_DISTRIBUTOR_LUT_H_

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
    ctrl_gt im;    // current amplitude
    ctrl_gt alpha; // advance d-axis
} current_lookup_table_t;


//#ifndef CURRENT_DISTRIBUTION_LUT
static const current_lookup_table_t CURRENT_DISTRIBUTION_LUT[] = {
    {0.0000f, 0.3056f}, // im = 0.0, alpha = 110.0бу
    {0.0623f, 0.3222f}, // im = 2.0, alpha = 116.0бу
    {0.1246f, 0.3394f}, // im = 4.0, alpha = 122.2бу
    {0.1870f, 0.3500f}, // im = 6.0, alpha = 126.0бу
    {0.2493f, 0.3533f}, // im = 8.0, alpha = 127.2бу
    {0.3116f, 0.3589f}, // im = 10.0, alpha = 129.2бу
    {0.3739f, 0.3639f}, // im = 12.0, alpha = 131.0бу
    {0.4363f, 0.3700f}, // im = 14.0, alpha = 133.2бу
    {0.4986f, 0.3772f}, // im = 16.0, alpha = 135.8бу
    {0.5609f, 0.3844f}, // im = 18.0, alpha = 138.4бу
    {0.6232f, 0.3883f}, // im = 20.0, alpha = 139.8бу
    {0.6856f, 0.3944f}, // im = 22.0, alpha = 142.0бу
    {0.7479f, 0.3978f}, // im = 24.0, alpha = 143.2бу
    {0.8102f, 0.4000f}, // im = 26.0, alpha = 144.0бу
    {0.8725f, 0.4017f}, // im = 28.0, alpha = 144.6бу
    {0.9349f, 0.4050f}, // im = 30.0, alpha = 145.8бу
    {0.9972f, 0.4078f}, // im = 32.0, alpha = 146.8бу
    {1.0595f, 0.4094f}, // im = 34.0, alpha = 147.4бу
    {1.1218f, 0.4106f}, // im = 36.0, alpha = 147.8бу
    {1.1842f, 0.4117f}, // im = 38.0, alpha = 148.2бу
    {1.2465f, 0.4133f}, // im = 40.0, alpha = 148.8бу
    {1.3088f, 0.4144f}, // im = 42.0, alpha = 149.2бу
    {1.3711f, 0.4156f}, // im = 44.0, alpha = 149.6бу
    {1.4335f, 0.4167f}, // im = 46.0, alpha = 150.0бу
    {1.4958f, 0.4183f}, // im = 48.0, alpha = 150.6бу
    {1.5581f, 0.4211f}, // im = 50.0, alpha = 151.6бу
};
//#endif // CURRENT_DISTRIBUTION_LUT

//#ifndef CURRENT_DISTRIBUTION_LUT_SIZE
#define CURRENT_DISTRIBUTION_LUT_SIZE (sizeof(CURRENT_DISTRIBUTION_LUT) / sizeof(current_lookup_table_t))
//#endif

//#ifndef CURRENT_DISTRIBUTION_LUT_STEP
#define CURRENT_DISTRIBUTION_LUT_STEP (CURRENT_DISTRIBUTION_LUT[1].im - CURRENT_DISTRIBUTION_LUT[0].im)
//#endif

//#ifndef CURRENT_DISTRIBUTION_ALPHA
#define CURRENT_DISTRIBUTION_ALPHA (135.0f / 360.0f)
//#endif

#ifdef __cplusplus
}
#endif

#endif // _FILE_CURRENT_DISTRIBUTOR_LUT_H_
