
#ifndef _FILE_CTL_VECTOR3_H_
#define _FILE_CTL_VECTOR3_H_

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct _tag_ctl_vector3_t
{
    ctrl_gt dat[3];
} ctl_vector3_t, vector3_gt;

GMP_STATIC_INLINE
void ctl_vector3_clear(ctl_vector3_t *vec)
{
    vec->dat[0] = 0;
    vec->dat[1] = 0;
    vec->dat[2] = 0;
}

GMP_STATIC_INLINE void ctl_vector3_set(ctl_vector3_t* vec, ctrl_gt v)
{
    vec->dat[0] = v;
    vec->dat[1] = v;
    vec->dat[2] = v;
}

#ifdef __cplusplus
}
#endif

#endif // _FILE_CTL_VECTOR3_H_
