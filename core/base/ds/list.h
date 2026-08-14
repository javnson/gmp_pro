/**
 * @file list.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 */


#ifndef _FILE_GMP_DATASTRUCT_LIST_H_
#define _FILE_GMP_DATASTRUCT_LIST_H_

#include <gmp_type.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

    
typedef struct _tag_gmp_list
{
    struct _tag_gmp_list *next;
} gmp_list;


#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_GMP_DATASTRUCT_LIST_H_

