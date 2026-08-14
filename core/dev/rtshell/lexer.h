
#ifndef _FILE_RTSH_LEXER_H_
#define _FILE_RTSH_LEXER_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

#define RTSH_VARIABLE_PROPERTY_POINTER ((0x0001))

typedef enum _tag_variable_type
{
    RTSH_VAR_NULL = 0,
    RTSH_VAR_INT8 = 1,
    RTSH_VAR_UINT8 = 2,
    RTSH_VAR_INT16 = 3,
    RTSH_VAR_UINT16 = 4,
    RTSH_VAR_INT32 = 5,
    RTSH_VAR_UINT32 = 6,
    RTSH_VAR_BOOL = 7,
    RTSH_VAR_CTRL = 8,
    RTSH_VAR_REAL = 9,
    RTSH_VAR_STRING = 10
}rtsh_variable_type_t;

// #define RTSH_VARIABLE_BASE_CONENT \
//     /* fast_gt property;*/ \
//     fast_gt type; \ 
//     union{      \
//         int8_t value_int8;      \
//         uint8_t value_uint8;        \
//         int16_t value_int16;        \
//         uint16_t value_uint16;      \
//         int32_t value_int32;        \
//         uint32_t value_uint32;      \
//         fast_gt value_bool;     \
//         ctrl_gt value_ctrl;     \
//         parameter_gt value_real;        \
//         void *value_string; \
//     } content;\
    

typedef struct _tag_variable_base
{
    // property of the variable
    //fast_gt property;

    // type of variables  
    // select from @rtsh_variable_type_t
    fast_gt type;

    // content of variables
    union{
        int8_t value_int8;
        uint8_t value_uint8;
        int16_t value_int16;
        uint16_t value_uint16;
        int32_t value_int32;
        uint32_t value_uint32;
        fast_gt value_bool;
        ctrl_gt value_ctrl;
        parameter_gt value_real;
        void *value_pointer;
    } content;

}rtsh_var_base_t;

typedef struct _tag_variable_usr
{
    // property of the variable
    //fast_gt property;

    // type of variables  
    // select from @rtsh_variable_type_t
    fast_gt type;

    // content of variables
    union{
        int8_t value_int8;
        uint8_t value_uint8;
        int16_t value_int16;
        uint16_t value_uint16;
        int32_t value_int32;
        uint32_t value_uint32;
        fast_gt value_bool;
        ctrl_gt value_ctrl;
        parameter_gt value_real;
        void *value_pointer;
    } content;

    // name of variables
    // end string using '\0'
    byte_gt *name;

    // point to next variable
    // type is rtsh_var_usr
    rtsh_var_base_t * next;

}rtsh_var_usr_t;

typedef struct _tag_variable_intrisic
{
    // property of the variable
    //fast_gt property;

    // type of variables  
    // select from @rtsh_variable_type_t
    fast_gt type;

    // content of variables
    union{
        int8_t value_int8;
        uint8_t value_uint8;
        int16_t value_int16;
        uint16_t value_uint16;
        int32_t value_int32;
        uint32_t value_uint32;
        fast_gt value_bool;
        ctrl_gt value_ctrl;
        parameter_gt value_real;
        void *value_pointer;
    } content;

    // name of variables
    // end string using '\0'
    byte_gt *name;

}rtsh_var_intrinsic_t;

// typedef struct _tag_

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_RTSH_LEXER_H_
