
#ifndef _RTSH_CMD_H_
#define _RTSH_CMD_H_

typedef byte_gt rtsh_char_t ;

typedef int (*command_target_t)(int ,char*[]);

typedef struct _tag_rtsh_cmd_record 
{
    // prompt for command
    rtsh_char_t *prompt;

    // command target
    command_target_t target;

    // command description
    rtsh_char_t *desc;
}rtsh_cmd_record_t;

// This is a static array, end with {0, 0, 0}
extern rtsh_cmd_record_t internal_cmd[];


#endif // _RTSH_CMD_H_
