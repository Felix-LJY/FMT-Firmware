#if 0
#include <firmament.h>
#include "module/task_manager/task_manager.h"

static fmt_err_t task_init(void)
{
    return FMT_EOK;
}

static void task_entry(void* parameter)
{
    /* main loop */
    while (1) {
        printf("Hello FMT!\n");
        sys_msleep(1000);
    }
}

TASK_EXPORT __fmt_task_desc = {
    .name = "hello",
    .init = task_init,
    .entry = task_entry,
    .priority = 25,
    .auto_start = true,
    .stack_size = 4096,
    .param = NULL,
    .dependency = NULL
};
#endif
