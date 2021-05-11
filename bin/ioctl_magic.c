#include <sys/types.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <stdbool.h>
#include "../include/dev/rcrecv/rcrecv.h"

int
main()
{
    printf("RCRECV_READ_CODE      int:%lu hex:0x%lx size:%d\n", RCRECV_READ_CODE, RCRECV_READ_CODE, sizeof(unsigned long));
    printf("RCRECV_READ_CODE_INFO int:%lu hex:0x%lx size:%d\n", RCRECV_READ_CODE_INFO, RCRECV_READ_CODE_INFO, sizeof(struct rcrecv_code));

    return (0);
}
