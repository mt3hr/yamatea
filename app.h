#ifndef APP
#define APP
#ifdef __cplusplus
extern "C"
{
    void __sync_synchronize() {} // TODO なんかこれないとビルド失敗するけど、メモリガードが機能しなくなるらしいのでよくないっぽい https://support.xilinx.com/s/question/0D52E00006hpjyKSAQ/mfloatabisoft-causes-link-error-in-c-projects?language=ja
#endif

#include "ev3api.h"

#define MAIN_PRIORITY (TMIN_APP_TPRI + 1)
#define TRACER_PRIORITY (TMIN_APP_TPRI + 2)

#ifndef STACK_SIZE
#define STACK_SIZE (4096)
#endif /* STACK_SIZE */

#ifndef TOPPERS_MACRO_ONLY

    extern void main_task(intptr_t exinf);
    extern void tracer_task(intptr_t exinf);
    extern void tracer_cyc(intptr_t exinf);

#endif /* TOPPERS_MACRO_ONLY */

#ifdef __cplusplus
}
#endif
#endif
