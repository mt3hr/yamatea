#ifndef APP
#define APP
#ifdef __cplusplus
extern "C"
{
    void __sync_synchronize() {} // TODO なんかこれないとビルド失敗するけど、メモリガードが機能しなくなるらしいのでよくないっぽい https://support.xilinx.com/s/question/0D52E00006hpjyKSAQ/mfloatabisoft-causes-link-error-in-c-projects?language=ja
#endif

#include "ev3api.h"

#define MAIN_PRIORITY (TMIN_APP_TPRI + 1)
#define RUNNER_PRIORITY (TMIN_APP_TPRI + 2)
#define LISTEN_BLUETOOTH_COMMAND_PRIORITY (TMIN_APP_TPRI + 3)
#define RETURN_TO_START_POINT_PRIORITY (TMIN_APP_TPRI + 4)
#define FROGGY_SONG_PRIORITY (TMIN_APP_TPRI + 5)

#ifndef STACK_SIZE
#define STACK_SIZE (4096)
#endif /* STACK_SIZE */

#ifndef TOPPERS_MACRO_ONLY

    extern void main_task(intptr_t exinf);
    extern void runner_task(intptr_t exinf);
    extern void runner_cyc(intptr_t exinf);
    extern void listen_bluetooth_command_task(intptr_t exinf);
    extern void return_to_start_point_task(intptr_t exinf);
    extern void sing_a_song_task(intptr_t exinf);

#endif /* TOPPERS_MACRO_ONLY */

#ifdef __cplusplus
}
#endif
#endif
