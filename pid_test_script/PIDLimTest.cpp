#include "float.h"
#include "stdio.h"
#include "stdlib.h"
#include "unistd.h"
#include <random>

#define PRINT_PID_INFO // コメントアウトを外してpidの各値を出力する

using namespace std;

int main()
{
    double value;
    double target;
    double incrementValue;
    int errorValue;
    double ku;
    double tu;
    double kp;
    double ti;
    double td;
    double dt;

    // 環境値設定ここから
    value = 0;            // 初期値
    target = 0;           // 目標値
    incrementValue = 200; // targetの増減値
    errorValue = 0;       // targetの増減値のランダム要素（実機を想定したブレ）（値探しのときは0にしたほうがいい）
    // 環境値設定ここまで

    // 限界感度法を試す
    value = 500;          // 初期値
    target = 0;           // 目標値
    incrementValue = 500; // targetの増減値
    errorValue = 0;       // targetの増減値のランダム要素（実機を想定したブレ）（値探しのときは0にしたほうがいい）
    dt = 0.05;
    ku = 2;
    tu = dt * 2;
    kp = ku * 0.6;
    ti = 0.5 * tu;
    td = 0.125 * tu;

    // 持続振動を求める
    //  kp = 2;
    //  double ki = 0;
    //  double kd = 0;
    //  ti = kp / ki;
    //  td = kd / kp;

    // pidの設定ここまで

    double beforeP = 0;
    double integral = 0;
    const char *plus = "";
    plus = value < 0 ? "" : "+";
    printf("value : %s%01.5f\n", plus, value);
    plus = target < 0 ? "" : "+";
    printf("target: %s%01.5f\n", plus, target);

    // pid制御ここから
    for (int cnt = 1; true; cnt++)
    {
        // pid制御
        double p = target - value;
        integral += (p + beforeP) / 2.0 * dt;
        double i = integral;
        double d = (p - beforeP) / dt;
        double pid = kp * (p + 1 / ti * i + td * d);
        value += pid;

        // 誤差の追加
        double error = rand() % (-(errorValue / 2) - (errorValue / 2) + 1);
        value += incrementValue + error;

        // 情報の出力
#ifdef PRINT_PID_INFO
        plus = p < 0 ? "" : "+";
        printf("%05d p     : %s%01.5f\n", cnt, plus, p * kp);
        plus = i < 0 ? "" : "+";
        printf("%05d i     : %s%01.5f\n", cnt, plus, i * 1 / ti);
        plus = d < 0 ? "" : "+";
        printf("%05d d     : %s%01.5f\n", cnt, plus, d * td);
        printf("      pid   : %s%01.5f\n", plus, pid);
        plus = value < 0 ? "" : "+";
        printf("      value : %s%01.5f\n", plus, value);
        printf("\n");
#else
        plus = value < 0 ? "" : "+";
        printf("%05d value : %s%01.5f\n", cnt, plus, value);
#endif
        usleep(50000);
    }
    // pid制御ここまで
    return 0;
}
