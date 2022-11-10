#include "stdio.h"
#include "stdlib.h"
#include "unistd.h"
#include <random>

// #define PRINT_PID_INFO // コメントアウトを外してpidの各値を出力する

using namespace std;

int main()
{
	double value;
	double target;
	double incrementValue;
	int errorValue;
	double kp;
	double ki;
	double kd;
	double dt;

	// 環境値設定ここから
	value = 5000;		  // 初期値
	target = 0;			  // 目標値
	incrementValue = 500; // targetの増減値
	errorValue = 0;		  // targetの増減値のランダム要素（実機を想定したブレ）（値探しのときは0にしたほうがいい）
	// 環境値設定ここまで

	// pidの値設定ここから
	dt = 0.05; // komichi
	kp = 0.1;
	ki = 50;
	kd = 0.05;

	dt = 0.5;
	kp = 0.5;
	ki = 0.7;
	kd = 0.05;

	dt = 0.5;
	kp = 0.6;
	ki = 2;
	kd = 0.18;

	// このプログラムだといい感じな値
	dt = 0.5;
	kp = 0.7;
	ki = 2.0;
	kd = 0.18;

	// 実機で明日（2022-11-11）試す値1
	dt = 0.5;
	kp = 0.6;
	ki = 1.65;
	kd = 0.25;

	// 実機で明日（2022-11-11）試す値2
	dt = 0.5;
	kp = 0.6;
	ki = 1.65;
	kd = 0.1338;

	// 実機で明日（2022-11-11）試す値3
	dt = 0.75;
	kp = 0.6;
	ki = 1.9;
	kd = 0.29;

	// 実機で明日（2022-11-11）試す値4
	dt = 0.5;
	kp = 0.35;
	ki = 2.2;
	kd = 0.29;

	// 実機調節でdtをいじってもいいかもしれない。KiKdをいじるのはしんどい

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

		double pid = kp * p + ki * i + kd * d;
		value += pid;

		// 誤差の追加
		double error = rand() % (-(errorValue / 2) - (errorValue / 2) + 1);
		value += incrementValue + error;

		// 情報の出力
#ifdef PRINT_PID_INFO
		plus = p < 0 ? "" : "+";
		printf("%05d p     : %s%01.5f\n", cnt, plus, p);
		plus = i < 0 ? "" : "+";
		printf("      i     : %s%01.5f\n", plus, i);
		plus = d < 0 ? "" : "+";
		printf("      d     : %s%01.5f\n", plus, d);
		plus = pid < 0 ? "" : "+";
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
