# yamatea_manual

## yamateaのマニュアル
マニュアル？Q&A？

## yamateaの設計と使い方
### yamateaの基礎概念（？）
- Command: ロボットの挙動（例: pid走行（PIDTracer）, アーム上げ下げ（ArmController））
- CommandExecutor: 与えられたコマンドを逐次実行する実行者。コマンドを渡すときには終了条件も添え、その条件を満たしたときに次のコマンドへ遷移する。
- Predicate: 終了条件（例: モータ回転角度が指定角度を超えた（MotorCountPredicate）, ジャイロセンサの示す角度が指定角度を超えた（GyroRotateAnglePredicate））

### yamateaの使い方
サンプルプログラムについては[README.md](https://github.com/mt3hr/yamatea/blob/master/README.md)を参照してください。

## 基本的なこと

### このプログラムを使う用意
StartETRobo.batから環境を起動している前提で説明します。

#### 初回ダウンロード（git clone）
1. 次のコマンドを実行する
```bash
cd ~/workspace;
git clone git@github.com:mt3hr/yamatea.git;
```
1. workspaceフォルダ内にyamateaディレクトリが作られるので、作業する

#### 更新適用（git pull）
初回ダウンロード（git clone）が済んでいる前提で説明します。
1. 次のコマンドを実行する
```bash
cd ~/workspace/yamatea;
git add *;
git commit -m "update"; # -mはコミットメッセージなので必要に応じて変えて。
git pull;
```
1. なんかエディタっぽいのが起動した場合は、「CTRL+S」を押して、一旦キーを離し、「CTRL+S」を押す

### プログラム変更時にやること
StartETRobo.batから環境を起動している前提で説明します。

#### 実機に転送する場合
1. 変更したプログラムファイルを保存する
1. PCとEV3をケーブルで接続する
1. VSCodeで次のコマンドを入力し、実行する
```bash
make app=yamatea up
```
1. PCとEV3からケーブルを取り外す
1. EV3でyamateaを選択して実行する（大体のプログラムはロボット左腕のタッチセンサーを押すと動きはじめます）

#### シミュレータで試走する場合
1. 変更したプログラムファイルを保存する
1. VSCodeで次のコマンドを入力し、実行する
```bash
make app=yamatea sim up
```
1. 待つ（動き出すまでに結構時間がかかる）

## 動かない、期待通りの動きをしない

設定ミスの可能性が高いです。
原因別に説明します。

### モード設定が誤っている
設定1/2（Setting.h）にモード設定があります。
1つだけ有効化して、他をコメントアウトしてください。
ロボットは有効化されたモードの挙動をします。

### 「SimulatorMode」 を変更し忘れている
2022-08-19現在、実機とシミュレータとでジャイロセンサから取得できる角度の符号が異なります。
それに対応するための設定「SimulatorMode」があります。
設定し忘れていると、GyroRotateAnglePredicateなどが期待どおりに動かないです。
（内部でそれを使ったクラス、例えばUFORunnerなども期待どおりに動きません）