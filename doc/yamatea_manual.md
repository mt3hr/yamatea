# yamatea_manual

## yamatea のマニュアル

マニュアル？Q&A？

## yamatea の設計と使い方

### yamatea の基礎概念（？）

- Command: ロボットの挙動（例: pid 走行（PIDTracer）, アーム上げ下げ（ArmController））
- CommandExecutor: 与えられたコマンドを逐次実行する実行者。コマンドを渡すときには終了条件も添え、その条件を満たしたときに次のコマンドへ遷移する。
- Predicate: 終了条件（例: モータ回転角度が指定角度を超えた（MotorCountPredicate）, ジャイロセンサの示す角度が指定角度を超えた（GyroRotateAnglePredicate））

### yamatea の使い方

サンプルプログラムについては[README.md](https://github.com/mt3hr/yamatea/blob/master/README.md)を参照してください。

## 基本的なこと

### このプログラムを使う用意

StartETRobo.bat から環境を起動している前提で説明します。

#### 初回ダウンロード（git clone）

1. 次のコマンドを実行する

```bash
cd ~/etrobo/workspace; git clone https://github.com/mt3hr/yamatea.git;
```

1. workspace フォルダ内に yamatea ディレクトリが作られるので、作業する

#### 更新適用（git pull）

初回ダウンロード（git clone）が済んでいる前提で説明します。

1. 次のコマンドを実行する

```bash
cd ~/etrobo/workspace/yamatea; git add *; git commit -m "update"; git pull;
```

1. なんかエディタっぽいのが起動した場合は、「CTRL+S」を押して、一旦キーを離し、「CTRL+S」を押す

### プログラム変更時にやること

StartETRobo.bat から環境を起動している前提で説明します。

#### 実機に転送する場合

1. 変更したプログラムファイルを保存する
1. PC と EV3 をケーブルで接続する
1. VSCode で次のコマンドを入力し、実行する

```bash
make app=yamatea up
```

1. PC と EV3 からケーブルを取り外す
1. EV3 で yamatea を選択して実行する（大体のプログラムはロボット左腕のタッチセンサーを押すと動きはじめます）

#### シミュレータで試走する場合

1. 変更したプログラムファイルを保存する
1. VSCode で次のコマンドを入力し、実行する

```bash
make app=yamatea sim up
```

1. 待つ（動き出すまでに結構時間がかかる）

### 新しい挙動の記述方法（モード追加方法）

流れは次のとおりです。下で具体的な記述方法を説明します。

1. Setting.h にモードを定義する
1. app.cpp に追加したモードの initializeCommandExecutor()関数を定義する
1. Setting.h の他のモードをコメントアウトする
1. ビルドする

#### 1. Setting.h にモードを定義する

Setting.h に追加したいモードを定義します。
例えば、直進し続けるモード「StraightMode」を追加するには次のようにします。

変更前

```c:Setting.h
// モード設定ここから
#define LeftCourceMode
//#define RightCourceMode
// モード設定ここまで
```

変更後

```c:Setting.h
// モード設定ここから
#define LeftCourceMode
//#define RightCourceMode
#define StraightMode
// モード設定ここまで
```

#### 2. app.cpp に追加したモードの initializeCommandExecutor()関数を定義する

変更前

```c++:app.cpp
// ↑省略
#if defined(LeftCourceMode) | defined(RightCourceMode)
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI)
{
    // 省略
}
#endif
// ↓省略
```

変更後

```c++:app.cpp
// ↑省略
#if defined(LeftCourceMode) | defined(RightCourceMode)
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI)
{
    // 省略
}
#endif

// ここから追加
#if defined(StraightMode)
void initializeCommandExecutor(CommandExecutor *commandExecutor, RobotAPI *robotAPI)
{
  Predicate *startButtonPredicate = new StartButtonPredicate();
  commandExecutor->addCommand(new Command(), startButtonPredicate, GET_VARIABLE_NAME(stopper));

  int pwm = 50;
  float distanceCm = 10000;
  Walker *walker = new Walker(pwm, pwm);
  DistancePredicate *walkerPredicate = new DistancePredicate(distanceCm, robotAPI);
  commandExecutor->addCommand(walker, walkerPredicate, GET_VARIABLE_NAME(walker));

  Stopper *stopper = new Stopper();
  Predicate *stopperPredicate = new NumberOfTimesPredicate(1);
  commandExecutor->addCommand(stopper, stopperPredicate, GET_VARIABLE_NAME(stopper));
}
#endif
// ここまで追加
// ↓省略
```

#### 3. Setting.h の他のモードをコメントアウトする

モード設定で、起動するモード以外の行をコメントアウトします。
変更前

```c:Setting.h
// モード設定ここから
#define LeftCourceMode
//#define RightCourceMode
#define StraightMode
// モード設定ここまで
```

変更後

```c:Setting.h
// モード設定ここから
//#define LeftCourceMode   // ←コメントアウト
//#define RightCourceMode
#define StraightMode
// モード設定ここまで
```

#### 4. ビルドする

ビルドして通れば OK

#### モード管理でこのやりかたをする理由

これがいいとは思っていないけれど、  
コメントアウトで切り替えができるので一番マシな気がして。  
app.cpp のファイル分割はしない予定です。（include が面倒くさすぎるため、また、ファイルが増えすぎるからです。C++、ヘッダファイルも書かないといけないから。）  
ファイル内検索を使って頑張りましょう。

### 設計モデルと実装の差

#### Command などのコンストラクタ引数と Command.run()メソッドなどの引数

Command のサブクラスは、EV3API オブジェクト（例えば ColorSensor など）をコンストラクタ引数として受け取らなくなりました。  
かわりに、Command.run()メソッドなどの引数に、RobotAPI が追加されました。  
RobotAPI は、EV3API オブジェクトを集約したものです。センサ, モータなどは RobotAPI の各種 getter から取得できます。

うまく説明できないですが、つまりどういうことかというと、  
Command のサブクラスは EV3API オブジェクトを所有しなくなりました。  
実行時、準備時にしか使わないなら関連減らしたほうがよくない？という考えによるものです。フィールドに埋め込まなくても実現できるなら、引数で渡したほうがいいよな。

モデルとコードがきれいになるというメリットがあります。

実行時以外に API アクセスが必要な Command は、コンストラクタ引数に RobotAPI を追加して対処しましょ。

#### パッケージ名

パッケージ名が異なります。  
また、command パッケージは難所ごとにサブパッケージが設けられています。

## 動かない、期待通りの動きをしない

設定ミスの可能性が高いです。  
原因別に説明します。

### ケーブルがささりきっていない

アプリケーション読み込みまで進んで、キャリブレータが起動しない場合。
各種センサのケーブルが刺さりきっていない可能性があります。

### モード設定が誤っている

設定 1/2（Setting.h）にモード設定があります。  
1 つだけ有効化して、他をコメントアウトしてください。  
ロボットは有効化されたモードの挙動をします。

### 「SimulatorMode」 を変更し忘れている

2022-08-19 現在、実機とシミュレータとでジャイロセンサから取得できる角度の符号が異なります。  
それに対応するための設定「SimulatorMode」があります。  
設定し忘れていると、GyroRotateAnglePredicate などが期待どおりに動かないです。  
（内部でそれを使ったクラス、例えば UFORunner なども期待どおりに動きません）

### 「EnableBluetooth」を有効化しているのに Bluetooth 接続していない

Bluetooth 接続しない場合はコメントアウトしてください。
期待どおりの走行をしなくなります。
なんかよくわからない場合は、この文書の上の方にある本番用左コース設定で動かしてみてください
