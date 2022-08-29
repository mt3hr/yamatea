# encoding ANCI.

# このスクリプトの作成動機
# 1. etrobo環境の「make up」が遅い
# 2. 「make up」するとyamateaに上書きされる
#    モードごとに転送先ファイル名を分けることを自動化したい
# 3. 宿題からの逃避（CSS&JavaScriptの最終課題の案が思い浮かばない）

# 事前準備
# 1. wslでsshdがたっている
# 2. 「ssh localhost」でwslにつながる
# 3. このスクリプトがwindows上のどこかにある

# 実行方法
# 1. Windowsファイルシステムのどこかにこのスクリプトをコピーする（ホームディレクトリでいいと思う）
# 2. powershell上で「.\buildYamateaHackEV.ps1」を入力し、Enterで実行する
#   （「.\buildYamateaHackEV.ps1 -clean」でappsディレクトリの中身を全消ししてからビルド）

# 以上説明。以下コード。

Param([switch]$clean)

$projectName = "yamatea"

# EV3RTドライブを取得する
# 激遅なので決め打ちします
# 決め打ちしない場合は↓これ↓を有効化して
# $ev3rtDrive = Get-WmiObject Win32_LogicalDisk | ?{ ($_.VolumeName).toLower() -eq "ev3rt" } | %{ $_.VolumeName };
# 決め打ちしない場合は↓これ↓をコメントアウトして
$ev3rtDrive = "D:\"
if ($ev3rtDrive -eq "") {
    echo "ボリューム名EV3RTが見つかりませんでした。";
    echo "終了します。";
    exit;
}
if (!(Test-Path $ev3rtDrive)) {
    echo "EV3RTが接続されていません。";
    echo "終了します。";
    exit;
}

# cleanがtrueならEV3のappsディレクトリの中身を消す
if ($clean -eq $true) {
    Join-Path $ev3rtDrive "\ev3rt\apps\" | %{ Get-ChildItem $_ } | %{ rm $_.FullName };
}

# ビルドする。
ssh localhost "ETROBO_ROOT=~/etrobo; cd ~/etrobo; . ~/etrobo/scripts/etroboenv.sh silent; make app=$projectName;";

# EV3に転送するときのファイル名を決定する。（yamatea + コメントアウトされていないモードの名前）
$filename = ssh localhost "ETROBO_ROOT=~/etrobo; cd ~/etrobo/workspace/$projectName; . ~/etrobo/scripts/etroboenv.sh silent; grep -v '^//#' Setting.h | grep -v 'Bluetooth' | grep 'define' | grep -o '\s.*Mode' | xargs echo yamatea | sed 's/ //g' | sed 's/Mode//g'";

# 転送する（make upは遅いのでscpを使う）
scp "localhost:~/etrobo/workspace/app" (Join-Path $ev3rtDrive "ev3rt\apps\$filename");
