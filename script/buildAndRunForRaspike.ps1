# WSLのソースコードをRaspikeに転送して実行します。
# RasPikeにSSH接続できる状態にしてから走らせてください
# （我々はHackSPI使わないので、このスクリプトの説明は省きます）
# 実方
Param($projectName);
$temp = "$HOME\temp"; # 直接scpすると上手く動かない時があったりしたので
mkdir -Force $temp
scp -r localhost:~/etrobo/workspace/$projectName $temp;
ssh pi@raspberrypi rm -rf ~/work/RasPike/sdk/workspace/$projectName;
scp -r $temp\$projectName pi@raspberrypi:~/work/RasPike/sdk/workspace;
ssh pi@raspberrypi "cd ~/work/RasPike/sdk/workspace; make img=$projectName; make start;"