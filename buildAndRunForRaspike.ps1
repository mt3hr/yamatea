# WSLのソースコードをRaspikeに転送して実行します。
# RasPikeにSSH接続できる状態にしてから走らせてください
Param($projectName);
scp -r localhost:~/workspace/$projectName pi@raspberrypi:~/work/RasPike/sdk/workspace
ssh pi@raspberrypi "cd ~/work/RasPike/sdk/workspace; make img=$projectName; make start;"
