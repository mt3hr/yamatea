# encoding ANCI.

# ���̃X�N���v�g�̍쐬���@
# 1. etrobo���́umake up�v���x��
# 2. �umake up�v�����yamatea�ɏ㏑�������
#    ���[�h���Ƃɓ]����t�@�C�����𕪂��邱�Ƃ�������������
# 3. �h�肩��̓����iCSS&JavaScript�̍ŏI�ۑ�̈Ă��v�������΂Ȃ��j

# ���O����
# 1. wsl��sshd�������Ă���
# 2. �ussh localhost�v��wsl�ɂȂ���
# 3. ���̃X�N���v�g��windows��̂ǂ����ɂ���

# ���s���@
# 1. Windows�t�@�C���V�X�e���̂ǂ����ɂ��̃X�N���v�g���R�s�[����i�z�[���f�B���N�g���ł����Ǝv���j
# 2. powershell��Łu.\buildYamateaHackEV.ps1�v����͂��AEnter�Ŏ��s����
#   �i�u.\buildYamateaHackEV.ps1 -clean�v��apps�f�B���N�g���̒��g��S�������Ă���r���h�j

# �ȏ�����B�ȉ��R�[�h�B

Param([switch]$clean)

$projectName = "yamatea"

# EV3RT�h���C�u���擾����
# ���x�Ȃ̂Ō��ߑł����܂�
# ���ߑł����Ȃ��ꍇ�́����ꁫ��L��������
# $ev3rtDrive = Get-WmiObject Win32_LogicalDisk | ?{ ($_.VolumeName).toLower() -eq "ev3rt" } | %{ $_.VolumeName };
# ���ߑł����Ȃ��ꍇ�́����ꁫ���R�����g�A�E�g����
$ev3rtDrive = "D:\"
if ($ev3rtDrive -eq "") {
    echo "�{�����[����EV3RT��������܂���ł����B";
    echo "�I�����܂��B";
    exit;
}
if (!(Test-Path $ev3rtDrive)) {
    echo "EV3RT���ڑ�����Ă��܂���B";
    echo "�I�����܂��B";
    exit;
}

# clean��true�Ȃ�EV3��apps�f�B���N�g���̒��g������
if ($clean -eq $true) {
    Join-Path $ev3rtDrive "\ev3rt\apps\" | %{ Get-ChildItem $_ } | %{ rm $_.FullName };
}

# �r���h����B
ssh localhost "ETROBO_ROOT=~/etrobo; cd ~/etrobo; . ~/etrobo/scripts/etroboenv.sh silent; make app=$projectName;";

# EV3�ɓ]������Ƃ��̃t�@�C���������肷��B�iyamatea + �R�����g�A�E�g����Ă��Ȃ����[�h�̖��O�j
$filename = ssh localhost "ETROBO_ROOT=~/etrobo; cd ~/etrobo/workspace/$projectName; . ~/etrobo/scripts/etroboenv.sh silent; grep -v '^//#' Setting.h | grep -v 'Bluetooth' | grep 'define' | grep -o '\s.*Mode' | xargs echo yamatea | sed 's/ //g' | sed 's/Mode//g'";

# �]������imake up�͒x���̂�scp���g���j
scp "localhost:~/etrobo/workspace/app" (Join-Path $ev3rtDrive "ev3rt\apps\$filename");
