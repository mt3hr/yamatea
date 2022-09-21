# encoding ANCI.

# ���̃X�N���v�g�̍쐬���@
# 1. etrobo���́umake up�v���x��
# 2. �umake up�v�����yamatea�ɏ㏑�������
#    ���[�h���Ƃɓ]����t�@�C�����𕪂��邱�Ƃ�������������
# 3. �h�肩��̓����iCSS&JavaScript�̍ŏI�ۑ�̈Ă��v�������΂Ȃ��j
# 4. Bluetooth�]���Ń^�b�`�p�b�h���g�������Ȃ�

# ���O����
# 1. wsl��sshd�������Ă���
# 2. �ussh localhost�v��wsl�ɂȂ���
# 3. ���̃X�N���v�g��windows��̂ǂ����ɂ���
#
# Bluetooth�]���ݒ�
# 1. teraTerm�̐ݒ�t�@�C����ύX����BZmodemDataLen=128

# ���s���@
# 1. Windows�t�@�C���V�X�e���̂ǂ����ɂ��̃X�N���v�g���R�s�[����i�z�[���f�B���N�g���ł����Ǝv���j
# 2. powershell��Łu.\buildYamateaHackEV.ps1�v����͂��AEnter�Ŏ��s����
#   �i�u.\buildYamateaHackEV.ps1 -clean�v��apps�f�B���N�g���̒��g��S�������Ă���r���h���ē]���i�L���ڑ��̂ݗL���j�j
#   �i�u.\buildYamateaHackEV.ps1 -bluetoothPort 3�v��Port3��EV3�ɐڑ����ē]���j

# ����
# �ȏ�����B�ȉ��R�[�h�B

# �R�}���h���C������
Param([string]$projectName="yamatea", [switch]$clean, [int]$bluetoothPort=-1);

# TeraTerm��Path
$teraTermPath = "C:\Program Files (x86)\teraterm5\ttermpro.exe";

$bluetoothMode = $bluetoothPort -ne -1;

if (!$bluetoothMode) {
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
}

# �r���h����B
echo "build";

ssh localhost "ETROBO_ROOT=~/etrobo; cd ~/etrobo; . ~/etrobo/scripts/etroboenv.sh silent; make app=$projectName;" > $null;

echo "transfer";

# EV3�ɓ]������Ƃ��̃t�@�C���������肷��B
if ($projectName -eq "yamatea") {
    # yamatea�̏ꍇ�́uyamatea + ���ݗL���ȃ��[�h�̖��O�v
    $filename = ssh localhost "ETROBO_ROOT=~/etrobo; cd ~/etrobo/workspace/$projectName; . ~/etrobo/scripts/etroboenv.sh silent; grep -v '^//#' Setting.h | grep -v 'Bluetooth' | grep 'define' | grep -o '\s.*Mode' | xargs echo yamatea | sed 's/ //g' | sed 's/Mode//g'";
} else {
    # ����ȊO�̏ꍇ�̓v���W�F�N�g��
    $filename = $projectName;
}

if ($bluetoothMode) {
    $applicationFileTemp = Join-Path $env:TEMP $filename;
    $macroPath = Join-Path $env:TEMP "transferEV3Macro.ttl";
    
    # �A�v���P�[�V�����t�@�C����]������
    scp "localhost:~/etrobo/workspace/app" $applicationFileTemp;
    
    # �]���}�N�����ꎞ�t�@�C���ɏ�������
    Set-Content $macroPath @"
connect "/C=$bluetoothPort"
messagebox "load=>Bluetooth SPP�ɂ��Ă�������" "�]��"
zmodemsend "$applicationFileTemp" 1
"@;

    # teraTerm���N�����ē]������
    Start-Process $teraTermPath -ArgumentList "/M=$macroPath" -Wait;

    # �ꎞ�t�@�C�����폜����
    rm $applicationFileTemp;
    rm $macroPath;
} else {
    # clean��true�Ȃ�EV3��apps�f�B���N�g���̒��g������
    if ($clean -eq $true) {
        Join-Path $ev3rtDrive "\ev3rt\apps\" | %{ Get-ChildItem $_ } | %{ rm $_.FullName };
    }

    # �]������imake up�͒x���̂�scp���g���j
    scp "localhost:~/etrobo/workspace/app" (Join-Path $ev3rtDrive "ev3rt\apps\$filename");
}

echo "finish"