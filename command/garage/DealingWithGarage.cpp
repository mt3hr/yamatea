#include "DealingWithGarage.h"
#include "ColorReader.h"
#include "Stopper.h"
#include "CommandExecutor.h"
#include "Walker.h"
#include "NumberOfTimesPredicate.h"
#include "Walker.h"
#include "ColorPredicate.h"
#include "WheelDistancePredicate.h"
#include "CommandAndPredicate.h"
#include "RotateRobotUseGyroCommandAndPredicate.h"
#include "RawColorPredicate.h"

DealingWithGarage::~DealingWithGarage()
{

}
DealingWithGarage::DealingWithGarage(colorid_t* colorID,CommandExecutor* commandExecutor,bool reverse)
{
    //コマンドエグゼキューターポインタを渡し処理を追加させます。
    //reverseでコース反転させるか否かを決めます
    this->colorID = colorID;
    this->commandExecutor = commandExecutor;
    this->reverse = reverse;
}


// オーバーライドして使って
void DealingWithGarage::run(RobotAPI *robotAPI)
{
    if(executeState==false){
        int leftPow;
        int rightPow;
        Stopper *stopper1 = new Stopper();
        if(*colorID==COLOR_RED){
            //------14r、ガレージ赤------
            Predicate *predicateS0 = new NumberOfTimesPredicate(1);
            commandExecutor->addCommand(stopper1, predicateS0, "stopper1");
            
            //r1,赤エリアまで直進
            leftPow = 20;   
            rightPow = 20;
            Walker *walker1 = new Walker(leftPow, rightPow);
            Predicate *predicate1 = new ColorPredicate(COLOR_RED);
            commandExecutor->addCommand(walker1, predicate1, "walker1");
            Predicate *predicateS1 = new NumberOfTimesPredicate(1);
            commandExecutor->addCommand(stopper1, predicateS1, "stopper1");
            //r2,後ろに下がる
            leftPow = -20;
            rightPow = -20;
            Walker *walker2 = new Walker(leftPow, rightPow);
            Predicate *predicate2 = new WheelDistancePredicate(-5, robotAPI);
            commandExecutor->addCommand(walker2, predicate2, "walker2");
            Predicate *predicateS2 = new NumberOfTimesPredicate(1);
            commandExecutor->addCommand(stopper1, predicateS2, "stopper1");
            // r3,90ど右回転
            CommandAndPredicate *predicate3 = new RotateRobotUseGyroCommandAndPredicate(90,10,robotAPI);
            if (reverse) {
                predicate3 = new RotateRobotUseGyroCommandAndPredicate(-90,10,robotAPI);
            }
            commandExecutor->addCommand(predicate3->getCommand(), predicate3->getPredicate(), "90turn");
            Predicate *predicateS3 = new NumberOfTimesPredicate(1);
            commandExecutor->addCommand(stopper1, predicateS3, "stopper1");
            
            //r4,直進
            leftPow = 20;
            rightPow = 20;
            Walker *walker4 = new Walker(leftPow, rightPow);
            Predicate *predicate4 = new WheelDistancePredicate(14, robotAPI);
            commandExecutor->addCommand(walker4, predicate4, "walker4");
            Predicate *predicateS4 = new NumberOfTimesPredicate(1);
            commandExecutor->addCommand(stopper1, predicateS4, "stopper1");
            //r5,90度左回転
            CommandAndPredicate *predicate5 = new RotateRobotUseGyroCommandAndPredicate(-90,10,robotAPI);
            if (reverse) {
                predicate5 = new RotateRobotUseGyroCommandAndPredicate(90,10,robotAPI);
            }
            commandExecutor->addCommand(predicate5->getCommand(), predicate5->getPredicate(), "-90turn");
            Predicate *predicateS5 = new NumberOfTimesPredicate(1);
            commandExecutor->addCommand(stopper1, predicateS5, "stopper1");
            //r6,直進
            for(int i=0;i<3;i++){
                commandExecutor->addCommand(walker4, predicate4, "walker4");
                commandExecutor->addCommand(stopper1, predicateS1, "stopper1");
            }
            //r7,90度左回転()5の使いまわし
            commandExecutor->addCommand(predicate5->getCommand(), predicate5->getPredicate(), "-90turn");
            Predicate *predicateS7 = new NumberOfTimesPredicate(1);
            commandExecutor->addCommand(stopper1, predicateS7, "stopper1");
            //r8,直進
            Predicate *predicate8 = new ColorPredicate(COLOR_GREEN);
            commandExecutor->addCommand(walker4, predicate8, "walker8");
            
            Predicate *predicate8s = new WheelDistancePredicate(12, robotAPI);
            commandExecutor->addCommand(walker4, predicate8s, "walker48strate");
            Predicate *predicateS8 = new NumberOfTimesPredicate(1);
            commandExecutor->addCommand(stopper1, predicateS8, "stopper1");
            //r9, 90ど右回転(３の使いまわし)
            commandExecutor->addCommand(predicate3->getCommand(), predicate3->getPredicate(), "90turn");
            commandExecutor->addCommand(stopper1, predicateS1, "stopper1");
            
        }else if(*colorID==COLOR_GREEN){
            //ーーーーー14gガレージ緑ーーーーー
            leftPow = 30;
            rightPow = 30;
            Walker *walkerG = new Walker(leftPow, rightPow);
            Predicate *predicateG = new ColorPredicate(COLOR_GREEN);
            commandExecutor->addCommand(walkerG, predicateG, "walkerG");
        }else if(*colorID==COLOR_YELLOW){
            
            //ーーーーー14ｙガレージ黄色ーーーーー
            leftPow = 20;   
            rightPow = 20;
            Walker *walkerY = new Walker(leftPow, rightPow);
            Predicate *predicateY = new ColorPredicate(COLOR_GREEN);
            commandExecutor->addCommand(walkerY, predicateY, "walkerY");
            leftPow = 40;
            rightPow = 40;
            Walker *walker4 = new Walker(leftPow, rightPow);
            Predicate *predicate4 = new WheelDistancePredicate(20, robotAPI);
            commandExecutor->addCommand(walker4, predicate4, "walkerB");
        }else{
            //ーーーーー14gガレージ青ーーーーー
            leftPow = 20;
            rightPow = 20;
            Walker *walkerB = new Walker(leftPow, rightPow);
            Predicate *predicateB = new ColorPredicate(COLOR_GREEN);
            commandExecutor->addCommand(walkerB, predicateB, "walkerY");  
            leftPow = 40;
            rightPow = 40;
            Walker *walker4 = new Walker(leftPow, rightPow);
            Predicate *predicate4 = new WheelDistancePredicate(40, robotAPI);
            commandExecutor->addCommand(walker4, predicate4, "walkerB");
        }
        // 停止コマンドの初期化とCommandExecutorへの追加
        int numberOfTimes = 1;
        Stopper *stopper = new Stopper();
        Predicate *stopperPredicate = new NumberOfTimesPredicate(numberOfTimes);
        commandExecutor->addCommand(stopper, stopperPredicate, "stopper");
        executeState=true;
    }
    return;
}

void DealingWithGarage::preparation(RobotAPI *robotAPI)
{
    return;
}

// 左右反転したコマンドを生成する。
// オーバーライドして使って。
Command *DealingWithGarage::generateReverseCommand()
{
    DealingWithGarage *reversedDealingWithGarage = new DealingWithGarage(colorID, commandExecutor,true);
    return reversedDealingWithGarage;
}