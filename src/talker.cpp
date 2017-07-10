#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */

/*
 * =========== timer variable ===========
 */
#define LOOP_RATE_100Hz_ 100
const double deltaT=0.01;
unsigned int frame=0;

/*
 * =========== Task======================
 */
#define TASK_INIT_ 0
#define TASK_SITDOWN_  1
#define TASK_STANDING_  2
#define TASK_INIT_WALKING_  3
#define TASK_WALK_1STEP_   4
#define TASK_IDLE_   10
typedef struct{
    unsigned char startFlag:1;
    unsigned char finishFlag:1;
    unsigned char id;
    unsigned char scene;
}task_type;
task_type task;

/*
 * ============ constant =================
 */
const double degree = M_PI/180;

/*
 * ================================= Pose =======================
 */
//struct Pose{
//    unsigned int *NumOfFrams;
//    unsigned char *data;
//}pose;
const unsigned char pose_init[12]={127,130,105,140,70,150,127,120,124,145,100,167};//for example
const unsigned char softZeroPos[12]={127,127,127,127,48,206,127,127,127,127,127,127};// zero
//====== standing pose for walking=============//
const unsigned char Pose_init_walking[12]={125,127,104,150,88,166,145,109,127,127,127,127};
//====== sitdown=============//
const unsigned char Pose_sitdown[12]={127,127,81,173,190,64,250,4,127,127,127,127};
//====== standing pose============//
const unsigned char Pose_standing_0[12]={127,127,81,173,190,64,250,4,127,127,127,127};
const unsigned char Pose_standing_1[12]={127,127,87,167,156,98,240,14,127,127,127,127};
const unsigned char Pose_standing_2[12]={127,127,123,131,48,206,123,131,127,127,127,127};
const unsigned int numOfFrames_standing[2]={124,241};
/*
 * ============================== Scene =========================
 */
class Scene_class{
    unsigned int samTorq[12];
    unsigned char samP[12];
    unsigned char samI[12];
    unsigned char samD[12];
    double slope_begin[12];
    double slope_end[12];
    double slope_midle[12];
public:
    unsigned char *beginPose=NULL;
    unsigned char *endPose=NULL;
    unsigned int frame;
    unsigned int numOfFrame;
    struct Flag{
        unsigned char start:1;
        unsigned char finish:1;
        unsigned char enable:1;
    }flag;
    void setFrame(unsigned int value);
    void setBeginPose(const unsigned char *value);
    void setEndPose(const unsigned char *value);
    void setNumOfFrame(unsigned int value);
    void setUpMyScene(unsigned int fr, const unsigned char *beginpose,const unsigned char *endpose);
    Scene_class(){

    }
};

Scene_class myscene;
void setupMyScene(){
    myscene.setNumOfFrame(500);
    myscene.setBeginPose(softZeroPos);
    myscene.setEndPose(Pose_sitdown);
    myscene.frame=0;
    myscene.flag.start=0;
    myscene.flag.finish=0;
    myscene.flag.enable=1;
}


//====== test=============//
const unsigned char Pose_test[12]= {127,127,87,167,156,98,240,14,127,127,127,127};

unsigned char numPose_standing=0;


//======pose for walking 1 step- M04===========//
const unsigned char Pose_M04[10][12]={
    {125,127,102,150,89,166,146,109,127,127,127,127},
    {130,133,102,150,89,166,146,109,119,118,127,127},
    {132,136,87,153,129,166,166,109,124,121,127,127},
    {130,130,104,151,89,169,151,114,117,121,127,127},
    {125,125,106,151,89,169,151,114,122,124,127,127},
    {121,122,104,155,89,169,151,114,130,133,127,127},
    {119,119,103,163,89,143,148,94,133,137,127,127},
    {122,121,102,150,89,166,146,109,131,137,127,127},
    {125,127,102,150,89,166,146,109,127,127,127,127},
    {125,127,102,150,89,166,146,109,127,127,127,127},
};
const unsigned char Pose_M04_0[12]={125,127,102,150,89,166,146,109,127,127,127,127};
const unsigned char Pose_M04_1[12]={130,133,102,150,89,166,146,109,119,118,127,127};
const unsigned char Pose_M04_2[12]={132,136,87,153,129,166,166,109,124,121,127,127};
const unsigned char Pose_M04_3[12]={130,130,104,151,89,169,151,114,117,121,127,127};
const unsigned char Pose_M04_4[12]={125,125,106,151,89,169,151,114,122,124,127,127};
const unsigned char Pose_M04_5[12]={121,122,104,155,89,169,151,114,130,133,127,127};
const unsigned char Pose_M04_6[12]={119,119,103,163,89,143,148,94,133,137,127,127};
const unsigned char Pose_M04_7[12]={122,121,102,150,89,166,146,109,131,137,127,127};
const unsigned char Pose_M04_8[12]={125,127,102,150,89,166,146,109,127,127,127,127};
const unsigned char Pose_M04_9[12]={125,127,102,150,89,166,146,109,127,127,127,127};
unsigned char pose_M04=0;
const unsigned int numOfFrames_M04[10]={22,22,18,10,10,22,20,12,24};
//======pose for walking 1 step- M04===========//

const unsigned int numOfFrames[10]={100,100,0,0,0,0,0,0,0,0};
const double pos8bitTorad=1.08*M_PI/180;
//const unsigned int numOfFrame=500;
double angle[12];
char a;
unsigned char first_pose_cal_Flag=0;
int main(int argc, char **argv)
{

    ros::init(argc, argv, "talker");
    ros::NodeHandle n;


    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    ros::Publisher my_chatter_pub = n.advertise<sensor_msgs::JointState>("chatter_2", 1000);

    ros::Rate loop_rate(LOOP_RATE_100Hz_);

    sensor_msgs::JointState joint_state;
    joint_state.name.resize(12);
    joint_state.position.resize(12);
    joint_state.name[0] ="Joint0";
    joint_state.name[1] ="Joint1";
    joint_state.name[2] ="Joint2";
    joint_state.name[3] ="Joint3";
    joint_state.name[4] ="Joint4";
    joint_state.name[5] ="Joint5";
    joint_state.name[6] ="Joint6";
    joint_state.name[7] ="Joint7";
    joint_state.name[8] ="Joint8";
    joint_state.name[9] ="Joint9";
    joint_state.name[10] ="Joint10";
    joint_state.name[11] ="Joint11";
//    setupMyScene();
    double delta_pos[12];
    for(unsigned char i=0;i<12;i++)
    {
        angle[i]=(double)(Pose_M04_8[i]-softZeroPos[i])*pos8bitTorad;
    }
    unsigned char *test;
    test = (unsigned char*)Pose_test;
    //    a=10;
    //printf( "sizeof( s ) = %d\n",  sizeof( test ) );
    //    ROS_INFO("data: %d",sizeof(test));
    task.id=TASK_SITDOWN_;
    while (ros::ok())
    {
        //        frame++;
        //        if(frame>10000) frame=0;
        switch (task.id){
        case TASK_INIT_:
            if(task.startFlag==0){
                task.startFlag=1;
                myscene.setUpMyScene(200,pose_init,softZeroPos);
            }

            else if(task.finishFlag){
                task.finishFlag=0;
                task.startFlag=0;
                task.id=TASK_SITDOWN_;
            }
            else if(myscene.flag.finish)
            {
                task.finishFlag=1;
            }
            break;
        case  TASK_SITDOWN_:
            if(task.startFlag==0){
                task.startFlag=1;
                myscene.setUpMyScene(300,softZeroPos,Pose_sitdown);
            }

            else if(task.finishFlag){
                task.finishFlag=0;
                task.startFlag=0;
                task.id=TASK_STANDING_;
            }
            else if(myscene.flag.finish)
            {
                task.finishFlag=1;
            }
            break;
        case TASK_STANDING_:
            if(task.startFlag==0){
                task.startFlag=1;
                task.scene=0;
                myscene.setUpMyScene(numOfFrames_standing[task.scene],Pose_standing_0,Pose_standing_1);
            }
            else if(myscene.flag.finish)
            {
                myscene.flag.finish=0;
                task.scene++;
                switch(task.scene){
                case 0:
                    myscene.setUpMyScene(numOfFrames_standing[task.scene],Pose_standing_0,Pose_standing_1);
                    break;
                case 1:
                    myscene.setUpMyScene(numOfFrames_standing[task.scene],Pose_standing_1,Pose_standing_2);
                    break;
                default:
                    task.finishFlag=1;
                    break;
                }
                //                task.finishFlag=1;
            }
            else if(task.finishFlag){
                task.finishFlag=0;
                task.startFlag=0;
                task.id=TASK_INIT_WALKING_;
            }
            break;
        case TASK_INIT_WALKING_:
            if(task.startFlag==0){
                task.startFlag=1;
                myscene.setUpMyScene(300,Pose_standing_2,Pose_init_walking);
            }
            else if(myscene.flag.finish)
            {
                myscene.flag.finish=0;
                task.finishFlag=1;
            }
            else if(task.finishFlag){
                task.finishFlag=0;
                task.startFlag=0;
                task.id=TASK_WALK_1STEP_;
            }
            break;
        case TASK_WALK_1STEP_:
            if(task.startFlag==0){
                task.startFlag=1;
                task.scene=0;
                //===== you code begin from here====
                myscene.setUpMyScene(numOfFrames_M04[task.scene],Pose_M04_0,Pose_M04_1);
            }
            else if(myscene.flag.finish)
            {
                myscene.flag.finish=0;
                task.scene++;
                switch(task.scene){
                case 0:
                   myscene.setUpMyScene(numOfFrames_M04[task.scene],Pose_M04_0,Pose_M04_1);
                    break;
                case 1:
                   myscene.setUpMyScene(numOfFrames_M04[task.scene],Pose_M04_1,Pose_M04_2);
                    break;
                case 2:
                    myscene.setUpMyScene(numOfFrames_M04[task.scene],Pose_M04_2,Pose_M04_3);
                     break;
                case 3:
                    myscene.setUpMyScene(numOfFrames_M04[task.scene],Pose_M04_3,Pose_M04_4);
                     break;
                case 4:
                    myscene.setUpMyScene(numOfFrames_M04[task.scene],Pose_M04_4,Pose_M04_5);
                     break;
                case 5:
                    myscene.setUpMyScene(numOfFrames_M04[task.scene],Pose_M04_5,Pose_M04_6);
                     break;
                case 6:
                    myscene.setUpMyScene(numOfFrames_M04[task.scene],Pose_M04_6,Pose_M04_7);
                     break;
                case 7:
                    myscene.setUpMyScene(numOfFrames_M04[task.scene],Pose_M04_7,Pose_M04_8);
                     break;
                case 8:
                    myscene.setUpMyScene(numOfFrames_M04[task.scene],Pose_M04_8,Pose_M04_9);
                     break;
                default:
                    task.finishFlag=1;
                    break;
                }
                //                task.finishFlag=1;
            }
            else if(task.finishFlag){
                task.finishFlag=0;
                task.startFlag=0;
                //==========your code==========
                task.id=TASK_WALK_1STEP_;
            }
            break;
        default:
            break;
        }
        if(myscene.flag.enable){
            myscene.frame++;
            if(myscene.flag.start==0){
                myscene.flag.start=1;
                myscene.frame=0;
                for(unsigned char i=0;i<12;i++)
                {
                    angle[i]=(double)(*(myscene.beginPose+i)-softZeroPos[i])*pos8bitTorad;
                }

                for(unsigned char i=0;i<12;i++)
                {

                    delta_pos[i]=(double)(-*(myscene.beginPose+i)+*(myscene.endPose+i))*pos8bitTorad/(double)myscene.numOfFrame;
                }
            }
            else if(myscene.frame<myscene.numOfFrame)
            {
                for(unsigned char i=0;i<12;i++)
                {
                    angle[i]+=delta_pos[i];
                }
            }
            else{
                myscene.flag.enable=0;
                myscene.flag.finish=1;
                for(unsigned char i=0;i<12;i++)
                {
                    angle[i]=(double)(*(myscene.endPose+i)-softZeroPos[i])*pos8bitTorad;
                }
            }
        }
        std_msgs::String msg;
        std::stringstream ss;
        ss <<angle[1];
        msg.data = ss.str();

        //        for(unsigned char i=0;i<12;i++)
        //        {
        //            angle[i]+=delta_pos[i];
        //        }

        //        count++;
        //        if(count>numOfFrame){
        //            count=0;
        //            for(unsigned char i=0;i<12;i++)
        //            {
        //                angle[i]=0;
        //            }
        //        }
        joint_state.header.stamp = ros::Time::now();

        joint_state.position[0] = angle[0];
        joint_state.position[1] = angle[1];
        joint_state.position[2] = -angle[2];
        joint_state.position[3] = angle[3];
        joint_state.position[4] = -angle[4];
        joint_state.position[5] = angle[5];
        joint_state.position[6] = angle[6];
        joint_state.position[7] = -angle[7];
        joint_state.position[8] = angle[8];
        joint_state.position[9] = angle[9];
        joint_state.position[10] = angle[10];
        joint_state.position[11] = angle[11];


        ROS_INFO("%s", msg.data.c_str());

        my_chatter_pub.publish(joint_state);
        ros::spinOnce();

        loop_rate.sleep();
    }


    return 0;
}

//unsigned char *Scene_class::getBeginPose() const
//{
//return beginPose;
//}

//void Scene_class::setBeginPose(unsigned char *value)
//{
//beginPose = value;
//}

//void Scene_class::setFrame(unsigned int value)
//{
//frame = value;
//}

//void Scene_class::setBeginPose(unsigned char *value)
//{
//beginPose = value;
//}

//void Scene_class::setEndPose(const unsigned char *value)
//{
//endPose = value;
//}

void Scene_class::setBeginPose(const unsigned char *value)
{
    beginPose = (unsigned char*)value;
}

void Scene_class::setEndPose(const unsigned char *value)
{
    endPose = (unsigned char*)value;
}

void Scene_class::setNumOfFrame(unsigned int value)
{
    numOfFrame = value;
}

void Scene_class::setUpMyScene(unsigned int fr, const unsigned char *beginpose, const unsigned char *endpose)
{
    this->setNumOfFrame(fr);
    this->setBeginPose(beginpose);
    this->setEndPose(endpose);
    this->frame=0;
    this->flag.start=0;
    this->flag.finish=0;
    this->flag.enable=1;
}

void Scene_class::setFrame(unsigned int value)
{
    frame = value;
}
