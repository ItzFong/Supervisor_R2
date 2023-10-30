#include <webots/Supervisor.hpp>
#include <webots/Robot.hpp>
#include <webots/Node.hpp>
#include <webots/Camera.hpp>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#define TIME_STEP 32
using namespace webots;
using namespace std;

//Toa do
typedef struct 
{
  double x,z,dx,dz;
}Coord;

//Toa do duong tron
typedef struct 
{
  double I1,I2,R1,R2,x1,x2,z1,z2;
}Circle;
//Cac duong tron

// CheckPoint
Coord CP[5] = {{1.63, 2.5, 0.02,0.081}, {2.8, -0.56,0.081,0.02}, {-1.4, -0.59,0.081,0.02}, {-4.34, -1.28,0.081,0.02}, {-0.62, -2.51,0.02,0.081}};

//Start point of every line
Coord CL[51] = {{-3.11, 2.54,0.02,0.081},//0
{-3.11, 1.52,0.02,0.081},//1
{-3.08, 1.14,0.02,0.081},//2
{-3.1, 0.76,0.081,0.02},//3
{-3.06, 0.38,0.02,0.081},//4
{-3.05, -0.5,0.02,0.081},//5
{-2.45, -0.5,0.081,0.02},//6CP
{-2.45, -2.54,0.02,0.081},//7
{-2.18, -2.54,0.081,0.02},//8
{-2.18, -2.84, 0.02, 0.081},//9
{-1.6, -2.84, 0.081, 0.02},//10
{-1.6, -2.33, 0.02, 0.081},//11
{-1.6, -1.55, 0.081, 0.02},//12
{-1.6, 0.39, 0.02, 0.081},//13
{-2.36, 0.39, 0.081, 0.02},//14
{-2.36, 1.12, 0.02, 0.081},//15
{-1.27, 1.12, 0.081, 0.02},//16
{-1.27, 1.5, 0.02, 0.081},//17
{-1.97, 1.5, 0.081, 0.02},//18
{-1.97, 2.75, 0.02, 0.081},//19
{-0.56, 2.75, 0.081, 0.02},//20
{-0.57, 1.77, 0.02, 0.081},//21
{-0.21, 1.77, 0.081, 0.02},//22
{-0.21, -0.91, 0.02, 0.081},//23
{-0.36, -1.09, 0.02, 0.081},//24
{-0.2, -1.25, 0.02, 0.081},//25
{-0.37, -1.45, 0.02, 0.081},//26
{-0.2, -1.6, 0.02, 0.081},//27
{-0.37, -1.8, 0.02, 0.081},//28
{-0.21, -1.96, 0.02, 0.081},//29
{-0.38, -2.16, 0.02, 0.081},//30
{-0.2, -2.31, 0.02, 0.081},//31
{0.53, -2.31, 0.081, 0.02},//32
{0.53, -0.99, 0.02, 0.081},//33
{1.52, -0.99, 0.081, 0.02},//34
{1.52, -1.61, 0.02, 0.081},//35
{1.52, -2.59, 0.02, 0.081},//36
{1.52, -2.97, 0.02, 0.081},//37
{2.86, -2.97, 0.081, 0.02},//38
{2.86, -2.71, 0.02, 0.081},//39
{3.06, -2.69, 0.35, 0.081},//40
{3.06, -2.02, 0.35, 0.081},//41
{3.26, -2, 0.02, 0.081},//42
{3.26, -0.22, 0.02, 0.081},//43
{2.99, -0.22, 0.081, 0.02},//44
{2.97, -0.42, 0.081, 0.35},//45
{2.306, -0.42, 0.081, 0.35},//46
{2.286, -0.62, 0.081, 0.02},//47
{1.85, -0.62, 0.081, 0.02},//48
{1.846, -0.09, 0.02, 0.081},//49
{1.85, 0.21, 0.02, 0.081},//50
};

//Line type
int LineType[51] = {0,1,2,3,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,6,0,0,0,0,-1,0,0,0,0,-1,0,0,0,0};

//Kiem tra check line  +  check point
bool ForwardLine(Coord trans, Coord point)
{ 
  
  if(trans.x>=(point.x-point.dx) && trans.x<=(point.x+point.dx) && trans.z>=(point.z-point.dz) && trans.z<=(point.z+point.dz))
  {
    return true;
  }else return false;
}

//Rotation
Coord Rotation(const double *trans, Coord coord, const double *rotat)
{
  Coord Result;
  if(rotat[1]<0){
      Result.x =  trans[0]+ coord.x*cos(rotat[3]) - coord.z*sin(rotat[3]);
      Result.z =  trans[2]+ coord.x*sin(rotat[3]) + coord.z*cos(rotat[3]);
    }
    else
    {
      Result.x =  trans[0]+ coord.x*cos(-rotat[3]) - coord.z*sin(-rotat[3]);
      Result.z =  trans[2]+ coord.x*sin(-rotat[3]) + coord.z*cos(-rotat[3]);
    }
  return Result;
}

//Check out line o duong thang 
bool StraightLine(int index, Coord L, Coord R, double d)
{
  double a = CL[index-1].z - CL[index].z;
  double b = CL[index].x - CL[index-1].x;
  double c = -CL[index-1].x*a - CL[index-1].z*b;
  if((abs(a*L.x+b*L.z+c))/sqrt(a*a + b*b) > d || (abs(a*R.x+b*R.z+c)/sqrt(a*a+b*b)>d))
  {
    return true;
  }else return false;
}

//Kiem tra outline duong cong
bool CurveLine(Coord L, Coord R, double I1, double I2, double R1, double R2)
{
  if((L.x-I1)*(L.x-I1) + (L.z-I2)*(L.z-I2) > R1*R1 
  && (L.x-I1)*(L.x-I1) + (L.z-I2)*(L.z-I2) < R2*R2 
  && (R.x-I1)*(R.x-I1) + (R.z-I2)*(R.z-I2) > R1*R1 
  && (R.x-I1)*(R.x-I1) + (R.z-I2)*(R.z-I2) < R2*R2)  
  {  
      return false;
  }
  else return true;
}

//Check out Line
bool CheckOutLine(Coord L, Coord R, int numline)
{
  switch(LineType[numline-1]){
    case -1:
      return StraightLine(numline, L, R, 0.35);
    case 0:
      return StraightLine(numline, L, R, 0.15);
    case 1: 
      return CurveLine(L, R, -3.1, 1.33, 0.04, 0.33);
    case 2: 
      return CurveLine(L, R, -3.1, 0.95, 0.04, 0.33);
    case 3:
      return CurveLine(L, R, -3.1, 0.57, 0.04, 0.33);
    case 4: 
      return CurveLine(L, R, -1.6, -1.94, 0.235, 0.535);
    case 5: 
      return CurveLine(L, R, 1.522, -2.1013, 0.336, 0.64);
    default: return false;
    }
}



int main(int argc, char **argv) {
  const string argument[16] = {"DSC_ApolloCar_02", "THPT_NguyenHue_TeamKiNiem", "NH_smurfcat", "NH_a7_allinus", "crazy w1n",
  "NHH_STEM_Rhyder", "NguyenTeamDiVeHet", "2M2T", "NHH_STEM_HKN", "ClassicCar"};
  // create the Robot instance.
  Supervisor *supervisor = new Supervisor();
  Coord LPoint0, RPoint0, MPoint0;
  double startTime = 0.0;
  //Khai bao camera 
  //Index cho Check line va check point
  int CLine = 1;
  int CPoint = 0;
  //Get data from solid and car
  Node *R = supervisor->getFromDef("R");
  Node *L = supervisor->getFromDef("L");
  Node *M = supervisor->getFromDef("M");
  Node *Car = supervisor->getFromDef("Speed_Line_Follower_Robot_V4");
  //Node *cam = supervisor->getFromDef("cam");
  //Get translation of car and solid
  Field *R_trans_field = R->getField("translation");
  Field *L_trans_field = L->getField("translation");
  Field *M_trans_field = M->getField("translation");
  Field *Car_trans_field = Car->getField("translation");
  Field *CarController = Car->getField("controller");
  //Set up cho camera.
  /*Field *CamTrans = cam->getField("translation");
  Field *CamRots = cam->getField("rotation");
  double CamTranslation[3] = {0, 0, 0};
  double CamRotation[4] = {0, 0, 0, 0};*/
  //Get rotation of Car
  Field *Car_rot_field = Car->getField("rotation");
  
  //Vong lap
   while (supervisor->step(TIME_STEP) != -1) {
    supervisor->wwiSendText("T|" + to_string(supervisor->getTime()));
    string a = supervisor->wwiReceiveText();
    cout<<a<<endl;
    for(int i = 0; i<16; i++){
      if(a == argument[0])
      {
        CarController->setSFString("none");
        break;
      }
      if(a == argument[1])
      {
        CarController->setSFString("Controller_full_map");
        break;
      }
      if(a == argument[2])
      {
        CarController->setSFString("Bao_full_map");
        break;
      }
      if(a == argument[3])
      {
        CarController->setSFString("Bao_full_map_edited");
        break;
      }
      if(a == argument[4])
      {
        CarController->setSFString("Controller_full_map");
        break;
      }
      if(a == argument[5])
      {
        CarController->setSFString("none");
        break;
      }
    }
 
    //supervisor->wwiSendText("T|" + to_string(supervisor->getTime()));
    
   
    //Xu ly rotation o dau xe
    const double *Car_rot = Car_rot_field->getSFRotation();
    const double *Car_trans = Car_trans_field->getSFVec3f();
    
    //On the R
    LPoint0.x = -0.04;
    LPoint0.z = -0.14;
    Coord LPoint = Rotation(Car_trans,LPoint0,Car_rot);
    const double LTranslation[3] = {LPoint.x, Car_trans[1], LPoint.z};
    L_trans_field->setSFVec3f(LTranslation);

    //On the L
    RPoint0.x = 0.04;
    RPoint0.z = -0.14;
    Coord RPoint = Rotation(Car_trans,RPoint0,Car_rot);
    const double RTranslation[3] = {RPoint.x, Car_trans[1], RPoint.z};
    R_trans_field->setSFVec3f(RTranslation);

    //In the M
    MPoint0.x = 0;
    MPoint0.z = -0.14;
    Coord MPoint = Rotation(Car_trans,MPoint0,Car_rot);
    const double MTranslation[3] = {MPoint.x, Car_trans[1], MPoint.z};
    M_trans_field->setSFVec3f(MTranslation);
    
    
    
    //Check out line 
    if (CheckOutLine(LPoint, RPoint, CLine))
    {
     
      supervisor->wwiSendText("O|" + to_string(supervisor->getTime()));
      supervisor->simulationSetMode(webots::Supervisor::SimulationMode::SIMULATION_MODE_PAUSE);
      
    }

    //Kiem tra di qua Check line 
    if(ForwardLine(MPoint, CL[CLine])){
        
       CLine++;
    }
    
    /*if(CLine >= 42)
       {          
          Coord xedua;
          xedua.x = 0;
          xedua.z = 0;
          Coord Car_coord = Rotation(Car_trans,xedua,Car_rot);
          if(ForwardLine(Car_coord, CL[41]))
          {
            if(CPoint == 4)
            {
               if(startTime == 0.0)
              {
                startTime = supervisor->getTime();
              }
              cout<<startTime;
              if (supervisor->getTime() - startTime >= 06.00) {
                cout<<"\nHoan thanh";
                cout<<"\nHoan thanh";
                cout<<"\nHoan thanh";
                cout<<"\nHoan thanh";
                supervisor->wwiSendText("C.0" + to_string(CPoint+1) + "|" + to_string(supervisor->getTime()));
                supervisor->simulationSetMode(webots::Supervisor::SimulationMode::SIMULATION_MODE_PAUSE);
              }
              if (CLine == 44)
              {
                supervisor->wwiSendText("O|" + to_string(supervisor->getTime()));
                cout<<"\nOutline";
                supervisor->simulationSetMode(webots::Supervisor::SimulationMode::SIMULATION_MODE_PAUSE);
              }
            }
            
          }
   
        }*/
   
  
    //Kiem tra di qua Checkpoint
    if (ForwardLine(MPoint, CP[CPoint]))
    {
      if(CPoint<4)
      {
        supervisor->wwiSendText("C." + to_string(CPoint+1) + "|" + to_string(supervisor->getTime()));
        fflush(stdout);
        CPoint++;
      }
      
    }
    //Quy dinh goc chuyen camera.
    /*if(CLine == 1)
    {
      CamTranslation[0] = -4.19518;
      CamTranslation[1] = 0.526055;
      CamTranslation[2] = 1.03694;
      CamRotation[0] = 0.00501374;
      CamRotation[1] = -0.992721;
      CamRotation[2] = -0.12033;
      CamRotation[3] = 3.09533;
      CamTrans->setSFVec3f(CamTranslation);
      CamRots->setSFRotation(CamRotation);
    }
    if(CLine == 3)
    {
      CamTranslation[0] = -3.86154;
      CamTranslation[1] = 1.43229;
      CamTranslation[2] = 1.95288;
      CamRotation[0] = 0.188947;
      CamRotation[1] = 0.953241;
      CamRotation[2] = 0.23586;
      CamRotation[3] = -2.12815;
      CamTrans->setSFVec3f(CamTranslation);
      CamRots->setSFRotation(CamRotation);
    }
    
    if(CLine == 5)
    {
      CamTranslation[0] = 4.59356;
      CamTranslation[1] = 1.63659;
      CamTranslation[2] = 2.02519;
      CamRotation[0] = -0.255465;
      CamRotation[1] = 0.935091;
      CamRotation[2] = 0.245646;
      CamRotation[3] = 1.34589;
      CamTrans->setSFVec3f(CamTranslation);
      CamRots->setSFRotation(CamRotation);
    }
    if(CLine == 15)
    {
      CamTranslation[0] = -0.38103;
      CamTranslation[1] = 2.37438;
      CamTranslation[2] = 1.65994;
      CamRotation[0] = 0.609031;
      CamRotation[1] = 0.715395;
      CamRotation[2] = 0.342478;
      CamRotation[3] = -1.12604;
      CamTrans->setSFVec3f(CamTranslation);
      CamRots->setSFRotation(CamRotation);
    }
    if(CLine == 17)
    {
      CamTranslation[0] = 5.52191;
      CamTranslation[1] = 1.51643;
      CamTranslation[2] = 0.726193;
      CamRotation[0] = 0.23316;
      CamRotation[1] = -0.934358;
      CamRotation[2] = -0.269466;
      CamRotation[3] = -1.53467;
      CamTrans->setSFVec3f(CamTranslation);
      CamRots->setSFRotation(CamRotation);
    }
    if(CLine == 19)
    {
      CamTranslation[0] = 3.29367;
      CamTranslation[1] = 2.01217;
      CamTranslation[2] = 0.515339;
      CamRotation[0] = 0.42815;
      CamRotation[1] = -0.801944;
      CamRotation[2] = -0.416622;
      CamRotation[3] = -1.55423;
      CamTrans->setSFVec3f(CamTranslation);
      CamRots->setSFRotation(CamRotation);
    }
    if(CLine == 23)
    {
      CamTranslation[0] = 0.70778;
      CamTranslation[1] = 2.00136;
      CamTranslation[2] = 1.7154;
      CamRotation[0] = 0.408123;
      CamRotation[1] = -0.890044;
      CamRotation[2] = -0.20312;
      CamRotation[3] = -1.12897;
      CamTrans->setSFVec3f(CamTranslation);
      CamRots->setSFRotation(CamRotation);
    }
    if(CLine == 25)
    {
      CamTranslation[0] = -1.9367;
      CamTranslation[1] = 2.03581;
      CamTranslation[2] = 1.71497;
      CamRotation[0] = 0.956881;
      CamRotation[1] = -0.267677;
      CamRotation[2] = 0.112813;
      CamRotation[3] = -0.677025;
      CamTrans->setSFVec3f(CamTranslation);
      CamRots->setSFRotation(CamRotation);
    }
    if(CLine == 28)
    {
      CamTranslation[0] = -3.25375;
      CamTranslation[1] = 2.50058;
      CamTranslation[2] = -0.505339;
      CamRotation[0] = 0.58019;
      CamRotation[1] = 0.551656;
      CamRotation[2] = 0.599212;
      CamRotation[3] = -2.12583;
      CamTrans->setSFVec3f(CamTranslation);
      CamRots->setSFRotation(CamRotation);
    }
    if(CLine == 35)
    {
      CamTranslation[0] = -0.849429;
      CamTranslation[1] = 2.08782;
      CamTranslation[2] = -1.80297;
      CamRotation[0] = 0.58019;
      CamRotation[1] = 0.551656;
      CamRotation[2] = 0.599212;
      CamRotation[3] = -2.12583;
      CamTrans->setSFVec3f(CamTranslation);
      CamRots->setSFRotation(CamRotation);
    }
    if(CLine == 39)
    {
      CamTranslation[0] = -1.39513;
      CamTranslation[1] = 1.6367;
      CamTranslation[2] = -1.55793;
      CamRotation[0] = 0.4063;
      CamRotation[1] = 0.764231;
      CamRotation[2] = 0.50087;
      CamRotation[3] = -1.68791;
      CamTrans->setSFVec3f(CamTranslation);
      CamRots->setSFRotation(CamRotation);
    }*/
    

   }
   
  // Enter here exit cleanup code.

  delete supervisor;
  return 0;
}
