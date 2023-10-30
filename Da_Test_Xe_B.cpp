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

//Day la check line A
/*Coord CL_A[55] = {{1.09, -5.17,0.02,0.081},//0
{1.09, -1.12,0.02,0.081},//1
{1.47, -1.12,0.081,0.02},//2
{1.46, -0.777,0.081,0.02},//3
{1.425, -0.437,0.081,0.02},//4
{1.455, 0.263, 0.081, 0.02},//5
{1.455, 0.603, 0.081, 0.02},//6CP
{1.455, 1.303, 0.081 ,0.02},//7
{1.455, 1.643, 0.081 ,0.02},//8
{1.455, 1.983, 0.081 ,0.02},//9
{-1.325, 1.983, 0.081 ,0.02},//10
{-1.325, 2.373, 0.02 ,0.081},//11
{-1.465, 2.503, 0.081 ,0.02},//12
{-3.175, 2.503, 0.081 ,0.02},//13
{-3.175, 2.163, 0.02 ,0.081},//14
{-3.535, 2.163, 0.081 ,0.02},//15
{-3.535, 2.502, 0.02 ,0.081},//16
{-3.895, 2.502, 0.081 ,0.02},//17
{-3.895, 2.162, 0.02 ,0.081},//18
{-4.255, 2.162, 0.081 ,0.02},//19
{-4.255, 2.502, 0.02 ,0.081},//20
{-4.885, 2.502, 0.081 ,0.02},//21
{-5.24, 2.142, 0.02 ,0.081},//22
{-5.24, -0.458, 0.02 ,0.081},//23
{-4.9, -0.798, 0.02 ,0.081},//24
{-5.11, -1.028, 0.02 ,0.081},//25
{-4.9, -1.258, 0.02 ,0.081},//26
{-5.12, -1.478, 0.02 ,0.081},//27
{-4.9, -1.708, 0.02 ,0.081},//28
{-5.12, -1.938, 0.02 ,0.081},//29
{-4.9, -2.158, 0.02 ,0.081},//30
{-5.24, -2.508, 0.02 ,0.081},//31
{-5.24, -3.292, 0.02 ,0.081},//32
{-4.12, -3.292, 0.081 ,0.02},//33
{-3.77, -3.652, 0.02 ,0.081},//34
{-3.77, -5.389, 0.02 ,0.081},//35
{-3.16, -5.389, 0.081 ,0.02},//36
{-3.14, -5.189, 0.081 ,0.35},//37
{-2.48, -5.189, 0.081 ,0.35},//38
{-2.47, -4.989, 0.081 ,0.02},//39
{-1.36, -4.989, 0.081 ,0.02},//40
{-1.34, -5.189, 0.081 ,0.35},//41
{-0.6763, -5.189, 0.081 ,0.35},//42
{-0.6663, -5.389, 0.081 ,0.02},//43;
{-0.0093, -5.389, 0.081 ,0.02},//44;
{-0.0093, -2.994, 0.02 ,0.081},//45;
{-0.6593, -2.994, 0.081 ,0.02},//46;
{-1.3693, -2.994, 0.081 ,0.02},//47;
{-2.8593, -2.994, 0.081, 0.02},//48;
{-2.8593, -1.434, 0.02 ,0.081},//49;
{-2.1393, -1.434, 0.081, 0.02},//50;
{-2.1393, -0.714, 0.02 ,0.081},//51;
{-2.8593, -0.714, 0.081, 0.02},//52;
{-2.8593, 0.496, 0.02 ,0.081},//53;
{-2.8593, 0.796, 0.02, 0.081}//54;
};*/

//Line type A
//int LineTypeA[54] = {0,0,1,2,3,4,5,6,7,0,0,8,0,0,0,0,0,0,0,0,0,9,0,0,0,0,0,0,0,0,0,0,0,10,0,0,0,-1,0,0,0,-1,0,0,0,0,11,0,0,0,0,0,0,0};

//Check line B
Coord CL[55] = {{-2.856, 1.1, 0.02, 0.081},//0
{-2.856, 4.98, 0.02, 0.081},//1
{-2.156, 5.02, 0.02, 0.081},//2
{-1.816, 5.02, 0.02, 0.081},//3
{-1.476, 5.02, 0.02, 0.081},//4
{-1.136, 5.02, 0.02, 0.081},//5
{-0.796, 5.02, 0.02, 0.081},//6
{-0.456, 5.02, 0.081, 0.02},//7
{0.244, 5.03, 0.02, 0.081},//8
{0.244, 4.648, 0.02, 0.081},//9
{2.264, 4.648, 0.081, 0.02},//10
{2.457, 4.488, 0.02, 0.081},//11
{2.457, 3.931, 0.02, 0.081},//12
{1.077, 3.931, 0.081, 0.02},//13
{1.077, 3.591, 0.02, 0.081},//14
{0.717, 3.591, 0.081, 0.02},//15
{0.717, 3.931, 0.02, 0.081},//16
{0.357, 3.931, 0.081, 0.02},//17
{0.357, 3.591, 0.02, 0.081},//18
{-0.003, 3.591, 0.081, 0.02},//19
{-0.003, 3.93, 0.02, 0.081},//20
{-1.313, 3.93, 0.081, 0.02},//21
{-1.666, 3.571, 0.081, 0.02},//22
{-1.666, 1.761, 0.02, 0.081},//23
{-1.326, 1.401, 0.02, 0.081},//24
{-1.546, 1.181, 0.02, 0.081},//25
{-1.326, 0.951, 0.02, 0.081},//26
{-1.536, 0.731, 0.02, 0.081},//27
{-1.326, 0.501, 0.02, 0.081},//28
{-1.546, 0.281, 0.02, 0.081},//29
{-1.326, 0.051, 0.02, 0.081},//30
{-1.666, -0.289, 0.02, 0.081},//31
{-1.666, -1.462, 0.02, 0.081},//32
{2.42, -1.462, 0.081, 0.02},//33
{2.42, -0.812, 0.02, 0.081},//34
{2.42, -0.102, 0.02, 0.081},//35
{2.42, 0.435, 0.02, 0.081},//36
{3.62, 0.435, 0.081, 0.02},//37
{3.62, 1.04, 0.02, 0.081},//38
{3.82, 1.065, 0.35, 0.081},//39
{3.82, 1.73, 0.35, 0.081},//40
{4.02, 1.739, 0.02, 0.081},//41
{4.02, 2.845, 0.02, 0.081},//42
{3.82, 2.865, 0.35, 0.081},//43
{3.82, 3.53, 0.35, 0.081},//44
{3.621, 3.539, 0.02, 0.081},//45
{3.621, 4.139, 0.02, 0.081},//46
{3.981, 4.496, 0.081, 0.02},//47
{5.471, 4.496, 0.081, 0.02},//48
{5.471, 2.886, 0.02, 0.081},//49
{4.751, 2.886, 0.081, 0.02},//50
{4.751, 2.166, 0.02, 0.081},//51
{5.471, 2.166, 0.081, 0.02},//52
{5.471, 0.796, 0.02, 0.081},//53
{5.471, 0.506, 0.02, 0.081},//54
};

//Line type B
int LineType[54] = {0,1,2,3,4,5,6,7,0,0,8,0,0,0,0,0,0,0,0,0,0,9,0,0,0,0,0,0,0,0,0,0,0,0,10,0,0,0,0,-1,0,0,0,-1,0,0,11,0,0,0,0,0,0,0};


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
  cout<<LineType[numline-1]<<endl;
  cout<<"  "<<numline-1; 
  switch(LineType[numline-1]){
    case -1:
      return StraightLine(numline, L, R, 0.35);
    case 0:
      return StraightLine(numline, L, R, 0.15);
    case 1: 
      return CurveLine(L, R, -2.507, 5, 0.02, 0.5);
    case 2: 
      return CurveLine(L, R, -1.9866, 5.03, 0.024, 0.343);
    case 3:
      return CurveLine(L, R, -1.6466, 4.98, 0.024, 0.342);
    case 4: 
      return CurveLine(L, R, -1.3066, 5.03, 0.023, 0.342);
    case 5: 
      return CurveLine(L, R, -0.9666, 4.97, 0.024, 0.34);
    case 6:
      return CurveLine(L, R, -0.6266, 5.02, 0.024, 0.34);
    case 7:
      return CurveLine(L, R, -0.1066, 5, 0.2, 0.5);
    case 8:
      return CurveLine(L, R, 2.23, 4.417, 0.08, 0.4);
    case 9: 
      return CurveLine(L, R, -1.663, 3.933, 0.2, 0.5);
    case 10: 
      return CurveLine(L, R, 2.42, -0.463, 0.2, 0.5);
    case 11: 
      return CurveLine(L, R, 3.62, 4.497, 0.2, 0.5);
    default: return false;

    }
}



int main(int argc, char **argv) {
  //const string argument[6] = {"c1", "c2", "c3", "c4", "c5", "c6"};
  // create the Robot instance.
  Supervisor *supervisor = new Supervisor();
  Coord LPoint0, RPoint0, MPoint0;
  double startTime = 0.0;
  //Index cho Check line va check point
  int CLine = 1;
  int CPoint = 0;
  //Get data from solid and car
  Node *R = supervisor->getFromDef("R");
  Node *L = supervisor->getFromDef("L");
  Node *M = supervisor->getFromDef("M");
  Node *Car = supervisor->getFromDef("Speed_Line_Follower_Robot_V4_B");
  //Get translation of car and solid
  Field *R_trans_field = R->getField("translation");
  Field *L_trans_field = L->getField("translation");
  Field *M_trans_field = M->getField("translation");
  Field *Car_trans_field = Car->getField("translation");
  Field *CarController = Car->getField("controller");
  //Get rotation of Car
  Field *Car_rot_field = Car->getField("rotation");
  
  //Vong lap
   while (supervisor->step(TIME_STEP) != -1) {
    supervisor->wwiSendText("T|" + to_string(supervisor->getTime()));
    
   
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
    
    if(CLine >= 42)
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
   
        }
   
  
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
   }
   
  // Enter here exit cleanup code.

  delete supervisor;
  return 0;
}
