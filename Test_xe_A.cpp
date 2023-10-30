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
Coord CL[53] = {{-2.86, 1.1, 0.02, 0.081},//1
{-2.856, 5, 0.02, 0.081},//2
{-2.16, 5.01, 0.02, 0.081},//3
{-1.82, 5.01, 0.02, 0.081},//4
{-1.48, 5.01, 0.02, 0.081},//5
{-1.14, 5.01, 0.02, 0.081},//6
{-0.795, 5.01, 0.02, 0.081},//7
{-0.455, -1.48, 0.02, 0.081},//8
{0.245, 5.02, 0.02, 0.081},//9
{0.245, 4.65, 0.02, 0.081},//10
{2.305, 4.65, 0.081, 0.02},//11
{2.455, 4.49, 0.02, 0.081},//12
{2.455, 3.93, 0.02, 0.081},//13
{1.075, 3.93,0.081, 0.02},//14
{1.075, 3.59, 0.02, 0.081},//15
{0.715, 3.59,0.081, 0.02},//16
{0.715, 3.93, 0.02, 0.081},//17
{0.355, 3.93,0.081, 0.02},//18
{0.355, 3.59, 0.02, 0.081},//19
{-0.005, 3.59,0.081, 0.02},//20
{-0.005, 3.93, 0.02, 0.081},//21
{-1.315, 3.93, 0.081, 0.02},//22
{-1.665, 3.58, 0.02, 0.081},//23
{-1.665, 1.75, 0.02, 0.081},//24
{-1.325, 1.41, 0.02, 0.081},//25
{-1.545, 1.18, 0.02, 0.081},//26
{-1.325, 0.96, 0.02, 0.081},//27
{-1.545, 0.73, 0.02, 0.081},//28
{-1.325, 0.51, 0.02, 0.081},//29
{-1.545, 0.28, 0.02, 0.081},//30
{-1.325, 0.05, 0.02, 0.081},//31
{-1.665, -0.29, 0.02, 0.081},//32
{-1.665, -1.46, 0.02, 0.081},//33
{2.415, -1.46, 0.081, 0.02},//34
{2.415, -0.82, 0.02, 0.081},//35
{2.415, -0.11, 0.02, 0.081},//36
{2.415, 0.43, 0.02, 0.081},//37
{3.62, 0.435, 0.081, 0.02},//38
{3.62, 0.945, 0.02, 0.081},//39**
{3.77, 1.055, 0.3, 0.081},//40**
{4.02, 2.845, 0.02, 0.081},//41**
{3.87, 2.865, 0.3, 0.081},//42**
{3.77, 3.525, 0.02, 0.081},//43**
{3.63, 3.555, 0.02, 0.081},//44
{3.63, 4.145, 0.02, 0.081},//45
{3.97, 4.495, 0.081, 0.02},//46
{5.47, 4.495, 0.081, 0.02},//47
{5.47, 2.885, 0.02, 0.081},//48
{4.73, 2.885, 0.081, 0.02},//49
{4.75, 2.165, 0.02, 0.081},//50
{5.47, 2.155, 0.081, 0.02},//51
{5.47, 0.795, 0.02, 0.081},//52
{5.47, 0.495, 0.02, 0.081},//53
};
//Line type
int LineTypeA[53] = {0,1,2,3,4,5,6,7,0,0,8,0,0,0,0,0,0,0,0,0,0,9,0,0,0,0,0,0,0,0,0,0,0,0,0,10,0,0,0,0,-1,0,-1,0,0,11,0,0,0,0,0,0,0};

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
//Check out line duong cong parabol
bool CurveParabol(Coord L, Coord R, double I1, double I2, double R1, double R2)
{
  // Phương trình đường parabol: y = a*(x - I1)^2 + I2
  double a = 1.0 / ((R2 - R1) * (R2 - R1)); // Tính hệ số a dựa trên khoảng cách từ R1 đến R2

  double yL = a * (L.x - I1) * (L.x - I1) + I2; // Tọa độ y của L trên đường parabol
  double yR = a * (R.x - I1) * (R.x - I1) + I2; // Tọa độ y của R trên đường parabol

  if (L.z > yL && R.z > yR) // Kiểm tra nếu cả hai điểm đều nằm trên đường parabol
  {
    return false; // Xe không đi ra khỏi đường parabol
  }
  else
  {
    return true; // Xe đi ra khỏi đường parabol
  }
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
  
  switch(LineTypeA[numline-1]){
    case -2: 
      return CurveParabol(L, R, -1.985, 5.11, 0.04, 0.58);
    case -1: 
      return StraightLine(numline, L, R, 0.3);
    case 0:
      return StraightLine(numline, L, R, 0.15);
    case 1: 
      return CurveLine(L, R, -2.504, 5, 0.198, 0.49);
    case 2: 
      return CurveLine(L, R, -1.984, 5.01, 0.02, 0.32);
    case 3:
      return CurveLine(L, R, -1.645, 5.00111, 0.02, 0.32);
    case 4: 
      return CurveLine(L, R, -1.305, 5.01111, 0.02, 0.32);
    case 5: 
      return CurveLine(L, R, -0.966, 4.987, 0.02, 0.32);
    case 6:
      return CurveLine(L, R, -0.626, 4.999, 0.02, 0.32);
    case 7:
      return CurveLine(L, R, -0.106, 4.989, 0.2, 0.49);
     case 8:
       return CurveLine(L, R, -4.287, -0.89, 0.24, 0.53);
     case 9: 
       return CurveLine(L, R, -0.409, -1.6718, 0.2355, 0.53);
    default: return false;
    }
}



int main(int argc, char **argv) {
  const string argument[30] = {"01_NGUYEN_TEAM_DI_VE_HET", "02_crazy_w1n", "03_2M2T", "04_Fulushou",
                               "05_Cai_gi_cung_duoc", "06_Quai_xe_team", "NULL", "10_PRCar", "11_BIETDOIBAOTHU",
                               "13_Black_Eagle", "14_Tourismo", "15_G2D_Team","16_ClassicCar_1", "17_MIENTRUNGCHANCHAT_1",
                                "21_NHH_STEM_STRICKER","22_NHH_STEM_HKN", "23_NHH_STEM_Rhyder", "24_NHH_STEM_DOUBLE2T",
                                "28_DKT", "30_8_Bit", "31_Pencil","32_THPT_NguyenHue_TeamKiNiem", "33_B.A.D", "37_Thpt_Nguyen_Hue_Beluaga",
                                "38_THPTNguyenHue_ChampionUITCar", "39_Team_6", "40_THPT_NguyenHue_a7_allinus", "41_THPT_NguyenHue_welivewelovewelie",
                                 "46_DSC_Apollocar_02", "48_ComGaXoiMo"};
  string start = "Start";
  // create the Robot instance.
  Supervisor *supervisor = new Supervisor();
  Coord LPoint0_A, RPoint0_A, MPoint0_A;
  double startTimeA = 0.0;
  //Khai bao camera 
  //Camera* cm;
  //cm = supervisor->getCamera("camera");
  //cm->enable(TIME_STEP);
  //Index cho Check line va check point
  int CLine_A = 1;
  int CPoint_A = 0;
  //Get data from solid and car
  Node *R_A = supervisor->getFromDef("R_A");
  Node *L_A = supervisor->getFromDef("L_A");
  Node *M_A = supervisor->getFromDef("M_A");
  Node *Car_A = supervisor->getFromDef("Speed_Line_Follower_Robot_V4_A");
  //Node *cam = supervisor->getFromDef("cam");
  //Get translation of car and solid
  Field *R_trans_field_A = R_A->getField("translation");
  Field *L_trans_field_A = L_A->getField("translation");
  Field *M_trans_field_A = M_A->getField("translation");
  Field *Car_trans_field_A = Car_A->getField("translation");
  Field *CarControllerA = Car_A->getField("controller");
  //Set up cho camera.
  //Field *CamTrans = cam->getField("translation");
  //Field *CamRots = cam->getField("rotation");
  //double CamTranslation[3] = {0, 0, 0};
  //double CamRotation[4] = {0, 0, 0, 0};
  //Get rotation of Car
  Field *Car_rot_field_A = Car_A->getField("rotation");
  //Vong lap
   while (supervisor->step(TIME_STEP) != -1) {
    //cm->getImage();
    //supervisor->wwiSendText("T|" + to_string(supervisor->getTime()));   FIX XONG PHẢI MỞ
    
   
    //****************Xu ly rotation o dau xe A
    const double *Car_rot_A = Car_rot_field_A->getSFRotation();
    const double *Car_trans_A = Car_trans_field_A->getSFVec3f();
    
    //On the R
    LPoint0_A.x = -0.04;
    LPoint0_A.z = -0.14;
    Coord LPointA = Rotation(Car_trans_A,LPoint0_A,Car_rot_A);
    const double LTranslation_A[3] = {LPointA.x, Car_trans_A[1], LPointA.z};
    L_trans_field_A->setSFVec3f(LTranslation_A);

    //On the L
    RPoint0_A.x = 0.04;
    RPoint0_A.z = -0.14;
    Coord RPointA = Rotation(Car_trans_A,RPoint0_A,Car_rot_A);
    const double RTranslationA[3] = {RPointA.x, Car_trans_A[1], RPointA.z};
    R_trans_field_A->setSFVec3f(RTranslationA);

    //In the M
    MPoint0_A.x = 0;
    MPoint0_A.z = -0.14;
    Coord MPointA = Rotation(Car_trans_A,MPoint0_A,Car_rot_A);
    const double MTranslation_A[3] = {MPointA.x, Car_trans_A[1], MPointA.z};
    M_trans_field_A->setSFVec3f(MTranslation_A);
    
    
    
    //Check out line 
    if (CheckOutLine(LPointA, RPointA, CLine_A))
    {
     
      supervisor->wwiSendText("O|" + to_string(supervisor->getTime()));
      supervisor->simulationSetMode(webots::Supervisor::SimulationMode::SIMULATION_MODE_PAUSE);
      
    }

    //Kiem tra di qua Check line 
    if(ForwardLine(MPointA, CL[CLine_A])){

       CLine_A++;
    }
    
  
    //Kiem tra di qua Checkpoint
    if (ForwardLine(MPointA, CP[CPoint_A]))
    {
      if(CPoint_A<4)
      {
        supervisor->wwiSendText("C." + to_string(CPoint_A+1) + "|" + to_string(supervisor->getTime()));
        fflush(stdout);
        CPoint_A++;
      }
      
    }

    if(CLine_A >= 43)
    {          
      Coord xedua_A;
      xedua_A.x = 0;
      xedua_A.z = 0;
      Coord Car_coord_A = Rotation(Car_trans_A,xedua_A,Car_rot_A);
      if(ForwardLine(Car_coord_A, CL[42]))
      {
        if(CPoint_A == 4)
        {
          if(startTimeA == 0.0)
          {
            startTimeA = supervisor->getTime();
          }
          cout<<startTimeA;
          if (supervisor->getTime() - startTimeA >= 05.00) {
            cout<<"\nHoan thanh";
            cout<<"\nHoan thanh";
            cout<<"\nHoan thanh";
            cout<<"\nHoan thanh";
            supervisor->wwiSendText("C.0" + to_string(CPoint_A+1) + "|" + to_string(supervisor->getTime()));
            supervisor->simulationSetMode(webots::Supervisor::SimulationMode::SIMULATION_MODE_PAUSE);
          }
          if (CLine_A == 45)
          {
            supervisor->wwiSendText("O|" + to_string(supervisor->getTime()));
            cout<<"\nOutline";
            supervisor->simulationSetMode(webots::Supervisor::SimulationMode::SIMULATION_MODE_PAUSE);
            }
        }
      }
    }
   }
   
  // Enter here exit cleanup code.

  delete supervisor;
  return 0;
}
