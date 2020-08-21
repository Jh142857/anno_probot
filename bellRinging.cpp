#include <string>
#include <ros/ros.h>
#include <iostream>
#include <time.h>
#include "vector"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64MultiArray.h"
#include "controller_manager_msgs/SwitchController.h"
#include "controller_manager_msgs/ListControllers.h"
#include "ikfast.h"
#include "probot_anno_manipulator_ikfast_moveit_plugin.cpp"
using namespace std;
void planOnce(double theta[],double w[],double v[]){
double a1 = theta[0], a2 = theta[1], a3 = theta[2], a4 = theta[3], a5 = theta[4], a6 = theta[5];
double v1 = v[0], v2 = v[1], v3 = v[2], v4 = v[3], v5 = v[4], v6 = v[5];
w[0] = (10000*v2*cos(a1))/(2289*cos(a2 + a3) - 2250*sin(a2)) - (10000*v1*sin(a1))/(2289*cos(a2 + a3) - 2250*sin(a2)) - (v6*(550*cos(a2)*cos(a3)*sin(a5) - 550*sin(a2)*sin(a3)*sin(a5) + 550*cos(a2)*cos(a4)*cos(a5)*sin(a3) + 550*cos(a3)*cos(a4)*cos(a5)*sin(a2)))/(2289*cos(a2 + a3) - 2250*sin(a2)) + (550*v4*cos(a1)*(cos(a2)*sin(a3)*sin(a5) + cos(a3)*sin(a2)*sin(a5) - cos(a2)*cos(a3)*cos(a4)*cos(a5) + cos(a4)*cos(a5)*sin(a2)*sin(a3)))/(2289*cos(a2 + a3) - 2250*sin(a2)) + (550*v5*sin(a1)*(cos(a2)*sin(a3)*sin(a5) + cos(a3)*sin(a2)*sin(a5) - cos(a2)*cos(a3)*cos(a4)*cos(a5) + cos(a4)*cos(a5)*sin(a2)*sin(a3)))/(2289*cos(a2 + a3) - 2250*sin(a2));

w[1] = (11*v5*cos(a5)*(750*cos(a1)*cos(a4)*sin(a2) - 750*cos(a3)*sin(a1)*sin(a4) - 763*cos(a1)*cos(a2)*cos(a3)*cos(a4) + 763*cos(a1)*cos(a4)*sin(a2)*sin(a3) - 763*cos(a2)*sin(a1)*sin(a2)*sin(a4) - 763*cos(a3)*sin(a1)*sin(a3)*sin(a4) + 750*pow(cos(a2),2)*cos(a3)*sin(a1)*sin(a4) - 750*cos(a2)*sin(a1)*sin(a2)*sin(a3)*sin(a4) + 1526*cos(a2)*pow(cos(a3),2)*sin(a1)*sin(a2)*sin(a4) + 1526*pow(cos(a2),2)*cos(a3)*sin(a1)*sin(a3)*sin(a4)))/(45*cos(a3)*(763*cos(a2 + a3) - 750*sin(a2))) - (40*v1*cos(a1)*((763*cos(2*a2 + 2*a3))/2 + 375*sin(a3) - 375*sin(2*a2 + a3) + 763/2))/(9*cos(a3)*(763*cos(a2 + a3) - 750*sin(a2))) - (40*v2*sin(a1)*((763*cos(2*a2 + 2*a3))/2 + 375*sin(a3) - 375*sin(2*a2 + a3) + 763/2))/(9*cos(a3)*(763*cos(a2 + a3) - 750*sin(a2))) - (11*v4*cos(a5)*(750*cos(a1)*cos(a3)*sin(a4) + 750*cos(a4)*sin(a1)*sin(a2) - 763*cos(a2)*cos(a3)*cos(a4)*sin(a1) + 763*cos(a1)*cos(a2)*sin(a2)*sin(a4) + 763*cos(a1)*cos(a3)*sin(a3)*sin(a4) + 763*cos(a4)*sin(a1)*sin(a2)*sin(a3) - 750*cos(a1)*pow(cos(a2),2)*cos(a3)*sin(a4) + 750*cos(a1)*cos(a2)*sin(a2)*sin(a3)*sin(a4) - 1526*cos(a1)*cos(a2)*pow(cos(a3),2)*sin(a2)*sin(a4) - 1526*cos(a1)*pow(cos(a2),2)*cos(a3)*sin(a3)*sin(a4)))/(45*cos(a3)*(763*cos(a2 + a3) - 750*sin(a2))) - (40*v3*sin(a2 + a3))/(9*cos(a3)) - (11*v6*cos(a5)*sin(a4)*(763*cos(2*a2 + 2*a3) + 750*sin(a3) - 750*sin(2*a2 + a3) + 763))/(90*cos(a3)*(763*cos(a2 + a3) - 750*sin(a2)));

w[2] = (v3*(30520*sin(a2 + a3) + 30000*cos(a2)))/(6867*cos(a3)) + (v5*(6294750*cos(a3)*cos(a5)*sin(a1)*sin(a4) - 6187500*cos(a1)*cos(a3)*sin(a2)*sin(a5) - 12589500*cos(a1)*cos(a4)*cos(a5)*sin(a2) + 6294750*cos(a1)*cos(a2)*pow(cos(a3),2)*sin(a5) + 6403859*cos(a1)*cos(a2)*cos(a3)*cos(a4)*cos(a5) - 12591359*cos(a1)*cos(a4)*cos(a5)*sin(a2)*sin(a3) - 6294750*cos(a1)*cos(a3)*sin(a2)*sin(a3)*sin(a5) + 12591359*cos(a2)*cos(a5)*sin(a1)*sin(a2)*sin(a4) + 6403859*cos(a3)*cos(a5)*sin(a1)*sin(a3)*sin(a4) + 6294750*cos(a1)*pow(cos(a3),2)*cos(a4)*cos(a5)*sin(a2) - 12589500*pow(cos(a2),2)*cos(a3)*cos(a5)*sin(a1)*sin(a4) + 6294750*cos(a1)*cos(a2)*cos(a3)*cos(a4)*cos(a5)*sin(a3) + 12589500*cos(a2)*cos(a5)*sin(a1)*sin(a2)*sin(a3)*sin(a4) - 12807718*cos(a2)*pow(cos(a3),2)*cos(a5)*sin(a1)*sin(a2)*sin(a4) - 12807718*pow(cos(a2),2)*cos(a3)*cos(a5)*sin(a1)*sin(a3)*sin(a4)))/(34335*cos(a3)*(763*cos(a2 + a3) - 750*sin(a2))) + (v4*(6294750*cos(a1)*cos(a3)*cos(a5)*sin(a4) + 12589500*cos(a4)*cos(a5)*sin(a1)*sin(a2) + 6187500*cos(a3)*sin(a1)*sin(a2)*sin(a5) - 6294750*cos(a2)*pow(cos(a3),2)*sin(a1)*sin(a5) - 6403859*cos(a2)*cos(a3)*cos(a4)*cos(a5)*sin(a1) + 12591359*cos(a1)*cos(a2)*cos(a5)*sin(a2)*sin(a4) + 6403859*cos(a1)*cos(a3)*cos(a5)*sin(a3)*sin(a4) + 12591359*cos(a4)*cos(a5)*sin(a1)*sin(a2)*sin(a3) + 6294750*cos(a3)*sin(a1)*sin(a2)*sin(a3)*sin(a5) - 12589500*cos(a1)*pow(cos(a2),2)*cos(a3)*cos(a5)*sin(a4) - 6294750*pow(cos(a3),2)*cos(a4)*cos(a5)*sin(a1)*sin(a2) - 6294750*cos(a2)*cos(a3)*cos(a4)*cos(a5)*sin(a1)*sin(a3) + 12589500*cos(a1)*cos(a2)*cos(a5)*sin(a2)*sin(a3)*sin(a4) - 12807718*cos(a1)*cos(a2)*pow(cos(a3),2)*cos(a5)*sin(a2)*sin(a4) - 12807718*cos(a1)*pow(cos(a2),2)*cos(a3)*cos(a5)*sin(a3)*sin(a4)))/(34335*cos(a3)*(763*cos(a2 + a3) - 750*sin(a2))) + (40*v1*cos(a1)*((582169*cos(2*a2 + 2*a3))/2 - 281250*cos(2*a2) + 572250*sin(a3) - 572250*sin(2*a2 + a3) + 1144669/2))/(6867*cos(a3)*(763*cos(a2 + a3) - 750*sin(a2))) + (40*v2*sin(a1)*((582169*cos(2*a2 + 2*a3))/2 - 281250*cos(2*a2) + 572250*sin(a3) - 572250*sin(2*a2 + a3) + 1144669/2))/(6867*cos(a3)*(763*cos(a2 + a3) - 750*sin(a2))) + (11*v6*cos(a5)*sin(a4)*(582169*cos(2*a2 + 2*a3) - 562500*cos(2*a2) + 1144500*sin(a3) - 1144500*sin(2*a2 + a3) + 1144669))/(68670*cos(a3)*(763*cos(a2 + a3) - 750*sin(a2)));

w[3] = (v1*(7630000*pow(cos(a3),2)*cos(a5)*sin(a1)*sin(a2) - 7630000*cos(a1)*sin(a3)*sin(a4)*sin(a5) - 7500000*cos(a1)*sin(a4)*sin(a5) + 7500000*cos(a1)*pow(cos(a2),2)*sin(a4)*sin(a5) + 7630000*cos(a2)*cos(a3)*cos(a5)*sin(a1)*sin(a3) + 7630000*cos(a2)*pow(cos(a3),2)*cos(a4)*sin(a1)*sin(a5) + 7630000*cos(a1)*pow(cos(a2),2)*sin(a3)*sin(a4)*sin(a5) + 7630000*cos(a1)*cos(a2)*cos(a3)*sin(a2)*sin(a4)*sin(a5) - 7630000*cos(a3)*cos(a4)*sin(a1)*sin(a2)*sin(a3)*sin(a5)))/(2289*cos(a3)*cos(a5)*(763*cos(a2 + a3) - 750*sin(a2))) - (v4*(1746507*cos(a1)*pow(cos(a3),3)*cos(a5) - 1746507*cos(a1)*cos(a3)*cos(a5) - 3493014*cos(a1)*pow(cos(a2),2)*pow(cos(a3),3)*cos(a5) - 1716750*cos(a1)*cos(a3)*cos(a5)*sin(a3) + 412500*cos(a3)*sin(a1)*sin(a2)*sin(a4) + 1746507*cos(a1)*pow(cos(a2),2)*cos(a3)*cos(a5) - 419650*cos(a1)*pow(cos(a3),2)*cos(a4)*sin(a3) - 1716750*cos(a1)*pow(cos(a3),2)*cos(a4)*sin(a5) + 419650*cos(a1)*pow(cos(a3),3)*cos(a5)*sin(a5) - 419650*cos(a2)*pow(cos(a3),2)*sin(a1)*sin(a4) + 839300*cos(a1)*pow(cos(a2),2)*pow(cos(a3),2)*cos(a4)*sin(a3) + 1716750*cos(a1)*pow(cos(a2),2)*pow(cos(a3),2)*cos(a4)*sin(a5) + 839300*cos(a1)*pow(cos(a3),2)*cos(a4)*pow(cos(a5),2)*sin(a3) - 839300*cos(a1)*pow(cos(a2),2)*pow(cos(a3),3)*cos(a5)*sin(a5) + 419650*cos(a1)*pow(cos(a3),3)*pow(cos(a4),2)*cos(a5)*sin(a5) + 419650*cos(a2)*pow(cos(a3),2)*pow(cos(a5),2)*sin(a1)*sin(a4) - 419650*cos(a1)*cos(a2)*cos(a3)*cos(a4)*sin(a2) + 412500*cos(a1)*cos(a2)*cos(a5)*sin(a2)*sin(a5) + 419650*cos(a3)*sin(a1)*sin(a2)*sin(a3)*sin(a4) + 1716750*cos(a3)*sin(a1)*sin(a2)*sin(a4)*sin(a5) + 1716750*cos(a1)*cos(a2)*pow(cos(a3),2)*cos(a5)*sin(a2) + 839300*cos(a1)*cos(a2)*pow(cos(a3),3)*cos(a4)*sin(a2) + 1716750*cos(a1)*pow(cos(a2),2)*cos(a3)*cos(a5)*sin(a3) - 419650*cos(a1)*cos(a3)*pow(cos(a4),2)*cos(a5)*sin(a5) - 1746507*cos(a1)*pow(cos(a3),2)*cos(a4)*sin(a3)*sin(a5) - 1746507*cos(a2)*pow(cos(a3),2)*sin(a1)*sin(a4)*sin(a5) - 412500*cos(a3)*pow(cos(a5),2)*sin(a1)*sin(a2)*sin(a4) - 1746507*cos(a1)*cos(a2)*cos(a3)*cos(a4)*sin(a2)*sin(a5) - 1678600*cos(a1)*pow(cos(a2),2)*pow(cos(a3),2)*cos(a4)*pow(cos(a5),2)*sin(a3) - 839300*cos(a1)*pow(cos(a2),2)*pow(cos(a3),3)*pow(cos(a4),2)*cos(a5)*sin(a5) + 419650*cos(a1)*cos(a2)*cos(a5)*sin(a2)*sin(a3)*sin(a5) + 419650*cos(a4)*cos(a5)*sin(a1)*sin(a2)*sin(a4)*sin(a5) + 1746507*cos(a3)*sin(a1)*sin(a2)*sin(a3)*sin(a4)*sin(a5) + 839300*cos(a1)*cos(a2)*cos(a3)*cos(a4)*pow(cos(a5),2)*sin(a2) + 3493014*cos(a1)*cos(a2)*pow(cos(a3),2)*cos(a5)*sin(a2)*sin(a3) + 3493014*cos(a1)*cos(a2)*pow(cos(a3),3)*cos(a4)*sin(a2)*sin(a5) - 412500*cos(a1)*cos(a2)*pow(cos(a4),2)*cos(a5)*sin(a2)*sin(a5) - 419650*cos(a3)*pow(cos(a5),2)*sin(a1)*sin(a2)*sin(a3)*sin(a4) - 1678600*cos(a1)*cos(a2)*pow(cos(a3),3)*cos(a4)*pow(cos(a5),2)*sin(a2) + 839300*cos(a1)*pow(cos(a2),2)*cos(a3)*pow(cos(a4),2)*cos(a5)*sin(a5) + 3493014*cos(a1)*pow(cos(a2),2)*pow(cos(a3),2)*cos(a4)*sin(a3)*sin(a5) + 412500*cos(a4)*cos(a5)*sin(a1)*sin(a2)*sin(a3)*sin(a4)*sin(a5) + 839300*cos(a1)*cos(a2)*pow(cos(a3),2)*cos(a5)*sin(a2)*sin(a3)*sin(a5) - 419650*cos(a1)*cos(a2)*pow(cos(a4),2)*cos(a5)*sin(a2)*sin(a3)*sin(a5) - 419650*pow(cos(a3),2)*cos(a4)*cos(a5)*sin(a1)*sin(a2)*sin(a4)*sin(a5) - 1716750*cos(a1)*cos(a2)*cos(a3)*cos(a4)*sin(a2)*sin(a3)*sin(a5) + 839300*cos(a1)*cos(a2)*pow(cos(a3),2)*pow(cos(a4),2)*cos(a5)*sin(a2)*sin(a3)*sin(a5) - 419650*cos(a2)*cos(a3)*cos(a4)*cos(a5)*sin(a1)*sin(a3)*sin(a4)*sin(a5)))/(2289*cos(a3)*cos(a5)*(763*cos(a2 + a3) - 750*sin(a2))) - (v5*(1746507*pow(cos(a3),3)*cos(a5)*sin(a1) - 1746507*cos(a3)*cos(a5)*sin(a1) - 3493014*pow(cos(a2),2)*pow(cos(a3),3)*cos(a5)*sin(a1) - 412500*cos(a1)*cos(a3)*sin(a2)*sin(a4) - 1716750*cos(a3)*cos(a5)*sin(a1)*sin(a3) + 419650*cos(a1)*cos(a2)*pow(cos(a3),2)*sin(a4) + 1746507*pow(cos(a2),2)*cos(a3)*cos(a5)*sin(a1) - 419650*pow(cos(a3),2)*cos(a4)*sin(a1)*sin(a3) - 1716750*pow(cos(a3),2)*cos(a4)*sin(a1)*sin(a5) + 419650*pow(cos(a3),3)*cos(a5)*sin(a1)*sin(a5) - 419650*cos(a1)*cos(a2)*pow(cos(a3),2)*pow(cos(a5),2)*sin(a4) + 839300*pow(cos(a2),2)*pow(cos(a3),2)*cos(a4)*sin(a1)*sin(a3) + 1716750*pow(cos(a2),2)*pow(cos(a3),2)*cos(a4)*sin(a1)*sin(a5) + 839300*pow(cos(a3),2)*cos(a4)*pow(cos(a5),2)*sin(a1)*sin(a3) - 839300*pow(cos(a2),2)*pow(cos(a3),3)*cos(a5)*sin(a1)*sin(a5) + 419650*pow(cos(a3),3)*pow(cos(a4),2)*cos(a5)*sin(a1)*sin(a5) - 419650*cos(a2)*cos(a3)*cos(a4)*sin(a1)*sin(a2) - 419650*cos(a1)*cos(a3)*sin(a2)*sin(a3)*sin(a4) - 1716750*cos(a1)*cos(a3)*sin(a2)*sin(a4)*sin(a5) + 412500*cos(a2)*cos(a5)*sin(a1)*sin(a2)*sin(a5) + 1716750*cos(a2)*pow(cos(a3),2)*cos(a5)*sin(a1)*sin(a2) + 839300*cos(a2)*pow(cos(a3),3)*cos(a4)*sin(a1)*sin(a2) + 1716750*pow(cos(a2),2)*cos(a3)*cos(a5)*sin(a1)*sin(a3) + 1746507*cos(a1)*cos(a2)*pow(cos(a3),2)*sin(a4)*sin(a5) + 412500*cos(a1)*cos(a3)*pow(cos(a5),2)*sin(a2)*sin(a4) - 419650*cos(a3)*pow(cos(a4),2)*cos(a5)*sin(a1)*sin(a5) - 1746507*pow(cos(a3),2)*cos(a4)*sin(a1)*sin(a3)*sin(a5) - 1746507*cos(a2)*cos(a3)*cos(a4)*sin(a1)*sin(a2)*sin(a5) - 419650*cos(a1)*cos(a4)*cos(a5)*sin(a2)*sin(a4)*sin(a5) - 1678600*pow(cos(a2),2)*pow(cos(a3),2)*cos(a4)*pow(cos(a5),2)*sin(a1)*sin(a3) - 839300*pow(cos(a2),2)*pow(cos(a3),3)*pow(cos(a4),2)*cos(a5)*sin(a1)*sin(a5) - 1746507*cos(a1)*cos(a3)*sin(a2)*sin(a3)*sin(a4)*sin(a5) + 419650*cos(a2)*cos(a5)*sin(a1)*sin(a2)*sin(a3)*sin(a5) + 839300*cos(a2)*cos(a3)*cos(a4)*pow(cos(a5),2)*sin(a1)*sin(a2) + 3493014*cos(a2)*pow(cos(a3),2)*cos(a5)*sin(a1)*sin(a2)*sin(a3) + 419650*cos(a1)*cos(a3)*pow(cos(a5),2)*sin(a2)*sin(a3)*sin(a4) + 3493014*cos(a2)*pow(cos(a3),3)*cos(a4)*sin(a1)*sin(a2)*sin(a5) - 412500*cos(a2)*pow(cos(a4),2)*cos(a5)*sin(a1)*sin(a2)*sin(a5) - 1678600*cos(a2)*pow(cos(a3),3)*cos(a4)*pow(cos(a5),2)*sin(a1)*sin(a2) + 839300*pow(cos(a2),2)*cos(a3)*pow(cos(a4),2)*cos(a5)*sin(a1)*sin(a5) + 3493014*pow(cos(a2),2)*pow(cos(a3),2)*cos(a4)*sin(a1)*sin(a3)*sin(a5) + 419650*cos(a1)*pow(cos(a3),2)*cos(a4)*cos(a5)*sin(a2)*sin(a4)*sin(a5) + 839300*cos(a2)*pow(cos(a3),2)*cos(a5)*sin(a1)*sin(a2)*sin(a3)*sin(a5) - 419650*cos(a2)*pow(cos(a4),2)*cos(a5)*sin(a1)*sin(a2)*sin(a3)*sin(a5) - 1716750*cos(a2)*cos(a3)*cos(a4)*sin(a1)*sin(a2)*sin(a3)*sin(a5) - 412500*cos(a1)*cos(a4)*cos(a5)*sin(a2)*sin(a3)*sin(a4)*sin(a5) + 839300*cos(a2)*pow(cos(a3),2)*pow(cos(a4),2)*cos(a5)*sin(a1)*sin(a2)*sin(a3)*sin(a5) + 419650*cos(a1)*cos(a2)*cos(a3)*cos(a4)*cos(a5)*sin(a3)*sin(a4)*sin(a5)))/(2289*cos(a3)*cos(a5)*(763*cos(a2 + a3) - 750*sin(a2))) - (v6*(206250*sin(2*a5) + 1716750*pow(cos(a3),2)*cos(a5) + 419650*pow(cos(a3),3)*cos(a4) - 419650*cos(a3)*cos(a4) + 419650*pow(cos(a2),2)*cos(a3)*cos(a4) + 419650*cos(a3)*cos(a4)*pow(cos(a5),2) + 1746507*pow(cos(a3),2)*cos(a5)*sin(a3) - 412500*pow(cos(a2),2)*cos(a5)*sin(a5) + 1746507*pow(cos(a3),3)*cos(a4)*sin(a5) - 412500*pow(cos(a4),2)*cos(a5)*sin(a5) - 1716750*pow(cos(a2),2)*pow(cos(a3),2)*cos(a5) - 839300*pow(cos(a2),2)*pow(cos(a3),3)*cos(a4) - 839300*pow(cos(a3),3)*cos(a4)*pow(cos(a5),2) - 1746507*cos(a3)*cos(a4)*sin(a5) + 419650*cos(a5)*sin(a3)*sin(a5) - 839300*pow(cos(a2),2)*cos(a3)*cos(a4)*pow(cos(a5),2) - 3493014*pow(cos(a2),2)*pow(cos(a3),2)*cos(a5)*sin(a3) - 3493014*pow(cos(a2),2)*pow(cos(a3),3)*cos(a4)*sin(a5) + 412500*pow(cos(a2),2)*pow(cos(a4),2)*cos(a5)*sin(a5) + 1746507*cos(a2)*cos(a3)*cos(a5)*sin(a2) + 1678600*pow(cos(a2),2)*pow(cos(a3),3)*cos(a4)*pow(cos(a5),2) - 1716750*cos(a3)*cos(a4)*sin(a3)*sin(a5) - 3493014*cos(a2)*pow(cos(a3),3)*cos(a5)*sin(a2) + 1746507*pow(cos(a2),2)*cos(a3)*cos(a4)*sin(a5) - 419650*pow(cos(a2),2)*cos(a5)*sin(a3)*sin(a5) + 419650*pow(cos(a3),2)*cos(a5)*sin(a3)*sin(a5) - 419650*pow(cos(a4),2)*cos(a5)*sin(a3)*sin(a5) - 839300*pow(cos(a2),2)*pow(cos(a3),2)*cos(a5)*sin(a3)*sin(a5) + 419650*pow(cos(a2),2)*pow(cos(a4),2)*cos(a5)*sin(a3)*sin(a5) + 419650*pow(cos(a3),2)*pow(cos(a4),2)*cos(a5)*sin(a3)*sin(a5) + 1716750*cos(a2)*cos(a3)*cos(a5)*sin(a2)*sin(a3) + 839300*cos(a2)*pow(cos(a3),2)*cos(a4)*sin(a2)*sin(a3) + 1716750*cos(a2)*pow(cos(a3),2)*cos(a4)*sin(a2)*sin(a5) + 1716750*pow(cos(a2),2)*cos(a3)*cos(a4)*sin(a3)*sin(a5) - 839300*cos(a2)*pow(cos(a3),3)*cos(a5)*sin(a2)*sin(a5) - 839300*pow(cos(a2),2)*pow(cos(a3),2)*pow(cos(a4),2)*cos(a5)*sin(a3)*sin(a5) + 839300*cos(a2)*cos(a3)*pow(cos(a4),2)*cos(a5)*sin(a2)*sin(a5) + 3493014*cos(a2)*pow(cos(a3),2)*cos(a4)*sin(a2)*sin(a3)*sin(a5) - 1678600*cos(a2)*pow(cos(a3),2)*cos(a4)*pow(cos(a5),2)*sin(a2)*sin(a3) - 839300*cos(a2)*pow(cos(a3),3)*pow(cos(a4),2)*cos(a5)*sin(a2)*sin(a5)))/(2289*cos(a3)*cos(a5)*(763*cos(a2 + a3) - 750*sin(a2))) - (v2*(7500000*sin(a1)*sin(a4)*sin(a5) - 7500000*pow(cos(a2),2)*sin(a1)*sin(a4)*sin(a5) + 7630000*sin(a1)*sin(a3)*sin(a4)*sin(a5) + 7630000*cos(a1)*pow(cos(a3),2)*cos(a5)*sin(a2) + 7630000*cos(a1)*cos(a2)*cos(a3)*cos(a5)*sin(a3) + 7630000*cos(a1)*cos(a2)*pow(cos(a3),2)*cos(a4)*sin(a5) - 7630000*pow(cos(a2),2)*sin(a1)*sin(a3)*sin(a4)*sin(a5) - 7630000*cos(a1)*cos(a3)*cos(a4)*sin(a2)*sin(a3)*sin(a5) - 7630000*cos(a2)*cos(a3)*sin(a1)*sin(a2)*sin(a4)*sin(a5)))/(2289*cos(a3)*cos(a5)*(763*cos(a2 + a3) - 750*sin(a2))) - (10000*v3*cos(a2)*sin(a4)*sin(a5))/(2289*cos(a3)*cos(a5));


w[4] = (v1*(7500000*cos(a1)*pow(cos(a2),2)*cos(a4) - 7500000*cos(a1)*cos(a4) - 7630000*cos(a1)*cos(a4)*sin(a3) + 7630000*cos(a1)*pow(cos(a2),2)*cos(a4)*sin(a3) - 7630000*cos(a2)*pow(cos(a3),2)*sin(a1)*sin(a4) + 7630000*cos(a1)*cos(a2)*cos(a3)*cos(a4)*sin(a2) + 7630000*cos(a3)*sin(a1)*sin(a2)*sin(a3)*sin(a4)))/(2289*cos(a3)*(763*cos(a2 + a3) - 750*sin(a2))) - (v5*(1716750*pow(cos(a3),2)*sin(a1)*sin(a4) + 1746507*pow(cos(a3),2)*sin(a1)*sin(a3)*sin(a4) - 1716750*pow(cos(a2),2)*pow(cos(a3),2)*sin(a1)*sin(a4) - 1716750*cos(a1)*cos(a3)*cos(a4)*sin(a2) + 1746507*cos(a1)*cos(a2)*pow(cos(a3),2)*cos(a4) - 419650*cos(a1)*pow(cos(a4),2)*cos(a5)*sin(a2) + 419650*cos(a1)*pow(cos(a3),2)*pow(cos(a4),2)*cos(a5)*sin(a2) - 3493014*pow(cos(a2),2)*pow(cos(a3),2)*sin(a1)*sin(a3)*sin(a4) - 1746507*cos(a1)*cos(a3)*cos(a4)*sin(a2)*sin(a3) - 412500*cos(a1)*cos(a3)*cos(a4)*sin(a2)*sin(a5) + 419650*cos(a3)*cos(a4)*cos(a5)*sin(a1)*sin(a4) + 1746507*cos(a2)*cos(a3)*sin(a1)*sin(a2)*sin(a4) + 419650*cos(a1)*cos(a2)*pow(cos(a3),2)*cos(a4)*sin(a5) - 412500*cos(a1)*pow(cos(a4),2)*cos(a5)*sin(a2)*sin(a3) - 419650*pow(cos(a3),3)*cos(a4)*cos(a5)*sin(a1)*sin(a4) - 3493014*cos(a2)*pow(cos(a3),3)*sin(a1)*sin(a2)*sin(a4) + 419650*pow(cos(a3),2)*sin(a1)*sin(a3)*sin(a4)*sin(a5) - 419650*cos(a1)*cos(a3)*cos(a4)*sin(a2)*sin(a3)*sin(a5) + 412500*cos(a2)*cos(a4)*cos(a5)*sin(a1)*sin(a2)*sin(a4) + 1716750*cos(a2)*cos(a3)*sin(a1)*sin(a2)*sin(a3)*sin(a4) + 419650*cos(a2)*cos(a3)*sin(a1)*sin(a2)*sin(a4)*sin(a5) + 419650*cos(a1)*cos(a2)*cos(a3)*pow(cos(a4),2)*cos(a5)*sin(a3) - 839300*pow(cos(a2),2)*cos(a3)*cos(a4)*cos(a5)*sin(a1)*sin(a4) - 839300*cos(a2)*pow(cos(a3),3)*sin(a1)*sin(a2)*sin(a4)*sin(a5) + 839300*pow(cos(a2),2)*pow(cos(a3),3)*cos(a4)*cos(a5)*sin(a1)*sin(a4) - 839300*pow(cos(a2),2)*pow(cos(a3),2)*sin(a1)*sin(a3)*sin(a4)*sin(a5) + 419650*cos(a2)*cos(a4)*cos(a5)*sin(a1)*sin(a2)*sin(a3)*sin(a4) - 839300*cos(a2)*pow(cos(a3),2)*cos(a4)*cos(a5)*sin(a1)*sin(a2)*sin(a3)*sin(a4)))/(2289*cos(a3)*(763*cos(a2 + a3) - 750*sin(a2))) - (10000*v3*cos(a2)*cos(a4))/(2289*cos(a3)) + (v2*(7500000*pow(cos(a2),2)*cos(a4)*sin(a1) - 7500000*cos(a4)*sin(a1) - 7630000*cos(a4)*sin(a1)*sin(a3) + 7630000*cos(a1)*cos(a2)*pow(cos(a3),2)*sin(a4) + 7630000*pow(cos(a2),2)*cos(a4)*sin(a1)*sin(a3) + 7630000*cos(a2)*cos(a3)*cos(a4)*sin(a1)*sin(a2) - 7630000*cos(a1)*cos(a3)*sin(a2)*sin(a3)*sin(a4)))/(2289*cos(a3)*(763*cos(a2 + a3) - 750*sin(a2))) - (v4*(1716750*cos(a1)*pow(cos(a3),2)*sin(a4) - 1716750*cos(a1)*pow(cos(a2),2)*pow(cos(a3),2)*sin(a4) + 1716750*cos(a3)*cos(a4)*sin(a1)*sin(a2) - 1746507*cos(a2)*pow(cos(a3),2)*cos(a4)*sin(a1) + 1746507*cos(a1)*pow(cos(a3),2)*sin(a3)*sin(a4) + 419650*pow(cos(a4),2)*cos(a5)*sin(a1)*sin(a2) - 3493014*cos(a1)*pow(cos(a2),2)*pow(cos(a3),2)*sin(a3)*sin(a4) - 419650*pow(cos(a3),2)*pow(cos(a4),2)*cos(a5)*sin(a1)*sin(a2) + 419650*cos(a1)*cos(a3)*cos(a4)*cos(a5)*sin(a4) + 1746507*cos(a1)*cos(a2)*cos(a3)*sin(a2)*sin(a4) + 1746507*cos(a3)*cos(a4)*sin(a1)*sin(a2)*sin(a3) + 412500*cos(a3)*cos(a4)*sin(a1)*sin(a2)*sin(a5) - 419650*cos(a1)*pow(cos(a3),3)*cos(a4)*cos(a5)*sin(a4) - 3493014*cos(a1)*cos(a2)*pow(cos(a3),3)*sin(a2)*sin(a4) - 419650*cos(a2)*pow(cos(a3),2)*cos(a4)*sin(a1)*sin(a5) + 412500*pow(cos(a4),2)*cos(a5)*sin(a1)*sin(a2)*sin(a3) + 419650*cos(a1)*pow(cos(a3),2)*sin(a3)*sin(a4)*sin(a5) + 412500*cos(a1)*cos(a2)*cos(a4)*cos(a5)*sin(a2)*sin(a4) + 1716750*cos(a1)*cos(a2)*cos(a3)*sin(a2)*sin(a3)*sin(a4) + 419650*cos(a1)*cos(a2)*cos(a3)*sin(a2)*sin(a4)*sin(a5) + 419650*cos(a3)*cos(a4)*sin(a1)*sin(a2)*sin(a3)*sin(a5) - 839300*cos(a1)*pow(cos(a2),2)*cos(a3)*cos(a4)*cos(a5)*sin(a4) - 419650*cos(a2)*cos(a3)*pow(cos(a4),2)*cos(a5)*sin(a1)*sin(a3) - 839300*cos(a1)*cos(a2)*pow(cos(a3),3)*sin(a2)*sin(a4)*sin(a5) + 839300*cos(a1)*pow(cos(a2),2)*pow(cos(a3),3)*cos(a4)*cos(a5)*sin(a4) - 839300*cos(a1)*pow(cos(a2),2)*pow(cos(a3),2)*sin(a3)*sin(a4)*sin(a5) + 419650*cos(a1)*cos(a2)*cos(a4)*cos(a5)*sin(a2)*sin(a3)*sin(a4) - 839300*cos(a1)*cos(a2)*pow(cos(a3),2)*cos(a4)*cos(a5)*sin(a2)*sin(a3)*sin(a4)))/(2289*cos(a3)*(763*cos(a2 + a3) - 750*sin(a2))) + (v6*sin(a4)*(1746507*pow(cos(a3),3) - 1746507*cos(a3) - 858375*sin(2*a3) + 1746507*pow(cos(a2),2)*cos(a3) + 419650*pow(cos(a3),3)*sin(a5) - 3493014*pow(cos(a2),2)*pow(cos(a3),3) - 412500*cos(a4)*cos(a5) - 419650*cos(a3)*sin(a5) + 412500*pow(cos(a2),2)*cos(a4)*cos(a5) + 1716750*cos(a2)*pow(cos(a3),2)*sin(a2) + 1716750*pow(cos(a2),2)*cos(a3)*sin(a3) + 419650*pow(cos(a2),2)*cos(a3)*sin(a5) - 839300*pow(cos(a2),2)*pow(cos(a3),3)*sin(a5) - 419650*cos(a4)*cos(a5)*sin(a3) + 419650*pow(cos(a2),2)*cos(a4)*cos(a5)*sin(a3) + 419650*pow(cos(a3),2)*cos(a4)*cos(a5)*sin(a3) + 3493014*cos(a2)*pow(cos(a3),2)*sin(a2)*sin(a3) - 839300*pow(cos(a2),2)*pow(cos(a3),2)*cos(a4)*cos(a5)*sin(a3) + 839300*cos(a2)*cos(a3)*cos(a4)*cos(a5)*sin(a2) - 839300*cos(a2)*pow(cos(a3),3)
*cos(a4)*cos(a5)*sin(a2) + 839300*cos(a2)*pow(cos(a3),2)*sin(a2)*sin(a3)*sin(a5)))/(2289*cos(a3)*(763*cos(a2 + a3) - 750*sin(a2)));

w[5] = (v6*(412500*cos(a5) - 412500*pow(cos(a2),2)*cos(a5) + 1746507*pow(cos(a3),3)*cos(a4) - 412500*pow(cos(a4),2)*cos(a5) - 1746507*cos(a3)*cos(a4) + 419650*cos(a5)*sin(a3) + 1746507*pow(cos(a2),2)*cos(a3)*cos(a4) - 419650*pow(cos(a2),2)*cos(a5)*sin(a3) - 419650*pow(cos(a4),2)*cos(a5)*sin(a3) + 419650*pow(cos(a3),3)*cos(a4)*sin(a5) - 3493014*pow(cos(a2),2)*pow(cos(a3),3)*cos(a4) + 412500*pow(cos(a2),2)*pow(cos(a4),2)*cos(a5) - 1716750*cos(a3)*cos(a4)*sin(a3) - 419650*cos(a3)*cos(a4)*sin(a5) + 419650*pow(cos(a2),2)*pow(cos(a4),2)*cos(a5)*sin(a3) - 839300*pow(cos(a2),2)*pow(cos(a3),3)*cos(a4)*sin(a5) + 419650*pow(cos(a3),2)*pow(cos(a4),2)*cos(a5)*sin(a3) - 419650*cos(a2)*cos(a3)*cos(a5)*sin(a2) + 1716750*cos(a2)*pow(cos(a3),2)*cos(a4)*sin(a2) + 1716750*pow(cos(a2),2)*cos(a3)*cos(a4)*sin(a3) + 419650*pow(cos(a2),2)*cos(a3)*cos(a4)*sin(a5) - 839300*cos(a2)*pow(cos(a3),3)*pow(cos(a4),2)*cos(a5)*sin(a2) - 839300*pow(cos(a2),2)*pow(cos(a3),2)*pow(cos(a4),2)*cos(a5)*sin(a3) + 839300*cos(a2)*cos(a3)*pow(cos(a4),2)*cos(a5)*sin(a2) + 3493014*cos(a2)*pow(cos(a3),2)*cos(a4)*sin(a2)*sin(a3) + 839300*cos(a2)*pow(cos(a3),2)*cos(a4)*sin(a2)*sin(a3)*sin(a5)))/(2289*cos(a3)*cos(a5)*(763*cos(a2 + a3) - 750*sin(a2))) - (v5*(1716750*pow(cos(a3),2)*cos(a4)*sin(a1) - 1716750*pow(cos(a2),2)*pow(cos(a3),2)*cos(a4)*sin(a1) - 419650*pow(cos(a3),3)*pow(cos(a4),2)*cos(a5)*sin(a1) + 1716750*cos(a1)*cos(a3)*sin(a2)*sin(a4) - 412500*cos(a2)*cos(a5)*sin(a1)*sin(a2) - 1746507*cos(a1)*cos(a2)*pow(cos(a3),2)*sin(a4) + 419650*pow(cos(a2),2)*cos(a3)*cos(a5)*sin(a1) + 419650*cos(a3)*pow(cos(a4),2)*cos(a5)*sin(a1) + 1746507*pow(cos(a3),2)*cos(a4)*sin(a1)*sin(a3) - 839300*pow(cos(a2),2)*cos(a3)*pow(cos(a4),2)*cos(a5)*sin(a1) - 3493014*pow(cos(a2),2)*pow(cos(a3),2)*cos(a4)*sin(a1)*sin(a3) + 1746507*cos(a2)*cos(a3)*cos(a4)*sin(a1)*sin(a2) + 419650*cos(a1)*cos(a4)*cos(a5)*sin(a2)*sin(a4) + 839300*pow(cos(a2),2)*pow(cos(a3),3)*pow(cos(a4),2)*cos(a5)*sin(a1) + 1746507*cos(a1)*cos(a3)*sin(a2)*sin(a3)*sin(a4) - 419650*cos(a2)*cos(a5)*sin(a1)*sin(a2)*sin(a3) + 412500*cos(a1)*cos(a3)*sin(a2)*sin(a4)*sin(a5) - 3493014*cos(a2)*pow(cos(a3),3)*cos(a4)*sin(a1)*sin(a2) + 412500*cos(a2)*pow(cos(a4),2)*cos(a5)*sin(a1)*sin(a2) - 419650*cos(a1)*cos(a2)*pow(cos(a3),2)*sin(a4)*sin(a5) + 419650*pow(cos(a3),2)*cos(a4)*sin(a1)*sin(a3)*sin(a5) + 1716750*cos(a2)*cos(a3)*cos(a4)*sin(a1)*sin(a2)*sin(a3) + 419650*cos(a2)*cos(a3)*cos(a4)*sin(a1)*sin(a2)*sin(a5) + 412500*cos(a1)*cos(a4)*cos(a5)*sin(a2)*sin(a3)*sin(a4) + 419650*cos(a1)*cos(a3)*sin(a2)*sin(a3)*sin(a4)*sin(a5) - 419650*cos(a1)*pow(cos(a3),2)*cos(a4)*cos(a5)*sin(a2)*sin(a4) + 419650*cos(a2)*pow(cos(a4),2)*cos(a5)*sin(a1)*sin(a2)*sin(a3) - 839300*cos(a2)*pow(cos(a3),3)*cos(a4)*sin(a1)*sin(a2)*sin(a5) - 839300*pow(cos(a2),2)*pow(cos(a3),2)*cos(a4)*sin(a1)*sin(a3)*sin(a5) - 839300*cos(a2)*pow(cos(a3),2)*pow(cos(a4),2)*cos(a5)*sin(a1)*sin(a2)*sin(a3) - 419650*cos(a1)*cos(a2)*cos(a3)*cos(a4)*cos(a5)*sin(a3)*sin(a4)))/(2289*cos(a3)*cos(a5)*(763*cos(a2 + a3) - 750*sin(a2))) + (10000*v3*cos(a2)*sin(a4))/(2289*cos(a3)*cos(a5)) - (v1*(7500000*cos(a1)*pow(cos(a2),2)*sin(a4) - 7500000*cos(a1)*sin(a4) - 7630000*cos(a1)*sin(a3)*sin(a4) + 7630000*cos(a2)*pow(cos(a3),2)*cos(a4)*sin(a1) + 7630000*cos(a1)*pow(cos(a2),2)*sin(a3)*sin(a4) + 7630000*cos(a1)*cos(a2)*cos(a3)*sin(a2)*sin(a4) - 7630000*cos(a3)*cos(a4)*sin(a1)*sin(a2)*sin(a3)))/(2289*cos(a3)*cos(a5)*(763*cos(a2 + a3) - 750*sin(a2))) + (v4*(1716750*cos(a1)*pow(cos(a2),2)*pow(cos(a3),2)*cos(a4) - 1716750*cos(a1)*pow(cos(a3),2)*cos(a4) + 419650*cos(a1)*pow(cos(a3),3)*pow(cos(a4),2)*cos(a5) + 412500*cos(a1)*cos(a2)*cos(a5)*sin(a2) + 1716750*cos(a3)*sin(a1)*sin(a2)*sin(a4) - 419650*cos(a1)*pow(cos(a2),2)*cos(a3)*cos(a5) - 419650*cos(a1)*cos(a3)*pow(cos(a4),2)*cos(a5) - 1746507*cos(a1)*pow(cos(a3),2)*cos(a4)*sin(a3) - 1746507*cos(a2)*pow(cos(a3),2)*sin(a1)*sin(a4) + 3493014*cos(a1)*pow(cos(a2),2)*pow(cos(a3),2)*cos(a4)*sin(a3) - 1746507*cos(a1)*cos(a2)*cos(a3)*cos(a4)*sin(a2) - 839300*cos(a1)*pow(cos(a2),2)*pow(cos(a3),3)*pow(cos(a4),2)*cos(a5) + 419650*cos(a1)*cos(a2)*cos(a5)*sin(a2)*sin(a3) + 419650*cos(a4)*cos(a5)*sin(a1)*sin(a2)*sin(a4) + 1746507*cos(a3)*sin(a1)*sin(a2)*sin(a3)*sin(a4) + 412500*cos(a3)*sin(a1)*sin(a2)*sin(a4)*sin(a5) + 3493014*cos(a1)*cos(a2)*pow(cos(a3),3)*cos(a4)*sin(a2) - 412500*cos(a1)*cos(a2)*pow(cos(a4),2)*cos(a5)*sin(a2) - 419650*cos(a1)*pow(cos(a3),2)*cos(a4)*sin(a3)*sin(a5) - 419650*cos(a2)*pow(cos(a3),2)*sin(a1)*sin(a4)*sin(a5) + 839300*cos(a1)*pow(cos(a2),2)*cos(a3)*pow(cos(a4),2)*cos(a5) - 1716750*cos(a1)*cos(a2)*cos(a3)*cos(a4)*sin(a2)*sin(a3) - 419650*cos(a1)*cos(a2)*cos(a3)*cos(a4)*sin(a2)*sin(a5) + 412500*cos(a4)*cos(a5)*sin(a1)*sin(a2)*sin(a3)*sin(a4) + 419650*cos(a3)*sin(a1)*sin(a2)*sin(a3)*sin(a4)*sin(a5) - 419650*cos(a1)*cos(a2)*pow(cos(a4),2)*cos(a5)*sin(a2)*sin(a3) + 839300*cos(a1)*cos(a2)*pow(cos(a3),3)*cos(a4)*sin(a2)*sin(a5) - 419650*pow(cos(a3),2)*cos(a4)*cos(a5)*sin(a1)*sin(a2)*sin(a4) + 839300*cos(a1)*pow(cos(a2),2)*pow(cos(a3),2)*cos(a4)*sin(a3)*sin(a5) + 839300*cos(a1)*cos(a2)*pow(cos(a3),2)*pow(cos(a4),2)*cos(a5)*sin(a2)*sin(a3) - 419650*cos(a2)*cos(a3)*cos(a4)*cos(a5)*sin(a1)*sin(a3)*sin(a4)))/(2289*cos(a3)*cos(a5)*(763*cos(a2 + a3) - 750*sin(a2))) - (v2*(7500000*pow(cos(a2),2)*sin(a1)*sin(a4) - 7500000*sin(a1)*sin(a4) - 7630000*sin(a1)*sin(a3)*sin(a4) + 7630000*pow(cos(a2),2)*sin(a1)*sin(a3)*sin(a4) - 7630000*cos(a1)*cos(a2)*pow(cos(a3),2)*cos(a4) + 7630000*cos(a1)*cos(a3)*cos(a4)*sin(a2)*sin(a3) + 7630000*cos(a2)*cos(a3)*sin(a1)*sin(a2)*sin(a4)))/(2289*cos(a3)*cos(a5)*(763*cos(a2 + a3) - 750*sin(a2)));
                                                                            
}
 
 

void terminalVelocity(double t,double v[]){
    double dx = 0.0511, dy = -0.24, dz = -0.374, t_once = 4.9;
    double dt= t_once / 7;
    double vx_max = dx / 4 / dt;
    double vx1 = dx / 16 /dt;
    double ax_max = 2 * vx1 / dt; 
    double Jx = 2 * vx1 / dt /dt;

    double vy_max = dy / 4 / dt;
    double vy1 = dy / 16 /dt;
    double ay_max = 2 * vy1 / dt; 
    double Jy = 2 * vy1 / dt /dt;

    double vz_max = dz / 4 / dt;
    double vz1 = dz / 16 /dt;
    double az_max = 2 * vz1 / dt; 
    double Jz = 2 * vz1 / dt /dt;

    // cout << ax_max << " " << ax_max << " " <<az_max <<endl;
    if (t < dt){
        v[0] = 0.5 * Jx * t * t;
        v[1] = 0.5 * Jy * t * t;
        v[2] = 0.5 * Jz * t * t;
    }
    else if (t < 2 * dt){
        v[0] = 0.5 * Jx * dt *dt + ax_max * (t - dt);
        v[1] = 0.5 * Jy * dt *dt + ay_max * (t - dt);
        v[2] = 0.5 * Jz * dt *dt + az_max * (t - dt);
    }
    else if (t < 3 *dt)
    {
        v[0] = vx_max - 0.5 * Jx * (3*dt - t) * (3*dt -t);
        v[1] = vy_max - 0.5 * Jy * (3*dt - t)* (3*dt -t);
        v[2] = vz_max - 0.5 * Jz * (3*dt - t)* (3*dt -t);
    }
    else if(t < 4*dt){
        v[0] = vx_max;
        v[1] = vy_max;
        v[2] = vz_max;
    }

    else if (t < 5 *dt)
    {
       v[0] = vx_max - 0.5 * Jx *(t - 4*dt) *(t - 4*dt);
       v[1] = vy_max - 0.5 * Jy *(t - 4*dt) *(t - 4*dt);
       v[2] = vz_max - 0.5 * Jz *(t - 4*dt) *(t - 4*dt);
    }

    else if (t < 6 *dt)
    {
        v[0] = vx_max - 0.5 * Jx *dt * dt -ax_max*(t - 5 *dt);
        v[1] = vy_max - 0.5 * Jy *dt * dt -ay_max*(t - 5 *dt);
        v[2] = vz_max - 0.5 * Jz *dt * dt -az_max*(t - 5 *dt);
    }

    else if (t < t_once)
    {
        v[0]= 0.5 * Jx * (t_once - t) *(t_once- t);
        v[1] = 0.5 * Jy * (t_once - t) *(t_once - t);
        v[2] = 0.5 * Jz * (t_once - t) *(t_once - t);
    }
    else
    {
        v[0] = 0;
        v[1] = 0;
        v[2] = 0;
    }

    
    //4s
    /*
        if(t < 2){
		v[0] = 0.0511 * t / 8;
        v[1] = -0.03 * t;
		v[2] =-0.374 * t /8;
	}
	else if(t < 4){
		v[0] = 0.0511 / 4;
        v[1] = -0.06;
        v[2] = -0.374 / 4;
    }
	else if(t < 6){
		v[0] = 0.0511 * (6 - t) / 8;
        v[1] = -0.03 * (6 - t );
        v[2] = -0.374 * (6 - t) / 8;
	}
	else{
		v[0] = 0;
		v[2] = 0;
	}
    */
}

int main(int argc, char **argv) {

    bool ret;
    //节点初始化
    ros::init(argc, argv, "bellRinging");
    //创建节点句柄对象
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    ROS_INFO_STREAM("start");

    double RUNTIME = 4.9;
    double w[6], v[6] = {0, 0, 0, 0, 0, 0};
    static double theta[6] = {0,0,0,0,0,0};

    static double t = 0;
    double dt = 0.01;

    ros::Publisher vel_pub = node_handle.advertise<std_msgs::Float64MultiArray>("/probot_anno/arm_vel_controller/command", 1000);
    ros::Rate rate(50);
    std_msgs::Float64MultiArray init_vel;
    init_vel.data.push_back(0);
    init_vel.data.push_back(0);
    init_vel.data.push_back(0);
    init_vel.data.push_back(0);
    init_vel.data.push_back(0);
    init_vel.data.push_back(0);
    sleep(1);

    while (ros::ok && t <= RUNTIME)
    {
        cout << t << ":DO ONCE"<<  endl;
        /* code */
        terminalVelocity(t, v);

        planOnce(theta, w, v);
        for (int i = 0; i < 6; i++)
        {
            theta[i] += w[i] * dt;
        }
        init_vel.data.at(0) = w[0];
        init_vel.data.at(1) = w[1];
        init_vel.data.at(2) = w[2];
        init_vel.data.at(3) = w[3];
        init_vel.data.at(4) = w[4];
        init_vel.data.at(5) = w[5];
        for (int i=0; i < 6; i++){
            cout << "\nw:" <<  w[i] << " " << endl;
        }
        vel_pub.publish(init_vel);

        rate.sleep();
        t += dt;
    }
    ROS_INFO_STREAM("published");
}