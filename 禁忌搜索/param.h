#pragma once
//************************************************************

//参数配置

//************************************************************

#include <vector>
#define INF 0x3ffffff
#define Delivery_Number 2000   //算例中除仓库以外的最大顾客节点个数
#define Tabu_tenure 20   //禁忌时长
#define Initial_Time 0

int Delivery_num;
int Vehicle_Number;   //由于无车辆数量限制，因此将上限设为顾客总数
int Tabu[Delivery_Number + 10][Delivery_Number + 10];   //禁忌表用于禁忌节点插入操作
int TabuCreate[Delivery_Number + 10];   //禁忌表用于禁忌拓展新路径或使用新车辆
std::vector<int> Needs;//用于记录需要配送的结点
double Ans;
double Graph[Delivery_Number + 10][Delivery_Number + 10];
int lastVN;

struct Vehicle_Type
{
    double MaxLoad;     //最大负载
    double MaxMileage;  //最大巡回里程
    int Num;        //车辆数目
};

struct Param
{
    int Iter_Epoch;     //迭代次数
    int Vehicle_type;   //车辆种类数目
    std::vector<Vehicle_Type> vehicle;
    double ServiceTime;     //单配送点服务时长
    double Speed;       //车辆行驶速度
}param;

struct Delivery_Type
{
    int Number;   //节点自身编号
    int R;   //节点所属车辆路径编号
    double X, Y;   //节点横纵坐标
    double End;   //节点被访问的最晚时间
    double SerBegin;//节点被开始访问的时间
    double Demand;   //节点的需求量
} Delivery[Delivery_Number + 10];   //计算时，配送中心节点下标为1，顾客节点下标为2-max

struct Route_Type
{
    double Load;   //单条路径装载量
    double Dis;   //单条路径总长度
    int VT;      //哪种车辆配送
    std::vector<Delivery_Type> V;   //单条路径上顾客节点序列
} Route[Delivery_Number + 10], Route_Ans[Delivery_Number + 10];   //车辆路径及搜索到最优的车辆路径