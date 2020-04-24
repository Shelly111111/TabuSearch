//*****************************************************************

//禁忌搜索算法求解车辆路径问题

//*****************************************************************

#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <algorithm>
#include <ctime>
#include <vector>
#include "data.h"
using namespace std;

string m_json = "{ \"vertexes\": [{\"id\":1,\"name\" : \"P\",\"need\" : 0.0,\"typeId\" : 2},{\"id\":2,\"name\" : \"A\",\"need\" : 1.7,\"typeId\" : 1},{\"id\":3,\"name\" : \"B\",\"need\" : 0.8,\"typeId\" : 1},{\"id\":4,\"name\" : \"C\",\"need\" : 1.3,\"typeId\" : 1},{\"id\":5,\"name\" : \"D\",\"need\" : 2.8,\"typeId\" : 1},{\"id\":6,\"name\" : \"E\",\"need\" : 1.9,\"typeId\" : 1},{\"id\":7,\"name\" : \"F\",\"need\" : 3.5,\"typeId\" : 1},{\"id\":8,\"name\" : \"G\",\"need\" : 0.9,\"typeId\" : 1},{\"id\":9,\"name\" : \"H\",\"need\" : 0.3,\"typeId\" : 1},{\"id\":10,\"name\" : \"I\",\"need\" : 1.2,\"typeId\" : 1},{\"id\":11,\"name\" : \"中转站\",\"need\" : 0.0,\"typeId\" : 3}] ,\"edges\" : [{\"distance\":8.0,\"pointId1\" : 1,\"pointId2\" : 3},{\"distance\":7.0,\"pointId1\" : 1,\"pointId2\" : 4},{\"distance\":4.0,\"pointId1\" : 1,\"pointId2\" : 6},{\"distance\":12.0,\"pointId1\" : 1,\"pointId2\" : 7},{\"distance\":9.0,\"pointId1\" : 1,\"pointId2\" : 8},{\"distance\":12.0,\"pointId1\" : 1,\"pointId2\" : 9},{\"distance\":6.0,\"pointId1\" : 1,\"pointId2\" : 10},{\"distance\":5.0,\"pointId1\" : 1,\"pointId2\" : 11},{\"distance\":4.0,\"pointId1\" : 2,\"pointId2\" : 3},{\"distance\":3.0,\"pointId1\" : 3,\"pointId2\" : 4},{\"distance\":4.0,\"pointId1\" : 4,\"pointId2\" : 5},{\"distance\":5.0,\"pointId1\" : 4,\"pointId2\" : 11},{\"distance\":2.0,\"pointId1\" : 5,\"pointId2\" : 11},{\"distance\":2.0,\"pointId1\" : 6,\"pointId2\" : 11},{\"distance\":3.0,\"pointId1\" : 5,\"pointId2\" : 6},{\"distance\":10.0,\"pointId1\" : 6,\"pointId2\" : 7},{\"distance\":7.0,\"pointId1\" : 7,\"pointId2\" : 9},{\"distance\":4.0,\"pointId1\" : 7,\"pointId2\" : 8},{\"distance\":5.0,\"pointId1\" : 8,\"pointId2\" : 9},{\"distance\":9.0,\"pointId1\" : 9,\"pointId2\" : 10},{\"distance\":3.0,\"pointId1\" : 10,\"pointId2\" : 2}] }";

//************************************************************
//计算路径规划R的目标函数值
double Calculation(Route_Type R[])
{
    double D = 0;
    //若每条路径都符合约束条件，则计算总里程
    for (int i = 1; i <= Vehicle_Number; ++i)
        if (R[i].V.size() > 2)
        {
            if(R[i].Load > param.vehicle[R[i].VT].MaxLoad || R[i].Dis > param.vehicle[R[i].VT].MaxMileage || R[i].ServT > param.Max_ServiceTime)
                return INF;
            D += R[i].Dis;
        }
    return D;
}

//************************************************************
//检验路径规划R是否满足所有约束
bool Check(Route_Type R[])
{
    //检查是否满足容量约束、时间窗约束
    for (int i = 1; i <= Vehicle_Number; ++i)
    {
        if (R[i].VT == param.Vehicle_type)
            break;
        if (R[i].V.size() > 2 && (R[i].Load > param.vehicle[R[i].VT].MaxLoad || R[i].Dis > param.vehicle[R[i].VT].MaxMileage))
            return false;
    }
    return true;
}

//************************************************************
//将路径规划Route的内容复制给路径规划Route_Ans
void Copy_Route()
{
    for (int i = 1; i <= Vehicle_Number; ++i)
    {
        Route_Ans[i].Load = Route[i].Load;
        Route_Ans[i].Dis = Route[i].Dis;
        Route_Ans[i].ServT = Route[i].ServT;
        Route_Ans[i].VT = Route[i].VT;
        Route_Ans[i].V.clear();
        for (int j = 0; j < Route[i].V.size(); ++j)
            Route_Ans[i].V.push_back(Route[i].V[j]);
    }
}

//************************************************************
//构造初始路径
bool Construction()
{
    int Sizeof_Delivery_Set = Needs.size();
    int Current_Route = 1;
    int Delivery_Set = 0;
    random_shuffle(Needs.begin(), Needs.end());
    //以满足容量约束为目的的随机初始化
    //即随机挑选一个节点插入到第m条路径中，若超过容量约束，则插入第m+1条路径
    //且插入路径的位置由该路径上已存在的各节点最早时间的升序决定
    while (Delivery_Set < Sizeof_Delivery_Set)
    {
        int C = Needs[Delivery_Set];
        Delivery_Set++;
        //将当前服务过的节点赋值为最末节点值,数组容量减1
        if (Route[Current_Route].Load + Delivery[C].Demand > param.vehicle[Route[Current_Route].VT].MaxLoad)
            Current_Route++;
        if (Current_Route >= Delivery_num)
            return false;
        for (int i = 1; i < Route[Current_Route].V.size(); i++)
        {
            double Pre_Dis = Route[Current_Route].Dis - Graph[Route[Current_Route].V[i - 1].Number][Route[Current_Route].V[i].Number] +
                Graph[Route[Current_Route].V[i - 1].Number][Delivery[C].Number] +
                Graph[Route[Current_Route].V[i].Number][Delivery[C].Number];
            double ServTime = Pre_Dis / param.Speed * 60 + (Route[Current_Route].V.size() - 1) * param.ServiceTime;
            if (Pre_Dis <= param.vehicle[Route[Current_Route].VT].MaxMileage && ServTime <= param.Max_ServiceTime)
            {
                Route[Current_Route].Load += Delivery[C].Demand;
                Route[Current_Route].Dis = Pre_Dis;
                Route[Current_Route].ServT = ServTime;
                Delivery[C].R = Current_Route;
                Route[Current_Route].V.insert(Route[Current_Route].V.begin() + i, Delivery[C]);
                break;
            }
        }
        if (Delivery[C].R == 0)
        {
            Current_Route++;
            if (Current_Route >= Delivery_num)
                return false;
            Route[Current_Route].Load += Delivery[C].Demand;
            double Pre_Dis = Graph[Route[Current_Route].V[0].Number][Delivery[C].Number] + Graph[Route[Current_Route].V[1].Number][Delivery[C].Number];
            double ServTime = Pre_Dis / param.Speed * 60 + (Route[Current_Route].V.size() - 1) * param.ServiceTime;
            Route[Current_Route].Dis = Pre_Dis;
            Route[Current_Route].ServT = ServTime;
            Route[Current_Route].V.insert(Route[Current_Route].V.begin() + 1, Delivery[C]);
            Delivery[C].R = Current_Route;
        }
    }
    Copy_Route();
    int D = 0;
    for (int i = 1; i <= Vehicle_Number; ++i)
        D += Route[i].Dis;
    Ans = D;
}

//************************************************************
//用2-opt算子优化路径
//颠倒数组中下标begin到end的元素位置
void swap_element(int r, int begin, int end)
{
    Delivery_Type temp;
    while (begin < end)
    {
        temp = Route[r].V[begin];
        Route[r].V[begin] = Route[r].V[end];
        Route[r].V[end] = temp;
        if (Tabu[begin][r] != 0 || Tabu[end][r] != 0)
        {
            int temp2 = Tabu[begin][r];
            Tabu[begin][r] = Tabu[end][r];
            Tabu[end][r] = temp2;
        }
        begin++;
        end--;
    }
}

//2-opt算法优化路径
void opt(int r)
{
    int Bestbegin = 0, Bestend = 0;
    int _l = Route[r].V.size();
    double BestDist = Route[r].Dis;
    while (true)
    {
        for (int begin = 1; begin < _l - 2; begin++)
            for (int end = begin + 1; end < _l - 1; end++)
            {
                double distance = 0;
                swap_element(r, begin, end);
                for (int i = 1; i < _l; i++)
                    distance += Graph[Route[r].V[i - 1].Number][Route[r].V[i].Number];
                if (distance < BestDist)
                {
                    Bestbegin = begin;
                    Bestend = end;
                    BestDist = distance;
                }
                swap_element(r, begin, end);
            }
        if (BestDist == Route[r].Dis)
            break;
        swap_element(r, Bestbegin, Bestend);
        Route[r].Dis = BestDist;
        Route[r].ServT = Route[r].Dis / param.Speed * 60 + (_l - 3) * param.ServiceTime;
    }
}

//************************************************************
//禁忌搜索
void Tabu_Search()
{
    //初始化禁忌表
    for (int i = 2; i <= Delivery_num + 1; ++i)
    {
        for (int j = 1; j <= Vehicle_Number; ++j)
            Tabu[i][j] = 0;
        TabuCreate[i] = 0;
    }
    for (int Iteration = 0; Iteration < param.Iter_Epoch; Iteration++)
    {
        int BestDelivery = 0, BestRoute = 0, BestPoint = 0, P = 0;
        double BestV = INF;
        //Relocation算子搜索邻域，找到符合要求的最优插入方案
        for (int K = 0; K < Needs.size(); ++K)
        {
            int i = Needs[K];
            for (int j = 1; j < Route[Delivery[i].R].V.size(); ++j)
                if (Route[Delivery[i].R].V[j].Number == i)
                {
                    P = j;
                    break;
                }
            double Temp1 = Route[Delivery[i].R].ServT;
            //从节点原路径中去除该节点的需求
            Route[Delivery[i].R].Load -= Delivery[i].Demand;
            //从节点原路径中去除该节点所组成的路径并重组
            Route[Delivery[i].R].Dis = Route[Delivery[i].R].Dis - Graph[Route[Delivery[i].R].V[P - 1].Number][Route[Delivery[i].R].V[P].Number]
                - Graph[Route[Delivery[i].R].V[P].Number][Route[Delivery[i].R].V[P + 1].Number] + Graph[Route[Delivery[i].R].V[P - 1].Number][Route[Delivery[i].R].V[P + 1].Number];
            Route[Delivery[i].R].ServT = Route[Delivery[i].R].Dis / param.Speed * 60 + (Route[Delivery[i].R].V.size() - 3) * param.ServiceTime;
            //从节点原路径中去除节点
            Route[Delivery[i].R].V.erase(Route[Delivery[i].R].V.begin() + P);
            for (int j = 1; j <= Vehicle_Number; ++j)
            {
                if (Route[j].VT == param.Vehicle_type)
                    break;
                //禁忌插入操作，后者为禁止使用新的车辆
                if (((Route[j].V.size() > 2 && Tabu[i][j] <= Iteration) || (Route[j].V.size() == 2 && TabuCreate[i] <= Iteration)) && Delivery[i].R != j)
                    for (int l = 1; l < Route[j].V.size(); ++l)
                    {
                        //判断加上结点后是否会超出限制
                        if (Route[j].Load + Delivery[i].Demand > param.vehicle[Route[j].VT].MaxLoad)
                            break;
                        double Pre_Dis = Route[j].Dis - Graph[Route[j].V[l - 1].Number][Route[j].V[l].Number]
                            + Graph[Route[j].V[l - 1].Number][Delivery[i].Number] + Graph[Route[j].V[l].Number][Delivery[i].Number];
                        double ServTime = Pre_Dis / param.Speed * 60 + (Route[j].V.size() - 1) * param.ServiceTime;
                        if (Pre_Dis > param.vehicle[Route[j].VT].MaxMileage || ServTime > param.Max_ServiceTime)
                            continue;
                        //在节点新路径中加上该节点的需求
                        Route[j].Load += Delivery[i].Demand;
                        //在节点新路径中加上该节点插入后所组成的路径并断开原路径
                        Route[j].Dis = Pre_Dis;
                        double Temp2 = Route[j].ServT;
                        Route[j].ServT = ServTime;
                        //在节点新路径中插入节点
                        Route[j].V.insert(Route[j].V.begin() + l, Delivery[i]);
                        double TempV = Calculation(Route);
                        if (TempV < BestV)
                        {
                            BestV = TempV;
                            BestDelivery = i, BestRoute = j, BestPoint = l;
                        }
                        //节点新路径复原
                        Route[j].V.erase(Route[j].V.begin() + l);
                        Route[j].Load -= Delivery[i].Demand;
                        Route[j].Dis = Route[j].Dis + Graph[Route[j].V[l - 1].Number][Route[j].V[l].Number]
                            - Graph[Route[j].V[l - 1].Number][Delivery[i].Number] - Graph[Route[j].V[l].Number][Delivery[i].Number];
                        Route[j].ServT = Temp2;
                    }
            }
            //节点原路径复原
            Route[Delivery[i].R].V.insert(Route[Delivery[i].R].V.begin() + P, Delivery[i]);
            Route[Delivery[i].R].Load += Delivery[i].Demand;
            Route[Delivery[i].R].ServT = Temp1;
            Route[Delivery[i].R].Dis = Route[Delivery[i].R].Dis + Graph[Route[Delivery[i].R].V[P - 1].Number][Route[Delivery[i].R].V[P].Number]
                + Graph[Route[Delivery[i].R].V[P].Number][Route[Delivery[i].R].V[P + 1].Number] - Graph[Route[Delivery[i].R].V[P - 1].Number][Route[Delivery[i].R].V[P + 1].Number];
        }
        //将BestDelivery禁忌在该节点所在的路径中，直到超过禁忌时长才能重新插入到该路径里
        if (Route[BestRoute].V.size() == 2)
            TabuCreate[BestDelivery] = Iteration + 2 * Tabu_tenure + rand() % 10;
        Tabu[BestDelivery][Delivery[BestDelivery].R] = Iteration + Tabu_tenure + rand() % 10;
        for (int i = 1; i < Route[Delivery[BestDelivery].R].V.size(); ++i)
            if (Route[Delivery[BestDelivery].R].V[i].Number == BestDelivery)
            {
                P = i;
                break;
            }
        if (BestDelivery == 0)
        {
            //if (Iteration % 10 == 0)
                //cout << "Iteration: " << Iteration << ", BestV: " << BestV << ", Ans: " << Ans << endl;
            continue;
        }
        //依据上述循环中挑选的结果，生成新的总体路径规划
        //更新改变过的各单条路径的载重，距离长度的总量
        //将该结点从源路径中去除
        Route[Delivery[BestDelivery].R].Load -= Delivery[BestDelivery].Demand;
        Route[Delivery[BestDelivery].R].Dis = Route[Delivery[BestDelivery].R].Dis - Graph[Route[Delivery[BestDelivery].R].V[P - 1].Number][Route[Delivery[BestDelivery].R].V[P].Number]
            - Graph[Route[Delivery[BestDelivery].R].V[P].Number][Route[Delivery[BestDelivery].R].V[P + 1].Number] + Graph[Route[Delivery[BestDelivery].R].V[P - 1].Number][Route[Delivery[BestDelivery].R].V[P + 1].Number];
        Route[Delivery[BestDelivery].R].ServT = Route[Delivery[BestDelivery].R].Dis / param.Speed * 60 + (Route[Delivery[BestDelivery].R].V.size() - 3) * param.ServiceTime;
        Route[Delivery[BestDelivery].R].V.erase(Route[Delivery[BestDelivery].R].V.begin() + P);
        //在新路径中插入该结点
        Route[BestRoute].Dis = Route[BestRoute].Dis - Graph[Route[BestRoute].V[BestPoint - 1].Number][Route[BestRoute].V[BestPoint].Number]
            + Graph[Route[BestRoute].V[BestPoint - 1].Number][Delivery[BestDelivery].Number] + Graph[Route[BestRoute].V[BestPoint].Number][Delivery[BestDelivery].Number];
        Route[BestRoute].Load += Delivery[BestDelivery].Demand;
        Route[BestRoute].ServT = Route[BestRoute].Dis / param.Speed * 60 + (Route[BestRoute].V.size() - 1) * param.ServiceTime;
        Route[BestRoute].V.insert(Route[BestRoute].V.begin() + BestPoint, Delivery[BestDelivery]);
        //更新被操作的节点所属路径编号
        Delivery[BestDelivery].R = BestRoute;
        //如果当前解合法且较优则更新存储结果
        if ((Check(Route) == true) && (Ans > BestV))
        {
            Copy_Route();
            Ans = BestV;
        }
        //2-opt算子对BestRoute路径优化
        opt(BestRoute);
        if (Ans > Calculation(Route))
        {
            Copy_Route();
            Ans = Calculation(Route);
        }
        //if (Iteration % 10 == 0)
            //cout << "Iteration: " << Iteration << ", BestV: " << BestV << ", Ans: " << Ans << endl;
    }
}

//************************************************************
int main(int argc, char * argv[])
{
    //clock_t Start, Finish;
    //Start = clock();
    srand((unsigned)time(NULL));
    Init_Param();
    //cout << "InitParam is Ok!" << endl;
    //if (!get_Date_in_txt(Delivery_num))
        //return -1;
    get_Date_in_json(Delivery_num, argc, argv);
    //cout << "Initialization is Ok!" << endl;
    if (!Construction())
    {
        cerr << "{\"typeId\":1,\"data\":Error:Vehicle parameter setting is unreasonable, please check the parameters.}" << endl;
        return -1;
    }
    //cout << "Initial path construction is Ok!" << endl;
    Tabu_Search();
    //cout << "Tabu Search is Ok!" << endl;
    Output_by_json(Route_Ans);
    //Output(Route_Ans);
    //Finish = clock();
    //cout << "Total Running Time = " << ( Finish - Start ) / 1000.0 << endl;
    return 0;
}

//************************************************************
/*
The Minimum Total Distance = 2254
Concrete Schedule of Each Route as Following :
No.1 : Use VehicleType 0, Load 29.6, Distance 165, ServeTime 3.71
[0, 124, 68, 36, 159, 115, 47, 119, 165, 123, 164, 183, 59, 0]
No.2 : Use VehicleType 0, Load 27.5, Distance 205, ServeTime 4.61667
[0, 9, 24, 95, 73, 116, 104, 4, 70, 106, 132, 157, 122, 184, 88, 26, 0]
No.3 : Use VehicleType 0, Load 21.2, Distance 143, ServeTime 3.26333
[0, 0, 29, 156, 21, 110, 168, 117, 196, 49, 22, 113, 0]
No.4 : Use VehicleType 0, Load 29, Distance 175, ServeTime 4.03667
[0, 72, 77, 30, 87, 138, 28, 137, 66, 199, 187, 186, 107, 57, 198, 0]
No.5 : Use VehicleType 0, Load 29.1, Distance 152, ServeTime 3.73333
[0, 96, 109, 189, 93, 75, 177, 58, 51, 80, 142, 178, 40, 17, 181, 174, 0]
No.6 : Use VehicleType 0, Load 28.8, Distance 143, ServeTime 3.50333
[0, 37, 56, 55, 32, 82, 182, 191, 134, 170, 81, 100, 175, 89, 158, 0]
No.7 : Use VehicleType 0, Load 29.2, Distance 227, ServeTime 4.82333
[0, 118, 130, 76, 25, 19, 43, 50, 33, 143, 74, 111, 31, 149, 0]
No.8 : Use VehicleType 0, Load 29.2, Distance 201, ServeTime 4.31
[0, 45, 179, 84, 197, 44, 13, 8, 193, 15, 171, 2, 127, 0]
No.9 : Use VehicleType 0, Load 29.9, Distance 174, ServeTime 4.02
[0, 139, 18, 91, 172, 145, 180, 35, 121, 16, 103, 141, 155, 69, 3, 0]
No.10 : Use VehicleType 0, Load 26.5, Distance 120, ServeTime 3.04
[0, 152, 146, 194, 154, 78, 125, 147, 23, 169, 83, 192, 90, 67, 0]
No.11 : Use VehicleType 0, Load 28.1, Distance 179, ServeTime 4.26333
[0, 188, 79, 166, 6, 64, 10, 46, 7, 176, 120, 135, 126, 114, 98, 163, 94, 0]
No.12 : Use VehicleType 0, Load 29.1, Distance 172, ServeTime 3.98667
[0, 153, 190, 140, 52, 108, 129, 148, 161, 39, 42, 53, 173, 102, 128, 0]
No.13 : Use VehicleType 1, Load 11.6, Distance 115, ServeTime 2.39667
[0, 54, 131, 86, 11, 63, 71, 0]
No.14 : Use VehicleType 1, Load 11.1, Distance 83, ServeTime 1.78333
[0, 160, 97, 167, 60, 101, 0]
Check_Ans = 2254
************************************************************
Total Running Time = 2.865
*/
