//*****************************************************************

//禁忌搜索算法求解带时间窗的车辆路径问题

//*****************************************************************

#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <cmath>
#include <ctime>
#include <vector>
using namespace std;
#define INF 0x3ffffff
#define Customer_Number 200   //算例中除仓库以外的最大顾客节点个数
#define Itinerant_Mileage 400   //最大巡回里程，单位km
#define Tabu_tenure 20   //禁忌时长

struct Vehicle_Type
{
    double MaxLoad;
    double MaxMileage;
    int Num;
};

struct Param
{
    int Iter_Epoch;
    int Vehicle_type;
    vector<Vehicle_Type> vehicle;
    double ServiceTime;
    double Max_ServiceTime;
    double Speed;
}param;

struct Customer_Type
{
    int Number;   //节点自身编号
    int R;   //节点所属车辆路径编号
    double X, Y;   //节点横纵坐标
    double Begin, End, Service;   //节点被访问的最早时间，最晚时间以及服务时长
    double Demand;   //节点的需求量
} Customer[Customer_Number + 10];   //仓库节点编号为1，顾客节点编号为2-101

struct Route_Type
{
    double Load;   //单条路径装载量
    double SubT;   //单条路径违反各节点时间窗约束时长总和
    double Dis;   //单条路径总长度
    int VT;      //哪种车辆配送
    vector<Customer_Type> V;   //单条路径上顾客节点序列
} Route[Customer_Number + 10], Route_Ans[Customer_Number + 10];   //车辆路径及搜索到最优的车辆路径

int Vehicle_Number = Customer_Number;   //由于无车辆数量限制，因此将上限设为顾客总数
int Tabu[Customer_Number + 10][Customer_Number + 10];   //禁忌表用于禁忌节点插入操作
int TabuCreate[Customer_Number + 10];   //禁忌表用于禁忌拓展新路径或使用新车辆
double Ans;
//double Alpha = 1, Beta = 1, Sita = 0.5;
double Graph[Customer_Number + 10][Customer_Number + 10];
int Customer_num;
//************************************************************
//初始化参数
/*
train_opt={
    'epoch' : 4,#迭代次数
    'data_path' : './graph.json',
    'log_file' : './log',
    'checkpoint' : False,#检查点
    'max' : 9999999,
    'file_path' : './checkpoint.json',
    'dataDict' : {
        'Vehicle_type_num' : 2,
        0 : {
            'MaxLoad' : 30.0,#车辆最大负载
            'MaxMileage' : 400,#最大巡回里程
            'Num' : 12
            },
        1 : {
            'MaxLoad' : 12.0,
            'MaxMileage' : 400,#最大巡回里程
            'Num' : 10
            },
        'ServiceTime' : 5,#服务时间，单位分钟
        'MaxServiceTime' : 480,#单位分钟
        'speed' : 60#单位千米/小时
    }
}
*/
void Init_Param()
{
    param.Iter_Epoch = 400;
    param.Max_ServiceTime = 8.0;
    param.ServiceTime = 0.08;
    param.Speed = 60.0;
    param.Vehicle_type = 2;
    param.vehicle.resize(param.Vehicle_type);
    param.vehicle[0].MaxLoad = 30.0;
    param.vehicle[0].MaxMileage = 400.0;
    param.vehicle[0].Num = 12;
    param.vehicle[1].MaxLoad = 12.0;
    param.vehicle[1].MaxMileage = 400.0;
    param.vehicle[1].Num = 10;
}

//************************************************************
//计算图上各节点间的距离
double Distance(Customer_Type C1, Customer_Type C2)
{
    return sqrt((C1.X - C2.X) * (C1.X - C2.X) + (C1.Y - C2.Y) * (C1.Y - C2.Y));
}

//************************************************************
//计算路径规划R的目标函数值
double Calculation(Route_Type R[], int Cus, int NewR)
{
    //目标函数主要由三个部分组成：D路径总长度（优化目标），Q超出容量约束总量，T超出时间窗约束总量
    //目标函数结构为 f(R) = D + Alpha * Q + Beta * T, 第一项为问题最小化目标，后两项为惩罚部分
    //其中Alpha与Beta为可变参数，分别根据当前解是否满足两个约束来进行变化（在Check函数中更新，由于Check针对每轮迭代得到的解）
    //double Q = 0;
    //double T = 0;
    double D = 0;
    //计算单条路径超出容量约束的总量
    for (int i = 1; i <= Vehicle_Number; ++i)
    {
        if (R[i].VT == param.Vehicle_type)
            break;
        if (R[i].V.size() > 2 && R[i].Load > param.vehicle[R[i].VT].MaxLoad)
            return INF;
    }
            //Q = Q + R[i].Load - Capacity;
    //计算单条路径上各个节点超出时间窗约束的总量（仅更新进行移除和插入节点操作的两条路径）
    double ArriveTime = 0;
    R[Customer[Cus].R].SubT = 0;
    for (int j = 1; j < R[Customer[Cus].R].V.size(); ++j)
    {
        ArriveTime = ArriveTime + R[Customer[Cus].R].V[j - 1].Service + Graph[R[Customer[Cus].R].V[j - 1].Number][R[Customer[Cus].R].V[j].Number];
        if (ArriveTime > R[Customer[Cus].R].V[j].End)
            R[Customer[Cus].R].SubT = R[Customer[Cus].R].SubT + ArriveTime - R[Customer[Cus].R].V[j].End;
        else if (ArriveTime < R[Customer[Cus].R].V[j].Begin)
            ArriveTime = R[Customer[Cus].R].V[j].Begin;
    }
    ArriveTime = 0;
    R[NewR].SubT = 0;
    for (int j = 1; j < R[NewR].V.size(); ++j)
    {
        ArriveTime = ArriveTime + R[NewR].V[j - 1].Service + Graph[R[NewR].V[j - 1].Number][R[NewR].V[j].Number];
        if (ArriveTime > R[NewR].V[j].End)
            R[NewR].SubT = R[NewR].SubT + ArriveTime - R[NewR].V[j].End;
        else if (ArriveTime < R[NewR].V[j].Begin)
            ArriveTime = R[NewR].V[j].Begin;
    }
    //for (int i = 1; i <= Vehicle_Number; ++i)
        //T += R[i].SubT;
    //计算路径总长度
    for (int i = 1; i <= Vehicle_Number; ++i)
    {
        if (R[i].Dis <= Itinerant_Mileage)
            D += R[i].Dis;
        else
            return INF;
    }
    return D;
    //return D + Alpha * Q + Beta * T;
}

//************************************************************
//检验路径规划R是否满足所有约束
bool Check(Route_Type R[])
{
    //double Q = 0;
    //double T = 0;
    //double D = 0;
    //检查是否满足容量约束、时间窗约束
    for (int i = 1; i <= Vehicle_Number; ++i)
    {
        if (R[i].VT == param.Vehicle_type)
            break;
        if (R[i].V.size() > 2 && (R[i].Load > param.vehicle[R[i].VT].MaxLoad || R[i].Dis > Itinerant_Mileage))
            return false;
            //Q += R[i].Load - Capacity;
        //T += R[i].SubT;
        //if (R[i].V.size() > 2 && R[i].Dis > Itinerant_Mileage)
            //D += R[i].Dis - Itinerant_Mileage;
    }
    return true;
    //分别根据约束满足的情况更新Alpha和Beta值
    /*
        if (Q == 0 && Alpha >= 0.001)
        Alpha /= (1 + Sita);
    else if (Q != 0 && Alpha <= 2000)
        Alpha *= (1 + Sita);
    if (T == 0 && Beta >= 0.001)
        Beta /= (1 + Sita);
    else if (T != 0 && Beta <= 2000)
        Beta *= (1 + Sita);
    if (T == 0 && Q == 0)
        return true;
    else
        return false;
    */
}

//************************************************************
//将路径规划Route的内容复制给路径规划Route_Ans
void Copy_Route()
{
    for (int i = 1; i <= Vehicle_Number; ++i)
    {
        Route_Ans[i].Load = Route[i].Load;
        Route_Ans[i].V.clear();
        for (int j = 0; j < Route[i].V.size(); ++j)
            Route_Ans[i].V.push_back(Route[i].V[j]);
    }
}

//************************************************************
//结果输出
void Output(Route_Type R[])
{
    cout << "************************************************************" << endl;
    cout << "The Minimum Total Distance = " << Ans << endl;
    cout << "Concrete Schedule of Each Route as Following : " << endl;
    int M = 0;
    for (int i = 1; i <= Vehicle_Number; ++i)
        if (R[i].V.size() > 2)
        {
            M++;
            cout << "No." << M << " : ";
            for (int j = 0; j < R[i].V.size() - 1; ++j)
                cout << R[i].V[j].Number - 1 << " -> ";
            cout << R[i].V[R[i].V.size() - 1].Number - 1 << endl;
        }
    //检验距离计算是否正确
    double Check_Ans = 0;
    for (int i = 1; i <= Vehicle_Number; ++i)
        for (int j = 1; j < R[i].V.size(); ++j)
            Check_Ans += Graph[R[i].V[j - 1].Number][R[i].V[j].Number];
    cout << "Check_Ans = " << Check_Ans << endl;
    cout << "************************************************************" << endl;
}

//************************************************************
//迪杰斯特拉算两点之间最短路径
void Dijkstra(double (*Graph)[Customer_Number + 10],int s)
{
    bool flag[Customer_Number + 10];
    double distance[Customer_Number + 10];
    for (int i = 1; i <= Customer_num + 1; i++)
        flag[i] = false;
    for (int i = 1; i <= Customer_num + 1; i++)
        distance[i] = INF;
    distance[s] = 0.0;
    while (true)
    {
        int v = -1;
        for (int u = 1; u <= Customer_num + 1; u++)
            if (!flag[u] && (v == -1 || distance[u] < distance[v]))
                v = u;
        if (v == -1)
            break;
        flag[v] = true;
        for (int u = 1; u <= Customer_num + 1; u++)
            if (distance[u] > distance[v] + Graph[v][u])
                distance[u] = distance[v] + Graph[v][u];
    }
    for (int i = 1; i <= Customer_num + 1; i++)
        Graph[s][i] = distance[i];       
}
//************************************************************
//数据读入及初始化
void ReadIn_and_Initialization()
{
    ifstream ifs;
    //ifs.open("Node.txt", ios::in);
    ifs.open("Vex.txt", ios::in);
    if (!ifs.is_open())
    {
        cout << "open file failed." << endl;
        return;
    }
    ifs >> Customer_num;
    for (int i = 1; i <= Customer_num + 1; ++i)
    {
        ifs >> Customer[i].Number >> Customer[i].X >> Customer[i].Y >> Customer[i].Demand;
        Customer[i].Begin = 0.0;
        Customer[i].End = 3600.0;
        Customer[i].Service = 10.0;
    }
    /*
    for (int i = 1; i <= Customer_Number + 1; ++i)
        ifs >> Customer[i].Number >> Customer[i].X >> Customer[i].Y >> Customer[i].Demand
        >> Customer[i].Begin >> Customer[i].End >> Customer[i].Service;
    ifs.close();
    */
    FILE* fp;
    fp = fopen("Edges.txt", "r");
    if (fp == NULL)
    {
        cout << "open file failed." << endl;
        return;
    }
    for (int i = 1; i <= Customer_num + 1; ++i)
        for (int j = 1; j <= Customer_num + 1; ++j)
        {
            fscanf(fp, "%lf ", &Graph[i][j]);
            if (Graph[i][j] == 0.0)
                Graph[i][j] = INF;
        }
    fclose(fp);
    for (int i = 1; i <= Customer_num + 1; ++i)
        Dijkstra(&Graph[0], i);
    //初始化每条路径，默认路径收尾为仓库，且首仓库最早最晚时间均为原仓库最早时间，尾仓库则均为原仓库最晚时间
    Customer[1].R = -1;
    int Current_VT = 0, temp = 1;
    for (int i = 1; i <= Vehicle_Number; ++i)
    {
        if (!Route[i].V.empty())
            Route[i].V.clear();
        Route[i].V.push_back(Customer[1]);
        Route[i].V.push_back(Customer[1]);
        Route[i].V[0].End = Route[i].V[0].Begin;
        Route[i].V[1].Begin = Route[i].V[1].End;
        Route[i].Load = 0.0;
        Route[i].Dis = 0.0;
        if (Current_VT < param.Vehicle_type && i == temp + param.vehicle[Current_VT].Num)
        {
            temp = i;
            Current_VT++;
        }
        Route[i].VT = Current_VT;
    }
    Ans = INF;
    /*
    for (int i = 1; i <= Customer_Number + 1; ++i)
        for (int j = 1; j <= Customer_Number + 1; ++j)
            Graph[i][j] = Distance(Customer[i], Customer[j]);
    */
}

//************************************************************
//构造初始路径
bool Construction()
{
    int Customer_Set[Customer_Number + 10];
    for (int i = 1; i <= Customer_num + 1; ++i)
        Customer_Set[i] = i + 1;
    int Sizeof_Customer_Set = Customer_num;
    int Current_Route = 1;
    //以满足容量约束为目的的随机初始化
    //即随机挑选一个节点插入到第m条路径中，若超过容量约束，则插入第m+1条路径
    //且插入路径的位置由该路径上已存在的各节点最早时间的升序决定
    while (Sizeof_Customer_Set > 0)
    {
        int K = rand() % Sizeof_Customer_Set + 1;
        int C = Customer_Set[K];
        Customer_Set[K] = Customer_Set[Sizeof_Customer_Set];
        Sizeof_Customer_Set--;
        /*int K = rand() % Sizeof_Customer_Set + 1;
        int C = Customer_Set[K];
        Customer_Set[K] = Customer_Set[Sizeof_Customer_Set];
        Sizeof_Customer_Set--;*/
        //将当前服务过的节点赋值为最末节点值,数组容量减1
        if (Route[Current_Route].VT < param.Vehicle_type && Route[Current_Route].Load + Customer[C].Demand > param.vehicle[Route[Current_Route].VT].MaxLoad)
            Current_Route++;
        else if (Route[Current_Route].VT == param.Vehicle_type)
            return false;
        for (int i = 1; i < Route[Current_Route].V.size(); i++)
        {

            double Pre_Dis = Route[Current_Route].Dis - Graph[Route[Current_Route].V[i - 1].Number][Route[Current_Route].V[i].Number] +
                Graph[Route[Current_Route].V[i - 1].Number][Customer[C].Number] +
                Graph[Route[Current_Route].V[i].Number][Customer[C].Number];
            if ((Route[Current_Route].V[i - 1].Begin <= Customer[C].Begin) && (Customer[C].Begin <= Route[Current_Route].V[i].Begin) &&
                (Pre_Dis  <= Itinerant_Mileage))
            {
                Route[Current_Route].Load += Customer[C].Demand;
                Route[Current_Route].Dis = Pre_Dis;
                Customer[C].R = Current_Route;
                Route[Current_Route].V.insert(Route[Current_Route].V.begin() + i, Customer[C]);
                break;
            }
        }
        if (Customer[C].R == 0)
        {
            Current_Route++;
            Route[Current_Route].Load += Customer[C].Demand;
            double Pre_Dis = Graph[Route[Current_Route].V[0].Number][Customer[C].Number] + Graph[Route[Current_Route].V[1].Number][Customer[C].Number];
            Route[Current_Route].Dis = Pre_Dis;
            Route[Current_Route].V.insert(Route[Current_Route].V.begin() + 1, Customer[C]);
            Customer[C].R = Current_Route;
        }
    }
    //初始化计算超过容量约束的总量和超过时间窗约束的总量
    for (int i = 1; i <= Vehicle_Number; ++i)
    {
        double ArriveTime = Route[i].V[0].Begin;
        Route[i].SubT = 0;
        //Route[i].Dis = 0;
        for (int j = 1; j < Route[i].V.size(); ++j)
        {
            ArriveTime = ArriveTime + Route[i].V[j - 1].Service + Graph[Route[i].V[j - 1].Number][Route[i].V[j].Number];
            //Route[i].Dis += Graph[Route[i].V[j - 1].Number][Route[i].V[j].Number];
            if (ArriveTime > Route[i].V[j].End)
                Route[i].SubT = Route[i].SubT + ArriveTime - Route[i].V[j].End;
            else if (ArriveTime < Route[i].V[j].Begin)
                ArriveTime = Route[i].V[j].Begin;
        }
    }
    Copy_Route();
    int D = 0;
    for (int i = 1; i <= Vehicle_Number; ++i)
        D += Route[i].Dis;
    Ans = D;
    return true;
}

//************************************************************
//禁忌搜索
void Tabu_Search()
{
    //禁忌搜索采取插入算子，即从一条路径中选择一点插入到另一条路径中
    //在该操作下形成的邻域中选取使目标函数最小的非禁忌解或者因满足藐视法则而被解禁的解
    double Temp1;
    double Temp2;
    //初始化禁忌表
    for (int i = 2; i <= Customer_num + 1; ++i)
    {
        for (int j = 1; j <= Vehicle_Number; ++j)
            Tabu[i][j] = 0;
        TabuCreate[i] = 0;
    }
    for (int Iteration = 0; Iteration < param.Iter_Epoch; Iteration++)
    {        
        int BestCustomer = 0, BestRoute = 0, BestPoint = 0, P = 0;
        double BestV = INF;
        for (int i = 2; i <= Customer_num + 1; ++i)
        {
            for (int j = 1; j < Route[Customer[i].R].V.size(); ++j)
                if (Route[Customer[i].R].V[j].Number == i)
                {
                    P = j;
                    break;
                }
            //从节点原路径中去除该节点的需求
            Route[Customer[i].R].Load -= Customer[i].Demand;
            //从节点原路径中去除该节点所组成的路径并重组
            Route[Customer[i].R].Dis = Route[Customer[i].R].Dis - Graph[Route[Customer[i].R].V[P - 1].Number][Route[Customer[i].R].V[P].Number]
                - Graph[Route[Customer[i].R].V[P].Number][Route[Customer[i].R].V[P + 1].Number] + Graph[Route[Customer[i].R].V[P - 1].Number][Route[Customer[i].R].V[P + 1].Number];
            //从节点原路径中去除节点
            Route[Customer[i].R].V.erase(Route[Customer[i].R].V.begin() + P);
            for (int j = 1; j <= Vehicle_Number; ++j)
            {
                if (Route[j].VT == param.Vehicle_type)
                    break;
                //禁忌插入操作，后者为禁止使用新的车辆
                if (((Route[j].V.size() > 2 && Tabu[i][j] <= Iteration) || (Route[j].V.size() == 2 && TabuCreate[i] <= Iteration)) && Customer[i].R != j)
                    for (int l = 1; l < Route[j].V.size(); ++l)
                    {
                        //判断加上结点后是否会超出限制
                        if (Route[j].Load + Customer[i].Demand > param.vehicle[Route[j].VT].MaxMileage)
                            break;
                        double Pre_Dis = Route[j].Dis - Graph[Route[j].V[l - 1].Number][Route[j].V[l].Number]
                            + Graph[Route[j].V[l - 1].Number][Customer[i].Number] + Graph[Route[j].V[l].Number][Customer[i].Number];
                        if (Pre_Dis > Itinerant_Mileage)
                            continue;
                        //在节点新路径中加上该节点的需求
                        Route[j].Load += Customer[i].Demand;
                        //在节点新路径中加上该节点插入后所组成的路径并断开原路径
                        Route[j].Dis = Pre_Dis;
                        //在节点新路径中插入节点
                        Route[j].V.insert(Route[j].V.begin() + l, Customer[i]);
                        Temp1 = Route[Customer[i].R].SubT;
                        Temp2 = Route[j].SubT;
                        double TempV = Calculation(Route, i, j);
                        if (TempV < BestV)
                        {
                            BestV = TempV;
                            BestCustomer = i, BestRoute = j, BestPoint = l;
                        }
                        //节点新路径复原
                        Route[Customer[i].R].SubT = Temp1;
                        Route[j].SubT = Temp2;
                        Route[j].V.erase(Route[j].V.begin() + l);
                        Route[j].Load -= Customer[i].Demand;
                        Route[j].Dis = Route[j].Dis + Graph[Route[j].V[l - 1].Number][Route[j].V[l].Number]
                            - Graph[Route[j].V[l - 1].Number][Customer[i].Number] - Graph[Route[j].V[l].Number][Customer[i].Number];
                    }
            }
            //节点原路径复原
            Route[Customer[i].R].V.insert(Route[Customer[i].R].V.begin() + P, Customer[i]);
            Route[Customer[i].R].Load += Customer[i].Demand;
            Route[Customer[i].R].Dis = Route[Customer[i].R].Dis + Graph[Route[Customer[i].R].V[P - 1].Number][Route[Customer[i].R].V[P].Number]
                + Graph[Route[Customer[i].R].V[P].Number][Route[Customer[i].R].V[P + 1].Number] - Graph[Route[Customer[i].R].V[P - 1].Number][Route[Customer[i].R].V[P + 1].Number];
        }
        //将BestCustomer禁忌在该节点所在的路径中，直到超过禁忌时长才能重新插入到该路径里
        if (Route[BestRoute].V.size() == 2)
            TabuCreate[BestCustomer] = Iteration + 2 * Tabu_tenure + rand() % 10;
        Tabu[BestCustomer][Customer[BestCustomer].R] = Iteration + Tabu_tenure + rand() % 10;
        for (int i = 1; i < Route[Customer[BestCustomer].R].V.size(); ++i)
            if (Route[Customer[BestCustomer].R].V[i].Number == BestCustomer)
            {
                P = i;
                break;
            }
        //依据上述循环中挑选的结果，生成新的总体路径规划
        //更新改变过的各单条路径的载重，距离长度，超出时间窗的总量
        //将该结点从源路径中去除
        Route[Customer[BestCustomer].R].Load -= Customer[BestCustomer].Demand;
        Route[Customer[BestCustomer].R].Dis = Route[Customer[BestCustomer].R].Dis - Graph[Route[Customer[BestCustomer].R].V[P - 1].Number][Route[Customer[BestCustomer].R].V[P].Number]
            - Graph[Route[Customer[BestCustomer].R].V[P].Number][Route[Customer[BestCustomer].R].V[P + 1].Number] + Graph[Route[Customer[BestCustomer].R].V[P - 1].Number][Route[Customer[BestCustomer].R].V[P + 1].Number];
        Route[Customer[BestCustomer].R].V.erase(Route[Customer[BestCustomer].R].V.begin() + P);
        //在新路径中插入该结点
        Route[BestRoute].Dis = Route[BestRoute].Dis - Graph[Route[BestRoute].V[BestPoint - 1].Number][Route[BestRoute].V[BestPoint].Number]
            + Graph[Route[BestRoute].V[BestPoint - 1].Number][Customer[BestCustomer].Number] + Graph[Route[BestRoute].V[BestPoint].Number][Customer[BestCustomer].Number];
        Route[BestRoute].Load += Customer[BestCustomer].Demand;
        Route[BestRoute].V.insert(Route[BestRoute].V.begin() + BestPoint, Customer[BestCustomer]);
        //更新时间窗
        double ArriveTime = 0;
        Route[BestRoute].SubT = 0;
        for (int j = 1; j < Route[BestRoute].V.size(); ++j)
        {
            ArriveTime = ArriveTime + Route[BestRoute].V[j - 1].Service + Graph[Route[BestRoute].V[j - 1].Number][Route[BestRoute].V[j].Number];
            if (ArriveTime > Route[BestRoute].V[j].End)
                Route[BestRoute].SubT = Route[BestRoute].SubT + ArriveTime - Route[BestRoute].V[j].End;
            else if (ArriveTime < Route[BestRoute].V[j].Begin)
                ArriveTime = Route[BestRoute].V[j].Begin;
        }
        ArriveTime = 0;
        Route[Customer[BestCustomer].R].SubT = 0;
        for (int j = 1; j < Route[Customer[BestCustomer].R].V.size(); ++j)
        {
            ArriveTime = ArriveTime + Route[Customer[BestCustomer].R].V[j - 1].Service + Graph[Route[Customer[BestCustomer].R].V[j - 1].Number][Route[Customer[BestCustomer].R].V[j].Number];
            if (ArriveTime > Route[Customer[BestCustomer].R].V[j].End)
                Route[Customer[BestCustomer].R].SubT = Route[Customer[BestCustomer].R].SubT + ArriveTime - Route[Customer[BestCustomer].R].V[j].End;
            else if (ArriveTime < Route[Customer[BestCustomer].R].V[j].Begin)
                ArriveTime = Route[Customer[BestCustomer].R].V[j].Begin;
        }
        //更新被操作的节点所属路径编号
        Customer[BestCustomer].R = BestRoute;
        //如果当前解合法且较优则更新存储结果
        if ((Check(Route) == true) && (Ans > BestV))
        {
            Copy_Route();
            Ans = BestV;
        }
        if (Iteration % 10 == 0)
            cout << "Iteration: " << Iteration << ", BestV: " << BestV << ", Ans: " << Ans << endl;
    }
}

//************************************************************

int main()
{
    clock_t Start, Finish;
    Start = clock();
    srand((unsigned)time(NULL));
    Init_Param();
    cout << "InitParam is Ok!" << endl;
    ReadIn_and_Initialization();
    cout << "Initialization is Ok!" << endl;
    bool flag = false;
    int trystep = 0;
    while (!flag)
    {
        if (trystep >= 5)
        {
            cerr << "Some parameters currently entered may be too small for effective operation, please check the corresponding parameter settings\n";
            exit(-1);
        }
        flag = Construction();
        trystep++;
    } 
    cout << "Initial path construction is Ok!" << endl;
    Tabu_Search();
    cout << "Tabu Search is Ok!" << endl;
    Output(Route_Ans);
    Finish = clock();
    cout << "Total Running Time = " << ( Finish - Start ) / 1000.0 << endl;
    return 0;
}
//************************************************************

/*
The Minimum Total Distance = 2596
Concrete Schedule of Each Route as Following :
No.1 : 0 -> 37 -> 119 -> 108 -> 160 -> 183 -> 198 -> 125 -> 163 -> 105 -> 4 -> 185 -> 118 -> 44 -> 0
No.2 : 0 -> 140 -> 92 -> 130 -> 150 -> 138 -> 141 -> 170 -> 162 -> 20 -> 121 -> 79 -> 131 -> 149 -> 0
No.3 : 0 -> 74 -> 135 -> 35 -> 102 -> 19 -> 25 -> 156 -> 28 -> 117 -> 6 -> 10 -> 0
No.4 : 0 -> 89 -> 111 -> 88 -> 61 -> 172 -> 7 -> 22 -> 176 -> 152 -> 99 -> 97 -> 51 -> 190 -> 13 -> 87 -> 0
No.5 : 0 -> 49 -> 93 -> 18 -> 164 -> 182 -> 81 -> 123 -> 171 -> 57 -> 104 -> 194 -> 0
No.6 : 0 -> 71 -> 145 -> 95 -> 197 -> 16 -> 65 -> 36 -> 32 -> 84 -> 167 -> 180 -> 166 -> 56 -> 133 -> 101 -> 90 -> 116 -> 0
No.7 : 0 -> 31 -> 83 -> 86 -> 66 -> 188 -> 103 -> 98 -> 174 -> 15 -> 34 -> 132 -> 82 -> 85 -> 0
No.8 : 0 -> 179 -> 109 -> 55 -> 165 -> 94 -> 158 -> 70 -> 59 -> 62 -> 186 -> 187 -> 199 -> 192 -> 9 -> 0
No.9 : 0 -> 144 -> 129 -> 143 -> 45 -> 72 -> 106 -> 136 -> 137 -> 191 -> 29 -> 75 -> 114 -> 0
No.10 : 0 -> 69 -> 27 -> 39 -> 52 -> 151 -> 38 -> 155 -> 147 -> 26 -> 139 -> 50 -> 0
No.11 : 0 -> 142 -> 67 -> 8 -> 177 -> 120 -> 60 -> 73 -> 43 -> 100 -> 122 -> 68 -> 78 -> 0
No.12 : 0 -> 168 -> 64 -> 134 -> 107 -> 17 -> 24 -> 193 -> 153 -> 195 -> 146 -> 48 -> 189 -> 161 -> 0
No.13 : 0 -> 77 -> 41 -> 154 -> 115 -> 124 -> 3 -> 40 -> 46 -> 1 -> 181 -> 113 -> 96 -> 33 -> 42 -> 2 -> 0
No.14 : 0 -> 21 -> 23 -> 30 -> 53 -> 14 -> 12 -> 58 -> 196 -> 5 -> 0
No.15 : 0 -> 54 -> 110 -> 159 -> 126 -> 157 -> 184 -> 148 -> 128 -> 0
No.16 : 0 -> 63 -> 91 -> 76 -> 178 -> 47 -> 173 -> 0
No.17 : 0 -> 80 -> 112 -> 11 -> 175 -> 169 -> 127 -> 0
Check_Ans = 2596
Total Running Time = 204.022
*/