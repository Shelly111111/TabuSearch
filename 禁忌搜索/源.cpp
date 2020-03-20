//*****************************************************************

//禁忌搜索算法求解带时间窗的车辆路径问题

//*****************************************************************

#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <cmath>
#include<random>
#include <ctime>
#include <vector>
using namespace std;
#define INF 0x3ffffff
#define Customer_Number 200   //算例中除仓库以外的最大顾客节点个数
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
    double Demand;   //节点的需求量
} Customer[Customer_Number + 10];   //仓库节点编号为1，顾客节点编号为2-101

struct Route_Type
{
    double Load;   //单条路径装载量
    double SubT;   //单条路径服务时长总和
    double Dis;   //单条路径总长度
    int VT;      //哪种车辆配送
    vector<Customer_Type> V;   //单条路径上顾客节点序列
} Route[Customer_Number + 10], Route_Ans[Customer_Number + 10];   //车辆路径及搜索到最优的车辆路径

int Vehicle_Number = Customer_Number;   //由于无车辆数量限制，因此将上限设为顾客总数
int Tabu[Customer_Number + 10][Customer_Number + 10];   //禁忌表用于禁忌节点插入操作
int TabuCreate[Customer_Number + 10];   //禁忌表用于禁忌拓展新路径或使用新车辆
vector<int> Needs;//用于记录需要配送的结点
double Ans;
double Graph[Customer_Number + 10][Customer_Number + 10];
int Customer_num;
int lastVN;
//************************************************************
//初始化参数
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
    param.vehicle[1].Num = Customer_Number + 10;
    lastVN = 10;
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
    double D = 0;
    //计算单条路径超出容量约束的总量
    for (int i = 1; i <= Vehicle_Number; ++i)
    {
        if (R[i].VT == param.Vehicle_type)
            break;
        if (R[i].V.size() > 2 && R[i].Load > param.vehicle[R[i].VT].MaxLoad)
            return INF;
    }
    for (int i = 1; i <= Vehicle_Number; ++i)
    {
        if (R[i].Dis <= param.vehicle[R[i].VT].MaxMileage)
            D += R[i].Dis;
        else
            return INF;
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
    int ChecklastVN = 0;
    for (int i = 1; i <= Vehicle_Number; ++i)
        if (R[i].V.size() > 2)
        {
            M++;
            cout << "No." << M << " : " << "Use VehicleType " << R[i].VT << ",Load " << R[i].Load << ",Distance " 
                << R[i].Dis << endl;
            for (int j = 0; j < R[i].V.size() - 1; ++j)
                cout << R[i].V[j].Number - 1 << " -> ";
            cout << R[i].V[R[i].V.size() - 1].Number - 1 << endl;
            if (R[i].VT == param.Vehicle_type - 1)
                ChecklastVN++;
        }
    //检验距离计算是否正确
    double Check_Ans = 0;
    for (int i = 1; i <= Vehicle_Number; ++i)
        for (int j = 1; j < R[i].V.size(); ++j)
            Check_Ans += Graph[R[i].V[j - 1].Number][R[i].V[j].Number];
    cout << "Check_Ans = " << Check_Ans << endl;
    if (ChecklastVN >= lastVN)
        cerr << "Wornning：Some parameters currently entered may be too small for effective operation, please check the corresponding parameter settings." << endl;
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
    Needs.clear();
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
        if (Customer[i].Demand != 0.0)
            Needs.push_back(i);
    }
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
}

//************************************************************
//构造初始路径
void Construction()
{
    int Sizeof_Customer_Set = Needs.size();
    int Current_Route = 1;
    int Customer_Set = 0;
    random_shuffle(Needs.begin(), Needs.end());
    //以满足容量约束为目的的随机初始化
    //即随机挑选一个节点插入到第m条路径中，若超过容量约束，则插入第m+1条路径
    //且插入路径的位置由该路径上已存在的各节点最早时间的升序决定
    while (Customer_Set < Sizeof_Customer_Set)
    {
        //int K = rand() % Sizeof_Customer_Set + 1;
        int C = Needs[Customer_Set];
        Customer_Set++;
        //将当前服务过的节点赋值为最末节点值,数组容量减1
        if (Route[Current_Route].Load + Customer[C].Demand > param.vehicle[Route[Current_Route].VT].MaxLoad)
            Current_Route++;
        for (int i = 1; i < Route[Current_Route].V.size(); i++)
        {

            double Pre_Dis = Route[Current_Route].Dis - Graph[Route[Current_Route].V[i - 1].Number][Route[Current_Route].V[i].Number] +
                Graph[Route[Current_Route].V[i - 1].Number][Customer[C].Number] +
                Graph[Route[Current_Route].V[i].Number][Customer[C].Number];
            if (Pre_Dis  <= param.vehicle[Route[Current_Route].VT].MaxMileage)
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
    Copy_Route();
    int D = 0;
    for (int i = 1; i <= Vehicle_Number; ++i)
        D += Route[i].Dis;
    Ans = D;
}

//************************************************************
//禁忌搜索
void Tabu_Search()
{
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
        for (int K = 0; K < Needs.size(); ++K)
        {
            int i = Needs[K];
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
                        if (Pre_Dis > param.vehicle[Route[j].VT].MaxMileage)
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
        //更新改变过的各单条路径的载重，距离长度的总量
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
    Construction(); 
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
The Minimum Total Distance = 2244
Concrete Schedule of Each Route as Following :
No.1 : Use VehicleType 0,Load 29.1,Distance 129
0 -> 63 -> 43 -> 182 -> 67 -> 8 -> 16 -> 188 -> 103 -> 98 -> 97 -> 134 -> 64 -> 0
No.2 : Use VehicleType 0,Load 28.1,Distance 171
0 -> 168 -> 37 -> 119 -> 33 -> 42 -> 159 -> 126 -> 157 -> 120 -> 197 -> 61 -> 191 -> 0
No.3 : Use VehicleType 0,Load 21.5,Distance 119
0 -> 73 -> 156 -> 28 -> 198 -> 183 -> 147 -> 160 -> 129 -> 93 -> 18 -> 0
No.4 : Use VehicleType 0,Load 30,Distance 164
0 -> 9 -> 199 -> 192 -> 121 -> 15 -> 106 -> 124 -> 177 -> 66 -> 86 -> 83 -> 31 -> 24 -> 0
No.5 : Use VehicleType 0,Load 25.2,Distance 216
0 -> 122 -> 68 -> 78 -> 35 -> 88 -> 133 -> 56 -> 101 -> 172 -> 95 -> 131 -> 0
No.6 : Use VehicleType 0,Load 24.8,Distance 138
0 -> 58 -> 196 -> 5 -> 87 -> 109 -> 54 -> 13 -> 179 -> 89 -> 25 -> 0
No.7 : Use VehicleType 0,Load 28.9,Distance 143
0 -> 21 -> 127 -> 169 -> 161 -> 189 -> 48 -> 12 -> 14 -> 167 -> 84 -> 92 -> 130 -> 140 -> 0
No.8 : Use VehicleType 0,Load 29.4,Distance 141
0 -> 173 -> 55 -> 165 -> 47 -> 151 -> 155 -> 38 -> 91 -> 107 -> 170 -> 75 -> 123 -> 81 -> 0
No.9 : Use VehicleType 0,Load 29.4,Distance 183
0 -> 116 -> 90 -> 144 -> 114 -> 141 -> 186 -> 138 -> 36 -> 32 -> 181 -> 113 -> 174 -> 125 -> 51 -> 190 -> 0
No.10 : Use VehicleType 0,Load 28.7,Distance 112
0 -> 74 -> 10 -> 6 -> 143 -> 45 -> 79 -> 132 -> 34 -> 82 -> 145 -> 2 -> 85 -> 71 -> 0
No.11 : Use VehicleType 0,Load 24.5,Distance 122
0 -> 142 -> 50 -> 108 -> 139 -> 23 -> 59 -> 62 -> 70 -> 39 -> 27 -> 0
No.12 : Use VehicleType 0,Load 29.7,Distance 160
0 -> 153 -> 193 -> 102 -> 136 -> 20 -> 162 -> 44 -> 184 -> 148 -> 65 -> 175 -> 11 -> 112 -> 0
No.13 : Use VehicleType 0,Load 9.3,Distance 73
0 -> 154 -> 41 -> 80 -> 0
No.14 : Use VehicleType 0,Load 10.3,Distance 101
0 -> 158 -> 149 -> 1 -> 46 -> 100 -> 0
No.15 : Use VehicleType 0,Load 12,Distance 104
0 -> 171 -> 57 -> 19 -> 176 -> 152 -> 0
No.16 : Use VehicleType 0,Load 7.9,Distance 87
0 -> 166 -> 7 -> 22 -> 0
No.17 : Use VehicleType 0,Load 11.7,Distance 81
0 -> 128 -> 4 -> 118 -> 185 -> 163 -> 69 -> 0
Check_Ans = 2244
Total Running Time = 520.234
*/