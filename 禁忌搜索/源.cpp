//*****************************************************************

//���������㷨����ʱ�䴰�ĳ���·������

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
#define Customer_Number 200   //�����г��ֿ���������˿ͽڵ����
#define Tabu_tenure 20   //����ʱ��

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
    int Number;   //�ڵ�������
    int R;   //�ڵ���������·�����
    double X, Y;   //�ڵ��������
    double Demand;   //�ڵ��������
} Customer[Customer_Number + 10];   //�ֿ�ڵ���Ϊ1���˿ͽڵ���Ϊ2-101

struct Route_Type
{
    double Load;   //����·��װ����
    double SubT;   //����·������ʱ���ܺ�
    double Dis;   //����·���ܳ���
    int VT;      //���ֳ�������
    vector<Customer_Type> V;   //����·���Ϲ˿ͽڵ�����
} Route[Customer_Number + 10], Route_Ans[Customer_Number + 10];   //����·�������������ŵĳ���·��

int Vehicle_Number = Customer_Number;   //�����޳����������ƣ���˽�������Ϊ�˿�����
int Tabu[Customer_Number + 10][Customer_Number + 10];   //���ɱ����ڽ��ɽڵ�������
int TabuCreate[Customer_Number + 10];   //���ɱ����ڽ�����չ��·����ʹ���³���
vector<int> Needs;//���ڼ�¼��Ҫ���͵Ľ��
double Ans;
double Graph[Customer_Number + 10][Customer_Number + 10];
int Customer_num;
int lastVN;
//************************************************************
//��ʼ������
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
//����ͼ�ϸ��ڵ��ľ���
double Distance(Customer_Type C1, Customer_Type C2)
{
    return sqrt((C1.X - C2.X) * (C1.X - C2.X) + (C1.Y - C2.Y) * (C1.Y - C2.Y));
}

//************************************************************
//����·���滮R��Ŀ�꺯��ֵ
double Calculation(Route_Type R[], int Cus, int NewR)
{
    double D = 0;
    //���㵥��·����������Լ��������
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
//����·���滮R�Ƿ���������Լ��
bool Check(Route_Type R[])
{
    //����Ƿ���������Լ����ʱ�䴰Լ��
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
//��·���滮Route�����ݸ��Ƹ�·���滮Route_Ans
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
//������
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
    //�����������Ƿ���ȷ
    double Check_Ans = 0;
    for (int i = 1; i <= Vehicle_Number; ++i)
        for (int j = 1; j < R[i].V.size(); ++j)
            Check_Ans += Graph[R[i].V[j - 1].Number][R[i].V[j].Number];
    cout << "Check_Ans = " << Check_Ans << endl;
    if (ChecklastVN >= lastVN)
        cerr << "Wornning��Some parameters currently entered may be too small for effective operation, please check the corresponding parameter settings." << endl;
    cout << "************************************************************" << endl;
}

//************************************************************
//�Ͻ�˹����������֮�����·��
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
//���ݶ��뼰��ʼ��
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
    //��ʼ��ÿ��·����Ĭ��·����βΪ�ֿ⣬���ײֿ���������ʱ���Ϊԭ�ֿ�����ʱ�䣬β�ֿ����Ϊԭ�ֿ�����ʱ��
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
//�����ʼ·��
void Construction()
{
    int Sizeof_Customer_Set = Needs.size();
    int Current_Route = 1;
    int Customer_Set = 0;
    random_shuffle(Needs.begin(), Needs.end());
    //����������Լ��ΪĿ�ĵ������ʼ��
    //�������ѡһ���ڵ���뵽��m��·���У�����������Լ����������m+1��·��
    //�Ҳ���·����λ���ɸ�·�����Ѵ��ڵĸ��ڵ�����ʱ����������
    while (Customer_Set < Sizeof_Customer_Set)
    {
        //int K = rand() % Sizeof_Customer_Set + 1;
        int C = Needs[Customer_Set];
        Customer_Set++;
        //����ǰ������Ľڵ㸳ֵΪ��ĩ�ڵ�ֵ,����������1
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
//��������
void Tabu_Search()
{
    double Temp1;
    double Temp2;
    //��ʼ�����ɱ�
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
            //�ӽڵ�ԭ·����ȥ���ýڵ������
            Route[Customer[i].R].Load -= Customer[i].Demand;
            //�ӽڵ�ԭ·����ȥ���ýڵ�����ɵ�·��������
            Route[Customer[i].R].Dis = Route[Customer[i].R].Dis - Graph[Route[Customer[i].R].V[P - 1].Number][Route[Customer[i].R].V[P].Number]
                - Graph[Route[Customer[i].R].V[P].Number][Route[Customer[i].R].V[P + 1].Number] + Graph[Route[Customer[i].R].V[P - 1].Number][Route[Customer[i].R].V[P + 1].Number];
            //�ӽڵ�ԭ·����ȥ���ڵ�
            Route[Customer[i].R].V.erase(Route[Customer[i].R].V.begin() + P);
            for (int j = 1; j <= Vehicle_Number; ++j)
            {
                if (Route[j].VT == param.Vehicle_type)
                    break;
                //���ɲ������������Ϊ��ֹʹ���µĳ���
                if (((Route[j].V.size() > 2 && Tabu[i][j] <= Iteration) || (Route[j].V.size() == 2 && TabuCreate[i] <= Iteration)) && Customer[i].R != j)
                    for (int l = 1; l < Route[j].V.size(); ++l)
                    {
                        //�жϼ��Ͻ����Ƿ�ᳬ������
                        if (Route[j].Load + Customer[i].Demand > param.vehicle[Route[j].VT].MaxMileage)
                            break;
                        double Pre_Dis = Route[j].Dis - Graph[Route[j].V[l - 1].Number][Route[j].V[l].Number]
                            + Graph[Route[j].V[l - 1].Number][Customer[i].Number] + Graph[Route[j].V[l].Number][Customer[i].Number];
                        if (Pre_Dis > param.vehicle[Route[j].VT].MaxMileage)
                            continue;
                        //�ڽڵ���·���м��ϸýڵ������
                        Route[j].Load += Customer[i].Demand;
                        //�ڽڵ���·���м��ϸýڵ���������ɵ�·�����Ͽ�ԭ·��
                        Route[j].Dis = Pre_Dis;
                        //�ڽڵ���·���в���ڵ�
                        Route[j].V.insert(Route[j].V.begin() + l, Customer[i]);
                        Temp1 = Route[Customer[i].R].SubT;
                        Temp2 = Route[j].SubT;
                        double TempV = Calculation(Route, i, j);
                        if (TempV < BestV)
                        {
                            BestV = TempV;
                            BestCustomer = i, BestRoute = j, BestPoint = l;
                        }
                        //�ڵ���·����ԭ
                        Route[Customer[i].R].SubT = Temp1;
                        Route[j].SubT = Temp2;
                        Route[j].V.erase(Route[j].V.begin() + l);
                        Route[j].Load -= Customer[i].Demand;
                        Route[j].Dis = Route[j].Dis + Graph[Route[j].V[l - 1].Number][Route[j].V[l].Number]
                            - Graph[Route[j].V[l - 1].Number][Customer[i].Number] - Graph[Route[j].V[l].Number][Customer[i].Number];
                    }
            }
            //�ڵ�ԭ·����ԭ
            Route[Customer[i].R].V.insert(Route[Customer[i].R].V.begin() + P, Customer[i]);
            Route[Customer[i].R].Load += Customer[i].Demand;
            Route[Customer[i].R].Dis = Route[Customer[i].R].Dis + Graph[Route[Customer[i].R].V[P - 1].Number][Route[Customer[i].R].V[P].Number]
                + Graph[Route[Customer[i].R].V[P].Number][Route[Customer[i].R].V[P + 1].Number] - Graph[Route[Customer[i].R].V[P - 1].Number][Route[Customer[i].R].V[P + 1].Number];
        }
        //��BestCustomer�����ڸýڵ����ڵ�·���У�ֱ����������ʱ���������²��뵽��·����
        if (Route[BestRoute].V.size() == 2)
            TabuCreate[BestCustomer] = Iteration + 2 * Tabu_tenure + rand() % 10;
        Tabu[BestCustomer][Customer[BestCustomer].R] = Iteration + Tabu_tenure + rand() % 10;
        for (int i = 1; i < Route[Customer[BestCustomer].R].V.size(); ++i)
            if (Route[Customer[BestCustomer].R].V[i].Number == BestCustomer)
            {
                P = i;
                break;
            }
        //��������ѭ������ѡ�Ľ���������µ�����·���滮
        //���¸ı���ĸ�����·�������أ����볤�ȵ�����
        //���ý���Դ·����ȥ��
        Route[Customer[BestCustomer].R].Load -= Customer[BestCustomer].Demand;
        Route[Customer[BestCustomer].R].Dis = Route[Customer[BestCustomer].R].Dis - Graph[Route[Customer[BestCustomer].R].V[P - 1].Number][Route[Customer[BestCustomer].R].V[P].Number]
            - Graph[Route[Customer[BestCustomer].R].V[P].Number][Route[Customer[BestCustomer].R].V[P + 1].Number] + Graph[Route[Customer[BestCustomer].R].V[P - 1].Number][Route[Customer[BestCustomer].R].V[P + 1].Number];
        Route[Customer[BestCustomer].R].V.erase(Route[Customer[BestCustomer].R].V.begin() + P);
        //����·���в���ý��
        Route[BestRoute].Dis = Route[BestRoute].Dis - Graph[Route[BestRoute].V[BestPoint - 1].Number][Route[BestRoute].V[BestPoint].Number]
            + Graph[Route[BestRoute].V[BestPoint - 1].Number][Customer[BestCustomer].Number] + Graph[Route[BestRoute].V[BestPoint].Number][Customer[BestCustomer].Number];
        Route[BestRoute].Load += Customer[BestCustomer].Demand;
        Route[BestRoute].V.insert(Route[BestRoute].V.begin() + BestPoint, Customer[BestCustomer]);
        //���±������Ľڵ�����·�����
        Customer[BestCustomer].R = BestRoute;
        //�����ǰ��Ϸ��ҽ�������´洢���
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