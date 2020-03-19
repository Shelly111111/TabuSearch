//*****************************************************************

//���������㷨����ʱ�䴰�ĳ���·������

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
#define Customer_Number 200   //�����г��ֿ���������˿ͽڵ����
#define Itinerant_Mileage 400   //���Ѳ����̣���λkm
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
    double Begin, End, Service;   //�ڵ㱻���ʵ�����ʱ�䣬����ʱ���Լ�����ʱ��
    double Demand;   //�ڵ��������
} Customer[Customer_Number + 10];   //�ֿ�ڵ���Ϊ1���˿ͽڵ���Ϊ2-101

struct Route_Type
{
    double Load;   //����·��װ����
    double SubT;   //����·��Υ�����ڵ�ʱ�䴰Լ��ʱ���ܺ�
    double Dis;   //����·���ܳ���
    int VT;      //���ֳ�������
    vector<Customer_Type> V;   //����·���Ϲ˿ͽڵ�����
} Route[Customer_Number + 10], Route_Ans[Customer_Number + 10];   //����·�������������ŵĳ���·��

int Vehicle_Number = Customer_Number;   //�����޳����������ƣ���˽�������Ϊ�˿�����
int Tabu[Customer_Number + 10][Customer_Number + 10];   //���ɱ����ڽ��ɽڵ�������
int TabuCreate[Customer_Number + 10];   //���ɱ����ڽ�����չ��·����ʹ���³���
double Ans;
//double Alpha = 1, Beta = 1, Sita = 0.5;
double Graph[Customer_Number + 10][Customer_Number + 10];
int Customer_num;
//************************************************************
//��ʼ������
/*
train_opt={
    'epoch' : 4,#��������
    'data_path' : './graph.json',
    'log_file' : './log',
    'checkpoint' : False,#����
    'max' : 9999999,
    'file_path' : './checkpoint.json',
    'dataDict' : {
        'Vehicle_type_num' : 2,
        0 : {
            'MaxLoad' : 30.0,#���������
            'MaxMileage' : 400,#���Ѳ�����
            'Num' : 12
            },
        1 : {
            'MaxLoad' : 12.0,
            'MaxMileage' : 400,#���Ѳ�����
            'Num' : 10
            },
        'ServiceTime' : 5,#����ʱ�䣬��λ����
        'MaxServiceTime' : 480,#��λ����
        'speed' : 60#��λǧ��/Сʱ
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
//����ͼ�ϸ��ڵ��ľ���
double Distance(Customer_Type C1, Customer_Type C2)
{
    return sqrt((C1.X - C2.X) * (C1.X - C2.X) + (C1.Y - C2.Y) * (C1.Y - C2.Y));
}

//************************************************************
//����·���滮R��Ŀ�꺯��ֵ
double Calculation(Route_Type R[], int Cus, int NewR)
{
    //Ŀ�꺯����Ҫ������������ɣ�D·���ܳ��ȣ��Ż�Ŀ�꣩��Q��������Լ��������T����ʱ�䴰Լ������
    //Ŀ�꺯���ṹΪ f(R) = D + Alpha * Q + Beta * T, ��һ��Ϊ������С��Ŀ�꣬������Ϊ�ͷ�����
    //����Alpha��BetaΪ�ɱ�������ֱ���ݵ�ǰ���Ƿ���������Լ�������б仯����Check�����и��£�����Check���ÿ�ֵ����õ��Ľ⣩
    //double Q = 0;
    //double T = 0;
    double D = 0;
    //���㵥��·����������Լ��������
    for (int i = 1; i <= Vehicle_Number; ++i)
    {
        if (R[i].VT == param.Vehicle_type)
            break;
        if (R[i].V.size() > 2 && R[i].Load > param.vehicle[R[i].VT].MaxLoad)
            return INF;
    }
            //Q = Q + R[i].Load - Capacity;
    //���㵥��·���ϸ����ڵ㳬��ʱ�䴰Լ���������������½����Ƴ��Ͳ���ڵ����������·����
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
    //����·���ܳ���
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
//����·���滮R�Ƿ���������Լ��
bool Check(Route_Type R[])
{
    //double Q = 0;
    //double T = 0;
    //double D = 0;
    //����Ƿ���������Լ����ʱ�䴰Լ��
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
    //�ֱ����Լ��������������Alpha��Betaֵ
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
//��·���滮Route�����ݸ��Ƹ�·���滮Route_Ans
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
//������
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
    //�����������Ƿ���ȷ
    double Check_Ans = 0;
    for (int i = 1; i <= Vehicle_Number; ++i)
        for (int j = 1; j < R[i].V.size(); ++j)
            Check_Ans += Graph[R[i].V[j - 1].Number][R[i].V[j].Number];
    cout << "Check_Ans = " << Check_Ans << endl;
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
    //��ʼ��ÿ��·����Ĭ��·����βΪ�ֿ⣬���ײֿ���������ʱ���Ϊԭ�ֿ�����ʱ�䣬β�ֿ����Ϊԭ�ֿ�����ʱ��
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
//�����ʼ·��
bool Construction()
{
    int Customer_Set[Customer_Number + 10];
    for (int i = 1; i <= Customer_num + 1; ++i)
        Customer_Set[i] = i + 1;
    int Sizeof_Customer_Set = Customer_num;
    int Current_Route = 1;
    //����������Լ��ΪĿ�ĵ������ʼ��
    //�������ѡһ���ڵ���뵽��m��·���У�����������Լ����������m+1��·��
    //�Ҳ���·����λ���ɸ�·�����Ѵ��ڵĸ��ڵ�����ʱ����������
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
        //����ǰ������Ľڵ㸳ֵΪ��ĩ�ڵ�ֵ,����������1
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
    //��ʼ�����㳬������Լ���������ͳ���ʱ�䴰Լ��������
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
//��������
void Tabu_Search()
{
    //����������ȡ�������ӣ�����һ��·����ѡ��һ����뵽��һ��·����
    //�ڸò������γɵ�������ѡȡʹĿ�꺯����С�ķǽ��ɽ�������������ӷ����������Ľ�
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
        for (int i = 2; i <= Customer_num + 1; ++i)
        {
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
                        if (Pre_Dis > Itinerant_Mileage)
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
        //���¸ı���ĸ�����·�������أ����볤�ȣ�����ʱ�䴰������
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
        //����ʱ�䴰
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