//*****************************************************************

//���������㷨����ʱ�䴰�ĳ���·������(VRPTW_TS)

//*****************************************************************

//Reference

//J-F Cordeau, Laporte, G., & Mercier, A. (2001). A Unified Tabu Search Heuristic for Vehicle Routing Problems with Time Windows. The Journal of the Operational Research Society, 52(8), 928-936. Retrieved from http://www.jstor.org/stable/822953

//Solomon, M. (1987). Algorithms for the Vehicle Routing and Scheduling Problems with Time Window Constraints. Operations Research, 35(2), 254-265. Retrieved from http://www.jstor.org/stable/170697

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
#define Customer_Number 200   //�����г��ֿ�����Ĺ˿ͽڵ����
#define Capacity 30   //����������
#define Iter_Max 400   //��������������
#define Tabu_tenure 20   //����ʱ��

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
    vector<Customer_Type> V;   //����·���Ϲ˿ͽڵ�����
} Route[Customer_Number + 10], Route_Ans[Customer_Number + 10];   //����·�������������ŵĳ���·��

int Vehicle_Number = Customer_Number;   //�����޳����������ƣ���˽�������Ϊ�˿�����
int Tabu[Customer_Number + 10][Customer_Number + 10];   //���ɱ����ڽ��ɽڵ�������
int TabuCreate[Customer_Number + 10];   //���ɱ����ڽ�����չ��·����ʹ���³���
double Ans;
double Alpha = 1, Beta = 1, Sita = 0.5;
double Graph[Customer_Number + 10][Customer_Number + 10];
int Customer_num;

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
    double Q = 0;
    double T = 0;
    double D = 0;
    //���㵥��·����������Լ��������
    for (int i = 1; i <= Vehicle_Number; ++i)
        if (R[i].V.size() > 2 && R[i].Load > Capacity)
            Q = Q + R[i].Load - Capacity;
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
    for (int i = 1; i <= Vehicle_Number; ++i)
        T += R[i].SubT;
    //����·���ܳ���
    for (int i = 1; i <= Vehicle_Number; ++i)
        D += R[i].Dis;
    return D + Alpha * Q + Beta * T;
}

//************************************************************
//����·���滮R�Ƿ���������Լ��
bool Check(Route_Type R[])
{
    double Q = 0;
    double T = 0;
    double D = 0;
    //����Ƿ���������Լ��
    for (int i = 1; i <= Vehicle_Number; ++i)
        if (R[i].V.size() > 2 && R[i].Load > Capacity)
            Q = Q + R[i].Load - Capacity;
    //����Ƿ�����ʱ�䴰Լ��
    for (int i = 1; i <= Vehicle_Number; ++i)
        T += R[i].SubT;
    //�ֱ����Լ��������������Alpha��Betaֵ
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
                cout << R[i].V[j].Number << " -> ";
            cout << R[i].V[R[i].V.size() - 1].Number << endl;
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
    for (int i = 1; i < Customer_num + 1; i++)
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
    for (int i = 1; i <= Vehicle_Number; ++i)
    {
        if (!Route[i].V.empty())
            Route[i].V.clear();
        Route[i].V.push_back(Customer[1]);
        Route[i].V.push_back(Customer[1]);
        Route[i].V[0].End = Route[i].V[0].Begin;
        Route[i].V[1].Begin = Route[i].V[1].End;
        Route[i].Load = 0;
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
void Construction()
{
    int Customer_Set[Customer_Number + 10];
    //for (int i = 1; i <= Customer_Number + 1; ++i)
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
        if (Route[Current_Route].Load + Customer[C].Demand > Capacity)
            Current_Route++;
        for (int i = 0; i < Route[Current_Route].V.size() - 1; i++)
            if ((Route[Current_Route].V[i].Begin <= Customer[C].Begin) && (Customer[C].Begin <= Route[Current_Route].V[i + 1].Begin))
            {
                Route[Current_Route].V.insert(Route[Current_Route].V.begin() + i + 1, Customer[C]);
                Route[Current_Route].Load += Customer[C].Demand;
                Customer[C].R = Current_Route;
                break;
            }
    }
    //��ʼ�����㳬������Լ���������ͳ���ʱ�䴰Լ��������
    for (int i = 1; i <= Vehicle_Number; ++i)
    {
        double ArriveTime = Route[i].V[0].Begin;
        Route[i].SubT = 0;
        Route[i].Dis = 0;
        for (int j = 1; j < Route[i].V.size(); ++j)
        {
            ArriveTime = ArriveTime + Route[i].V[j - 1].Service + Graph[Route[i].V[j - 1].Number][Route[i].V[j].Number];
            Route[i].Dis += Graph[Route[i].V[j - 1].Number][Route[i].V[j].Number];
            if (ArriveTime > Route[i].V[j].End)
                Route[i].SubT = Route[i].SubT + ArriveTime - Route[i].V[j].End;
            else if (ArriveTime < Route[i].V[j].Begin)
                ArriveTime = Route[i].V[j].Begin;
        }
    }
}

//************************************************************
//��������
void Tabu_Search()
{
    //����������ȡ�������ӣ�����һ��·����ѡ��һ����뵽��һ��·����
    //�ڸò������γɵ�������ѡȡʹĿ�꺯����С�ķǽ��ɽ�������������ӷ����������Ľ�
    double Temp1;
    double Temp2;
    //for (int i = 2; i <= Customer_Number + 1; ++i)
    for (int i = 2; i <= Customer_num + 1; ++i)
    {
        for (int j = 1; j <= Vehicle_Number; ++j)
            Tabu[i][j] = 0;
        TabuCreate[i] = 0;
    }
    for (int Iteration = 0; Iteration < Iter_Max; Iteration++)
    {        
        int BestC = 0, BestR = 0, BestP = 0, P = 0;
        double BestV = INF;
        //for (int i = 2; i <= Customer_Number + 1; ++i)
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
                //���ɲ������������Ϊ��ֹʹ���µĳ���
                if ((Route[j].V.size() > 2 && Tabu[i][j] <= Iteration) || (Route[j].V.size() == 2 && TabuCreate[i] <= Iteration))
                {
                    for (int l = 1; l < Route[j].V.size(); ++l)
                        if (Customer[i].R != j)
                        {
                            //�ڽڵ���·���м��ϸýڵ������
                            Route[j].Load += Customer[i].Demand;
                            //�ڽڵ���·���м��ϸýڵ���������ɵ�·�����Ͽ�ԭ·��
                            Route[j].Dis = Route[j].Dis - Graph[Route[j].V[l - 1].Number][Route[j].V[l].Number]
                                + Graph[Route[j].V[l - 1].Number][Customer[i].Number] + Graph[Route[j].V[l].Number][Customer[i].Number];
                            //�ڽڵ���·���в���ڵ�
                            Route[j].V.insert(Route[j].V.begin() + l, Customer[i]);
                            Temp1 = Route[Customer[i].R].SubT;
                            Temp2 = Route[j].SubT;
                            double TempV = Calculation(Route, i, j);
                            if (TempV < BestV)
                            {
                                BestV = TempV;
                                BestC = i, BestR = j, BestP = l;
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
        if (Route[BestR].V.size() == 2)
            TabuCreate[BestC] = Iteration + 2 * Tabu_tenure + rand() % 10;
        Tabu[BestC][Customer[BestC].R] = Iteration + Tabu_tenure + rand() % 10;
        for (int i = 1; i < Route[Customer[BestC].R].V.size(); ++i)
            if (Route[Customer[BestC].R].V[i].Number == BestC)
            {
                P = i;
                break;
            }
        //��������ѭ������ѡ�Ľ���������µ�����·���滮
        //���¸ı���ĸ�����·�������أ����볤�ȣ�����ʱ�䴰������
        Route[Customer[BestC].R].Load -= Customer[BestC].Demand;
        Route[Customer[BestC].R].Dis = Route[Customer[BestC].R].Dis - Graph[Route[Customer[BestC].R].V[P - 1].Number][Route[Customer[BestC].R].V[P].Number]
            - Graph[Route[Customer[BestC].R].V[P].Number][Route[Customer[BestC].R].V[P + 1].Number] + Graph[Route[Customer[BestC].R].V[P - 1].Number][Route[Customer[BestC].R].V[P + 1].Number];
        Route[Customer[BestC].R].V.erase(Route[Customer[BestC].R].V.begin() + P);
        Route[BestR].Dis = Route[BestR].Dis - Graph[Route[BestR].V[BestP - 1].Number][Route[BestR].V[BestP].Number]
            + Graph[Route[BestR].V[BestP - 1].Number][Customer[BestC].Number] + Graph[Route[BestR].V[BestP].Number][Customer[BestC].Number];
        Route[BestR].Load += Customer[BestC].Demand;
        Route[BestR].V.insert(Route[BestR].V.begin() + BestP, Customer[BestC]);
        double ArriveTime = 0;
        Route[BestR].SubT = 0;
        for (int j = 1; j < Route[BestR].V.size(); ++j)
        {
            ArriveTime = ArriveTime + Route[BestR].V[j - 1].Service + Graph[Route[BestR].V[j - 1].Number][Route[BestR].V[j].Number];
            if (ArriveTime > Route[BestR].V[j].End)
                Route[BestR].SubT = Route[BestR].SubT + ArriveTime - Route[BestR].V[j].End;
            else if (ArriveTime < Route[BestR].V[j].Begin)
                ArriveTime = Route[BestR].V[j].Begin;
        }
        ArriveTime = 0;
        Route[Customer[BestC].R].SubT = 0;
        for (int j = 1; j < Route[Customer[BestC].R].V.size(); ++j)
        {
            ArriveTime = ArriveTime + Route[Customer[BestC].R].V[j - 1].Service + Graph[Route[Customer[BestC].R].V[j - 1].Number][Route[Customer[BestC].R].V[j].Number];
            if (ArriveTime > Route[Customer[BestC].R].V[j].End)
                Route[Customer[BestC].R].SubT = Route[Customer[BestC].R].SubT + ArriveTime - Route[Customer[BestC].R].V[j].End;
            else if (ArriveTime < Route[Customer[BestC].R].V[j].Begin)
                ArriveTime = Route[Customer[BestC].R].V[j].Begin;
        }
        //���±������Ľڵ�����·�����
        Customer[BestC].R = BestR;
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