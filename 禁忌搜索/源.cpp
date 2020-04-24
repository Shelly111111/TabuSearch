//*****************************************************************

//���������㷨��⳵��·������

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

//************************************************************
//����·���滮R��Ŀ�꺯��ֵ
double Calculation(Route_Type R[])
{
    double D = 0;
    //��ÿ��·��������Լ������������������
    for (int i = 1; i <= Vehicle_Number; ++i)
        if (R[i].V.size() > 2)
        {
            double ArriveTime = 0;
            if (R[i].Load > param.vehicle[R[i].VT].MaxLoad || R[i].Dis > param.vehicle[R[i].VT].MaxMileage)
                return INF;
            for (int j = 1; j < R[i].V.size(); j++)
            {
                ArriveTime += Graph[R[i].V[j - 1].Number][R[i].V[j].Number] / param.Speed * 60;
                if (ArriveTime > R[i].V[j].End)
                    return INF;
                ArriveTime += param.ServiceTime;
            }
            D += R[i].Dis;
        }
    return D;
}

//************************************************************
//����·���滮R�Ƿ���������Լ��,�Ƿ���ȷ
bool Check(Route_Type R[])
{
    //����Ƿ���������Լ����ʱ�䴰Լ��
    for (int i = 1; i <= Vehicle_Number; ++i)
    {
        if (R[i].VT == param.Vehicle_type)
            break;
        if (R[i].V.size() > 2)
        {
            double ArriveTime = 0.0;
            if (R[i].Load > param.vehicle[R[i].VT].MaxLoad || R[i].Dis > param.vehicle[R[i].VT].MaxMileage)
                return false;
            for (int j = 1; j < R[i].V.size() - 1; j++)
            {
                ArriveTime += Graph[R[i].V[j - 1].Number][R[i].V[j].Number] / param.Speed * 60.0;
                if (ArriveTime != R[i].V[j].SerBegin)
                    return false;
                ArriveTime += param.ServiceTime;
            }
        }
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
        Route_Ans[i].VT = Route[i].VT;
        Route_Ans[i].V.clear();
        for (int j = 0; j < Route[i].V.size(); ++j)
            Route_Ans[i].V.push_back(Route[i].V[j]);
    }
}

//************************************************************
//�����ʼ·��
bool Construction()
{
    int Sizeof_Delivery_Set = Needs.size();
    int Current_Route = 1;
    random_shuffle(Needs.begin(), Needs.end());
    //����������Լ��ΪĿ�ĵ������ʼ��
    //�������ѡһ���ڵ���뵽��m��·���У�����������Լ����������m+1��·��
    //�������Լ����������Ըõ����
    for (int Delivery_Set = 0; Delivery_Set < Sizeof_Delivery_Set; Delivery_Set++)
    {
        int C = Needs[Delivery_Set];
        //����ǰ������Ľڵ㸳ֵΪ��ĩ�ڵ�ֵ,·��+1,����ʱ���ʼ��
        if (Route[Current_Route].Load + Delivery[C].Demand > param.vehicle[Route[Current_Route].VT].MaxLoad)
            Current_Route++;
        if (Current_Route >= Delivery_num)
            return false;
        //�ҵ���ǰ·����һ���ɲ����λ�ò���õ�
        for (int i = 1; i < Route[Current_Route].V.size(); i++)
        {
            double Differ_Dis = Graph[Route[Current_Route].V[i - 1].Number][Delivery[C].Number] + Graph[Route[Current_Route].V[i].Number][Delivery[C].Number]
                - Graph[Route[Current_Route].V[i - 1].Number][Route[Current_Route].V[i].Number];
            if (Route[Current_Route].Dis + Differ_Dis > param.vehicle[Route[Current_Route].VT].MaxMileage)
                continue;
            double ArriveTime = Route[Current_Route].V[i - 1].SerBegin + param.ServiceTime + 
                Graph[Route[Current_Route].V[i - 1].Number][Delivery[C].Number] / param.Speed * 60.0;
            //������㣬�Ƿ�����ʱ�䴰Լ��
            if (ArriveTime > Delivery[C].End)
                break;
            //���������Ƿ�����ʱ�䴰Լ������
            int P = 0;
            for (int t = i; t < Route[Current_Route].V.size(); t++)
                if (Route[Current_Route].V[t].SerBegin + Differ_Dis / param.Speed * 60.0 + param.ServiceTime > Route[Current_Route].V[t].End)
                    P = t;
            if (P != 0)
            {
                i = P;
                continue;
            }
            //�Ըõ����
            Route[Current_Route].Load += Delivery[C].Demand;
            Route[Current_Route].Dis += Differ_Dis;
            Delivery[C].R = Current_Route;
            Delivery[C].SerBegin = ArriveTime;
            //���º����ڵ�ķ���ʼʱ��
            for (int t = i; t < Route[Current_Route].V.size() - 1; t++)
                Route[Current_Route].V[t].SerBegin += Differ_Dis / param.Speed * 60.0 + param.ServiceTime;
            Route[Current_Route].V.insert(Route[Current_Route].V.begin() + i, Delivery[C]);
            break;
        }
        //����Ҳ����ɲ���Ľڵ㣬����뵽��·����
        if (Delivery[C].R == 0)
        {
            Current_Route++;
            if (Current_Route >= Delivery_num)
                return false;
            Route[Current_Route].Load += Delivery[C].Demand;
            Route[Current_Route].Dis = Graph[Route[Current_Route].V[0].Number][Delivery[C].Number] + Graph[Route[Current_Route].V[1].Number][Delivery[C].Number];
            Delivery[C].R = Current_Route;
            Delivery[C].SerBegin = Graph[Route[Current_Route].V[0].Number][Delivery[C].Number] / param.Speed * 60.0;
            Route[Current_Route].V.insert(Route[Current_Route].V.begin() + 1, Delivery[C]);
        }
    }
    Copy_Route();
    int D = 0;
    for (int i = 1; i <= Vehicle_Number; ++i)
        D += Route[i].Dis;
    Ans = D;
}

//************************************************************
//��2-opt�����Ż�·��
//�ߵ��������±�begin��end��Ԫ��λ��
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

//2-opt�㷨�Ż�·��
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
                double ArriveTime = Initial_Time;
                swap_element(r, begin, end);
                for (int i = 1; i < _l; i++)
                {
                    distance += Graph[Route[r].V[i - 1].Number][Route[r].V[i].Number];
                    //���㵽��ʱ���Ƿ�����ʱ��Լ��
                    ArriveTime += Graph[Route[r].V[i - 1].Number][Route[r].V[i].Number] / param.Speed * 60.0;
                    if (ArriveTime > Route[r].V[i].End)
                    {
                        distance = INF;
                        break;
                    }
                    ArriveTime += param.ServiceTime;
                }
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
        //���·���ʼʱ��
        for (int i = Bestbegin; i < _l - 1; i++)
            Route[r].V[i].SerBegin = Route[r].V[i - 1].SerBegin + param.ServiceTime + Graph[Route[r].V[i - 1].Number][Route[r].V[i].Number] / param.Speed * 60.0;
    }
}

//************************************************************
//��������
void Tabu_Search()
{
    //��ʼ�����ɱ�
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
        //Relocation�������������ҵ�����Ҫ������Ų��뷽��
        for (int K = 0; K < Needs.size(); ++K)
        {
            int i = Needs[K];
            for (int j = 1; j < Route[Delivery[i].R].V.size(); ++j)
                if (Route[Delivery[i].R].V[j].Number == Delivery[i].Number)
                {
                    P = j;
                    break;
                }
            //�ӽڵ�ԭ·����ȥ���ýڵ������
            Route[Delivery[i].R].Load -= Delivery[i].Demand;
            //�ӽڵ�ԭ·����ȥ���ýڵ�����ɵ�·��������
            Route[Delivery[i].R].Dis = Route[Delivery[i].R].Dis - Graph[Route[Delivery[i].R].V[P - 1].Number][Delivery[i].Number]
                - Graph[Delivery[i].Number][Route[Delivery[i].R].V[P + 1].Number] + Graph[Route[Delivery[i].R].V[P - 1].Number][Route[Delivery[i].R].V[P + 1].Number];
            //�ӽڵ�ԭ·����ȥ���ڵ�
            Delivery[i].SerBegin = Route[Delivery[i].R].V[P].SerBegin;
            Route[Delivery[i].R].V.erase(Route[Delivery[i].R].V.begin() + P);
            for (int j = 1; j <= Vehicle_Number; ++j)
            {
                if (Route[j].VT == param.Vehicle_type)
                    break;
                //���ɲ������������Ϊ��ֹʹ���µĳ���
                if (((Route[j].V.size() > 2 && Tabu[i][j] <= Iteration) || (Route[j].V.size() == 2 && TabuCreate[i] <= Iteration)) && Delivery[i].R != j)
                    for (int l = 1; l < Route[j].V.size(); ++l)
                    {
                        //�жϼ��Ͻ����Ƿ�ᳬ������
                        if (Route[j].Load + Delivery[i].Demand > param.vehicle[Route[j].VT].MaxLoad)
                            break;
                        double Differ_Dis = Graph[Route[j].V[l - 1].Number][Delivery[i].Number] + Graph[Route[j].V[l].Number][Delivery[i].Number]
                            - Graph[Route[j].V[l - 1].Number][Route[j].V[l].Number];
                        if (Route[j].Dis + Differ_Dis > param.vehicle[Route[j].VT].MaxMileage)
                            continue;
                        double ArriveTime = Route[j].V[l - 1].SerBegin + param.ServiceTime +
                            Graph[Route[j].V[l - 1].Number][Delivery[i].Number] / param.Speed * 60.0;
                        //������㣬�Ƿ�����ʱ�䴰Լ��
                        if (ArriveTime > Delivery[i].End)
                            break;
                        //���������Ƿ�����ʱ�䴰Լ������
                        int TempP = 0;
                        for (int t = l; t < Route[j].V.size() - 1; t++)
                            if (Route[j].V[t].SerBegin + Differ_Dis / param.Speed * 60.0 + param.ServiceTime > Route[j].V[t].End)
                                TempP = t;
                        if (TempP != 0)
                        {
                            l = TempP;
                            continue;
                        }
                        //�ڽڵ���·���м��ϸýڵ������
                        Route[j].Load += Delivery[i].Demand;
                        //�ڽڵ���·���м��ϸýڵ���������ɵ�·�����Ͽ�ԭ·��
                        Route[j].Dis += Differ_Dis;
                        //�ڽڵ���·���в���ڵ�
                        Route[j].V.insert(Route[j].V.begin() + l, Delivery[i]);
                        double TempV = Calculation(Route);
                        if (TempV < BestV)
                        {
                            BestV = TempV;
                            BestDelivery = i, BestRoute = j, BestPoint = l;
                        }
                        //�ڵ���·����ԭ
                        Route[j].V.erase(Route[j].V.begin() + l);
                        Route[j].Load -= Delivery[i].Demand;
                        Route[j].Dis = Route[j].Dis + Graph[Route[j].V[l - 1].Number][Route[j].V[l].Number]
                            - Graph[Route[j].V[l - 1].Number][Delivery[i].Number] - Graph[Route[j].V[l].Number][Delivery[i].Number];
                    }
            }
            //�ڵ�ԭ·����ԭ
            Route[Delivery[i].R].V.insert(Route[Delivery[i].R].V.begin() + P, Delivery[i]);
            Route[Delivery[i].R].Load += Delivery[i].Demand;
            Route[Delivery[i].R].Dis = Route[Delivery[i].R].Dis + Graph[Route[Delivery[i].R].V[P - 1].Number][Delivery[i].Number]
                + Graph[Delivery[i].Number][Route[Delivery[i].R].V[P + 1].Number] - Graph[Route[Delivery[i].R].V[P - 1].Number][Route[Delivery[i].R].V[P + 1].Number];
        }
        //��BestDelivery�����ڸýڵ����ڵ�·���У�ֱ����������ʱ���������²��뵽��·����
        if (Route[BestRoute].V.size() == 2)
            TabuCreate[BestDelivery] = Iteration + 2 * Tabu_tenure + rand() % 10;
        Tabu[BestDelivery][Delivery[BestDelivery].R] = Iteration + Tabu_tenure + rand() % 10;
        for (int i = 1; i < Route[Delivery[BestDelivery].R].V.size(); ++i)
            if (Route[Delivery[BestDelivery].R].V[i].Number == Delivery[BestDelivery].Number)
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
        //��������ѭ������ѡ�Ľ���������µ�����·���滮
        //���¸ı���ĸ�����·�������أ����볤�ȵ�����,����ʼʱ��
        //���ý���Դ·����ȥ��
        Route[Delivery[BestDelivery].R].Load -= Delivery[BestDelivery].Demand;
        double Differ_D = Graph[Route[Delivery[BestDelivery].R].V[P - 1].Number][Route[Delivery[BestDelivery].R].V[P + 1].Number] -
            Graph[Route[Delivery[BestDelivery].R].V[P - 1].Number][Delivery[BestDelivery].Number] -
            Graph[Delivery[BestDelivery].Number][Route[Delivery[BestDelivery].R].V[P + 1].Number];
        Route[Delivery[BestDelivery].R].Dis += Differ_D;
        //���º����ڵ�ķ���ʼʱ��
        for (int t = P + 1; t < Route[Delivery[BestDelivery].R].V.size() - 1; t++)
            Route[Delivery[BestDelivery].R].V[t].SerBegin += Differ_D / param.Speed * 60.0 - param.ServiceTime;
        Route[Delivery[BestDelivery].R].V.erase(Route[Delivery[BestDelivery].R].V.begin() + P);
        //����·���в���ý��
        Differ_D = Graph[Route[BestRoute].V[BestPoint - 1].Number][Delivery[BestDelivery].Number] + Graph[Route[BestRoute].V[BestPoint].Number][Delivery[BestDelivery].Number]
            - Graph[Route[BestRoute].V[BestPoint - 1].Number][Route[BestRoute].V[BestPoint].Number];
        Route[BestRoute].Dis += Differ_D;
        Route[BestRoute].Load += Delivery[BestDelivery].Demand;
        //���º����ڵ�ķ���ʼʱ��
        Delivery[BestDelivery].SerBegin = Route[BestRoute].V[BestPoint - 1].SerBegin + param.ServiceTime +
            Graph[Route[BestRoute].V[BestPoint - 1].Number][Delivery[BestDelivery].Number] / param.Speed * 60.0;
        for (int t = BestPoint; t < Route[BestRoute].V.size() - 1; t++)
            Route[BestRoute].V[t].SerBegin += Differ_D / param.Speed * 60.0 + param.ServiceTime;
        //���±������Ľڵ�����·�����
        Delivery[BestDelivery].R = BestRoute;
        Route[BestRoute].V.insert(Route[BestRoute].V.begin() + BestPoint, Delivery[BestDelivery]);
        //2-opt���Ӷ�BestRoute·���Ż�
        opt(BestRoute);
        BestV = Calculation(Route);
        //�����ǰ��Ϸ��ҽ�������´洢���
        if (Ans > BestV)
        {
            Copy_Route();
            Ans = BestV;
        }
        //if (Iteration % 10 == 0)
            //cout << "Iteration: " << Iteration << ", BestV: " << BestV << ", Ans: " << Ans << endl;
    }
}

//************************************************************
int main()
{
    clock_t Start, Finish;
    Start = clock();
    srand((unsigned)time(NULL));
    Init_Param();
    //cout << "InitParam is Ok!" << endl;
    if (!get_Date_in_txt(Delivery_num))
        return -1;
    //get_Date_in_json(Delivery_num);
    //cout << "Initialization is Ok!" << endl;
    if (!Construction())
    {
        cerr << "{\"typeId\":1,\"data\":Error:Vehicle parameter setting is unreasonable, please check the parameters.}" << endl;
        return -1;
    }
    //cout << "Initial path construction is Ok!" << endl;
    Tabu_Search();
    cout << "Tabu Search is Ok!" << endl;
    //Output_by_json(Route_Ans);
    Output(Route_Ans);
    //if (!Check(Route))
        //cout << "Error" << endl;
    Finish = clock();
    cout << "Total Running Time = " << ( Finish - Start ) / 1000.0 << endl;
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