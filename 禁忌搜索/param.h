#pragma once
//************************************************************

//��������

//************************************************************

#include <vector>
#define INF 0x3ffffff
#define Delivery_Number 2000   //�����г��ֿ���������˿ͽڵ����
#define Tabu_tenure 20   //����ʱ��
#define Initial_Time 0

int Delivery_num;
int Vehicle_Number;   //�����޳����������ƣ���˽�������Ϊ�˿�����
int Tabu[Delivery_Number + 10][Delivery_Number + 10];   //���ɱ����ڽ��ɽڵ�������
int TabuCreate[Delivery_Number + 10];   //���ɱ����ڽ�����չ��·����ʹ���³���
std::vector<int> Needs;//���ڼ�¼��Ҫ���͵Ľ��
double Ans;
double Graph[Delivery_Number + 10][Delivery_Number + 10];
int lastVN;

struct Vehicle_Type
{
    double MaxLoad;     //�����
    double MaxMileage;  //���Ѳ�����
    int Num;        //������Ŀ
};

struct Param
{
    int Iter_Epoch;     //��������
    int Vehicle_type;   //����������Ŀ
    std::vector<Vehicle_Type> vehicle;
    double ServiceTime;     //�����͵����ʱ��
    double Speed;       //������ʻ�ٶ�
}param;

struct Delivery_Type
{
    int Number;   //�ڵ�������
    int R;   //�ڵ���������·�����
    double X, Y;   //�ڵ��������
    double End;   //�ڵ㱻���ʵ�����ʱ��
    double SerBegin;//�ڵ㱻��ʼ���ʵ�ʱ��
    double Demand;   //�ڵ��������
} Delivery[Delivery_Number + 10];   //����ʱ���������Ľڵ��±�Ϊ1���˿ͽڵ��±�Ϊ2-max

struct Route_Type
{
    double Load;   //����·��װ����
    double Dis;   //����·���ܳ���
    int VT;      //���ֳ�������
    std::vector<Delivery_Type> V;   //����·���Ϲ˿ͽڵ�����
} Route[Delivery_Number + 10], Route_Ans[Delivery_Number + 10];   //����·�������������ŵĳ���·��