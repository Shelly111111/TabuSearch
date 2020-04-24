//************************************************************

//数据读入及初始化

//************************************************************

#pragma once
#include "param.h"
//#include "Base64.h"
#include <fstream>
#include <stdlib.h>
#include <iostream>
#pragma comment(lib,"jsoncpp.lib")//windows
#include <json/json.h>
//#include <unistd.h>
using namespace std;

Json::Value out_vex_mapping;//顶点输出映射

//************************************************************
//初始化参数
void Init_Param()
{
    param.Iter_Epoch = 400;//迭代次数
    //param.Max_ServiceTime = 8.0;//最大服务时间
    param.ServiceTime = 5.0;//每个结点服务时间
    param.Speed = 60.0;//车辆行驶速度
    param.Vehicle_type = 2;//车辆种类数目
    param.vehicle.resize(param.Vehicle_type);
    param.vehicle[0].MaxLoad = 30.0;//车辆最大负载
    param.vehicle[0].MaxMileage = 400.0;//最大巡回里程
    param.vehicle[0].Num = 12;//每种数目
    param.vehicle[1].MaxLoad = 12.0;
    param.vehicle[1].MaxMileage = 400.0;
    param.vehicle[1].Num = Delivery_Number + 10;
    lastVN = 10;//最后一种车辆数目写这里，由于算法必须有充足的车辆才能运行，
                //所以最后一种车辆数目预先设置为max，另用一个参数保留车辆数目，
                //直到计算完毕后判断是否符合车辆限制条件
}

//************************************************************
//迪杰斯特拉算配送点到其余各个配送点之间最短距离
void Dijkstra(double(*Graph)[Delivery_Number + 10], int s, int Delivery_num)
{
    bool flag[Delivery_Number + 10];
    double distance[Delivery_Number + 10];
    for (int i = 1; i <= Delivery_num + 1; i++)
        flag[i] = false;
    for (int i = 1; i <= Delivery_num + 1; i++)
        distance[i] = INF;
    distance[s] = 0.0;
    while (true)
    {
        int v = -1;
        //从未使用过的顶点中选择一个距离最小的顶点
        for (int u = 1; u <= Delivery_num + 1; u++)
            if (!flag[u] && (v == -1 || distance[u] < distance[v]))
                v = u;
        if (v == -1)
            break;
        //将选定的顶点加入到S中, 同时进行距离更新
        flag[v] = true;
        //更新U中各个顶点到起点s的距离。之所以更新U中顶点的距离，是由于上一步中确定了k是求出最短路径的顶点，从而可以利用k来更新其它顶点的距离；例如，(s,v)的距离可能大于(s,k)+(k,v)的距离。
        for (int u = 1; u <= Delivery_num + 1; u++)
            if (distance[u] > distance[v] + Graph[v][u])
                distance[u] = distance[v] + Graph[v][u];
    }
    for (int i = 1; i <= Delivery_num + 1; i++)
        Graph[s][i] = distance[i];
}

//数据处理
void Process_Date(int Delivery_num)
{
    for (int i = 1; i <= Delivery_num + 1; ++i)
        Dijkstra(&Graph[0], i, Delivery_num);
    //初始化每条路径，默认路径收尾为仓库，且首仓库最早最晚时间均为原仓库最早时间，尾仓库则均为原仓库最晚时间
    Delivery[1].R = -1;
    int Current_VT = 0, temp = 1;
    for (int i = 1; i <= Vehicle_Number; ++i)
    {
        if (!Route[i].V.empty())
            Route[i].V.clear();
        Route[i].V.push_back(Delivery[1]);
        Route[i].V.push_back(Delivery[1]);
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
//数据录入
//从txt中录入
bool get_Date_in_txt(int& Delivery_num)
{
    ifstream ifs;
    Needs.clear();
    ifs.open("Vex.txt", ios::in);
    if (!ifs.is_open())
    {
        cerr << "Error:open file failed." << endl;
        return false;
    }
    ifs >> Delivery_num;
    Vehicle_Number = Delivery_num;
    for (int i = 1; i <= Delivery_num + 1; ++i)
    {
        ifs >> Delivery[i].Number >> Delivery[i].X >> Delivery[i].Y >> Delivery[i].Demand >> Delivery[i].End;
        Delivery[i].SerBegin = Initial_Time - param.ServiceTime;
        if (Delivery[i].Demand != 0.0)
            Needs.push_back(i);
    }
    ifs.close();
    ifs.open("Edges.txt", ios::in);
    if (!ifs.is_open())
    {
        cerr << "Error:open file failed." << endl;
        return false;
    }
    for (int i = 1; i <= Delivery_num + 1; ++i)
        for (int j = 1; j <= Delivery_num + 1; ++j)
        {
            ifs >> Graph[i][j];
            if (Graph[i][j] == 0.0)
                Graph[i][j] = INF;
        }
    ifs.close();
    Process_Date(Delivery_num);
    return true;
}

/*
//从json字符串中录入
void get_Date_in_json(int& Delivery_num,int argc, char* argv[])
{
    string vehicle_json;
    string map_json;
    int ch;
    string json_base;
    while ((ch = getopt(argc, argv, "e:T:t:s:v:g:")) != -1)
        {
            switch (ch)
            {
                    case 'e':
                            param.Iter_Epoch = atoi(optarg);//迭代次数
                            break;
                    case 'T':
                            param.Max_ServiceTime = atof(optarg);//最大服务时长
                            break;
                    case 't':
                            param.ServiceTime = atof(optarg);//单配送点服务时长
                            break;
                    case 's':
                        param.Speed = atof(optarg);//速度
                            break;
                    case 'v':
                json_base = optarg;
                Crypt::Base64::Decode(json_base,vehicle_json);//车辆信息
                        break;
                    case 'g':
                json_base = optarg;
                Crypt::Base64::Decode(json_base,map_json);//图信息
                        break;
                    case '?':
                            cout<<"{\"typeId\":1,\"data\":Unknown option: "<<(char)optopt<<".}"<<endl;
                            break;
            default:
                break;
            }
    }
    Json::Reader reader;
    Json::Value root;
    Json::Value vex_mapping;//顶点映射
    //录入车辆信息
    if (reader.parse(vehicle_json, root))
    {
        if (root.isArray())
        {
            int nArraySize = root.size();
            param.Vehicle_type = nArraySize;
            param.vehicle.resize(param.Vehicle_type);
            for (int i = 0; i < nArraySize - 1; i++)
            {
                param.vehicle[i].MaxMileage = root[i]["vehicleDistanceLimit"].asDouble();
                param.vehicle[i].MaxLoad = root[i]["vehicleLoad"].asDouble();
                param.vehicle[i].Num = root[i]["vehicleNum"].asInt();
            }
            param.vehicle[nArraySize - 1].MaxMileage = root[nArraySize - 1]["vehicleDistanceLimit"].asDouble();
            param.vehicle[nArraySize - 1].MaxLoad = root[nArraySize - 1]["vehicleLoad"].asDouble();
            param.vehicle[nArraySize - 1].Num = Delivery_Number + 10;
            lastVN = root[nArraySize - 1]["vehicleNum"].asInt();
        }
    }
    if (reader.parse(map_json, root))
    {
        //录入顶点信息
        if (root["vertexes"].isArray())
        {
            Delivery_num = root["vertexes"].size() - 1;
            Vehicle_Number = Delivery_num + 1;
            for (int i = 1; i <= Delivery_num + 1; i++)
            {
                Delivery[i].Number = i;
                vex_mapping[root["vertexes"][i - 1]["id"].asString()] = i;
                out_vex_mapping[i]["vertexId"] = root["vertexes"][i - 1]["id"].asInt();
                out_vex_mapping[i]["vertexTypeId"] = root["vertexes"][i - 1]["typeId"].asInt();
                out_vex_mapping[i]["vertexName"] = root["vertexes"][i - 1]["name"].asString();
                Delivery[i].Demand = root["vertexes"][i - 1]["need"].asDouble();
                if (Delivery[i].Demand != 0.0)
                    Needs.push_back(i);
            }
        }
        //录入边信息
        if (root["edges"].isArray())
        {
            int nArrySize = root["edges"].size();
            for (int i = 0; i < nArrySize; i++)
            {
                int p1 = vex_mapping[root["edges"][i]["pointId1"].asString()].asInt();
                int p2 = vex_mapping[root["edges"][i]["pointId2"].asString()].asInt();
                Graph[p1][p2] = root["edges"][i]["distance"].asDouble();
                Graph[p2][p1] = root["edges"][i]["distance"].asDouble();
            }
        }
    }
    for (int i = 1; i <= Delivery_num + 1; i++)
        for (int j = 1; j <= Delivery_num + 1; j++)
            if (Graph[i][j] == 0.0)
                Graph[i][j] = INF;
    Process_Date(Delivery_num);
}
*/



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
            cout << "No." << M << " : " << "Use VehicleType " << R[i].VT << ", Load " << R[i].Load << ", Distance "
                //<< R[i].Dis << ", ServeTime " << R[i].ServT << endl;
                << R[i].Dis << endl;
            cout << "[";
            for (int j = 0; j < R[i].V.size() - 1; ++j)
                cout << R[i].V[j].Number - 1 << ", ";
            cout << R[i].V[R[i].V.size() - 1].Number - 1 << "]" << endl;
            if (R[i].VT == param.Vehicle_type - 1)
                ChecklastVN++;
        }
    //检验距离计算是否正确
    double Check_Ans = 0;
    for (int i = 1; i <= Vehicle_Number; ++i)
        for (int j = 1; j < R[i].V.size(); ++j)
            Check_Ans += Graph[R[i].V[j - 1].Number][R[i].V[j].Number];
    cout << "Check_Ans = " << Check_Ans << endl;
    //cout << "Point that can't ship:[";
    //for (int i = 0; i < Remainder.size(); i++)
        //cout << Delivery[Remainder[i]].Number - 1 << ",";
    //cout << "]" << endl;
    if (ChecklastVN >= lastVN)
        cerr << "Wornning:Some parameters currently entered may be too small for effective operation, please check the corresponding parameter settings." << endl;
    cout << "************************************************************" << endl;
}

//结果输出为json
void Output_by_json(Route_Type R[])
{
    int ChecklastVN = 0;
    Json::Value json_out;
    json_out["typeId"] = 1;
    for (int i = 0; i < param.Vehicle_type - 1; i++)
    {
        Json::Value v;
        v["vehiclelLoad"] = param.vehicle[i].MaxLoad;
        v["vehicleDistanceLimit"] = param.vehicle[i].MaxMileage;
        v["vehicleNum"] = param.vehicle[i].Num;
        json_out["date"].append(v);
    }
    Json::Value v;
    v["vehiclelLoad"] = param.vehicle[param.Vehicle_type - 1].MaxLoad;
    v["vehicleDistanceLimit"] = param.vehicle[param.Vehicle_type - 1].MaxMileage;
    v["vehicleNum"] = lastVN;
    json_out["date"].append(v);
    //json格式化路径信息
    for (int i = 1; i <= Vehicle_Number; ++i)
    {
        int vt = R[i].VT;
        if (R[i].V.size() > 2)
        {
            Json::Value path;
            for (int j = 0; j < R[i].V.size(); ++j)
            {
                Json::Value Node;
                Node["vertexId"] = out_vex_mapping[R[i].V[j].Number]["vertexId"].asInt();
                Node["vertexTypeId"] = out_vex_mapping[R[i].V[j].Number]["vertexTypeId"].asInt();
                Node["vertexName"] = out_vex_mapping[R[i].V[j].Number]["vertexName"].asString();
                path.append(Node);
            }
            json_out["date"][vt]["path"].append(path);
            if (R[i].VT == param.Vehicle_type - 1)
                ChecklastVN++;
        }
    }
    if (ChecklastVN >= lastVN)
        cerr << "{\"typeId\":1,\"data\":Wornning:Some parameters currently entered may be too small for effective operation, please check the corresponding parameter settings.}" << endl;
    string out = json_out.toStyledString();
    //去除回车和Tab
    std::string::size_type startpos = 0;
    while (startpos != std::string::npos)
    {
        startpos = out.find('\n');
        if (startpos != std::string::npos) //std::string::npos表示没有找到该字符
            out.replace(startpos, 1, ""); //实施替换，注意后面一定要用""引起来，表示字符串
        startpos = out.find('\t');
        if (startpos != std::string::npos) //std::string::npos表示没有找到该字符
            out.replace(startpos, 1, ""); //实施替换，注意后面一定要用""引起来，表示字符串
    }
    cout << out << endl;
    //cout << "************************************************************" << endl;
}
