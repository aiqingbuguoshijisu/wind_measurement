#pragma once
/*
* 1.函数的命名第一个字母大写，中间不加符号。
* 2.变量名中，如果出现-90°这种数据，使用Neg90命名。
* 3.计算出角度后需转换成角度制。
* 4.函数的输入参数尽量不要是文件路径。
*/
//C++17及以上版本运行 
#include <iostream>
#include <sstream>
#include <filesystem>
#include <fstream>
#include<vector>
#include<map>
#include<future>
#include<thread>
#include<queue>
//下面两行是使用Eigen库用的，根据自己计算机配置即可
#include "../eigen-3.4.0/Eigen/Dense"
#include "../eigen-3.4.0/Eigen/Core"
#define M_PI 3.14159265358979323846
#define Map map<int, vector<double>>
using namespace std;
using namespace Eigen;
namespace fs = std::filesystem;
enum COL {Y_COL,MZ_COL,MX_COL,X_COL,Z_COL,MY_COL};
static int it_nums = 7;//迭代次数
extern Map coef;//同一次运行中，天平的系数和修正值不会发生变化，因此使用全局变量。
extern VectorXd offset;

class Pose_Angle//风轴系外的姿态角
{
    public:
        double THETA;
        double PSI;
        double PHI;
        Pose_Angle(double theta, double psi, double phi) : THETA(theta), PSI(psi), PHI(phi) {}
        virtual void TransformMatrix()  = 0;
        virtual ~Pose_Angle() = default;  // 默认虚析构函数
};

class R_z_theta : public Pose_Angle//俯仰角转换矩阵
{
    public:
        R_z_theta(double theta, double psi, double phi) : Pose_Angle(theta, psi, phi) {}
        Matrix3d R_z_theta_N;
        Matrix3d R_z_theta_M;
        void TransformMatrix() override
        {
                R_z_theta_N << cos(THETA), -sin(THETA), 0,
                                sin(THETA),  cos(THETA), 0,
                                0,0,1;
                R_z_theta_M << cos(this->THETA), sin(this->THETA), 0,
                            -sin(this->THETA), cos(this->THETA), 0,
                            0,0,1;
        }
};

class R_y_psi : public Pose_Angle//偏航角转换矩阵
{
    public:
        R_y_psi(double theta, double psi, double phi) : Pose_Angle(theta, psi, phi) {}
        Matrix3d R_y_psi_N;
        Matrix3d R_y_psi_M;
        void TransformMatrix() override
        {
            R_y_psi_N << cos(this->PSI), 0, sin(this->PSI),
                        0, 1, 0,
                        -sin(this->PSI), 0, cos(this->PSI);
            R_y_psi_M << cos(this->PSI), 0, -sin(this->PSI),
                        0, 1, 0,
                        sin(this->PSI), 0, cos(this->PSI);
        }
};

class R_x_phi : public Pose_Angle//滚转角转换矩阵
{
    public:
        R_x_phi(double theta, double psi, double phi) : Pose_Angle(theta, psi, phi) {}
        Matrix3d R_x_phi_N;
        Matrix3d R_x_phi_M;
        void TransformMatrix() override
        {
            R_x_phi_N << 1, 0, 0,
                        0, cos(this->PHI), sin(this->PHI),
                        0, -sin(this->PHI), cos(this->PHI);
            R_x_phi_M << 1, 0, 0,
                        0, cos(this->PHI), sin(this->PHI),
                        0, -sin(this->PHI), cos(this->PHI);
        }
};

R_z_theta R_theta_sa{0,0,0};//支杆和支撑机构姿态角
R_y_psi R_psi_sa{0,0,0};
R_x_phi R_phi_sa{0,0,0};

R_z_theta R_theta_ss{0,0,0};//支杆几何姿态角
R_y_psi R_psi_ss{0,0,0};
R_x_phi R_phi_ss{0,0,0};

void printResult(const map<int, std::vector<double>>& result) {
    // 打印第一个元素：map<int, vector<double>>
    std::cout << "First element (map<int, vector<double>>):\n";
    for (const auto& elem : result) {
        std::cout << "Key: " << elem.first << ", Values: ";
        for (double val : elem.second) {
            std::cout <<fixed<<setprecision(2)<< val << " ";
        }
        std::cout << "\n";
    }
}
// 读取常规的数据文件，如安装角，原始数据等。并且删除一些没用的行。
MatrixXd ReadNormalData(const string &filename ,int DeleteLinesCounts)
{
    std::filesystem::path fsPath;
    try {
        // Use u8path to interpret the input string as UTF-8
        fsPath = std::filesystem::u8path(filename);
    } catch (const std::exception& e) {
        cerr << "Error: Invalid file path format: " << filename << " - " << e.what() << endl;
        return {};
    }
    ifstream inFile(fsPath);
    if (!inFile.is_open()) {
        cerr << "Error: Unable to open file " << fsPath << endl;
        return {};
    }

    vector<string> lines;
    string line;
    while (getline(inFile, line)) { 
        if(line.find_first_not_of(" \t") == string::npos)
        {
            continue;
        }
        lines.push_back(line);
    }
    inFile.close();

    if (lines.empty()) {
    cerr << "Error: File is empty" << endl;
    return {};
    }
    //删除前四行包括表头
    lines.erase(lines.begin(), lines.begin() + DeleteLinesCounts);
    
    // 获取列数
    int cols = 0;
    stringstream ss(lines[0]);
    string time1, time2;
    ss >> time1 >> time2;// 跳过两个时间戳
    double val;
    while (ss >> val) {
        ++cols;
    }
    if (cols == 0) {
        cerr << "Error: First data line contains no numeric values." << endl;
        return MatrixXd();
    }

    // 读取数据
    MatrixXd data(lines.size(), cols);//不带表头的数据大小
    int row = 0;
    for(const auto& line : lines)
    {
        stringstream ss(line);
        string time;

        for(int i = 0; i < 2; ++i)
        {
            if (!(ss >> time)) break; // 跳过两个时间戳
        }

        int col = 0;
        double val;
        while(ss>>val && col < cols)
        {
            data(row, col++) = val;
        }
        ++row;
    }
    
    return data;
}

//弹性角系数计算要读取整个文件夹的数据，并对每个文件中的载荷数据求均值用作拟合系数。
vector<MatrixXd> ReadFolderAllData(const string &folderPath)
{
    int deleteLinesCounts = 4;
    vector<MatrixXd> dataList;
    vector<std::pair<fs::path, fs::file_time_type>> files;// 用于存储文件路径及其最后修改时间

    for(const auto& entry : fs::directory_iterator(folderPath)){
        if (entry.is_regular_file()) {
            files.emplace_back(entry.path(), entry.last_write_time());
        }
    }

    // 按照修改时间升序排序
    sort(files.begin(), files.end(),[](const auto& a, const auto& b) {
        return a.second < b.second;  // 升序排列
    });

    for(const auto& [path,_] :files){
        dataList.push_back(ReadNormalData(path.string(), deleteLinesCounts));
    }
    
    return dataList;
}

map<int ,vector<double>> ReadFactorFile(const string& coffFilePath)//读系数文件，并转换成6*27，方便后续使用
{
    std::filesystem::path fsPath;
    try {
         // Use u8path to interpret the input string as UTF-8
        fsPath = std::filesystem::u8path(coffFilePath);
    } catch (const std::exception& e) {
        cerr << "Error: Invalid file path format: " << coffFilePath << " - " << e.what() << endl;
            return map<int, vector<double>>();
    }
    ifstream inFile(fsPath);
    if (!inFile.is_open()) {
        cerr << "Error: Unable to open file " << fsPath << endl;
        return map<int, vector<double>>();
    }

    vector<string> lines;
    string line;
    int rowCount = 0;
    while (getline(inFile, line) && rowCount < 28) {//拿28行数据，但是第一行是表头，后面会删除。
        if(line.find_first_not_of(" \t") == string::npos)
        {
            continue;
        }
        lines.push_back(line);
        ++rowCount;
    }
    inFile.close();

    if (lines.empty()) {
        cerr << "Error: File is empty" << endl;
        return map<int, vector<double>>();
    }

    //如果第一行是标题行，可以删除（根据实际情况决定）
    lines.erase(lines.begin());

    map<int, vector<double>> data;
    int dataIndex = 0;

    for (int i = 0; i < lines.size(); ++i) {
        stringstream ss(lines[i]);
        vector<double> row;
        double value;
        while (ss >> value) {
            row.push_back(value);
        }
        data[dataIndex] = row;
        dataIndex++;
    }

    map<int, vector<double>> data_T;
    for(int j =0;j<6;j++)
    {
        vector<double> row;
        for(int i=0;i<data.size();i++)
        {
            row.push_back(data[i][j]);
        }
        data_T[j] = row;
    }

    return data_T;
}

map<int ,vector<double>> MatToMap(MatrixXd &m)
{
    map<int,vector<double>> result;
    for(int i = 0;  i < m.rows(); ++i)
    {
        vector<double> tmp(m.cols());
        for(int j =0;j<m.cols();++j)
        {
            tmp[j] = m(i,j);
        }
        result.emplace(i, std::move(tmp));
    }
    return result;
}

MatrixXd MapToMat(map<int ,vector<double>> &data)//将map<int ,vector<double>>数据转化为MatrixXd数据
{
    int rowCount = data.size();
    int colCount = data.begin()->second.size();
    MatrixXd result(rowCount, colCount);
    int index = 0;
    for (const auto& pair : data) 
    {
        for (int j = 0; j < colCount; j++) 
        {
            result(index, j) = pair.second[j];
        }
        index++;
    }
    return result;
}

double Rad2Deg(double rad)
{
    return rad*180/M_PI;
}
double Deg2Rad(double deg)
{
    return deg*M_PI/180;
}

map<int, vector<double>> CombinedVariables (vector<double> &Y)//计算干扰项的自变量，按顺序放
{
    /*
    Y：载荷值。
    */
    map<int, vector<double>> m1;//一阶项
    for(int i=0;i<6;i++)
    {
        m1[i] = vector<double>{Y[0], Y[1], Y[2], Y[3], Y[4], Y[5]};
        m1[i].erase(m1[i].begin()+i);//剔除对角线上的自变量
    }

    vector<double> m2;//二阶项
    for(int i=0;i<6;i++)
    {
        m2.push_back(Y[i]*Y[i]);
    }
    for(int j=0;j<5;j++)
    {
        for(int k=j+1;k<6;k++)
        {
            m2.push_back(Y[j]*Y[k]);
        }
    }
    for(auto& it:m1)
    {
        it.second.insert(it.second.end(),m2.begin(),m2.end());
    }

    return m1;//返回自变量矩阵
}

map<int, vector<double>> IterativeCalculation(vector<double> &U)
{
    /*
    coff：6*27个系数
    U：各载荷下产生的电压，修正组桥后的
    */
    //计算主项力Y_0
    vector <double> Y_0(6,0);
    for(int i=0;i<6;i++)
    {
        //Y_0[i] = coff[i][i]*U[i]/1000;
        Y_0[i] = coef[i][i]*U[i]/1000;
    }
    //map<int, vector<double>> tmp_coff = coff;
    map<int, vector<double>> tmp_coff = coef;
    int index = 0;//剔除对角线上的系数，只保留干扰项的系数
    for (auto& it:tmp_coff)
    {
        it.second.erase(it.second.begin()+index);
        index++;
    }
    //迭代过程
    map<int, vector<double>> Y_i;//存放迭代结果

    vector <double> sum(6,0);//每个载荷的干扰项值
    map<int, vector<double>> x = CombinedVariables(Y_0);//利用Y_0计算出自变量矩阵
    vector<double> tmp;
    for (int i=0;i<6;i++)
    {
        for (int j=0;j<tmp_coff[i].size();j++)
        {
            sum[i] += x[i][j] * tmp_coff[i][j] ;//将自变量与对应的系数相乘得到各载荷的干扰项值
        }
        tmp.push_back(Y_0[i] + sum[i]);
    }
    Y_i[0] = tmp;//第一次迭代结果

    for(int i=0;i<(it_nums-1);i++)//上面已经迭代一次了，这里再迭代6次
    {
        vector<double> sum1(6,0);
        map<int, vector<double>> x = CombinedVariables(Y_i[i]);//使用上一次的载荷计算下一次的自变量矩阵
        vector<double> tmp1;
        for (int j=0;j<6;j++)
        {
            for (int k=0;k<tmp_coff[j].size();k++)
            {
                sum1[j] += x[j][k] * tmp_coff[j][k] ;
            }
            tmp1.push_back(Y_0[j] + sum1[j]);
        }
        Y_i[i+1] = tmp1;
    }

    return Y_i;//返回迭代结果
}

MatrixXd BridgeCorrect(MatrixXd &rawVol)//先修正，后组桥
{
    /*
    rawVol: 原始7元应变。
    offset: 修正值。
    */
    //修正电压值怎么给，每次都给在Balance Parameters (17-N6-80A)20250228.dat里面吗？
    MatrixXd correctedVol = rawVol.rowwise() - offset.transpose();
    // 13.11727924	,110.0940311	,93.70432627	,-79.92551249	,448.9582912	,-197.00641     ,-27.41440414
    MatrixXd result(rawVol.rows(), 6);
    result.col(0) = correctedVol.col(1) - correctedVol.col(0);//uy
    result.col(1) = correctedVol.col(1) + correctedVol.col(0);//umz
    result.col(2) = correctedVol.col(2);//umx
    result.col(3) = correctedVol.col(3) + correctedVol.col(6);//ux
    result.col(4) = correctedVol.col(4) - correctedVol.col(5);//uz
    result.col(5) = correctedVol.col(4) + correctedVol.col(5);//umy

    return result;
}

// map<int ,vector<double>> _Result(string dataFilePath,string coffFilePath)//测试用
// {
//     map<int, vector<double>> result_seven;

//     map<int, vector<double>> coef = ReadFactorFile(coffFilePath);
//     MatrixXd data = ReadNormalData(dataFilePath,4);

//     MatrixXd datablock = data.block(0,24,data.rows(),7);
//     MatrixXd datablockFix = BridgeCorrect(datablock);

//     Map data_map = MatToMap(datablockFix);
//     for(auto& it:data_map)
//     {
//         map<int, vector<double>> Y_i = IterativeCalculation(it.second);
//         result_seven[it.first] = Y_i.rbegin()->second;
//     }
//     return result_seven;
// }

MatrixXd ItResult(MatrixXd &data)//迭代结果
{
    /*
    data : 原始数据，全部都出来，函数里面有选取7元应变的过程。
    offset ：修正值。
    coef ：6*27个系数。
    */
    Map result_seven;
    MatrixXd datablock = data.block(0,24,data.rows(),7);
    MatrixXd datablockFix = BridgeCorrect(datablock);
    Map data_map = MatToMap(datablockFix);
    for(auto& it:data_map)
    {
        map<int, vector<double>> Y_i = IterativeCalculation(it.second);
        result_seven[it.first] = Y_i.rbegin()->second;
    }
    return MapToMat(result_seven);
}

class ThreadPool {
public:
    explicit ThreadPool(size_t numThreads = std::thread::hardware_concurrency()) 
        : stop(false) {
        // 创建指定数量的工作线程
        for (size_t i = 0; i < numThreads; ++i) {
            workers.emplace_back([this] {
                while (true) {
                    std::function<void()> task;
                    {
                        std::unique_lock<std::mutex> lock(queueMutex);
                        // 等待任务或停止信号
                        condition.wait(lock, [this] { return stop || !tasks.empty(); });
                        
                        if (stop && tasks.empty()) {
                            return;
                        }
                        
                        // 取出任务
                        task = std::move(tasks.front());
                        tasks.pop();
                    }
                    // 执行任务
                    task();
                }
            });
        }
    }

    // 提交任务的通用模板函数
    template<class F, class... Args>
    auto enqueue(F&& f, Args&&... args) 
        -> std::future<typename std::result_of<F(Args...)>::type> {
        
        using return_type = typename std::result_of<F(Args...)>::type;
        
        // 创建任务包装器
        auto task = std::make_shared<std::packaged_task<return_type()>>(
            std::bind(std::forward<F>(f), std::forward<Args>(args)...)
        );
        
        std::future<return_type> result = task->get_future();
        
        {
            std::unique_lock<std::mutex> lock(queueMutex);
            
            if (stop) {
                throw std::runtime_error("ThreadPool已停止，无法添加新任务");
            }
            
            // 将任务添加到队列
            tasks.emplace([task]() { (*task)(); });
        }
        
        condition.notify_one();
        return result;
    }

    // 析构函数
    ~ThreadPool() {
        {
            std::unique_lock<std::mutex> lock(queueMutex);
            stop = true;
        }
        
        condition.notify_all();
        
        for (std::thread &worker : workers) {
            worker.join();
        }
    }

    // 获取当前任务队列大小
    size_t getQueueSize() const {
        std::unique_lock<std::mutex> lock(queueMutex);
        return tasks.size();
    }

    // 获取线程数量
    size_t getThreadCount() const {
        return workers.size();
    }

private:
    std::vector<std::thread> workers;           // 工作线程
    std::queue<std::function<void()>> tasks;    // 任务队列
    
    mutable std::mutex queueMutex;              // 队列互斥锁
    std::condition_variable condition;          // 条件变量
    bool stop;                                  // 停止标志
};



VectorXd ElasticAngleCoef (vector<MatrixXd> &dataList,vector<MatrixXd> &loadHead710List)//弹性角系数计算
{
    /*
    dataList:计算弹性角系数的数据，好几个文件得出。
    loadHead710List：加载头和710角度，由象限仪给出。
    */
    //计算弹性角系数时，各个姿态角下的文件数量怎么考虑？
    //给的弹性角加载文件中，俯仰角文件36个，偏航26个，滚转14个，总计76个。
    ThreadPool pool;//线程池
    vector<future<MatrixXd>> futures;//异步任务结果
    vector<MatrixXd> itResultList;//迭代结果列表
    VectorXd result;//弹性角系数，5个，顺序为KY,KMZ,KZ,KMY,KMX

    for(auto &data:dataList)
    {
        futures.push_back(pool.enqueue(ItResult, std::ref(data)));
    }
    for(auto &it:futures)
    {
        itResultList.push_back(it.get());
    }

    return result;
}

tuple<class R_z_theta, class R_y_psi, class R_x_phi> GetInstallAngleTransMatrix(MatrixXd &InstallAngleData_0,
    MatrixXd &InstallAngleData_180,MatrixXd &InstallAngleData_90,MatrixXd &InstallAngleData_Neg90
)
{
    /*
    InstallAngleData:各角度下的安装角数据。
    offset: 修正值。
    coef: 6*27个系数。
    */

    // MatrixXd itResult_0 = ItResult(InstallAngleData_0,offset,coef);
    // MatrixXd itResult_180 = ItResult(InstallAngleData_180,offset,coef);
    // MatrixXd itResult_90 = ItResult(InstallAngleData_90,offset,coef);
    // MatrixXd itResult_Neg90 = ItResult(InstallAngleData_Neg90,offset,coef);

    // 启动异步任务
    auto future_0 = std::async(std::launch::async, ItResult, std::ref(InstallAngleData_0));
    auto future_180 = std::async(std::launch::async, ItResult, std::ref(InstallAngleData_180));
    auto future_90 = std::async(std::launch::async, ItResult, std::ref(InstallAngleData_90));
    auto future_neg90 = std::async(std::launch::async, ItResult, std::ref(InstallAngleData_Neg90));

    // 获取结果（会自动等待线程完成）
    MatrixXd itResult_0 = future_0.get();
    MatrixXd itResult_180 = future_180.get();
    MatrixXd itResult_90 = future_90.get();
    MatrixXd itResult_Neg90 = future_neg90.get();

    double Theta_mb , Psi_mb, Phi_mb;
    double tmp1 = 0;
    for(int i=0;i<itResult_0.rows();i++)
    {
        tmp1 += atan(itResult_0(i,X_COL)/itResult_0(i,Y_COL));
    }
    double tmp2 = 0;
    for(int i=0;i<itResult_180.rows();i++)
    {
        tmp2 += atan(itResult_180(i,X_COL)/itResult_180(i,Y_COL));
    }
    double tmp3 = 0;
    for(int i=0;i<itResult_90.rows();i++)
    {
        tmp3 += atan(itResult_90(i,X_COL)/itResult_90(i,Z_COL));
    }
    double tmp4 = 0;
    for(int i=0;i<itResult_Neg90.rows();i++)
    {
        tmp4 += atan(itResult_Neg90(i,X_COL)/itResult_Neg90(i,Z_COL));
    }
    double tmp5 = 0;
    for(int i=0;i<itResult_0.rows();i++)
    {
        tmp5 += atan(itResult_0(i,Z_COL)/itResult_0(i,Y_COL)); 
    }

    Theta_mb = Rad2Deg((tmp1/itResult_0.rows()+tmp2/itResult_180.rows())/2);//弧度制
    Psi_mb = Rad2Deg((tmp3/itResult_90.rows()+tmp4/itResult_Neg90.rows())/(-2));
    Phi_mb = Rad2Deg(tmp5/itResult_0.rows());

    R_z_theta R_theta_mb{Theta_mb,0,0};//支杆和支撑机构姿态角
    R_y_psi R_psi_mb{0,Psi_mb,0};
    R_x_phi R_phi_mb{0,0,Phi_mb};

    return make_tuple(R_theta_mb, R_psi_mb, R_phi_mb);
}