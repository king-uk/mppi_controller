#pragma once

#include "guide_mppi_param.h"


#include <Eigen/Dense>
#include <EigenRand/EigenRand>   
#include <random>                
#include <chrono>          
#include <ctime>            
#include <cstdint>           
#include <fstream>         
#include <iostream>         
#include <vector>
#include <string>

#include "collision_checker.h"   
#include "matplotlibcpp.h"      

class GuideMPPI {
public:
    template<typename ModelClass>   // Model자체는 자료형이 다를 수 있음
    GuideMPPI(ModelClass model);
    ~GuideMPPI(); 

    void init(GuideMPPIParam mppi_param); 
    void setCollisionChecker(CollisionChecker *collision_checker);
    virtual Eigen::MatrixXd getNoise(const int &T);
    void move();
    Eigen::VectorXd solve();
    void show();
    void showTraj();

    void getTrajectory(std::string path);
    void getNextRefTraj(const Eigen::VectorXd& cur_pos);

    Eigen::VectorXd mppi_controller(const Eigen::VectorXd& cur_pos);

    std::chrono::time_point<std::chrono::high_resolution_clock> start;
    std::chrono::time_point<std::chrono::high_resolution_clock> finish;
    std::chrono::duration<double> elapsed_1;        // 걸린 시간
    double elapsed;

    Eigen::MatrixXd U_0;        // 얘는 main.cpp에서 크기를 초기화해줌
    Eigen::VectorXd x_init;
    Eigen::VectorXd x_target;
    Eigen::MatrixXd Uo;         // u의 sequence 한 묶음
    Eigen::MatrixXd Xo;

    Eigen::MatrixXd X_ref;


    double acceptance_dist = 0.1;     // 기본값, init에서 덮어씀

    int lastwaypoint = 0;
    int nextwaypoint = 0;

    Eigen::MatrixXd global_traj;

    bool with_theta = false;

protected:
    int dim_x;      
    int dim_u;

    // Discrete Time System
    std::function<Eigen::MatrixXd(Eigen::VectorXd, Eigen::VectorXd)> f;
    // Stage Cost Function
    std::function<double(Eigen::VectorXd, Eigen::VectorXd)> q;
    // Terminal Cost Function
    std::function<double(Eigen::VectorXd, Eigen::VectorXd)> p;
    // Projection
    std::function<void(Eigen::Ref<Eigen::MatrixXd>)> h; // U입력들을 프로젝션

    std::mt19937_64 urng{static_cast<std::uint_fast64_t>(std::time(nullptr))};
    // std::mt19937_64 urng{1};
    Eigen::Rand::NormalGen<double> norm_gen{0.0, 1.0};

    // Parameters
    float dt;
    int T;
    int N;  // # of time horizon
    double gamma_u; // temparature param
    Eigen::MatrixXd sigma_u;    // 노이즈 공분산
    
    CollisionChecker *collision_checker;

    Eigen::VectorXd u0;

    std::vector<Eigen::VectorXd> visual_traj;

    double getGuideCost(const Eigen::MatrixXd& Xi);  // 후보경로를 넣어주면 이와 X_ref 간의 GuideCost를 반환함.

    Eigen::VectorXd last_col;

    int guide_x_dim;
};

template<typename ModelClass>
GuideMPPI::GuideMPPI(ModelClass model) {  // 걍 다 받아온 ModelClass꺼 그대로 쓰는거
    this->dim_x = model.dim_x;
    this->dim_u = model.dim_u;

    this->f = model.f;
    this->q = model.q;
    this->p = model.p;
    this->h = model.h;
}

GuideMPPI::~GuideMPPI() {
}

void GuideMPPI::init(GuideMPPIParam mppi_param) {
    this->dt = mppi_param.dt;
    this->T = mppi_param.T;
    this->x_init = mppi_param.x_init;
    this->x_target = mppi_param.x_target;
    this->N = mppi_param.N;
    this->gamma_u = mppi_param.gamma_u;
    this->sigma_u = mppi_param.sigma_u;
    this->acceptance_dist = mppi_param.acceptance_dist;

    this->guide_x_dim = mppi_param.guide_x_dim;

    u0 = Eigen::VectorXd::Zero(dim_u);
    Xo = Eigen::MatrixXd::Zero(dim_x, T+1);
}

void GuideMPPI::setCollisionChecker(CollisionChecker *collision_checker) {
    this->collision_checker = collision_checker;
}

Eigen::MatrixXd GuideMPPI::getNoise(const int &T) {
    return sigma_u * norm_gen.template generate<Eigen::MatrixXd>(dim_u, T, urng);
}   // 시간을 seed로 표준편차가 sigma_u안의 것을 쓴 랜덤변수 불러오기

void GuideMPPI::getTrajectory(std::string path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("Error opening file: " + path);
    }

    std::string line;
    std::vector<std::vector<double>> values;

    bool first_line = true;
    while (std::getline(file, line)) {
        if (first_line) {  // 헤더 스킵
            first_line = false;
            continue;
        }
        std::stringstream ss(line);
        std::string cell;
        std::vector<double> row;

        while (std::getline(ss, cell, ',')) {
            cell.erase(remove_if(cell.begin(), cell.end(), ::isspace), cell.end());
            if (!cell.empty()) {
                try { row.push_back(std::stod(cell)); }
                catch (...) { std::cerr << "Warning: invalid cell -> " << cell << std::endl; }
            }
        }
        if (!row.empty()) values.push_back(row);
    }
    file.close();

    if (values.empty()) {
        throw std::runtime_error("getTrajectory: no data rows found in file.");
    }

    // --- 열 수 고정(모든 행이 동일한 열 수라고 가정; 다르면 첫 행 기준으로 자름) ---
    const int orig_cols = static_cast<int>(values.front().size());
    for (auto& r : values) {
        if ((int)r.size() > orig_cols) r.resize(orig_cols);
    }

    // --- 행 다운샘플링: 50개 초과 시 균등 간격으로 50개 샘플 ---
    const int target = 50;
    
    std::vector<std::vector<double>> sampled;
    if ((int)values.size() > target) {
        sampled.resize(target);
        const int R = static_cast<int>(values.size());
        const double step = (R - 1) / static_cast<double>(target - 1);

        int last_idx = -1;
        for (int k = 0; k < target; ++k) {
            int idx = static_cast<int>(std::round(k * step));
            if (idx == last_idx && idx + 1 < R) idx++;   // 중복 인덱스 방지
            if (idx >= R) idx = R - 1;
            last_idx = idx;
            sampled[k] = values[idx];
        }
    } else {
        sampled = std::move(values);
    }

    // --- Eigen 행렬로 변환 ---
    const int rows = static_cast<int>(sampled.size());
    const int use_cols = std::min(orig_cols, guide_x_dim); // x,y 또는 x,y,theta
    if (use_cols <= 0) {
        throw std::runtime_error("getTrajectory: guide_dim_x must be positive and <= number of CSV columns.");
    }

    Eigen::MatrixXd mat(rows, use_cols);
    for (int i = 0; i < rows; ++i)
        for (int j = 0; j < use_cols; ++j)
            mat(i, j) = sampled[i][j];

    // x,y(,theta)만 사용
    Eigen::MatrixXd ref_traj = mat.leftCols(use_cols);

    // 열 = 시간축이 되도록 전치
    global_traj = ref_traj.transpose();
    last_col = global_traj.col(global_traj.cols() - 1);
}


void GuideMPPI::move() {
    x_init = x_init + (dt * f(x_init, u0));
    U_0.leftCols(T-1) = Uo.rightCols(T-1);
    U_0.col(T - 1).setZero();

}   // 진짜 x_init업데이트할때 씀


Eigen::VectorXd GuideMPPI::solve() {
    start = std::chrono::high_resolution_clock::now();

    Eigen::MatrixXd Ui = U_0.replicate(N, 1);   // 외부 레퍼런스 입력
    Eigen::VectorXd costs(N);
    Eigen::VectorXd weights(N); 
    #pragma omp parallel for    // 해당 for에 대해서 CPU로 병렬로 실행
    for (int i = 0; i < N; ++i) {
        Eigen::MatrixXd Xi(dim_x, T+1);
        Eigen::MatrixXd noise = getNoise(T);    
        Ui.middleRows(i * dim_u, dim_u) += noise; 
        h(Ui.middleRows(i * dim_u, dim_u));     // 생성된 입력을 프로젝션함. 제한조건에 맞게

        Xi.col(0) = x_init;
        double cost = 0.0;
        for (int j = 0; j < T; ++j) {
            // cost += p(Xi.col(j), x_target);
            // cost += q(Xi.col(j), Ui.block(i * dim_u, j, dim_u, 1));
            Xi.col(j+1) = Xi.col(j) + (dt * f(Xi.col(j), Ui.block(i * dim_u, j, dim_u, 1)));
        }

        cost += getGuideCost(Xi);        // X_ref는 timestep이 행이 커질수록 커지지만, Xi는 timestep이 열이 커질수록 커짐.

        // cost += p(Xi.col(T), x_target);
        for (int j = 1; j < T+1; ++j) {
            if (collision_checker->getCollisionGrid(Xi.col(j))) {
                cost = 1e8;
                break;
            }
        }
        costs(i) = cost;        // input나오면 그걸로 롤아웃해서 state 계산하고 그걸로부터 해당 input의 cost까지 계산함
    }

    double min_cost = costs.minCoeff(); 
    weights = (-gamma_u * (costs.array() - min_cost)).exp();
    double total_weight =  weights.sum();
    weights /= total_weight;        

    Uo = Eigen::MatrixXd::Zero(dim_u, T);   
    for (int i = 0; i < N; ++i) {
        Uo += Ui.middleRows(i * dim_u, dim_u) * weights(i);
    }       // weight 계산 끝난 애들로 최종 U를 구함
    h(Uo);  // 이를 projection함. 사용가능한 범위로.


    finish = std::chrono::high_resolution_clock::now();
    elapsed_1 = finish - start;     // 걸린 시간

    elapsed = elapsed_1.count();    // count는 double로 변환해주는애

    u0 = Uo.col(0);     // 결국 계산된 U의 첫 행만 가져다 씀

    Xo.col(0) = x_init;
    for (int j = 0; j < T; ++j) {
        Xo.col(j+1) = Xo.col(j) + (dt * f(Xo.col(j), Uo.col(j)));
    }   // 현재까지 나온 최적 U sequence에 대한 rollout

    visual_traj.push_back(x_init);
    return u0;
}


void GuideMPPI::show() {     // 이번 solve로 내놓은 최적 궤적 하나에 대한 플롯
    namespace plt = matplotlibcpp;
    // plt::subplot(1,2,1);

    double resolution = 0.1;
    double hl = resolution / 2;
    for (int i = 0; i < collision_checker->map.size(); ++i) {
        for (int j = 0; j < collision_checker->map[0].size(); ++j) {
            if ((collision_checker->map[i])[j] == 10) {
                double mx = i*resolution;
                double my = j*resolution;
                std::vector<double> oX = {mx-hl, mx+hl, mx+hl, mx-hl, mx-hl};
                std::vector<double> oY = {my-hl,my-hl,my+hl,my+hl,my-hl};
                plt::plot(oX, oY, "k"); // oX, oY를 하나씩 매칭하면 플롯에 필요한 사각형의 꼭짓점과 초기점으로 5개의 점으로 구성됨
            }
        }
    }

    std::vector<std::vector<double>> X_MPPI(dim_x, std::vector<double>(Xo.cols()));
    for (int i = 0; i < dim_x; ++i) {
        for (int j = 0; j < Xo.cols(); ++j) {
            X_MPPI[i][j] = Xo(i, j);
        }
    }
    // std::string color = "C" + std::to_string(9 - index%10);
    plt::plot(X_MPPI[0], X_MPPI[1], {{"color", "black"}, {"linewidth", "10.0"}});


if (X_ref.size() > 0 && X_ref.rows() >= 2) {
    std::vector<double> RX(X_ref.cols()), RY(X_ref.cols());
    for (int j = 0; j < X_ref.cols(); ++j) {
        RX[j] = X_ref(0, j);  // x
        RY[j] = X_ref(1, j);  // y
    }
    plt::plot(RX, RY, {{"color","blue"},{"linestyle","--"},{"linewidth","1.5"}});
}

    plt::xlim(0, 3);
    plt::ylim(0, 5);
    plt::grid(true);
    plt::show();
}

void GuideMPPI::showTraj() {     // 실제로 지나온 footprint에 대한 궤적 플롯
    namespace plt = matplotlibcpp;

    // --- 0) (선택) 이전 그림 지우기 ---
    // plt::clf();

    // --- 1) 장애물 그리기: 검은 윤곽선 "k-" ---
    const double res = 0.1;
    const double hl  = res / 2.0;
    for (int i = 0; i < static_cast<int>(collision_checker->map.size()); ++i) {
        for (int j = 0; j < static_cast<int>(collision_checker->map[0].size()); ++j) {
            if (collision_checker->map[i][j] == 10) {
                const double mx = i * res;
                const double my = j * res;
                std::vector<double> oX = {mx - hl, mx + hl, mx + hl, mx - hl, mx - hl};
                std::vector<double> oY = {my - hl, my - hl, my + hl, my + hl, my - hl};
                plt::plot(oX, oY, "k-");
            }
        }
    }

    std::vector<std::vector<double>> X_MPPI(dim_x, std::vector<double>(visual_traj.size()));
    for (int i = 0; i < dim_x; ++i) {
        for (int j = 0; j < (int)visual_traj.size(); ++j) {
            X_MPPI[i][j] = visual_traj[j](i);
        }
    }

    // --- 최소 변경: guide_x_dim -> guide_dim_x, 그리고 사용 가능한 행만 사용(gd)
    const int gd = std::min(guide_x_dim, (int)global_traj.rows());
    std::vector<std::vector<double>> X_r(gd, std::vector<double>(global_traj.cols()));
    for (int i = 0; i < gd; ++i) {
        for (int j = 0; j < global_traj.cols(); ++j) {
            X_r[i][j] = global_traj(i, j);
        }
    }

    // MPPI 궤적 → 빨간색
    if (!X_MPPI[0].empty() && !X_MPPI[1].empty()) {
        plt::plot(X_MPPI[0], X_MPPI[1], {{"color", "red"}, {"linewidth", "1.0"}});
        plt::plot(X_MPPI[0], X_MPPI[1], "ro");
    }

    // global_traj → 검정색
    if (gd >= 2 && global_traj.cols() > 0) {
        plt::plot(X_r[0], X_r[1], {{"color", "black"}, {"linewidth", "0.5"}});
        plt::plot(X_r[0], X_r[1], "ko");
    }

    plt::xlim(0, 3);
    plt::ylim(0, 5);
    plt::grid(true);
    plt::show();
}

void GuideMPPI::getNextRefTraj(const Eigen::VectorXd& cur_pos)
{

    const int nx = global_traj.rows();
    const int N  = global_traj.cols();

    // --- 1) lastwaypoint부터 가장 가까운 지점(minIndex) 탐색 ---
    const int start = std::min(std::max(lastwaypoint, 0), std::max(0, N - 1));

    int   minIndex = start;
    double minDist = std::numeric_limits<double>::infinity();

    for (int i = start; i < N; ++i) {
        const double d = (cur_pos - global_traj.col(i)).norm();   // 비교만 필요하면 squaredNorm()도 가능
        if (d < minDist) {
            minDist  = d;
            minIndex = i;
        }
    }
    // --- 2) acceptance 로직 ---
    int idx = minIndex;
    if (minDist < acceptance_dist) {
        int j = minIndex;
        while (j < N) {
            const double d = (cur_pos - global_traj.col(j)).norm();
            if (d > acceptance_dist) break;
            ++j;
        }
        idx = std::min(j, N - 1);  // 경계 보정
    }
    nextwaypoint = idx;

    // --- 3) X_ref 구성: nextwaypoint 기준으로 (T+1)열 채우기 ---
    const int need   = T + 1;              // 필요 열 수
    const int remain = N - nextwaypoint;   // 남은 열 수
    X_ref.resize(nx, need);

    if (remain >= need) {
        // 그대로 T+1 열 슬라이스
        X_ref = global_traj.middleCols(nextwaypoint, need);
    } else {
        // 남은 만큼 복사 + 마지막 열 반복
        if (remain > 0) {
            X_ref.leftCols(remain) = global_traj.middleCols(nextwaypoint, remain);
        }
        const Eigen::VectorXd last_col = global_traj.col(N - 1);
        for (int c = std::max(0, remain); c < need; ++c) {
            X_ref.col(c) = last_col;
        }
    }
    // --- 4) 다음 탐색 시작점 갱신 ---
    lastwaypoint = nextwaypoint;
}


double GuideMPPI::getGuideCost(const Eigen::MatrixXd& Xi){
    const int r = X_ref.rows();
    Eigen::MatrixXd diff = Xi.topRows(r) - X_ref;   // r × (T+1)
    return diff.colwise().squaredNorm().sum();      // 각 열(시점)의 2-노름^2 합
}



Eigen::VectorXd GuideMPPI::mppi_controller(const Eigen::VectorXd& cur_pos){
    getNextRefTraj(cur_pos);
    return solve();
}