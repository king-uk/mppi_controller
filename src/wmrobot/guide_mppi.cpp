#include <wmrobot_map.h>
#include "GuideMPPI.h"
#include "collision_checker.h"
#include "bicycle.h"

#include <iostream>
#include <Eigen/Dense>

int main(){
    // auto model = WMRobotMap();
    auto model = Bicycle(0.17, 0.13, 0.0, 3.0, 0.35);

    using Solver = GuideMPPI;        
    using SolverParam = GuideMPPIParam;

    SolverParam param;     
    param.dt = 0.1;    
    param.T = 100;      // time horizon
    param.x_init.resize(model.dim_x);
    param.x_init << 0.5, 0.0, M_PI_2;
    param.x_target.resize(model.dim_x);
    param.x_target << 0.5, 5.0, M_PI_2;
    param.N = 6000;     // 샘플은 6천개
    param.gamma_u = 1;       // 온도파라미터
    Eigen::VectorXd sigma_u(model.dim_u);       // 노이즈 정도
    sigma_u << 0.25, 0.25;      // 0.25로 설정
    param.sigma_u = sigma_u.asDiagonal();       // 대각행렬로 만듦
    param.acceptance_dist = 0.01;
    
    param.guide_x_dim=2;
    
    double f_err;

    int maxiter = 200;      // 최대 시도는 200번

    bool is_collision = false;

    double total_elapsed = 0.0;


    CollisionChecker collision_checker = CollisionChecker(); 
    collision_checker.loadMap("../BARN_dataset/txt_files/output_78.txt", 0.1);     // 0.1은 해상도임
    Solver solver(model); 
    solver.U_0 = Eigen::MatrixXd::Zero(model.dim_u, param.T);
    solver.init(param);
    solver.setCollisionChecker(&collision_checker);

    // Eigen::Matrix<double, model.dim_x, param.T+1> traj_block;   // 이 역시 steer포함되면 바꿔져야함.

    std::string path = "../ref_traj/path_78_0.500000Bi_RRT.csv";   
    // std::string path = "../ref_traj/path_78_0.500000BiC_MPPI.csv";   


    solver.getTrajectory(path);     // global_traj가 채워짐. last_col도 채워지는데 이는 GuideMPPI의 protected임

    bool is_success = false;
    
    
    for(int i=0; i<maxiter; i++){

        solver.mppi_controller(solver.x_init.head(2));
        solver.move();      // move하면 x_init이 한칸 앞으로 가게 됨

        total_elapsed += solver.elapsed;        // 이동에 걸린 시간 축적

        if (collision_checker.getCollisionGrid(solver.x_init)) {
            is_collision = true;
            break;
        }
        else {
            f_err = (solver.x_init.head(2) - param.x_target.head(2)).norm();
            // std::cout<<"f_err = "<<f_err<<std::endl;
            if (f_err < 0.1) {
                is_success = true;

                std::cout << f_err << std::endl;
                break;
            }       // 만약 maxiter안에 한계 내에 도달했다면 성공! 
        }
        std::cout << i << std::endl;
        
        // solver.show();
    }

    solver.showTraj();
    std::cout<<'\t'<<is_success<<'\t'<<total_elapsed<<std::endl;

    return 0;
		
		}