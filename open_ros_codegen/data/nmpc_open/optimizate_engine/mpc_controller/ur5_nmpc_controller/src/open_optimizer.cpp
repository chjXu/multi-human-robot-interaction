/**
 * This is an auto-generated file by Optimization Engine (OpEn)
 * OpEn is a free open-source software - see doc.optimization-engine.xyz
 * dually licensed under the MIT and Apache v2 licences.
 *
 * Generated at 2021-01-12 20:41:21.004088.
 */
#include "ros/ros.h"
#include "ur5_nmpc_controller/OptimizationResult.h"
#include "ur5_nmpc_controller/OptimizationParameters.h"
#include "mpc_controller_bindings.hpp"
#include "open_optimizer.hpp"

#include <open_ur5_msgs/ref.h>
#include <open_ur5_msgs/cur.h>
#include <open_ur5_msgs/control.h>
#include <fstream>

using namespace std;

namespace ur5_nmpc_controller {
/**
 * Class ur5_nmpc_controller::OptimizationEngineManager manages the
 * exchange of data between the input and output topics
 * of this node
 */
class OptimizationEngineManager {
/**
 * Private fields and methods
 */
private:
    /**
     * Optimization parameters announced on the corresponding
     * topic (ur5_nmpc_controller/parameters)
     */
    ur5_nmpc_controller::OptimizationParameters params;
    /**
     * Object containing the result (solution and solver
     * statistics), which will be announced on ur5_nmpc_controller/results
     */
    ur5_nmpc_controller::OptimizationResult results;
    /**
     * Vector of parameters (provided by the client on
     * ur5_nmpc_controller/parameters)
     */
    double p[MPC_CONTROLLER_NUM_PARAMETERS] = { 0 };
    /**
     * Solution vector
     */
    double u[MPC_CONTROLLER_NUM_DECISION_VARIABLES] = { 0 };
    /**
     * Vector of Lagrange multipliers (if any)
     */
    double *y = NULL;

    static const int NX = 2;
    static const int NU = 1;
    static const int JOINT_NUM = 6;

    double current_pos[JOINT_NUM][NX] = {};
    double current_ref[JOINT_NUM][NX] = {};


    /**
     * Workspace variable used by the solver - initialised once
     */
    mpc_controllerCache* cache;
    /**
     * Initial guess for the penalty parameter
     */
    double init_penalty = ROS_NODE_MPC_CONTROLLER_DEFAULT_INITIAL_PENALTY;

    /**
     * Publish obtained results to output topic
     */
    void publishToTopic(ros::Publisher& publisher)
    {
        publisher.publish(results);
    }

    /**
     * Updates the input data based on the data that are posted
     * on /mpc/open_parameters (copies value from topic data to
     * local variables). This method is responsible for parsing
     * the data announced on the input topic.
     */
    void updateInputData()
    {
        init_penalty = (params.initial_penalty > 1.0)
            ? params.initial_penalty
            : ROS_NODE_MPC_CONTROLLER_DEFAULT_INITIAL_PENALTY;

        if (params.parameter.size() > 0) {
            for (size_t i = 0; i < MPC_CONTROLLER_NUM_PARAMETERS; ++i)
                p[i] = params.parameter[i];
        }

        if (params.initial_guess.size() == MPC_CONTROLLER_NUM_DECISION_VARIABLES) {
            for (size_t i = 0; i < MPC_CONTROLLER_NUM_DECISION_VARIABLES; ++i)
                u[i] = params.initial_guess[i];
        }

		if (params.initial_y.size() == MPC_CONTROLLER_N1) {
            for (size_t i = 0; i < MPC_CONTROLLER_N1; ++i)
                y[i] = params.initial_y[i];
		}
    }

    /**
     * Call OpEn to solve the problem
     */
    mpc_controllerSolverStatus solve()
    {
        return mpc_controller_solve(cache, u, p, y, &init_penalty);
    }
/**
 * Public fields and methods
 */
public:
    /**
     * Constructor of OptimizationEngineManager
     */
    OptimizationEngineManager()
    {
	    y = new double[MPC_CONTROLLER_N1];
        cache = mpc_controller_new();
    }

    /**
     * Destructor of OptimizationEngineManager
     */
    ~OptimizationEngineManager()
    {
		if (y!=NULL) delete[] y;
        mpc_controller_free(cache);
    }

    /**
     * Copies results from `status` to the local field `results`
     */
    void updateResults(mpc_controllerSolverStatus& status)
    {
        std::vector<double> sol(u, u + MPC_CONTROLLER_NUM_DECISION_VARIABLES);
        results.solution = sol;
        std::vector<double> y(status.lagrange, status.lagrange + MPC_CONTROLLER_N1);
        results.lagrange_multipliers = y;
        results.inner_iterations = status.num_inner_iterations;
        results.outer_iterations = status.num_outer_iterations;
        results.norm_fpr = status.last_problem_norm_fpr;
        results.cost = status.cost;
        results.penalty = status.penalty;
        results.status = (int)status.exit_status;
        results.solve_time_ms = (double)status.solve_time_ns / 1000000.0;
        results.infeasibility_f2 = status.f2_norm;
        results.infeasibility_f1 = status.delta_y_norm_over_c;
    }

    /**
     * Callback that obtains data from topic `/ur5_nmpc_controller/open_params`
     */
    void mpcReceiveRequestCallback(
        const ur5_nmpc_controller::OptimizationParameters::ConstPtr& msg)
    {
        params = *msg;
    }

    void solveAndPublish(ros::Publisher& publisher)
    {
        updateInputData(); /* get input data */
        mpc_controllerSolverStatus status = solve(); /* solve!  */
        updateResults(status); /* pack results into `results` */
        publishToTopic(publisher);
    }

    void commandPoseRef(const open_ur5_msgs::ref::ConstPtr &msg)
    {
        for(int i=0; i < JOINT_NUM; ++i)
        {
            current_ref[i][0] = msg->q_ref[i];
            current_ref[i][1] = msg->v_ref[i];
        }        
    }
    
    void stateCallback(const open_ur5_msgs::cur::ConstPtr &msg)
    {
        for(int i=0; i < JOINT_NUM; ++i){
            current_pos[i][0] = msg->q_cur[i];
            current_pos[i][1] = msg->v_cur[i];
        }
    }

    void solveAndPublishCmd(ros::Publisher& publisher, ofstream& fs, char* write_buffer, bool record)
    {
        double current_par [MPC_CONTROLLER_NUM_PARAMETERS] = {0};
        double current_var [MPC_CONTROLLER_NUM_DECISION_VARIABLES] = {0};
        double control_cmd = 0;

        open_ur5_msgs::control control;
        double optimize_time = 0.0;
        for(int i=0; i < JOINT_NUM; ++i){
            for(int j=0; j < NX; ++j){
                // std::cout << "当前参靠：" << current_ref[i][j] << std::endl;
                // std::cout << "当前姿态：" << current_pos[i][j] << std::endl;
                
                current_par[j] = current_pos[i][j];
                current_par[j+NX] = current_ref[i][j];

                //solve
                mpc_controllerSolverStatus status = 
                    mpc_controller_solve(cache, current_var, current_par, 0, &init_penalty);

                // std::cout << current_var[0] << " "
                //           << current_var[1] << " "
                //           << current_var[2] << " "
                //           << current_var[3] << " "
                //           << current_var[4] << " "
                //           << current_var[5] << " "
                //           << current_var[6] << " "
                //           << current_var[7] << " "
                //           << current_var[8] << std::endl;
        
                control_cmd = current_var[0];
                control.u[i] = control_cmd;

                optimize_time += status.solve_time_ns;
                
            }   
        }

        if(record && fs){
            sprintf(write_buffer, "%.4f\n", 
                    (double)optimize_time / 1000000.0);
            fs.write(write_buffer, strlen(write_buffer));
        }

        publisher.publish(control);

        ROS_INFO("q1_ref: %f, q2_ref: %f, q3_ref: %f, q4_ref: %f, q5_ref: %f, q6_ref: %f", 
                    current_ref[0][0], current_ref[1][0], current_ref[2][0], 
                    current_ref[3][0], current_ref[4][0], current_ref[5][0]);
        ROS_INFO("q1_cur: %f, q2_cur: %f, q3_cur: %f, q4_cur: %f, q5_cur: %f, q6_cur: %f", 
                    current_pos[0][0], current_pos[1][0], current_pos[2][0], 
                    current_pos[3][0], current_pos[4][0], current_pos[5][0]);
        ROS_INFO("u1: %f, u2: %f, u3: %f, u4: %f, u5: %f, u6: %f", 
                    control.u[0], control.u[1], control.u[2], 
                    control.u[3], control.u[4], control.u[5]);
    
        //ROS_INFO("q_cur: %f, v_cur: %f", current_pos[0][0], current_pos[0][1]);
        ROS_INFO("Solve time: %f ms.", (double)optimize_time / 1000000.0);
    }


}; /* end of class OptimizationEngineManager */
} /* end of namespace ur5_nmpc_controller */

/**
 * Main method
 *
 * This advertises a new (private) topic to which the optimizer
 * announces its solution and solution status and details. The
 * publisher topic is 'ur5_nmpc_controller/result'.
 *
 * It obtains inputs from 'ur5_nmpc_controller/parameters'.
 *
 */
int main(int argc, char** argv)
{
    std::string result_topic, params_topic;  /* parameter and result topics */
    std::string out_status, in_status;
    double rate; /* rate of node (specified by parameter) */

    std::string record_dir = "/home/xuchengjun/open_ros_codegen/data/time.txt";
    //std::string record_dir_2 = "/home/xuchengjun/open_ros_codegen/data/ee_track.txt";
    bool record = false;
    char* write_buffer = NULL;
    std::ofstream fs;
    if(record){
        fs.open(record_dir.c_str());
        if(fs){
            fs << "time\n";
        }
        write_buffer = (char*)malloc(256);
    }

    ur5_nmpc_controller::OptimizationEngineManager mng;
    ros::init(argc, argv, ROS_NODE_MPC_CONTROLLER_NODE_NAME);
    ros::NodeHandle nh, private_nh("~");

    /* obtain parameters from config/open_params.yaml file */
    private_nh.param("result_topic", result_topic,
                     std::string(ROS_NODE_MPC_CONTROLLER_RESULT_TOPIC));
    private_nh.param("params_topic", params_topic,
                     std::string(ROS_NODE_MPC_CONTROLLER_PARAMS_TOPIC));
    private_nh.param("rate", rate,
                     double(ROS_NODE_MPC_CONTROLLER_RATE));

    private_nh.param("out_status_name", out_status, std::string("/ur5_joint_accle"));
    private_nh.param("in_status_name", in_status, std::string("/ur5_cur_joint_state"));

    ros::Publisher mpc_pub
        = private_nh.advertise<ur5_nmpc_controller::OptimizationResult>(
            ROS_NODE_MPC_CONTROLLER_RESULT_TOPIC,
            ROS_NODE_MPC_CONTROLLER_RESULT_TOPIC_QUEUE_SIZE);
    ros::Subscriber sub
        = private_nh.subscribe(
            ROS_NODE_MPC_CONTROLLER_PARAMS_TOPIC,
            ROS_NODE_MPC_CONTROLLER_PARAMS_TOPIC_QUEUE_SIZE,
            &ur5_nmpc_controller::OptimizationEngineManager::mpcReceiveRequestCallback,
            &mng);

    ros::Subscriber command_joint_subscriber
        = nh.subscribe(in_status,
                        1,
                        &ur5_nmpc_controller::OptimizationEngineManager::stateCallback,
                        &mng);

    ros::Subscriber joint_ref
        = private_nh.subscribe("/ur5_ref_joint_state",
                                1,
                                &ur5_nmpc_controller::OptimizationEngineManager::commandPoseRef,
                                &mng);
    
    ros::Publisher pub_joint_cmd = nh.advertise<open_ur5_msgs::control>(out_status, 1);
    ros::Rate loop_rate(rate);

    while (ros::ok()) {
        mng.solveAndPublishCmd(pub_joint_cmd, fs, write_buffer, record);
        //mng.solveAndPublish(mpc_pub);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}