#include <memory>
#include <cmath>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "my_first_package_msgs/action/dist_turtle.hpp"

class DistTurtleServer : public rclcpp::Node {
public:
    using DistTurtle = my_first_package_msgs::action::DistTurtle;
    using GoalHandleDistTurtle = rclcpp_action::ServerGoalHandle<DistTurtle>;

    DistTurtleServer() : Node("dist_turtle_action_server") {

        total_dist_ = 0.0;
        is_first_time_ = true;

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10, std::bind(&DistTurtleServer::pose_callback, this, std::placeholders::_1));

        action_server_ = rclcpp_action::create_server<DistTurtle>(
            this,
            "dist_turtle",
            std::bind(&DistTurtleServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&DistTurtleServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&DistTurtleServer::handle_accepted, this, std::placeholders::_1));

        this->declare_parameter("quantile_time", 0.75);
        this->declare_parameter("almost_goal_time", 0.95);

        quantile_time_ = this->get_parameter("quantile_time").as_double();
        almosts_time_ = this->get_parameter("almost_goal_time").as_double();

        RCLCPP_INFO(this->get_logger(), "Dist turtle action server is started.");
    }

private:
    double total_dist_;
    bool is_first_time_;
    turtlesim::msg::Pose current_pose_;
    turtlesim::msg::Pose previous_pose_;
    double quantile_time_;
    double almosts_time_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
    rclcpp_action::Server<DistTurtle>::SharedPtr action_server_;

    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg) {
        current_pose_ = *msg;
    }

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const DistTurtle::Goal> goal) 
    {
        (void)uuid; (void)goal;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleDistTurtle> goal_handle) {
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleDistTurtle> goal_handle) {

        std::thread{std::bind(&DistTurtleServer::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    double calc_diff_pose() {
        if (is_first_time_) {
            previous_pose_ = current_pose_;
            is_first_time_ = false;
        }
        double diff = std::sqrt(std::pow(current_pose_.x - previous_pose_.x, 2) + 
                                std::pow(current_pose_.y - previous_pose_.y, 2));
        previous_pose_ = current_pose_;
        return diff;
    }

    void execute(const std::shared_ptr<GoalHandleDistTurtle> goal_handle) {
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<DistTurtle::Feedback>();
        auto result = std::make_shared<DistTurtle::Result>();
        geometry_msgs::msg::Twist msg;

        msg.linear.x = goal->linear_x;
        msg.angular.z = goal->angular_z;

        rclcpp::WallRate loop_rate(100); // 100Hz (0.01s)

        while (rclcpp::ok()) {
            total_dist_ += calc_diff_pose();
            feedback->remained_dist = goal->dist - total_dist_;
            goal_handle->publish_feedback(feedback);
            publisher_->publish(msg);

            double tmp = std::abs(feedback->remained_dist - (goal->dist * quantile_time_));

            if (tmp < 0.02) {
                RCLCPP_INFO(this->get_logger(), "The turtle passes the %f point. : %f", quantile_time_, tmp);
            }

            if (feedback->remained_dist < 0.2) {
                break;
            }
            loop_rate.sleep();
        }

        if (rclcpp::ok()) {
            result->pos_x = current_pose_.x;
            result->pos_y = current_pose_.y;
            result->pos_theta = current_pose_.theta;
            result->result_dist = total_dist_;
            goal_handle->succeed(result);
            
            total_dist_ = 0.0;
            is_first_time_ = true;
        }
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DistTurtleServer>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}