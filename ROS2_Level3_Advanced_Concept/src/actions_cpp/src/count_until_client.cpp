#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_robot_interfaces/action/count_until.hpp"


using CountUntil = my_robot_interfaces::action::CountUntil;
using CountUntilGoalHandle = rclcpp_action::ClientGoalHandle<CountUntil>;
using namespace std::placeholders;

class CountUntilClientNode : public rclcpp::Node
{
public:
    CountUntilClientNode() : Node("count_until_client")
    {
        count_until_client_ = rclcpp_action::create_client<CountUntil>(this, "count_until");
    }

    void send_goal(int target_number, double period)
    {
        //wait for the server
        count_until_client_->wait_for_action_server();

        //create a goal
        auto goal = CountUntil::Goal();
        goal.target_number = target_number;
        goal.period = period;

        // add callbacks
        auto options = rclcpp_action::Client<CountUntil>::SendGoalOptions();
        options.goal_response_callback = std::bind(&CountUntilClientNode::goal_response_callback, this, _1);
        options.result_callback = std::bind(&CountUntilClientNode::goal_result_callback, this, _1);
        options.feedback_callback = std::bind(&CountUntilClientNode::goal_feedback_callback, this, _1, _2);

        //send the goal
        RCLCPP_INFO(this->get_logger(), "Sending goal: target_number=%d, period=%.2f", target_number, period);
        count_until_client_->async_send_goal(goal, options);
    }

private:

    //callbakck to know if goal is accepted or rejected
    void goal_response_callback(const CountUntilGoalHandle::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    //callback to receive the result
    void goal_result_callback(const CountUntilGoalHandle::WrappedResult & result)
    {
        auto status = result.code;
        if (status == rclcpp_action::ResultCode::SUCCEEDED){
            RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
        } else if (status == rclcpp_action::ResultCode::ABORTED){
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        }
        int reached_number = result.result->reached_number;
        RCLCPP_INFO(this->get_logger(), "Result received: reached_number=%d", reached_number);
    }

    //feedback callback
    void goal_feedback_callback(CountUntilGoalHandle::SharedPtr goal_handle, const std::shared_ptr<const CountUntil::Feedback> feedback)
    {
        (void)goal_handle;
        int number = feedback->current_number;
        RCLCPP_INFO(this->get_logger(), "Received feedback: current_number=%d", number);
    }

    rclcpp_action::Client<CountUntil>::SharedPtr count_until_client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CountUntilClientNode>();
    node->send_goal(10, 0.5);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}