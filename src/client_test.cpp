#include <voice_control/TaskServerAction.h>
//#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib/client/simple_action_client.h>

//这样定义下会用起来简洁许多
typedef actionlib::SimpleActionClient<voice_control::TaskServerAction> Client;

class TaskServerActionClient {
private:
    // Called once when the goal completes
    void DoneCb(const actionlib::SimpleClientGoalState& state,
            const voice_control::TaskServerResultConstPtr& result) {
        ROS_INFO("Finished in state [%s]", state.toString().c_str());
        ROS_INFO("Toal dish cleaned: %i", result->finished_status);
        ros::shutdown();
    }

    // 当目标激活的时候，会调用一次
    void ActiveCb() {
        ROS_INFO("Goal just went active");
    }

    // 接收服务器的反馈信息
    void FeedbackCb(
            const voice_control::TaskServerFeedbackConstPtr& feedback) {
        // ROS_INFO("Got Feedback Complete Rate: %f", feedback->step);
    }
public:
    TaskServerActionClient(const std::string client_name, bool flag = true) :
            client(client_name, flag) {
    }

    //客户端开始
    void Start() {
        //等待服务器初始化完成
        client.waitForServer();
        //定义要做的目标
        voice_control::TaskServerGoal goal;
        // goal.items = 1;
        // goal.task = {"BASEMOVE"};
        // goal.value = {0.0, 0.1};
        // goal.task = {"FINGERPOSITION"};
        // goal.value = {6500, 6500, 6500};
        // goal.task = {"ARMPOSE","out"};
        // goal.task = {"ARMFORCECONTROL","start"};
        goal.task = {"BASENAVGOAL","pose2"};
        //发送目标至服务器
        client.sendGoal(goal,
                boost::bind(&TaskServerActionClient::DoneCb, this, _1, _2),
                boost::bind(&TaskServerActionClient::ActiveCb, this),
                boost::bind(&TaskServerActionClient::FeedbackCb, this, _1));
        //等待结果，时间间隔5秒
        client.waitForResult(ros::Duration(50.0));

        //根据返回结果，做相应的处理
        if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            // printf("Yay! The dishes are now clean");
            ROS_INFO("SUCCEEDED");
        else {
            ROS_INFO("Cancel Goal!");
            client.cancelAllGoals();
        }

        ROS_INFO("Current State: %s\n", client.getState().toString().c_str());
    }
private:
    Client client;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pick_task_client");
    TaskServerActionClient actionclient("task_server", true);
    //启动客户端
    actionclient.Start();
    ros::spin();
    return 0;
}