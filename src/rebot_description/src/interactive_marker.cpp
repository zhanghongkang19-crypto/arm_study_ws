#include <interactive_markers/interactive_marker_server.hpp>

class TargetMarker : public rclcpp::Node
{
public:
    TargetMarker():Node("target_marker")
    {
        //这里创建一个：InteractiveMarkerServer它负责：创建球，接收 RViz 鼠标拖动，返回反馈
        server_ =std::make_shared<interactive_markers::InteractiveMarkerServer>("target_marker",this);
        target_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("/target_pose",1);
        create_marker();
    }

    void create_marker()
    {
        visualization_msgs::msg::InteractiveMarker marker;  //创建一个交互物体。
        marker.header.frame_id="base_link"; //表示这个球在哪个坐标系下。
        marker.name="target_pose";  //这个 marker ID。
        marker.description="Move Target";   //RViz显示文字：Move Target

        //初始位置
        marker.pose.position.x=0.3;
        marker.pose.position.y=0;
        marker.pose.position.z=0.4;

        // 初始姿态 quaternion
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale=0.2;   //球大小

        //平移
        visualization_msgs::msg::InteractiveMarkerControl move_control;
        move_control.name = "move_3d";
        move_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_3D;
        marker.controls.push_back(move_control);

        //旋转
        visualization_msgs::msg::InteractiveMarkerControl rotate_control;
        rotate_control.name = "rotate_3d";
        rotate_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_3D;
        marker.controls.push_back(rotate_control);

        //这个很重要。意思：把 marker 放入 server。同时绑定回调。以后 RViz：拖一下： -> server收到 -> 调用：
        server_->insert(marker,std::bind(&TargetMarker::feedback,this,std::placeholders::_1));

        server_->applyChanges();    //提交修改。
    }

    void feedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback)
    {
        auto pose = feedback->pose;
        target_pub_->publish(pose);
    }


private:
    std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr target_pub_;
};