#include <interactive_markers/interactive_marker_server.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <visualization_msgs/msg/marker.hpp> // 需包含基础 Marker 头文件
#include <Eigen/Core>
#include <Eigen/Geometry>

class TargetMarker : public rclcpp::Node
{
public:
    TargetMarker():Node("target_marker")
    {
        server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>("target_marker", this);
        target_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("/target_pose", 1);
        create_marker();
    }

    void create_marker()
    {
        visualization_msgs::msg::InteractiveMarker marker;
        marker.header.frame_id = "base_link"; 
        marker.name = "target_pose";
        marker.description = "Move Target";

        // 初始位置
        marker.pose.position.x = 0.3;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.4;

        // 初始姿态
        marker.pose.orientation.w = 1.0;
        
        marker.scale = 0.15; // 整体缩放比例

        
        visualization_msgs::msg::Marker sphere_marker;
        sphere_marker.type = visualization_msgs::msg::Marker::SPHERE;
        sphere_marker.scale.x = 0.02; // 球体 X 直径 (单位: 米)
        sphere_marker.scale.y = 0.02; // 球体 Y 直径
        sphere_marker.scale.z = 0.02; // 球体 Z 直径

        // 设置球体颜色 (RGBA: 0.0 ~ 1.0)
        sphere_marker.color.r = 0.0f;
        sphere_marker.color.g = 0.8f;
        sphere_marker.color.b = 1.0f;
        sphere_marker.color.a = 0.5f; // 半透明

       
        visualization_msgs::msg::InteractiveMarkerControl control;
        control.always_visible = true;
        // 设置为 MOVE_3D 模式（鼠标按住球体可在 3D 空间自由平移拖拽）
        control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_3D;
        
        // 关键步骤：把可视化的球体放入 control 中，这样按住球体才能拖拽！
        control.markers.push_back(sphere_marker);
        marker.controls.push_back(control);

        // 如果你需要像 MoveIt 那样的 XYZ 旋转环，可以在这里追加控制轴：
        visualization_msgs::msg::InteractiveMarkerControl rotate_control;
        rotate_control.name = "rotate_x";
        rotate_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
        rotate_control.orientation.w = 1.0;
        rotate_control.orientation.x = 1.0;
        marker.controls.push_back(rotate_control);

        // 把 marker 提交给 Server 并注册回调
        server_->insert(marker, std::bind(&TargetMarker::feedback, this, std::placeholders::_1));
        server_->applyChanges();
    }

    void feedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback)
    {
        // 只在释放鼠标时发送
        if (feedback->event_type != visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP)
        {
            return;
        }

        const auto& pose = feedback->pose;
        Eigen::Vector3d p(pose.position.x,pose.position.y,pose.position.z);

        RCLCPP_INFO(this->get_logger(),"Target final: [%.4f %.4f %.4f]",p.x(), p.y(), p.z());
        target_pub_->publish(pose);
    }

private:
    std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr target_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TargetMarker>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}