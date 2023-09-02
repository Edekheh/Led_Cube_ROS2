#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "led_cube_interfaces/msg/colors.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "led_cube_interfaces/msg/color.hpp"

using namespace std::chrono_literals;

class LedCube : public rclcpp::Node
{
  public:
    LedCube()
    : Node("led_cube")
    {
      create_marker();
      _vis_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("led_cube", 10);
      publish_marker();

      _sub = this->create_subscription<led_cube_interfaces::msg::Color>("color", 512, std::bind(&LedCube::color_callback, this, std::placeholders::_1));
      _colors_sub = this->create_subscription<led_cube_interfaces::msg::Colors>("colors", 10, std::bind(&LedCube::colors_callback, this, std::placeholders::_1));
    }

  private:
    void publish_marker()
    {
      //fill MarkerArray with random colors from 0 to 1
      for(unsigned long i=0; i<_marker_array.markers.size(); i++) {
        _marker_array.markers[i].color.r = 0.1;
        _marker_array.markers[i].color.g = 0.1;
        _marker_array.markers[i].color.b = 0.1;
        _marker_array.markers[i].color.a = 0.1;
        //update the marker
        _marker_array.markers[i].header.stamp = rclcpp::Time();
        _marker_array.markers[i].header.frame_id = "map";
        _marker_array.markers[i].id = i;
        _marker_array.markers[i].ns = "cube" + std::to_string(i);
      }
      _vis_pub->publish(_marker_array);
      //print size of marker
      RCLCPP_INFO(this->get_logger(), "Publishing marker with %ld points", _marker_array.markers.size());
      //ros sleep for 20ms
      rclcpp::sleep_for(20ms);     
    }
    void color_callback(const led_cube_interfaces::msg::Color::SharedPtr msg) 
    {
        _marker_array.markers[msg->position.data].color= msg->color;
        _marker_array.markers[msg->position.data].header.stamp = rclcpp::Time();
        _vis_pub->publish(_marker_array);
    }
    void colors_callback(const led_cube_interfaces::msg::Colors::SharedPtr msg) 
    {
        for(unsigned long i=0; i<msg->colors.size(); i++) {
          _marker_array.markers.at(i).color= msg->colors.at(i);
          _marker_array.markers.at(i).header.stamp = rclcpp::Time();
        }
        _vis_pub->publish(_marker_array);
    }
    void create_marker()
    {
      _marker.header.frame_id = "map";
      _marker.header.stamp = rclcpp::Time();
      _marker.type = visualization_msgs::msg::Marker::CUBE;
      _marker.action = 0;
      _marker.pose.position.x = 0;
      _marker.pose.position.y = 0;
      _marker.pose.position.z = 0;
      _marker.pose.orientation.x = 0.0;
      _marker.pose.orientation.y = 0.0;
      _marker.pose.orientation.z = 0.0;
      _marker.pose.orientation.w = 1.0;
      _marker.scale.x = 0.5;
      _marker.scale.y = 0.5;
      _marker.scale.z = 0.5;
      _marker.color.a = 1.0;
      _marker.color.r = 1.0;
      double r = 9.656854;
      // generate octagon
      for (int i = 0; i < 8; i++)
      {
        geometry_msgs::msg::Point p;
        p.y = r;
        p.z = i;
        for(int j = 0; j < 8; j++)  {
          p.x = -3.5 + j;
          // _marker.points.push_back(p);
          // _marker.colors.push_back(_marker.color);
          _marker.pose.position = p;
          _marker_array.markers.push_back(_marker);
        }
      }
      //rotate by 45 degrees
      _marker.pose.orientation.x = 0.0;
      _marker.pose.orientation.y = 0.0;
      _marker.pose.orientation.z = 0.3826834;
      _marker.pose.orientation.w = 0.9238795;
      // next wall of octagon
      for(int i=0; i<8; i++) {
        geometry_msgs::msg::Point p;
        p.z = i;
        for(int j=0; j<8; j++) {
          p.x = 4.353553 + j*sqrt(2)/2;
          p.y = 9.303301 - j*sqrt(2)/2;
          // _marker.points.push_back(p);
          // _marker.colors.push_back(_marker.color);
          _marker.pose.position = p;
          _marker_array.markers.push_back(_marker);
        }
      }
      _marker.pose.orientation.x = 0.0;
      _marker.pose.orientation.y = 0.0;
      _marker.pose.orientation.z = 0.0;
      _marker.pose.orientation.w = 1.0;
     
      // next wall of octagon
      for(int i=0; i<8; i++) {
        geometry_msgs::msg::Point p;
        p.x = r;
        p.z = i;
        for(int j=0; j<8; j++) {
          p.y = 3.5 - j;
          // _marker.points.push_back(p);
          // _marker.colors.push_back(_marker.color);
          _marker.pose.position = p;
          _marker_array.markers.push_back(_marker);
        }
      }
      //rotate by 45 degrees
      _marker.pose.orientation.x = 0.0;
      _marker.pose.orientation.y = 0.0;
      _marker.pose.orientation.z = 0.3826834;
      _marker.pose.orientation.w = 0.9238795;
      // next wall of octagon
      for(int i=0; i<8; i++) {
        geometry_msgs::msg::Point p;
        p.z = i;
        for(int j=0; j<8; j++) {
          p.x = 9.303301 - j*sqrt(2)/2;
          p.y = -4.353553 - j*sqrt(2)/2;
          // _marker.points.push_back(p);
          // _marker.colors.push_back(_marker.color);
          _marker.pose.position = p;
          _marker_array.markers.push_back(_marker);
        }
      }
      _marker.pose.orientation.x = 0.0;
      _marker.pose.orientation.y = 0.0;
      _marker.pose.orientation.z = 0.0;
      _marker.pose.orientation.w = 1.0;
      // next wall of octagon
      for(int i=0; i<8; i++) {
        geometry_msgs::msg::Point p;
        p.y = -r;
        p.z = i;
        for(int j=0; j<8; j++) {
          p.x = 3.5 - j;
          // _marker.points.push_back(p);
          // _marker.colors.push_back(_marker.color);
          _marker.pose.position = p;
          _marker_array.markers.push_back(_marker);
        }
      }
      //rotate by 45 degrees
      _marker.pose.orientation.x = 0.0;
      _marker.pose.orientation.y = 0.0;
      _marker.pose.orientation.z = 0.3826834;
      _marker.pose.orientation.w = 0.9238795;
      // next wall of octagon
      for(int i=0; i<8; i++) {
        geometry_msgs::msg::Point p;
        p.z = i;
        for(int j=0; j<8; j++) {
          p.x = -4.353553 - j*sqrt(2)/2;
          p.y = -9.303301 + j*sqrt(2)/2;
          // _marker.points.push_back(p);
          // _marker.colors.push_back(_marker.color);
          _marker.pose.position = p;
          _marker_array.markers.push_back(_marker);
        }
      }
      _marker.pose.orientation.x = 0.0;
      _marker.pose.orientation.y = 0.0;
      _marker.pose.orientation.z = 0.0;
      _marker.pose.orientation.w = 1.0;
      // next wall of octagon
      for(int i=0; i<8; i++) {
        geometry_msgs::msg::Point p;
        p.x = -r;
        p.z = i;
        for(int j=0; j<8; j++) {
          p.y = -3.5 + j;
          // _marker.points.push_back(p);
          // _marker.colors.push_back(_marker.color);
          _marker.pose.position = p;
          _marker_array.markers.push_back(_marker);
        }
      }
      //rotate by 45 degrees
      _marker.pose.orientation.x = 0.0;
      _marker.pose.orientation.y = 0.0;
      _marker.pose.orientation.z = 0.3826834;
      _marker.pose.orientation.w = 0.9238795;
      // next wall of octagon
      for(int i=0; i<8; i++) {
        geometry_msgs::msg::Point p;
        p.z = i;
        for(int j=0; j<8; j++) {
          p.x = -9.303301 + j*sqrt(2)/2;
          p.y = 4.353553 + j*sqrt(2)/2;
          // _marker.points.push_back(p);
          // _marker.colors.push_back(_marker.color);
          _marker.pose.position = p;
          _marker_array.markers.push_back(_marker);
        }
      }
      //get first row of octagon 0,1,2,3,4,5,6,7 then 64,65,66,67,68,69,70,71 ....
      visualization_msgs::msg::MarkerArray temp_marker_array;
      for(int j=0; j<8; j++) {
        for(int i=0; i<8; i++) {
          temp_marker_array.markers.push_back(_marker_array.markers[i + 8*j]);
        }
        for(int i=0; i<8; i++) {
          temp_marker_array.markers.push_back(_marker_array.markers[i+64+ 8*j]);
        }
        for(int i=0; i<8; i++) {
          temp_marker_array.markers.push_back(_marker_array.markers[i+128+ 8*j]);
        }
        for(int i=0; i<8; i++) {
          temp_marker_array.markers.push_back(_marker_array.markers[i+192+ 8*j]);
        }
        for(int i=0; i<8; i++) {
          temp_marker_array.markers.push_back(_marker_array.markers[i+256+ 8*j]);
        }
        for(int i=0; i<8; i++) {
          temp_marker_array.markers.push_back(_marker_array.markers[i+320+ 8*j]);
        }
        for(int i=0; i<8; i++) {
          temp_marker_array.markers.push_back(_marker_array.markers[i+384+ 8*j]);
        }
        for(int i=0; i<8; i++) {
          temp_marker_array.markers.push_back(_marker_array.markers[i+448+ 8*j]);
        }
      }
        _marker_array = temp_marker_array;
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _vis_pub;
    rclcpp::Subscription<led_cube_interfaces::msg::Color>::SharedPtr _sub;
    visualization_msgs::msg::Marker _marker;
    visualization_msgs::msg::MarkerArray _marker_array;
    rclcpp::Subscription<led_cube_interfaces::msg::Colors>::SharedPtr _colors_sub;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LedCube>());
  rclcpp::shutdown();
  return 0;
}