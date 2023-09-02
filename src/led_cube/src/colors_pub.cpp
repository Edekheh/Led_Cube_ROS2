#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "led_cube_interfaces/msg/color.hpp"
#include "led_cube_interfaces/msg/colors.hpp"

using namespace std::chrono_literals;

led_cube_interfaces::msg::Colors add_matrix(led_cube_interfaces::msg::Colors& msg, std_msgs::msg::ColorRGBA colors[8][8])
{
  for (int i = 0; i < 8; i++)
  {
    for (int j = 0; j < 64; j++)
    {
      msg.colors.at(i * 64 + j).r = colors[i][j].r;
      msg.colors.at(i * 64 + j).g = colors[i][j].g;
      msg.colors.at(i * 64 + j).b = colors[i][j].b;
      msg.colors.at(i * 64 + j).a = colors[i][j].a;
    }
  }
  return msg;
}



led_cube_interfaces::msg::Colors get_A()
{
  led_cube_interfaces::msg::Colors colors;
  colors.colors.at(0).b = 1;
  colors.colors.at(1).b = 1;
  colors.colors.at(6).b = 1;
  colors.colors.at(7).b = 1;
  colors.colors.at(64).b = 1;
  colors.colors.at(64 + 1).b = 1;
  colors.colors.at(64 + 6).b = 1;
  colors.colors.at(64 + 7).b = 1;
  for (int i = 0; i < 8; i++)
  {
    colors.colors.at(64 * 2 + i).b = 1;
  }
  // fill third row with 8 blue leds
  for (int i = 0; i < 8; i++)
  {
    colors.colors.at(64 * 3 + i).b = 1;
  }
  colors.colors.at(64 * 4).b = 1;
  colors.colors.at(64 * 4 + 1).b = 1;
  colors.colors.at(64 * 4 + 6).b = 1;
  colors.colors.at(64 * 4 + 7).b = 1;
  colors.colors.at(64 * 5 + 1).b = 1;
  colors.colors.at(64 * 5 + 2).b = 1;
  colors.colors.at(64 * 5 + 5).b = 1;
  colors.colors.at(64 * 5 + 6).b = 1;
  colors.colors.at(64 * 6 + 2).b = 1;
  colors.colors.at(64 * 6 + 3).b = 1;
  colors.colors.at(64 * 6 + 4).b = 1;
  colors.colors.at(64 * 6 + 5).b = 1;
  colors.colors.at(64 * 7 + 3).b = 1;
  colors.colors.at(64 * 7 + 4).b = 1;
  // fill .a=1
  for (int i = 0; i < 512; i++)
  {
    colors.colors.at(i).a = 1;
  }
  return colors;
}
led_cube_interfaces::msg::Colors push_left(led_cube_interfaces::msg::Colors colors)
{
  led_cube_interfaces::msg::Colors new_colors;
  for (int i = 0; i < 8; i++)
  {
    for (int j = 0; j < 64; j++)
    {
      new_colors.colors.at(i * 64 + j) = colors.colors.at(i * 64 + (j + 1) % 64);
    }
  }
  return new_colors;
}
led_cube_interfaces::msg::Colors push_left_x_times(led_cube_interfaces::msg::Colors colors, int x)
{
  for (int i = 0; i < x; i++)
  {
    colors = push_left(colors);
  }
  return colors;
}

class ColorsPub : public rclcpp::Node
{
public:
  ColorsPub()
      : Node("colors_pub")
  {
    colors = get_A();
    // _timer = this->create_wall_timer(
    // 0.01ms, std::bind(&ColorsPub::publish_color, this));
    _timer = this->create_wall_timer(
        300ms, std::bind(&ColorsPub::publish_colors, this));
        
    _color_pub = this->create_publisher<led_cube_interfaces::msg::Color>("color", 512);
    _colors_pub = this->create_publisher<led_cube_interfaces::msg::Colors>("colors", 10);
    rclcpp::sleep_for(3s);
    
  }

private:
  void publish_color()
  {
    // fill Color with random colors from 0 to 1
    led_cube_interfaces::msg::Color color;
    color.color.r = 0;
    color.color.g = 0;
    color.color.b = 1;
    color.color.a = 1.0;
    color.position.data = iterator;
    iterator++;
    if (iterator == 512)
    {
      iterator = 0;
    }
    _color_pub->publish(color);
    rclcpp::sleep_for(20ms);
  }
  void publish_colors()
  {
    colors = push_left_x_times(colors, 4);
    _colors_pub->publish(colors);
  }
  rclcpp::TimerBase::SharedPtr _timer;
  rclcpp::Publisher<led_cube_interfaces::msg::Color>::SharedPtr _color_pub;
  rclcpp::Publisher<led_cube_interfaces::msg::Colors>::SharedPtr _colors_pub;
  led_cube_interfaces::msg::Colors colors;
  unsigned int iterator = 0;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ColorsPub>());
  rclcpp::shutdown();
  return 0;
}