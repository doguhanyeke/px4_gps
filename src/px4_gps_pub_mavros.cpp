#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <eigen3/Eigen/Eigen>
#include <gz/math.hh>
#include <gz/msgs.hh>
#include <gz/transport.hh>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "std_msgs/msg/string.hpp"
#include "mavros_msgs/msg/hil_gps.hpp"
#include <gz/sensors/NavSatMultipathSensor.hh>
#include <gz/msgs/navsat_multipath.pb.h>
#include "px4_gps/tsqueue.hpp"
#include <std_msgs/msg/bool.hpp>
#include "offboard_detector/msg/detector_output.hpp"

using namespace std::chrono_literals;

class PX4GPSMavrosPublisher : public rclcpp::Node
{
  public:
    PX4GPSMavrosPublisher()
    : Node("mavros_gps_real_publisher")
    {
      this->declare_parameter("gz_world_name", "AbuDhabi");
      this->declare_parameter("gz_model_name", "x500_1");
      this->declare_parameter("gz_spoofer_model_name", "spoofer");
      this->declare_parameter("gps_delay", 0.0);

      std::string _world_name = this->get_parameter("gz_world_name").as_string();
      std::string _model_name = this->get_parameter("gz_model_name").as_string();
      std::string _spoofer_model_name = this->get_parameter("gz_spoofer_model_name").as_string();

      RCLCPP_INFO(this->get_logger(),"World Name: %s", _world_name.c_str());
      RCLCPP_INFO(this->get_logger(),"Drone Model Name: %s", _model_name.c_str());
      RCLCPP_INFO(this->get_logger(),"Spoofer Model Name: %s", _spoofer_model_name.c_str());

      std::string navsat_topic = "/world/" + _world_name + "/model/" + _model_name + "/link/base_link/sensor/navsat_sensor/navsat_multipath";
      std::string spoofer_topic = "/world/" + _world_name + "/model/" + _spoofer_model_name + "/link/base_link/sensor/navsat_sensor/navsat_multipath";
      std::string mavros_gps_topic = "mavros/gps_input/hil_gps";

      if (!_node.Subscribe(navsat_topic, &PX4GPSMavrosPublisher::navsatCallback, this)) {
            RCLCPP_ERROR(this->get_logger(),"failed to subscribe to %s", navsat_topic.c_str());
      }
      if (!_node.Subscribe(spoofer_topic, &PX4GPSMavrosPublisher::navsatSpooferCallback, this)) {
            RCLCPP_ERROR(this->get_logger(),"failed to subscribe to %s", spoofer_topic.c_str());
      }  
      hil_gps_publisher_ = this->create_publisher<mavros_msgs::msg::HilGPS>(mavros_gps_topic, 1);  
      attack_flag_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        "/attack_flag", 10, std::bind(&PX4GPSMavrosPublisher::attackFlagCallback, this, std::placeholders::_1));
      count =0;
      
      rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
      qos_profile.history=RMW_QOS_POLICY_HISTORY_KEEP_LAST;
      qos_profile.depth=1;
      qos_profile.reliability=RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
      qos_profile.durability=RMW_QOS_POLICY_DURABILITY_VOLATILE;
      auto qos_ = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth), qos_profile);
      detector_flag_subscriber_ = this->create_subscription<offboard_detector::msg::DetectorOutput>(
        "/detector/out/observer_output", qos_, std::bind(&PX4GPSMavrosPublisher::detectorFlagCallback, this, std::placeholders::_1));
      //timer_ = this->create_wall_timer(std::chrono::microseconds(100), std::bind(&PX4GPSRealPublisher::timerCallback, this));
    }
  private:
    // void timerCallback()
    // { 
    //   rclcpp::Time t = this->now();
    //   double time_now = t.seconds();
    //   auto hil_gps = mavros_msgs::msg::HilGPS();

    //   if (!time_queue.empty())
    //     if ( time_now - time_queue.front() > this->get_parameter("gps_delay").as_double())
    //     {
    //       time_queue.pop();
    //       hil_gps = hilgps_msg_queue.pop();
    //       hil_gps_publisher_->publish(hil_gps);
    //       //RCLCPP_ERROR(this->get_logger(), "time after: %f", time_now);
    //     }
    // }

    void navsatCallback(const gz::msgs::NavSatMultipath &navsat)
    {   
        // publish gps
        auto hil_gps = mavros_msgs::msg::HilGPS();
        // Fill in and send message
        hil_gps.header.stamp.sec = navsat.header().stamp().sec();                 // [useconds]
        hil_gps.header.stamp.nanosec = navsat.header().stamp().nsec();  
        hil_gps.geo.latitude = navsat.latitude_deg();//+ 0.0000001*count;                         // [degrees * 1e7]
        hil_gps.geo.longitude = navsat.longitude_deg();                         // [degrees * 1e7]
        hil_gps.geo.altitude = navsat.altitude();    // [meters * 1e3]
        hil_gps.vel = sqrtf(
                navsat.velocity_east() * navsat.velocity_east()
                + navsat.velocity_north() * navsat.velocity_north()
                + navsat.velocity_up() * navsat.velocity_up())*1e2;            // [cm/s]
        hil_gps.vn = navsat.velocity_north()*1e2;                                  // [cm/s]
        hil_gps.ve = navsat.velocity_east()*1e2;                                   // [cm/s]
        hil_gps.vd = -navsat.velocity_up()*1e2;                                 // [cm/s]
        hil_gps.cog = atan2(navsat.velocity_east(), navsat.velocity_north()) * 1e2;                                  // [degrees * 1e2]
        hil_gps.eph = 0.9f * 1e2;                                  // [cm]
        hil_gps.epv = 1.78f * 1e2;                                  // [cm]
        hil_gps.fix_type = 3;
        hil_gps.satellites_visible = 10;
        
        if (attack_flag == 1)
        {
          hil_gps.geo.latitude = spoofer_latitude;
          hil_gps.geo.longitude = spoofer_longitude;
        }
        hil_gps_publisher_->publish(hil_gps);
        count++;
        // rclcpp::Time t = this->now();
        // double time_now = t.seconds();
        // hilgps_msg_queue.push(hil_gps);
        // time_queue.push(time_now);
        //RCLCPP_ERROR(this->get_logger(), "time before: %f", time_now);
    }

    void navsatSpooferCallback(const gz::msgs::NavSatMultipath &msg_spoofer)
    { 
      spoofer_latitude = msg_spoofer.latitude_deg();
      spoofer_longitude =  msg_spoofer.longitude_deg();
      spoofer_velocity_east = msg_spoofer.velocity_east();
      spoofer_velocity_north = msg_spoofer.velocity_north();      
    }
    
    void attackFlagCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
      attack_flag = msg->data;
    }
    
    void detectorFlagCallback(const offboard_detector::msg::DetectorOutput::SharedPtr msg)
    {
      detect_flag = msg->attack_detect;
      attack_flag = 0;
    }
    
    gz::transport::Node _node;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<mavros_msgs::msg::HilGPS>::SharedPtr hil_gps_publisher_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr attack_flag_subscriber_;
    rclcpp::Subscription<offboard_detector::msg::DetectorOutput>::SharedPtr detector_flag_subscriber_;

    TSQueue<mavros_msgs::msg::HilGPS> hilgps_msg_queue; 
    TSQueue<double> time_queue; 
    int count;
    double spoofer_latitude;
    double spoofer_longitude;
    double spoofer_velocity_east;
    double spoofer_velocity_north;
    int attack_flag;
    int detect_flag;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PX4GPSMavrosPublisher>());
  rclcpp::shutdown();
  return 0;
}
