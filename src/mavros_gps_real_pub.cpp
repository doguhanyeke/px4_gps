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
using namespace std::chrono_literals;

class PX4GPSRealPublisher : public rclcpp::Node
{
  public:
    PX4GPSRealPublisher()
    : Node("mavros_gps_real_publisher")
    {
      this->declare_parameter("gz_world_name", "AbuDhabi");
      this->declare_parameter("gz_model_name", "x500_1");
      this->declare_parameter("gz_spoofer_model_name", "spoofer");
      this->declare_parameter("gps_delay", 0.0);

      std::string _world_name = this->get_parameter("gz_world_name").as_string();
      std::string _model_name = this->get_parameter("gz_model_name").as_string();
      std::string _spoofer_model_name = this->get_parameter("gz_spoofer_model_name").as_string();
      std::string navsat_topic = "/world/" + _world_name + "/model/" + _model_name + "/link/base_link/sensor/navsat_sensor/navsat_multipath";
      std::string spoofer_topic = "/world/" + _world_name + "/model/" + _spoofer_model_name + "/link/base_link/sensor/navsat_sensor/navsat_multipath";
      std::string mavros_gps_topic = "mavros/gps_input/hil_gps";

      if (!_node.Subscribe(navsat_topic, &PX4GPSRealPublisher::navsatCallback, this)) {
            RCLCPP_ERROR(this->get_logger(),"failed to subscribe to %s", navsat_topic.c_str());
      }
      // if (!_node.Subscribe(spoofer_topic, &PX4GPSRealPublisher::navsatSpooferCallback, this)) {
      //       RCLCPP_ERROR(this->get_logger(),"failed to subscribe to %s", spoofer_topic.c_str());
      // }  
      publisher_ = this->create_publisher<mavros_msgs::msg::HilGPS>(mavros_gps_topic, 1);  
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
    //       publisher_->publish(hil_gps);
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
        hil_gps.geo.latitude = navsat.latitude_deg();                         // [degrees * 1e7]
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
        publisher_->publish(hil_gps);
        // rclcpp::Time t = this->now();
        // double time_now = t.seconds();
        // hilgps_msg_queue.push(hil_gps);
        // time_queue.push(time_now);
        //RCLCPP_ERROR(this->get_logger(), "time before: %f", time_now);
    }

    void navsatSpooferCallback(const gz::msgs::NavSatMultipath &msg_spoofer)
    { 
      gz::sensors::NavSatConverter navsat_converter;
      //gz::sensors::NavSatMultipathSensor navsat_sensor;

      std::vector<double> spoofer_sat_range_meas;
      std::vector<std::vector<double>> spoofer_sat_ECEF;
      double world_origin_lat = 24.484043629238872;
      double world_origin_lon = 54.36068616768677;
      double world_origin_alt = 10.0;
      navsat_converter.initialiseReference(world_origin_lat, world_origin_lon, world_origin_alt);

      spoofer_latitude = msg_spoofer.latitude_deg();
      spoofer_longitude =  msg_spoofer.longitude_deg();
      spoofer_velocity_east = msg_spoofer.velocity_east();
      spoofer_velocity_north = msg_spoofer.velocity_north();

      
      for (int i = 0; i < msg_spoofer.satellite_range().size(); i++)
      {
        spoofer_sat_range_meas.push_back(msg_spoofer.satellite_range(i));        
      }

      for (int i = 0; i < msg_spoofer.satellite_ecef().size(); i++)
      {
        std::vector<double> ecef = {0,0,0};

        ecef[0] = msg_spoofer.satellite_ecef(i).x();
        ecef[1] = msg_spoofer.satellite_ecef(i).y();
        ecef[2] = msg_spoofer.satellite_ecef(i).z();
        spoofer_sat_ECEF.push_back(ecef);        
      }
      double latitude=0, longitude=0, altitude=0;
      Eigen::Vector3d recECEF(0,0,0);
      
      // GetLeastSquaresEstimate(spoofer_sat_range_meas,  spoofer_sat_ECEF, recECEF);
      // navsat_converter.ecef2Geodetic(recECEF(0), recECEF(1),recECEF(2), &latitude,
      //                  &longitude, &altitude);                
    }

    bool GetLeastSquaresEstimate(std::vector<double> _meas,
                  std::vector<std::vector<double>> _sat_ecef, Eigen::Vector3d &_rec_ecef)
    {
      const int nsat = _meas.size();
      Eigen::MatrixXd A, b;
      A.resize(nsat, 4);
      b.resize(nsat, 1);
      Eigen::Matrix<double, 4, 1> dx;
      Eigen::ColPivHouseholderQR<Eigen::MatrixXd> solver;
      double cdt = 0;
      int iter = 0;
      do
      {
        iter++;
        for (int i = 0 ; i < nsat; i++)
        {
          Eigen::Vector3d sat_pos(_sat_ecef[i][0], _sat_ecef[i][1], _sat_ecef[i][2]);
          double dist = (_rec_ecef - sat_pos).norm();
          A.block<1,3>(i,0) = (_rec_ecef - sat_pos).normalized();
          b(i) = _meas[i] - (dist + cdt);
          A(i,3) = -1;
          
        }
        solver.compute(A);
        dx = solver.solve(b);
        _rec_ecef += dx.topRows<3>();
      } while (dx.norm() > 1e-6 && iter < 10);
      return iter < 10;
    }
    
    
    gz::transport::Node _node;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<mavros_msgs::msg::HilGPS>::SharedPtr publisher_;
    TSQueue<mavros_msgs::msg::HilGPS> hilgps_msg_queue; 
    TSQueue<double> time_queue; 

    double spoofer_latitude;
    double spoofer_longitude;
    double spoofer_velocity_east;
    double spoofer_velocity_north;
    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PX4GPSRealPublisher>());
  rclcpp::shutdown();
  return 0;
}
