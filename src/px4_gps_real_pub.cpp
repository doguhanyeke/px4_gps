#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <eigen3/Eigen/Eigen>
#include <gz/math.hh>
#include <gz/msgs.hh>
#include <gz/transport.hh>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "px4_msgs/msg/sensor_gps.hpp"
#include <gz/sensors/NavSatMultipathSensor.hh>
#include <gz/msgs/navsat_multipath.pb.h>


using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class PX4GPSRealPublisher : public rclcpp::Node
{
  public:
    PX4GPSRealPublisher()
    : Node("px4_gps_real_publisher")
    {
      std::string navsat_topic = "/world/AbuDhabi/model/x500_1/link/base_link/sensor/navsat_sensor/navsat_multipath";
      std::string px4_gps_ros_topic = "/fmu/in/vehicle_gps_position";
      
      if (!_node.Subscribe(navsat_topic, &PX4GPSRealPublisher::navsatCallback, this)) {
            RCLCPP_ERROR(this->get_logger(),"failed to subscribe to %s", navsat_topic.c_str());
      }
      // if (!_node.Subscribe(spoofer_topic, &PX4GPSRealPublisher::navsatSpooferCallback, this)) {
      //       RCLCPP_ERROR(this->get_logger(),"failed to subscribe to %s", spoofer_topic.c_str());
      // }  
      publisher_ = this->create_publisher<px4_msgs::msg::SensorGps>(px4_gps_ros_topic, 10);  
      count=0;
    }
  private:
    void navsatCallback(const gz::msgs::NavSatMultipath &navsat)
    {   
        const uint64_t time_us = (navsat.header().stamp().sec() * 1000000) + (navsat.header().stamp().nsec() / 1000);
        // publish gps
        auto sensor_gps = px4_msgs::msg::SensorGps();
      
        // sensor_gps.device_id =11010053; // (Type: 0xA8, SERIAL:0 (0x00))
        // sensor_gps.timestamp_sample = time_us;
        // sensor_gps.timestamp = time_us;

        // sensor_gps.fix_type = 3; // 3D fix
        // sensor_gps.s_variance_m_s = 0.4f;
        // sensor_gps.c_variance_rad = 0.1f;
        // sensor_gps.eph = 0.9f;
        // sensor_gps.epv = 1.78f;
        // sensor_gps.hdop = 0.7f;
        // sensor_gps.vdop = 1.1f;

        // sensor_gps.lat = navsat.latitude_deg()*1e7; //+ 0.00000025*count;
        // sensor_gps.lon = navsat.longitude_deg()*1e7;
        // sensor_gps.alt = navsat.altitude()*1e3;
        // sensor_gps.alt_ellipsoid = navsat.altitude()*1e3;
        // sensor_gps.noise_per_ms = 0;
        // sensor_gps.jamming_indicator = 0;
        // sensor_gps.vel_m_s = sqrtf(
        //         navsat.velocity_east() * navsat.velocity_east()
        //         + navsat.velocity_north() * navsat.velocity_north()
        //         + navsat.velocity_up() * navsat.velocity_up());
        // sensor_gps.vel_n_m_s = navsat.velocity_north();
        // sensor_gps.vel_e_m_s = navsat.velocity_east();
        // sensor_gps.vel_d_m_s = -navsat.velocity_up();
        // sensor_gps.cog_rad = atan2(navsat.velocity_east(), navsat.velocity_north());
        // sensor_gps.timestamp_time_relative = 0;
        // sensor_gps.heading = NAN;
        // sensor_gps.heading_offset = NAN;
        // sensor_gps.heading_accuracy = 0;
        // sensor_gps.automatic_gain_control = 0;
        // sensor_gps.jamming_state = 0;
        // sensor_gps.spoofing_state = 0;
        // sensor_gps.vel_ned_valid = true;
        // sensor_gps.satellites_used = 10;
        count++;
        publisher_->publish(sensor_gps);
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
      
      GetLeastSquaresEstimate(spoofer_sat_range_meas,  spoofer_sat_ECEF, recECEF);
      navsat_converter.ecef2Geodetic(recECEF(0), recECEF(1),recECEF(2), &latitude,
                       &longitude, &altitude);
      RCLCPP_ERROR(this->get_logger(),"spoofer latitude LS navsat %lf", msg_spoofer.latitude_deg());
                
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
    rclcpp::Publisher<px4_msgs::msg::SensorGps>::SharedPtr publisher_;
    gz::transport::Node _node;
    int count;
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
