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
#include "mavros_msgs/msg/hil_gps.hpp"
#include <gz/sensors/NavSatMultipathSensor.hh>
#include <gz/msgs/navsat_multipath.pb.h>


using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class PX4GPSRealPublisher : public rclcpp::Node
{
  public:
    PX4GPSRealPublisher()
    : Node("mavros_gps_real_publisher")
    {
      if (!_node.Subscribe(navsat_topic, &PX4GPSRealPublisher::navsatCallback, this)) {
            RCLCPP_ERROR(this->get_logger(),"failed to subscribe to %s", navsat_topic.c_str());
      }
      if (!_node.Subscribe(spoofer_topic, &PX4GPSRealPublisher::navsatSpooferCallback, this)) {
            RCLCPP_ERROR(this->get_logger(),"failed to subscribe to %s", spoofer_topic.c_str());
      }  
      publisher_ = this->create_publisher<mavros_msgs::msg::HilGPS>(mavros_gps_topic, 10);  
    }
  private:
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
        RCLCPP_ERROR(this->get_logger(),"navsat latitude LS navsat %lf", navsat.latitude_deg());

        publisher_->publish(hil_gps);
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
    rclcpp::Publisher<mavros_msgs::msg::HilGPS>::SharedPtr publisher_;
    gz::transport::Node _node;
    std::string _world_name = "AbuDhabi";
    std::string _model_name = "x500_1";
    std::string _spoofer_model_name = "spoofer";
    std::string navsat_topic = "/world/" + _world_name + "/model/" + _model_name + "/link/base_link/sensor/navsat_sensor/navsat_multipath";
    std::string spoofer_topic = "/world/" + _world_name + "/model/" + _spoofer_model_name + "/link/base_link/sensor/navsat_sensor/navsat_multipath";
    std::string px4_gps_ros_topic = "/px4_1/fmu/in/vehicle_gps_position";
    std::string mavros_gps_topic = "mavros/gps_input/hil_gps";

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
