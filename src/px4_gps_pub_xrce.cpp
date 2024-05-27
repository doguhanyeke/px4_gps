#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <eigen3/Eigen/Eigen>
#include <gz/math.hh>
#include <gz/msgs.hh>
#include <gz/transport.hh>
#include <gz/msgs/navsat_multipath.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/qos.hpp>
#include "std_msgs/msg/string.hpp"
#include "px4_msgs/msg/sensor_gps.hpp"
#include <gz/sensors/NavSatMultipathSensor.hh>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/bool.hpp>
#include "offboard_detector/msg/detector_output.hpp"
#include <vector>
#include <math.h>
#include <cmath>
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */
class Attack {
public:
    double time_t;
    double x;
    double y;
    double z;
    int victim_robot_id;
};

// Geodetic system parameters
static double kSemimajorAxis = 6378137;
static double kSemiminorAxis = 6356752.3142;
static double kFirstEccentricitySquared = 6.69437999014 * 0.001;
static double kSecondEccentricitySquared = 6.73949674228 * 0.001;
static double kFlattening = 1 / 298.257223563;

class NavSatConverter
    {
    public:

      NavSatConverter()
      {
        haveReference_ = false;
      }

      ~NavSatConverter()
      {
      }

      // Default copy constructor and assignment operator are OK.

      bool isInitialised()
      {
        return haveReference_;
      }

      void getReference(double* latitude, double* longitude, double* altitude)
      {
        *latitude = initial_latitude_;
        *longitude = initial_longitude_;
        *altitude = initial_altitude_;
      }

      void initialiseReference(const double latitude, const double longitude, const double altitude)
      {
        // Save NED origin
        initial_latitude_ = deg2Rad(latitude);
        initial_longitude_ = deg2Rad(longitude);
        initial_altitude_ = altitude;

        // Compute ECEF of NED origin
        geodetic2Ecef(latitude, longitude, altitude, &initial_ecef_x_, &initial_ecef_y_, &initial_ecef_z_);

        // Compute ECEF to NED and NED to ECEF matrices
        double phiP = atan2(initial_ecef_z_, sqrt(pow(initial_ecef_x_, 2) + pow(initial_ecef_y_, 2)));

        ecef_to_ned_matrix_ = nRe(phiP, initial_longitude_);
        ned_to_ecef_matrix_ = nRe(initial_latitude_, initial_longitude_).transpose();

        haveReference_ = true;
      }

      void geodetic2Ecef(const double latitude, const double longitude, const double altitude, double* x,
                        double* y, double* z)
      {
        // Convert geodetic coordinates to ECEF.
        // http://code.google.com/p/pysatel/source/browse/trunk/coord.py?r=22
        double lat_rad = deg2Rad(latitude);
        double lon_rad = deg2Rad(longitude);
        double xi = sqrt(1 - kFirstEccentricitySquared * sin(lat_rad) * sin(lat_rad));
        *x = (kSemimajorAxis / xi + altitude) * cos(lat_rad) * cos(lon_rad);
        *y = (kSemimajorAxis / xi + altitude) * cos(lat_rad) * sin(lon_rad);
        *z = (kSemimajorAxis / xi * (1 - kFirstEccentricitySquared) + altitude) * sin(lat_rad);
      }

      void ecef2Geodetic(const double x, const double y, const double z, double* latitude,
                        double* longitude, double* altitude)
      {
        // Convert ECEF coordinates to geodetic coordinates.
        // J. Zhu, "Conversion of Earth-centered Earth-fixed coordinates
        // to geodetic coordinates," IEEE Transactions on Aerospace and
        // Electronic Systems, vol. 30, pp. 957-961, 1994.

        double r = sqrt(x * x + y * y);
        double Esq = kSemimajorAxis * kSemimajorAxis - kSemiminorAxis * kSemiminorAxis;
        double F = 54 * kSemiminorAxis * kSemiminorAxis * z * z;
        double G = r * r + (1 - kFirstEccentricitySquared) * z * z - kFirstEccentricitySquared * Esq;
        double C = (kFirstEccentricitySquared * kFirstEccentricitySquared * F * r * r) / pow(G, 3);
        double S = cbrt(1 + C + sqrt(C * C + 2 * C));
        double P = F / (3 * pow((S + 1 / S + 1), 2) * G * G);
        double Q = sqrt(1 + 2 * kFirstEccentricitySquared * kFirstEccentricitySquared * P);
        double r_0 = -(P * kFirstEccentricitySquared * r) / (1 + Q)
            + sqrt(
                0.5 * kSemimajorAxis * kSemimajorAxis * (1 + 1.0 / Q)
                    - P * (1 - kFirstEccentricitySquared) * z * z / (Q * (1 + Q)) - 0.5 * P * r * r);
        double U = sqrt(pow((r - kFirstEccentricitySquared * r_0), 2) + z * z);
        double V = sqrt(
            pow((r - kFirstEccentricitySquared * r_0), 2) + (1 - kFirstEccentricitySquared) * z * z);
        double Z_0 = kSemiminorAxis * kSemiminorAxis * z / (kSemimajorAxis * V);
        *altitude = U * (1 - kSemiminorAxis * kSemiminorAxis / (kSemimajorAxis * V));
        *latitude = rad2Deg(atan((z + kSecondEccentricitySquared * Z_0) / r));
        *longitude = rad2Deg(atan2(y, x));
      }

      void ecef2Ned(const double x, const double y, const double z, double* north, double* east,
                    double* down)
      {
        // Converts ECEF coordinate position into local-tangent-plane NED.
        // Coordinates relative to given ECEF coordinate frame.

        Eigen::Vector3d vect, ret;
        vect(0) = x - initial_ecef_x_;
        vect(1) = y - initial_ecef_y_;
        vect(2) = z - initial_ecef_z_;
        ret = ecef_to_ned_matrix_ * vect;
        *north = ret(0);
        *east = ret(1);
        *down = -ret(2);
      }

      void ned2Ecef(const double north, const double east, const double down, double* x, double* y,
                    double* z)
      {
        // NED (north/east/down) to ECEF coordinates
        Eigen::Vector3d ned, ret;
        ned(0) = north;
        ned(1) = east;
        ned(2) = -down;
        ret = ned_to_ecef_matrix_ * ned;
        *x = ret(0) + initial_ecef_x_;
        *y = ret(1) + initial_ecef_y_;
        *z = ret(2) + initial_ecef_z_;
      }

      void geodetic2Ned(const double latitude, const double longitude, const double altitude,
                        double* north, double* east, double* down)
      {
        // Geodetic position to local NED frame
        double x, y, z;
        geodetic2Ecef(latitude, longitude, altitude, &x, &y, &z);
        ecef2Ned(x, y, z, north, east, down);
      }

      void ned2Geodetic(const double north, const double east, const double down, double* latitude,
                        double* longitude, double* altitude)
      {
        // Local NED position to geodetic coordinates
        double x, y, z;
        ned2Ecef(north, east, down, &x, &y, &z);
        ecef2Geodetic(x, y, z, latitude, longitude, altitude);
      }

      void geodetic2Enu(const double latitude, const double longitude, const double altitude,
                        double* east, double* north, double* up)
      {
        // Geodetic position to local ENU frame
        double x, y, z;
        geodetic2Ecef(latitude, longitude, altitude, &x, &y, &z);

        double aux_north, aux_east, aux_down;
        ecef2Ned(x, y, z, &aux_north, &aux_east, &aux_down);

        *east = aux_east;
        *north = aux_north;
        *up = -aux_down;
      }

      void enu2Geodetic(const double east, const double north, const double up, double* latitude,
                        double* longitude, double* altitude)
      {
        // Local ENU position to geodetic coordinates

        const double aux_north = north;
        const double aux_east = east;
        const double aux_down = -up;
        double x, y, z;
        ned2Ecef(aux_north, aux_east, aux_down, &x, &y, &z);
        ecef2Geodetic(x, y, z, latitude, longitude, altitude);
      }

    private:
      inline Eigen::Matrix3d nRe(const double lat_radians, const double lon_radians)
      {
        const double sLat = sin(lat_radians);
        const double sLon = sin(lon_radians);
        const double cLat = cos(lat_radians);
        const double cLon = cos(lon_radians);

        Eigen::Matrix3d ret;
        ret(0, 0) = -sLat * cLon;
        ret(0, 1) = -sLat * sLon;
        ret(0, 2) = cLat;
        ret(1, 0) = -sLon;
        ret(1, 1) = cLon;
        ret(1, 2) = 0.0;
        ret(2, 0) = cLat * cLon;
        ret(2, 1) = cLat * sLon;
        ret(2, 2) = sLat;

        return ret;
      }

      inline
      double rad2Deg(const double radians)
      {
        return (radians / M_PI) * 180.0;
      }

      inline
      double deg2Rad(const double degrees)
      {
        return (degrees / 180.0) * M_PI;
      }

      double initial_latitude_;
      double initial_longitude_;
      double initial_altitude_;

      double initial_ecef_x_;
      double initial_ecef_y_;
      double initial_ecef_z_;

      Eigen::Matrix3d ecef_to_ned_matrix_;
      Eigen::Matrix3d ned_to_ecef_matrix_;

      bool haveReference_;

    }; // class NavSatConverter

class PX4GPSXrcePublisher : public rclcpp::Node
{
  public:
    PX4GPSXrcePublisher()
    : Node("px4_gps_xrce_publisher")
    {
      this->declare_parameter("px4_ns", "px4_1");
      this->declare_parameter("gz_world_name", "AbuDhabi");
      this->declare_parameter("gz_model_name", "x500_1");
      this->declare_parameter("gz_spoofer_model_name", "spoofer");
      //this->declare_parameter("attack_parameters", std::vector<double>{});

      std::string _px4_ns = this->get_parameter("px4_ns").as_string();
      std::string _world_name = this->get_parameter("gz_world_name").as_string();
      std::string _model_name = this->get_parameter("gz_model_name").as_string();
      std::string _spoofer_model_name = this->get_parameter("gz_spoofer_model_name").as_string();
      this->declare_parameter<std::vector<double>>("attack_parameters",std::vector<double>{} );
      std::vector<double> attack_parameters;
      this->get_parameter("attack_parameters", attack_parameters);
      //std::vector<double> attack_parameters = this->get_parameter("attack_parameters").as_double_array();
      RCLCPP_INFO(this->get_logger(),"Size: %ld", attack_parameters.size());
      

      for (size_t i = 0; i < attack_parameters.size(); i += 5)
      {
          Attack attack;
          attack.time_t = attack_parameters[i];
          attack.x = attack_parameters[i + 1];
          attack.y = attack_parameters[i + 2];
          attack.z = attack_parameters[i + 3];
          attack.victim_robot_id = static_cast<int>(attack_parameters[i + 4]);
          RCLCPP_INFO(this->get_logger(), "%d %d",attack.victim_robot_id,  _px4_ns.back()-'0');

          if (attack.victim_robot_id == _px4_ns.back()-'0')
          {
            _attack_sequence.push_back(attack);
            RCLCPP_INFO(this->get_logger(), "%f %f %f %f %d", attack.time_t, attack.x, attack.y, attack.z, attack.victim_robot_id);
          }    
      }

      RCLCPP_INFO(this->get_logger(),"PX4 Namespace: %s", _px4_ns.c_str());
      RCLCPP_INFO(this->get_logger(),"World Name: %s", _world_name.c_str());
      RCLCPP_INFO(this->get_logger(),"Drone Model Name: %s", _model_name.c_str());
      RCLCPP_INFO(this->get_logger(),"Spoofer Model Name: %s", _spoofer_model_name.c_str());
      
      std::string navsat_topic = "/world/" + _world_name + "/model/" + _model_name + "/link/base_link/sensor/navsat_sensor/navsat_multipath";
      std::string spoofer_topic = "/world/" + _world_name + "/model/" + _spoofer_model_name + "/link/base_link/sensor/navsat_sensor/navsat_multipath";
      
      std::string px4_gps_ros_topic= "";
      std::string px4_all_hover_topic= "";
      std::string px4_attack_topic= "";
      if (_px4_ns.empty()){
        px4_gps_ros_topic = "/fmu/in/vehicle_gps_position";
      }
      else{
        px4_gps_ros_topic = "/" +  _px4_ns + "/fmu/in/vehicle_gps_position";
        px4_all_hover_topic = "/" +  _px4_ns + "/fmu/out/all_hover";
        px4_attack_topic =  "/" +  _px4_ns + "/attack_flag";
      }
      RCLCPP_INFO(this->get_logger(),"PX4 GPS Topic: %s", px4_gps_ros_topic.c_str());

      if (!_node.Subscribe(navsat_topic, &PX4GPSXrcePublisher::navsatCallback, this)) {
            RCLCPP_ERROR(this->get_logger(),"failed to subscribe to %s", navsat_topic.c_str());
      }
      if (!_node.Subscribe(spoofer_topic, &PX4GPSXrcePublisher::navsatSpooferCallback, this)) {
            RCLCPP_ERROR(this->get_logger(),"failed to subscribe to %s", spoofer_topic.c_str());
      }  
      
      rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
      qos_profile.history=RMW_QOS_POLICY_HISTORY_KEEP_LAST;
      qos_profile.depth=1;
      qos_profile.reliability=RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
      qos_profile.durability=RMW_QOS_POLICY_DURABILITY_VOLATILE;
      auto qos_ = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth), qos_profile);
      
      gps_publisher_ = this->create_publisher<px4_msgs::msg::SensorGps>(px4_gps_ros_topic, 10);  
      attack_flag_publisher_ = this->create_publisher<std_msgs::msg::Bool>(px4_attack_topic, 10);      
      detector_flag_subscriber_ = this->create_subscription<offboard_detector::msg::DetectorOutput>(
        "/px4_1/detector/out/observer_output", qos_, std::bind(&PX4GPSXrcePublisher::detectorFlagCallback, this, std::placeholders::_1));
      all_hover_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        px4_all_hover_topic , qos_, std::bind(&PX4GPSXrcePublisher::allHoverCallback, this, std::placeholders::_1)); 

      count=0;
      hover_flag = 0;
      time_offset = 0.0;
      attack_idx = 0;
      attack_counter = 10;
     
      double home_lat = 24.484043629238872;
      double home_lon = 54.36068616768677;
      double home_alt = 0.0;
      navsat_conv->initialiseReference(home_lat, home_lon, home_alt);
    }
  private:
    void navsatCallback(const gz::msgs::NavSatMultipath &navsat)
    {   
        
        const uint64_t time_us = (navsat.header().stamp().sec() * 1000000) + (navsat.header().stamp().nsec() / 1000);
        // publish gps
        auto sensor_gps = px4_msgs::msg::SensorGps();
        sensor_gps.device_id =11468804; // (Type: 0xAF, SIMULATION:0 (0x00))
        sensor_gps.timestamp_sample = time_us;
        sensor_gps.timestamp = time_us;
        sensor_gps.fix_type = 3; // 3D fix
        sensor_gps.s_variance_m_s = 0.4f;
        sensor_gps.c_variance_rad = 0.1f;
        sensor_gps.eph = 0.9f;
        sensor_gps.epv = 1.78f;
        sensor_gps.hdop = 0.7f;
        sensor_gps.vdop = 1.1f;
        sensor_gps.altitude_msl_m = navsat.altitude();
        sensor_gps.altitude_ellipsoid_m = navsat.altitude();
        sensor_gps.noise_per_ms = 0;
        sensor_gps.jamming_indicator = 0;
        // For normal operations
        sensor_gps.latitude_deg = navsat.latitude_deg();
        sensor_gps.longitude_deg = navsat.longitude_deg();
        sensor_gps.vel_m_s = sqrtf(
                navsat.velocity_east() * navsat.velocity_east()
                + navsat.velocity_north() * navsat.velocity_north()
                + navsat.velocity_up() * navsat.velocity_up());
        sensor_gps.vel_n_m_s = navsat.velocity_north();
        sensor_gps.vel_e_m_s = navsat.velocity_east();
        sensor_gps.vel_d_m_s = -navsat.velocity_up();
        sensor_gps.cog_rad = atan2(navsat.velocity_east(), navsat.velocity_north());
        
        auto attack_flag = std_msgs::msg::Bool();
        if (all_hover==1 && hover_flag ==0)
        {
          start_time = time_us;
          hover_flag = 1;
          attack_flag.data = false;
        }
        if (hover_flag == 1)
        {
          time_offset = std::round(((time_us - start_time)*1e-6)/0.001)*0.001;  
          double east, north, up, lat, lon, alt;
            
          navsat_conv->geodetic2Enu(navsat.latitude_deg(),navsat.longitude_deg(),navsat.altitude(), &east, &north, &up);
          RCLCPP_INFO(this->get_logger(),"Time Offset: %f enu: %f%f%f", time_offset, east, north, up);          
         
          if (attack_idx < _attack_sequence.size() && time_offset > _attack_sequence[attack_idx].time_t)
          {
            RCLCPP_INFO(this->get_logger(),"Time Offset: %f \t %f", time_offset ,_attack_sequence[attack_idx].time_t );          
            RCLCPP_INFO(this->get_logger(),"Old LatLonAlt: %f \t %f \t %f", sensor_gps.latitude_deg , sensor_gps.longitude_deg, sensor_gps.altitude_msl_m);  
            
            attack_flag.data = true;
            east = east + _attack_sequence[attack_idx].x;
            north = north + _attack_sequence[attack_idx].y;
            up = up + _attack_sequence[attack_idx].z;
            navsat_conv->enu2Geodetic(east, north, up, &lat, &lon, &alt);

            sensor_gps.latitude_deg  = lat;
            sensor_gps.longitude_deg =  lon;
            sensor_gps.altitude_msl_m = alt;
            
            RCLCPP_INFO(this->get_logger(),"Attack LatLonAlt: %f \t %f \t %f", sensor_gps.latitude_deg , sensor_gps.longitude_deg, sensor_gps.altitude_msl_m);  

            attack_counter--;
            if (attack_counter == 0 )
            {
              attack_counter = 10;
              attack_idx ++;
            }
          }
        }
        sensor_gps.timestamp_time_relative = 0;
        sensor_gps.heading = NAN;
        sensor_gps.heading_offset = NAN;
        sensor_gps.heading_accuracy = 0;
        sensor_gps.automatic_gain_control = 0;
        sensor_gps.jamming_state = 0;
        sensor_gps.spoofing_state = 0;
        sensor_gps.vel_ned_valid = true;
        sensor_gps.satellites_used = 10;
        count++;
        gps_publisher_->publish(sensor_gps);
        attack_flag_publisher_->publish(attack_flag);
    }
    
    void attackFlagCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
      attack_flag = msg->data;
    }
    
    void allHoverCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
      all_hover = msg->data;
    }

    void detectorFlagCallback(const offboard_detector::msg::DetectorOutput::SharedPtr msg)
    {
      detector_flag = msg->attack_detect;
      attack_flag = 0;
    }
    
    void navsatSpooferCallback(const gz::msgs::NavSatMultipath &msg_spoofer)
    { 
      spoofer_latitude = msg_spoofer.latitude_deg();
      spoofer_longitude =  msg_spoofer.longitude_deg();
      spoofer_velocity_east = msg_spoofer.velocity_east();
      spoofer_velocity_north = msg_spoofer.velocity_north();            
    }
  
    rclcpp::Publisher<px4_msgs::msg::SensorGps>::SharedPtr gps_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr attack_flag_publisher_;
    rclcpp::Subscription<offboard_detector::msg::DetectorOutput>::SharedPtr detector_flag_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr all_hover_subscriber_;


    gz::transport::Node _node;
    int count;
    double spoofer_latitude;
    double spoofer_longitude;
    double spoofer_velocity_east;
    double spoofer_velocity_north;
    int attack_flag;
    int detector_flag;
    int all_hover;
    int hover_flag;
    uint64_t start_time;
    double time_offset;
    std::vector<Attack> _attack_sequence;
    size_t attack_idx;
    int attack_counter;
    NavSatConverter *navsat_conv = new NavSatConverter();
    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PX4GPSXrcePublisher>());
  rclcpp::shutdown();
  return 0;
}
