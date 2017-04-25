#ifndef RADIO_NAV_HPP__
#define RADIO_NAV_HPP__

#include <math.h>
#include <vector>

#define PI 3.14159265

class radio_nav
{
  class base_radio
  {
  public:

    double x, y,z;

    base_radio(double x_value, double y_value, double z_value){
      x = x_value;
      y = y_value;
      z = z_value;
    };


  };

  class mobile_radio
  {
  public:

    std::vector <double> distance_to_base_rads;
    double x, y, z, bearing;

    mobile_radio(){
      distance_to_base_rads.reserve(3);
    };

    mobile_radio(std::vector <double> distance_to_base_radios){
      distance_to_base_rads = distance_to_base_radios;
    };

  };

  std::vector <base_radio> base_radios;
  std::vector <mobile_radio> mobile_radios;

public:

  radio_nav();

  int create_base_radio(double x_value, double y_value, double z_value);
  int create_mobile_radio();
  int create_mobile_radio(std::vector <double> distance_to_base_radios);
  int update_mobile_radio(int radio_index, std::vector <double> distance_to_base_radios);
  int get_mobile_radio_coordinate(int radio_index, int coordinate_index);
  int get_base_radio_coordinate(int radio_index, int coordinate_index);
  int triangulate_2Base(double z_estimate);
  double law_of_cosines(base_radio& base_rad_0, base_radio& base_rad_1,
                        mobile_radio& mob_rad, double z_estimate);
  double smart_atan(double x, double y);
  double calculate_base_station_distance(base_radio& b_rad_0, base_radio& b_rad_1);

};

radio_nav::radio_nav(){
  base_radios.reserve(3);
  mobile_radios.reserve(3);
};

int radio_nav::create_base_radio(double x_value, double y_value, double z_value){
  base_radio b_rad(x_value, y_value, z_value);
  base_radios.push_back(b_rad);
  return 0;
}

int radio_nav::create_mobile_radio(){
  mobile_radio m_rad;
  mobile_radios.push_back(m_rad);
  return 0;
}

int radio_nav::create_mobile_radio(std::vector <double> distance_to_base_radios){
  mobile_radio m_rad(distance_to_base_radios);
  mobile_radios.push_back(m_rad);
  return 0;
}

int radio_nav::update_mobile_radio(int radio_index, std::vector <double> distance_to_base_radios){
  mobile_radios[radio_index].distance_to_base_rads = distance_to_base_radios;
  return 0;
}

int radio_nav::get_mobile_radio_coordinate(int radio_index, int coordinate_index){
  switch(coordinate_index){
    case 0: return mobile_radios[radio_index].x;
    case 1: return mobile_radios[radio_index].y;
    case 2: return mobile_radios[radio_index].z;
    case 3: return mobile_radios[radio_index].bearing;
  }
  return -1;
}

int radio_nav::get_base_radio_coordinate(int radio_index, int coordinate_index){
  switch(coordinate_index){
    case 0: return base_radios[radio_index].x;
    case 1: return base_radios[radio_index].y;
    case 2: return base_radios[radio_index].z;
  }
  return -1;
}

int radio_nav::triangulate_2Base(double z_estimate){
  ROS_INFO("TEST: 3.1");
  for(int i = 0; i < mobile_radios.size(); i++){
    int angle = law_of_cosines(base_radios[0], base_radios[1], mobile_radios[i], z_estimate);
    if (angle == -500){
      return -1;
    }
    ROS_INFO("TEST: 3.2");

    mobile_radios[i].x = mobile_radios[i].distance_to_base_rads[0] * cos(angle) + base_radios[0].x;
    mobile_radios[i].y = mobile_radios[i].distance_to_base_rads[0] * sin(angle) + base_radios[0].y;
    mobile_radios[i].z = z_estimate;
    mobile_radios[i].bearing = smart_atan(mobile_radios[i].x, mobile_radios[i].y);

    ROS_INFO("TEST: 3.3");
  }
  return 0;
}


double radio_nav::law_of_cosines(base_radio& base_rad_0, base_radio& base_rad_1,
                                 mobile_radio& mob_rad, double z_estimate){
    double temp;
    double base_station_distance = calculate_base_station_distance(base_rad_0, base_rad_1);

    ROS_DEBUG("distrad1: %lf, distrad0: %lf", mob_rad.distance_to_base_rads[1], mob_rad.distance_to_base_rads[0]);
    ROS_INFO("TEST: 3.1.1");

    //triangulation of rad 0
    temp = pow(mob_rad.distance_to_base_rads[1] *
               cos( asin( (z_estimate - base_rad_1.z) / mob_rad.distance_to_base_rads[1])) ,2.0) +
           pow(base_station_distance,2.0) -
           pow(mob_rad.distance_to_base_rads[0] *
               cos( asin( (z_estimate - base_rad_0.z) / mob_rad.distance_to_base_rads[0])) ,2.0);
    ROS_INFO("TEST: 3.1.2");

    if (temp/(2.0 * base_station_distance * mob_rad.distance_to_base_rads[1]) >= -1
        && temp/(2.0 * base_station_distance * mob_rad.distance_to_base_rads[1]) <= 1){
      ROS_INFO("TEST: 3.1.3");
      return acos(temp/(2.0 * base_station_distance * mob_rad.distance_to_base_rads[1]));
  }

  return -500; //outside of domain of acos, measurement inaccurate

}

double radio_nav::smart_atan(double x, double y){
      if (x == 0 && y >= 0){
        return PI / 2.0;
      }else if (x == 0 && y < 0){
        return -1 * PI / 2.0;
      }else if (x < 0){
        return atan(y/x) + PI;
      }else {
        return atan(y/x);
      }
}


double radio_nav::calculate_base_station_distance(base_radio& b_rad_0, base_radio& b_rad_1){
  return sqrtl( pow(b_rad_0.x - b_rad_1.x, 2.0) + pow(b_rad_0.y - b_rad_1.y, 2.0));
}


// void radio_nav::set_base_station_distance(double& base_station_distance){
//   base_station = base_station_distance;
// }
//
// void radio_nav::set_rad0_distl(double& distance_to_left_rad){
//   rad0.distl = distance_to_left_rad;
// }
//
// void radio_nav::set_rad0_distr(double& distance_to_right_rad){
//   rad0.distr = distance_to_right_rad;
// }
//
// void radio_nav::set_rad1_distl(double& distance_to_left_rad){
//   rad1.distl = distance_to_left_rad;
// }
//
// void radio_nav::set_rad1_distr(double& distance_to_right_rad){
//   rad1.distr = distance_to_right_rad;
// }
//
// void radio_nav::set_all(double& dist_base_station, double& dist0_l, double& dist0_r, double& dist1_l, double& dist1_r){
//   base_station = dist_base_station;
//   rad0.distl = dist0_l;
//   rad0.distr = dist0_r;
//   rad1.distl = dist1_l;
//   rad1.distr = dist1_r;
// }
//
// void radio_nav::triangulate(){
//
//   double temp;
//
//   //triangulation of rad 0
//   temp = pow(rad0.distr,2.0) + pow(base_station,2.0) - pow(rad0.distl,2.0);
//   if (temp/(2.0 * base_station * rad0.distr) >= -1 && temp/(2.0 * base_station * rad0.distr) <= 1){
//     rad0.angle = acos(temp/(2.0 * base_station * rad0.distr));
//     rad0.x = (rad0.distr * cos(rad0.angle)) - (base_station/(2.0));
//     rad0.y = rad0.distr * sin(rad0.angle);
//
//     if (rad0.x == 0 && rad0.y >= 0){
//       rad0.angle = PI / 2.0;
//     }else if (rad0.x == 0 && rad0.y < 0){
//       rad0.angle = -1 * PI / 2.0;
//     }else if (rad0.x < 0){
//       rad0.angle = atan(rad0.y/rad0.x) + PI;
//     }else {
//       rad0.angle = atan(rad0.y/rad0.x);
//     }
//
//     rad0.isTriangle = true;
//   }
//
//   //triangulation of rad_1
//   temp = (pow(rad1.distr,2.0) + pow(base_station,2.0) - pow(rad1.distl,2.0));
//   if (temp/(2.0 * base_station * rad1.distr) >= -1 && temp/(2.0 * base_station * rad1.distr) <= 1){
//     rad1.angle = acos(temp/(2.0 * base_station * rad1.distr));
//     rad1.x = (rad1.distr * cos(rad1.angle)) - (base_station/(2.0));
//     rad1.y = rad1.distr * sin(rad1.angle);
//
//     if (rad1.x == 0 && rad1.y >= 0){
//       rad1.angle = PI / 2.0;
//     }else if (rad1.x == 0 && rad1.y < 0){
//       rad1.angle = -1 * PI / 2.0;
//     }else if (rad1.x < 0){
//       rad1.angle = atan(rad1.y/rad1.x) + PI;
//     }else {
//       rad1.angle = atan(rad1.y/rad1.x);
//     }
//
//     rad1.isTriangle = true;
//   }
//
//   if (rad0.isTriangle && rad1.isTriangle){
//     //calculate bearing
//     if ((rad1.x + rad0.x) == 0 && (rad1.y + rad0.y) >= 0){
//       bearing = PI / 2.0;
//     } else if ((rad1.x + rad0.x) == 0 && (rad1.y + rad0.y) < 0){
//       bearing = -1 * PI / 2.0;
//     }else if ((rad1.x + rad0.x) < 0){
//       bearing = atan(((rad1.y + rad0.y)/2.0) / ((rad1.x + rad0.x)/2.0)) + PI;
//     } else {
//       bearing = atan(((rad1.y + rad0.y)/2.0) / ((rad1.x + rad0.x)/2.0));
//     }
//
//     //calculate heading
//     if ((rad1.x - rad0.x) == 0 && (rad1.y - rad0.y) >= 0){
//       heading = PI / 2.0;
//     }else if ((rad1.x - rad0.x) == 0 && (rad1.y - rad0.y) < 0){
//       heading = -1 * PI / 2.0;
//     }else if ((rad1.x - rad0.x) < 0){
//       heading = atan((rad1.y - rad0.y) / (rad1.x - rad0.x)) + PI;
//     }else {
//       heading = atan((rad1.y - rad0.y) / (rad1.x - rad0.x));
//     }
//   }
//
// }
//
//
//
// double radio_nav::get_rad0_x(){
//   return rad0.x;
// }
//
// double radio_nav::get_rad0_y(){
//   return rad0.y;
// }
//
// double radio_nav::get_rad0_angle(){
//   return rad0.angle;
// }
//
// double radio_nav::get_rad1_x(){
//   return rad1.x;
// }
//
// double radio_nav::get_rad1_y(){
//   return rad1.y;
// }
//
// double radio_nav::get_rad1_angle(){
//   return rad1.angle;
// }
//
// double radio_nav::get_heading(){
//   return heading;
// }
//
// double radio_nav::get_bearing(){
//   return bearing;
// }
//
// bool radio_nav::success(){
//   return (rad0.isTriangle && rad1.isTriangle);
// }

#endif // RADIO_NAV_HPP__
