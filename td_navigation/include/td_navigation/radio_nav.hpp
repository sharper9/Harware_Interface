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
      distance_to_base_rads.resize(3);
      x = 0;
      y = 0;
      z = 0;
      bearing = 0;
    };

    mobile_radio(std::vector <double> distance_to_base_radios){
      distance_to_base_rads.reserve(3);
      distance_to_base_rads = distance_to_base_radios;
      x = 0;
      y = 0;
      z = 0;
      bearing = 0;
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
  double get_mobile_radio_coordinate(int radio_index, int coordinate_index);
  double get_base_radio_coordinate(int radio_index, int coordinate_index);
  int triangulate_2Base(double z_estimate);
  double law_of_cosines(base_radio& base_rad_0, base_radio& base_rad_1,
                        mobile_radio& mob_rad, double z_estimate, double estimate_angle_0, double estimate_angle_1);
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

double radio_nav::get_mobile_radio_coordinate(int radio_index, int coordinate_index){
  switch(coordinate_index){
    case 0: return mobile_radios[radio_index].x;
    case 1: return mobile_radios[radio_index].y;
    case 2: return mobile_radios[radio_index].z;
    case 3: return mobile_radios[radio_index].bearing;
  }
  return -1;
}

double radio_nav::get_base_radio_coordinate(int radio_index, int coordinate_index){
  switch(coordinate_index){
    case 0: return base_radios[radio_index].x;
    case 1: return base_radios[radio_index].y;
    case 2: return base_radios[radio_index].z;
  }
  return -1;
}


int radio_nav::triangulate_2Base(double z_estimate){
  double estimate_angle_0;
  double estimate_angle_1;


  for(int i = 0; i < mobile_radios.size(); i++){

    estimate_angle_0 = asin( (z_estimate) / mobile_radios[i].distance_to_base_rads[0]);

    estimate_angle_1 = asin( (z_estimate) / mobile_radios[i].distance_to_base_rads[1]);

    double angle = law_of_cosines(base_radios[0], base_radios[1], mobile_radios[i], z_estimate, estimate_angle_0, estimate_angle_1);
    if (angle == -500){
      return -1;
    }


    mobile_radios[i].x = (mobile_radios[i].distance_to_base_rads[0] * cos(estimate_angle_0) * sin(angle)) + base_radios[0].x;
    mobile_radios[i].y = (mobile_radios[i].distance_to_base_rads[0] * cos(estimate_angle_0) * cos(angle)) + base_radios[0].y;
    mobile_radios[i].z = z_estimate;
    mobile_radios[i].bearing = smart_atan(mobile_radios[i].y, mobile_radios[i].x);

  }
  return 0;
}


double radio_nav::law_of_cosines(base_radio& base_rad_0, base_radio& base_rad_1,
                                 mobile_radio& mob_rad, double z_estimate, double estimate_angle_0, double estimate_angle_1){
    double temp;
    double base_station_distance = calculate_base_station_distance(base_rad_0, base_rad_1);


  //  ROS_DEBUG("distrad1: %lf, distrad0: %lf, rad_0_y: %lf, rad_1_y: %lf", mob_rad.distance_to_base_rads[1], mob_rad.distance_to_base_rads[0],
  //            base_rad_0.y, base_rad_1.y);

    //triangulation of rad 0
    temp = pow(mob_rad.distance_to_base_rads[0] *
               cos(estimate_angle_0) ,2.0) +
           pow(base_station_distance,2.0) -
           pow(mob_rad.distance_to_base_rads[1] *
               cos(estimate_angle_1) ,2.0);

    if (temp/(2.0 * base_station_distance * mob_rad.distance_to_base_rads[0] * cos(estimate_angle_0)) >= -1
        && temp/(2.0 * base_station_distance * mob_rad.distance_to_base_rads[0]* cos(estimate_angle_0)) <= 1){

      return acos(temp/(2.0 * base_station_distance * mob_rad.distance_to_base_rads[0]) * cos(estimate_angle_0));
  }

  return -500; //outside of domain of acos, measurement inaccurate

}

double radio_nav::smart_atan(double adj, double opp){
      if (adj == 0 && opp >= 0){
        return PI / 2.0;
      }else if (adj == 0 && opp < 0){
        return -1 * PI / 2.0;
      }else if (adj < 0){
        return atan(opp/adj) + PI;
      }else {
        return atan(opp/adj);
      }
}


double radio_nav::calculate_base_station_distance(base_radio& b_rad_0, base_radio& b_rad_1){
  return sqrt( pow(b_rad_0.x - b_rad_1.x, 2.0) + pow(b_rad_0.y - b_rad_1.y, 2.0) + pow(b_rad_0.z - b_rad_1.z, 2.0) );
}

#endif // RADIO_NAV_HPP__
