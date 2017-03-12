#ifndef RADIO_NAV_HPP__
#define RADIO_NAV_HPP__

#include <math.h>

class radio_nav
{
  class radio
  {
  public:

    double distl, distr;
    double x, y;
    double angle;

    radio(){
    };


  };

  double base_station;
  radio rad0;
  radio rad1;

public:

  radio_nav();
  void triangulate(double& dist_base_station, double& dist0_l, double& dist0_r, double& dist1_l, double& dist1_r);
  double get_rad0_x();
  double get_rad0_y();
  double get_rad0_angle();
  double get_rad1_x();
  double get_rad1_y();
  double get_rad1_angle();

};

radio_nav::radio_nav(){};

void radio_nav::triangulate(double& dist_base_station, double& dist0_l, double& dist0_r, double& dist1_l, double& dist1_r){
  base_station = dist_base_station;
  double temp;

  rad0.distl = dist0_l;
  rad0.distr = dist0_r;
  rad1.distl = dist1_l;
  rad1.distr = dist1_r;

  //triangulation of rad 0
  temp = pow(rad0.distr,2.0) + pow(base_station,2.0) - pow(rad0.distl,2.0);
  rad0.angle = acos(temp/(2.0 * base_station * rad0.distl));
  rad0.x = (rad0.distr * cos(rad0.angle)) - (base_station/(2.0));
  rad0.y = rad0.distr * sin(rad0.angle);

  //triangulation of rad_1
  temp = (pow(rad1.distr,2.0) + pow(base_station,2.0) - pow(rad1.distl,2.0));
  rad1.angle = acos(temp/(2.0 * base_station * rad1.distl));
  rad1.x = (rad1.distr * cos(rad1.angle)) - (base_station/(2.0));
  rad1.y = rad1.distr * sin(rad1.angle);

}

double radio_nav::get_rad0_x(){
  return rad0.x;
}

double radio_nav::get_rad0_y(){
  return rad0.y;
}

double radio_nav::get_rad0_angle(){
  return rad0.angle;
}

double radio_nav::get_rad1_x(){
  return rad1.x;
}

double radio_nav::get_rad1_y(){
  return rad1.y;
}

double radio_nav::get_rad1_angle(){
  return rad1.angle;
}

#endif // RADIO_NAV_HPP__
