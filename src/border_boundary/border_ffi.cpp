//=======================================================================
// Copyright 1997, 1998, 1999, 2000 University of Notre Dame.
// Authors: Andrew Lumsdaine, Lie-Quan Lee, Jeremy G. Siek
//
// Distributed under the Boost Software License, Version 1.0. (See
// accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//=======================================================================
/* extern "C" makes the functions callable from C */
extern "C"
{
#include "TH.h"
}

#include <iostream>
#include <vector>
#include <cmath>
#include <tuple>
#include <unordered_map>


using namespace std;

/*struct Point {
  Point(int x1, int x2, int x3)
  : tuple(x1,x2,x3) {}
  boost::tuples::tuple<int, int, int> tuple;
};*/

typedef tuple<int, int, int> Point;

struct Value {

  bool neighbors[6] = {0};
  Value* neighbor_values[6] = {0};

  unsigned char r=255;
  unsigned char g=0;
  unsigned char b=0;
  void set_neighbor(int i, bool is_known) {
    neighbors[i] = is_known;
  }
void set_rgb(unsigned char r_new, unsigned char g_new, unsigned char b_new) {
    r=r_new;
    b=b_new;
    g=g_new;
  }

  bool all_neighbors_seen() {
    return neighbors[0] && neighbors[1] && neighbors[2] && neighbors[3] && neighbors[4] && neighbors[5];
  }
    void set_neighbor_value(int i, Value* v) {
    neighbor_values[i] = v;
  }
    void set_neighbor_null(int i) {
    neighbor_values[i] = 0;
  }

};

//start = locations[0-2], end = locations[3-5]
void generate_ray(double* location_start, int start, double* location_end, int end, double & slopeX, double& slopeY, double& slopeZ) {
  double num_x = (round(location_end[end + 0])-round(location_start[start + 0]));
  double num_y = (round(location_end[end + 1])-round(location_start[start + 1]));
  double num_z = (round(location_end[end + 2])-round(location_start[start + 2]));
  int num_locations = (int)round(max(max(fabs(num_x),fabs(num_y)),fabs(num_z))+1);
  slopeX = num_x/(double)(num_locations-1);
  slopeY = num_y/(double)(num_locations-1);
  slopeZ = num_z/(double)(num_locations-1);


}


struct iequal_to
  : std::binary_function<Point, Point, bool>
  {
  bool operator()(Point const& a,
   Point const& b) const
  {
    return get<0>(a) == get<0>(b) &&
      get<1>(a) == get<1>(b) &&
      get<2>(a) == get<2>(b);
    }
};


struct ihash
    : std::unary_function<Point, std::size_t>
{
    std::size_t operator()(Point const& e) const
    {
        //from http://stackoverflow.com/questions/4948780/magic-number-in-boosthash-combine
        std::size_t seed = 0;
        std::hash<int> hash_value;

        seed ^= hash_value(get<0>(e)) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= hash_value(get<1>(e)) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= hash_value(get<2>(e)) + 0x9e3779b9 + (seed << 6) + (seed >> 2);

        return seed;
    }
};


extern "C" {

  //project potential_loc onto line w slope slopeX,slopeY,slopeZ, and starting point center_d
  //line x=a+tn, point p
  // projection is norm of (a-p)-(dot(a-p,n)n)
  double get_dis_from_tuple_to_line(Point potential_loc, double* center_d, double slopeX, double slopeY, double slopeZ) {
    double diff_a_p_x = center_d[0]-get<0>(potential_loc);
    double diff_a_p_y = center_d[1]-get<1>(potential_loc);
    double diff_a_p_z = center_d[2]-get<2>(potential_loc);
    double dot = diff_a_p_x*slopeX + diff_a_p_y*slopeY+diff_a_p_z*slopeZ;

    double before_norm_x = diff_a_p_x - dot*slopeX;
    double before_norm_y = diff_a_p_y - dot*slopeY; 
    double before_norm_z = diff_a_p_z - dot*slopeZ;
    return sqrt(before_norm_x*before_norm_x+before_norm_y*before_norm_y+before_norm_z*before_norm_z);

  }
  void calculate_boundary(THDoubleTensor* points, THDoubleTensor* center, THByteTensor* rgb, THIntTensor* result, THByteTensor* result_rgb)
  {

    long npoints = points->size[0];
    long width  = points->size[1];
    if(width!=3) { 
      cout << "WIDTH NOT 3" << endl;
    }
    double * points_d       = THDoubleTensor_data(points);
    unsigned char * rgb_d  = THByteTensor_data(rgb);


    width  = center->size[1];
    if(width!=3) { 
      cout << "CENTER WIDTH NOT 3" << endl;
    }
    double * center_d       = THDoubleTensor_data(center);

    // constructor for a custom type
    unordered_map<Point, Value, ihash, iequal_to> map;
    unordered_map<Point, Value, ihash, iequal_to>::iterator loc_iterator;
    unordered_map<Point, Value, ihash, iequal_to>::iterator loc_new_iterator;

    //map.reserve(300000);
    Point center_point(center_d[0], center_d[1], center_d[2]);
    map[center_point] = Value();
    int x_addition[6] = {0,0,0,0,1,-1}; 
    int y_addition[6] = {0,0,1,-1,0,0};
    int z_addition[6] = {1,-1,0,0,0,0};
    int prev_neighbors[6] = {1, 0, 3, 2, 5, 4}; //reverse neighbor lookup, i.e. to neighbor what are you
    //for every ray, check each thing in ray
      #pragma omp for schedule(dynamic, 100000)
     for(int i=0; i<npoints; i++) {
      //cout << "i_new " << i << endl;
      int x,y,z;
      double slopeX, slopeY, slopeZ;
      generate_ray(center_d, 0, points_d, 3*i, slopeX, slopeY, slopeZ);
      int possible_x = slopeX <0 ? floor(slopeX) : ceil(slopeX);
      int possible_y = slopeY <0 ? floor(slopeY) : ceil(slopeY);
      int possible_z = slopeZ <0 ? floor(slopeZ) : ceil(slopeZ);
      double norm_factor = sqrt(slopeX*slopeX+slopeY*slopeY+slopeZ*slopeZ);

      slopeX = slopeX/norm_factor;
      slopeY = slopeY/norm_factor;
      slopeZ = slopeZ/norm_factor;

      bool inUnknown = false;
      bool done = false;
      Point loc(center_d[0],center_d[1],center_d[2]);
      Point next_loc(0,0,0);
      double best_dis;
      while (!done) {   
        done = (get<0>(loc)==round(points_d[3*i])) && 
              (get<1>(loc)==round(points_d[3*i+1])) &&
              (get<2>(loc)==round(points_d[3*i+2]));   
        best_dis = 10000000;
        if(possible_x !=0) {
          Point potential_loc(get<0>(loc)+possible_x, get<1>(loc),get<2>(loc));
          double dis = get_dis_from_tuple_to_line(potential_loc, center_d, slopeX, slopeY, slopeZ);
          if(dis<best_dis) {
            next_loc = potential_loc;
            best_dis = dis;
            x = possible_x;
            y=0;
            z=0;
          }
        } 
        if(possible_y !=0) {
          Point potential_loc(get<0>(loc), get<1>(loc)+possible_y,get<2>(loc));
          double dis = get_dis_from_tuple_to_line(potential_loc, center_d, slopeX, slopeY, slopeZ);
          if(dis<best_dis) {
            next_loc = potential_loc;
            best_dis = dis;
            x = 0;
            y=possible_y;
            z=0;
          }
        } 

        if(possible_z !=0) {
          Point potential_loc(get<0>(loc), get<1>(loc),get<2>(loc)+possible_z);
          double dis = get_dis_from_tuple_to_line(potential_loc, center_d, slopeX, slopeY, slopeZ);
          if(dis<best_dis) {
            next_loc = potential_loc;
            best_dis = dis;
            x = 0;
            y=0;
            z=possible_z;
          }
        }      
        loc_iterator = map.find(loc);
        bool exist = !(loc_iterator == map.end());
        Value neighbors_seen_temp;
        bool already_done = false;
        if(!exist && inUnknown) { //carve out new one
          already_done = true;
          for(int neighbors=0; neighbors <6; neighbors++) { //mark it's neighbors as good
            Point loc_new(get<0>(loc)+x_addition[neighbors],get<1>(loc)+y_addition[neighbors],get<2>(loc)+z_addition[neighbors]);
            loc_new_iterator = map.find(loc_new);
            if(!(loc_new_iterator == map.end())) {
              neighbors_seen_temp.set_neighbor(neighbors,true);
              Value& val = loc_new_iterator->second;
              val.set_neighbor(prev_neighbors[neighbors], true);
              val.set_neighbor_value(prev_neighbors[neighbors],&neighbors_seen_temp);
              neighbors_seen_temp.set_neighbor_value(neighbors,&val);
              
              if(val.all_neighbors_seen()) { //remove!
                map.erase(loc_new);
                neighbors_seen_temp.set_neighbor_null(neighbors);
              } 
            }
          }
          if(done) {
            neighbors_seen_temp.set_rgb(rgb_d[3*i], rgb_d[3*i+1], rgb_d[3*i+2]);
          }
          map[loc] = neighbors_seen_temp;

        }     

        bool next_one;
        Value& neighbors_seen = map[loc];

        if(exist || (!exist && inUnknown)) {
          int next_index = z*z*(1-z)/2.0+y*y*((1-y)/2.0+2.0) + x*x*((1-x)/2.0+4.0);
          next_one = neighbors_seen.neighbors[next_index];

          if(neighbors_seen.all_neighbors_seen()) { //remove!
            for(int n_counter=0; n_counter < 6; n_counter++) {
              if(neighbors_seen.neighbor_values[n_counter] != 0) {
                neighbors_seen.neighbor_values[n_counter]->set_neighbor_null(prev_neighbors[n_counter]);
              }
            }
            map.erase(loc); 
          } 
          inUnknown = !next_one; 
        }

       //shoot ray and update neighbors
        if(!already_done) {
          for(int neighbors=0; neighbors <6; neighbors++) {
            if(neighbors_seen.neighbor_values[neighbors] != 0) {
              Value* val = neighbors_seen.neighbor_values[neighbors];
              val->set_neighbor(prev_neighbors[neighbors], true);
              val->set_neighbor_value(prev_neighbors[neighbors],&neighbors_seen);
              if(val->all_neighbors_seen()) { //remove!

                for(int n_counter=0; n_counter < 6; n_counter++) {
                  if(val->neighbor_values[n_counter] != 0) {
                    val->neighbor_values[n_counter]->set_neighbor_null(prev_neighbors[n_counter]);
                  }
                }
                Point loc_new(get<0>(loc)+x_addition[neighbors],get<1>(loc)+y_addition[neighbors],get<2>(loc)+z_addition[neighbors]);
                                if(get<0>(loc_new) == 120 && get<1>(loc_new) == 0 && get<2>(loc_new) == -11) {
                  cout << "OHHH CRAP" << endl;
                }
                map.erase(loc_new);
              } 
            } 
          } 
        } 
        loc = next_loc; 
    }
  }

  THIntTensor_resize2d(result,map.size(),3);
  int * pts_d = THIntTensor_data(result);

  THByteTensor_resize2d(result_rgb,map.size(),3);
  unsigned char * result_rgb_d = THByteTensor_data(result_rgb);

  for ( auto it = map.begin(); it != map.end(); ++it ) {
    pts_d[0] = get<0>(it->first);
    pts_d[1] = get<1>(it->first);
    pts_d[2] = get<2>(it->first);

    result_rgb_d[0] = (it->second).r;
    result_rgb_d[1] = (it->second).g;
    result_rgb_d[2] = (it->second).b;

    pts_d+=3;
    result_rgb_d+=3;
  }


} //end calculate_boundary

} //end extern c