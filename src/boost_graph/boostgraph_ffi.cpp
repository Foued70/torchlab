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

#include <boost/graph/dag_shortest_paths.hpp>
#include <boost/graph/adjacency_list.hpp>

#include <iostream>

using namespace std;

// Example from Introduction to Algorithms by Cormen, et all p.537.

// Sample output:
//  r: inifinity
//  s: 0
//  t: 2
//  u: 6
//  v: 5
//  x: 3

extern "C" {

void get_graph_shortest_path(THDoubleTensor* points, int initializer, THDoubleTensor* result)
{
  using namespace boost;
  typedef adjacency_list<vecS, vecS, directedS, 
    property<vertex_distance_t, double>, property<edge_weight_t, double> > graph_t;

  long height = points->size[0];
  long width  = points->size[1];
  double * points_d       = THDoubleTensor_data(points);

  THDoubleTensor_resize2d(result,height, width);
  THDoubleTensor_fill(result,0);
  double* result_d = THDoubleTensor_data(result);

  graph_t g(height*width);

  for (int i=0;i<height;i++){
    for(int j=0; j<width; j++) {
//      cout << i << " " << j << " " << i*width+j << " " << points_d[i*width+j] << endl;
      if(i>0) add_edge((i-1)*width+j, i*width+j, points_d[i*width+j], g);
      if(i+1<height) add_edge((i+1)*width+j, i*width+j, points_d[i*width+j], g);
      if(j>0) add_edge(i*width+j-1, i*width+j, points_d[i*width+j], g);
      if(j+1<width) add_edge(i*width+j+1, i*width+j, points_d[i*width+j], g);
    }
  }

  property_map<graph_t, vertex_distance_t>::type
  d_map = get(vertex_distance, g);

  dijkstra_shortest_paths(g, initializer, distance_map(d_map));

  graph_traits<graph_t>::vertex_iterator vi , vi_end;
  for (boost::tie(vi, vi_end) = vertices(g); vi != vi_end; ++vi) {
      result_d[*vi]= d_map[*vi];
  }
}
}