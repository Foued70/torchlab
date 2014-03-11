/*
  Author  : Joshua Karges
  Date    : 06 March 2014
  Function: c++ helper function for floodFill
*/

extern "C"
{
#include <math.h>
#include <limits.h>
#define MIN(x,y) ( (x) < (y) ? (x) : (y) )
#define MAX(x,y) ( (x) > (y) ? (x) : (y) )
}

#include <iostream>
#include <queue>
#include <vector>
#include <array>
#include <cmath>
using namespace std;

template <typename T>
class A2D {
    T *m_buf;
    size_t m_h;
    size_t m_w;
public:
    A2D(T *buf, const size_t &h, const size_t &w)
        : m_buf(buf), m_h(h), m_w(w) { }
    ~A2D() { }

    T& operator()(const size_t &i, const size_t &j)
    {
    	if(i < this->m_h){
        	return this->m_buf[ i * this->m_w + j];
    	}
    }

    size_t getHeight(){	return this->m_h; }

    size_t getWidth(){ return this->m_w; }
};

template <typename T>
class CLIST
{
	vector< array<T,2> > content;
public:
	CLIST() {};
	~CLIST() {};
	
	int addEntry(T i, T j)
	{
		array<T,2> entry = {i,j};
		this->content.push_back(entry);
	}

	int addEntry(array<T,2> arr)
	{
		array<T,2> entry = {arr[0],arr[1]};
		this->content.push_back(entry);
	}

	array<T,2> operator[](const size_t &i)
	{
		if(i < this->content.size())
		{
			return this->content[i];
		}
	}

	void savePointer(T* ptr)
	{
		for(int i=0;i < this->content.size();i++){
			ptr[i*2] = this->content[i][0];
			ptr[i*2+1] = this->content[i][1];
		}
	}

	unsigned int getSize(){ return this->content.size(); }
};

extern "C"
{
	

	int walker_pixel_ranger(int* edges, unsigned char* mask, int height, int width)
	{
		//mask has already been padded with a 0 border
		int square_sum, old_uv_adj, new_uv_adj;
		array<int,2> uv = {-1,-1}; //top-left pixel of square window
		array<int,2> prev_dir = {1,0};
		array< array<int,2>,2 > new_half; //image coordinates of the pixels in the square that we haven't seen yet
		array< array<int,2>,2 > old_half; //image coordinates of the half that we have seen
		vector<int> edge_v;
		vector<int> edge_u;
		A2D<unsigned char> mask2(mask,height, width);

		CLIST<int> edge_list;

		//find the first 1 pixel
		for(int j=0;j<height;j++){
			for(int k=0;k<width;k++){
				if(mask2(j,k)==1 && uv[0] < 0){
					uv[0] = j;
					uv[1] = k-1;
					break;
				}
			}
			if(uv[0] >= 0){ break;}
		}
		edge_list.addEntry(uv);

		int cnt = 0,cnt2 = 0,cnt3 = 0;
		do {
			new_half[0][0] = uv[0] + MAX(prev_dir[0],0);
			new_half[0][1] = uv[1] + MAX(prev_dir[1],0);
			new_half[1][0] = uv[0] + MIN(prev_dir[0]+1,1);
			new_half[1][1] = uv[1] + MIN(prev_dir[1]+1,1);
			old_half[0][0] = uv[0] + MAX(-prev_dir[0],0);
			old_half[0][1] = uv[1] + MAX(-prev_dir[1],0);
			old_half[1][0] = uv[0] + MIN(-prev_dir[0]+1,1);
			old_half[1][1] = uv[1] + MIN(-prev_dir[1]+1,1);
			square_sum = mask2(new_half[0][0],new_half[0][1]) +
						 mask2(new_half[1][0],new_half[1][1]) +
						 mask2(old_half[0][0],old_half[0][1]) +
						 mask2(old_half[1][0],old_half[1][1]);

			switch(square_sum)
			{
				case 1:
					//add the 0 pixel next to the 1 pixel from the half you haven't seen yet because they are 0s
					old_uv_adj = mask2(old_half[0][0],old_half[0][1]);
					edge_list.addEntry( new_half[ 1-old_uv_adj ]);
					//change direction toward the pixel with the 1 on your known half (we assume there is one)
					prev_dir[0] = (-abs(prev_dir[0])+1)*(1-2*old_uv_adj);
					prev_dir[1] = (-abs(prev_dir[1])+1)*(1-2*old_uv_adj);
					break;
				case 2:
					cnt2++;
					//Two scenarios: the true pixels are adjacent and the false pixels are adjacent, OR it makes a "checkered" pattern
					if(mask2(new_half[0][0],new_half[0][1]) == mask2(old_half[0][0],old_half[0][1])){
						//add the 0 pixel from the half you haven't seen yet (we assume there is one)
						new_uv_adj = mask2(new_half[0][0],new_half[0][1]);
						edge_list.addEntry( new_half[ new_uv_adj ] );
						cnt3++;
						//keep the same direction
					}else{//"checkered" pattern.  
						//Treat it like a square_sum=3 scenario
						//don't add anything
						//change direction towards the pixel with the 0 on your known half
						old_uv_adj = mask2(old_half[0][0],old_half[0][1]);
						prev_dir[0] = (-abs(prev_dir[0])+1)*(2*old_uv_adj-1);
						prev_dir[1] = (-abs(prev_dir[1])+1)*(2*old_uv_adj-1);
					}
					break;
				case 3:
					//don't add anything
					//change direction towards the pixel with the 0 on your known half
					old_uv_adj = mask2(old_half[0][0],old_half[0][1]);
					prev_dir[0] = (-abs(prev_dir[0])+1)*(2*old_uv_adj-1);
					prev_dir[1] = (-abs(prev_dir[1])+1)*(2*old_uv_adj-1);
					break;
				default:
					break;
			}
			uv[0] += prev_dir[0];
			uv[1] += prev_dir[1];
			cnt++;
		}while((edge_list[0][0] != uv[0]) || (edge_list[0][1] != uv[1]));
		edge_list.savePointer(edges);

		return edge_list.getSize();

	}

	int walker_pixel_ranger2(int* edges, unsigned char* mask, int height, int width)
	{
		//this only uses primitive types, in totally un-official wishy washy time comparisons, the other walker_pixel_ranger is just a little faster in most cases by fractions of a millisecond
		//mask has already been padded with a 0 border
		int square_sum, old_uv_adj, new_uv_adj;
		int e_ind = 0;
		int uv[2] = {-1,-1}; //top-left pixel of square window
		int prev_dir[2] = {1,0};
		int (*new_half)[2]; //image coordinates of the pixels in the square that we haven't seen yet
		int (*old_half)[2]; //image coordinates of the half that we have seen

		new_half = new int[2][2];
		old_half = new int[2][2];

		//find the first 1 pixel
		for(int j=0;j<height;j++){
			for(int k=0;k<width;k++){
				if((int)mask[j*width + k]==1){
					uv[0] = j;
					uv[1] = k-1;
					break;
				}
			}
			if(uv[0] >= 0){ break;}
		}
		//edge_list.addEntry(uv);
		edges[2*e_ind] = uv[0];
		edges[2*e_ind+1] = uv[1];
		e_ind++;

		do {
			new_half[0][0] = uv[0] + MAX(prev_dir[0],0);
			new_half[0][1] = uv[1] + MAX(prev_dir[1],0);
			new_half[1][0] = uv[0] + MIN(prev_dir[0]+1,1);
			new_half[1][1] = uv[1] + MIN(prev_dir[1]+1,1);
			old_half[0][0] = uv[0] + MAX(-prev_dir[0],0);
			old_half[0][1] = uv[1] + MAX(-prev_dir[1],0);
			old_half[1][0] = uv[0] + MIN(-prev_dir[0]+1,1);
			old_half[1][1] = uv[1] + MIN(-prev_dir[1]+1,1);
			square_sum = (int)mask[new_half[0][0]*width + new_half[0][1]] +
						 (int)mask[new_half[1][0]*width + new_half[1][1]] +
						 (int)mask[old_half[0][0]*width + old_half[0][1]] +
						 (int)mask[old_half[1][0]*width + old_half[1][1]];

			switch(square_sum)
			{
				case 1:
					//add the 0 pixel next to the 1 pixel from the half you haven't seen yet because they are 0s
					old_uv_adj = (int)mask[ old_half[0][0]*width + old_half[0][1] ];
					//edge_list.addEntry( new_half[ 1-old_uv_adj ]);
					edges[2*e_ind] = new_half[ 1-old_uv_adj ][0];
					edges[2*e_ind+1] = new_half[ 1-old_uv_adj ][1];
					e_ind++;
					//change direction toward the pixel with the 1 on your known half (we assume there is one)
					prev_dir[0] = (-abs(prev_dir[0])+1)*(1-2*old_uv_adj);
					prev_dir[1] = (-abs(prev_dir[1])+1)*(1-2*old_uv_adj);
					break;
				case 2:
					//Two scenarios: the true pixels are adjacent and the false pixels are adjacent, OR it makes a "checkered" pattern
					if(mask[ new_half[0][0]*width + new_half[0][1] ] == mask[ old_half[0][0]*width + old_half[0][1]]){
						//add the 0 pixel from the half you haven't seen yet (we assume there is one)
						new_uv_adj = (int)mask[ new_half[0][0]*width + new_half[0][1] ];
						//edge_list.addEntry( new_half[ new_uv_adj ] );
						edges[2*e_ind] = new_half[ new_uv_adj ][0];
						edges[2*e_ind+1] = new_half[ new_uv_adj ][1];
						e_ind++;
						//keep the same direction
					}else{//"checkered" pattern.  
						//Treat it like a square_sum=3 scenario
						//don't add anything
						//change direction towards the pixel with the 0 on your known half
						old_uv_adj = (int)mask[ old_half[0][0]*width + old_half[0][1] ];
						prev_dir[0] = (-abs(prev_dir[0])+1)*(2*old_uv_adj-1);
						prev_dir[1] = (-abs(prev_dir[1])+1)*(2*old_uv_adj-1);
					}
					break;
				case 3:
					//don't add anything
					//change direction towards the pixel with the 0 on your known half
					old_uv_adj = (int)mask[ old_half[0][0]*width + old_half[0][1] ];
					prev_dir[0] = (-abs(prev_dir[0])+1)*(2*old_uv_adj-1);
					prev_dir[1] = (-abs(prev_dir[1])+1)*(2*old_uv_adj-1);
					break;
				default:
					break;
			}
			uv[0] += prev_dir[0];
			uv[1] += prev_dir[1];
		}while((edges[0] != uv[0]) || (edges[1] != uv[1]));
		//edge_list.savePointer(edges);

		return e_ind;

	}

	int flood_fill( char *flooded, char *mask, int seed_v, int seed_u, int height, int width )
	{
		//returns a mask of the connected component in the mask starting at the seed position (checking it's neighbors)
		int cur_node_v, cur_node_u, nbr_v, nbr_u;
		bool in_bounds;
		queue<int> open_v;
		queue<int> open_u;
		char *open_mask;

		open_mask = new char[height * width];
		for(int j=0;j<height;j++){
			for(int k=0;k<width;k++){
				open_mask[j*width + k] = 0;
			}
		}

		//seed is the starting position
		open_v.push(seed_v);
		open_u.push(seed_u);
		open_mask[seed_v*width + seed_u] = 1;

		while(!open_v.empty())
		{
			//remove the first element from the open list and set it to cur_node
			cur_node_v = open_v.front();
			cur_node_u = open_u.front();
			open_v.pop();
			open_u.pop();

			if(mask[cur_node_v*width + cur_node_u]==1){
				flooded[cur_node_v*width + cur_node_u] = 1;
			}
			//put the 1s neighbors into the open list.  Check that they're not already apart of the flooded or open_mask.
			for(int j=-1;j<=1;j++){
				for(int k=-1;k<=1;k++){
					nbr_v = cur_node_v+j;
					nbr_u = cur_node_u+k;
					in_bounds = (0 <= nbr_v && nbr_v < height && 0 <= nbr_u && nbr_u < width);
					if(in_bounds && mask[nbr_v*width + nbr_u]==1 && flooded[nbr_v*width + nbr_u]==0  && open_mask[nbr_v*width+ nbr_u]==0){
						open_v.push(nbr_v);
						open_u.push(nbr_u);
						open_mask[nbr_v*width + nbr_u] = 1;
					}
				}
			}
		}
		delete[] open_mask;
		return 0;
	}

	int find_first_true(int* seed, char* A,int height,int width)
	{
		int j,k;
		for( j=0;j<height;j++ ){
			for( k=0;k<width;k++ ){
				if( A[ j*width+k ] > 0 ){
					seed[0] = j;
					seed[1] = k;
					return j*width+k;
				}
			}
		}
	}
}