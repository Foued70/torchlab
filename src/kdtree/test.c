#include <flann/flann.h>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

float * read_points(char* filename, int *rows, int *cols)
{
	float * points;
	
	FILE *xyz_file = fopen(filename, "r");
	
	if(xyz_file == NULL)
	{
		printf("error opening file\n");
		exit(1);
	}
	
	int h, w, r, g, b, size;
	float x, y, z;
	size = 0;
	while(fscanf(xyz_file, "%d %d %f %f %f %d %d %d", &h, &w, &x, &y, &z, &r, &g, &b) > 0) {
		size++;
	}
	printf("read: number of lines: %d\n", size);
	fclose(xyz_file);

	*rows = size;
	*cols = 3;
	points = (float*) malloc((*rows) * (*cols) * sizeof(float));
		
	xyz_file = fopen(filename, "r");
	int index = 0;
	while(fscanf(xyz_file, "%d %d %f %f %f %d %d %d", &h, &w, &x, &y, &z, &r, &g, &b) > 0) {
		*(points+index) = x;
		*(points+index+1) = y;
		*(points+index+2) = z;
		index=index+3;
	}
	fclose(xyz_file);
	
	return points;
		
}

int main(int argc, char**argv)
{
	int rows, cols;
	int trows, tcols;
	float speedup;
	
	char * dset = "/Users/lihui815/Projects/PointCloud/Scans/2013_06_18_scans/dset.xyz";
	char * tset = "/Users/lihui815/Projects/PointCloud/Scans/2013_06_18_scans/tset.xyz";
	float * dataset = read_points(dset, &rows, &cols);
	float * testset = read_points(tset, &trows, &tcols);
	
	assert(cols == tcols);
	
	int nn = 15;
	int * result = (int*) malloc(trows*nn*sizeof(int));
	float * dists = (float*) malloc(trows*nn*sizeof(float));
	
	struct FLANNParameters p = DEFAULT_FLANN_PARAMETERS;
	p.algorithm = FLANN_INDEX_KDTREE;
	p.trees = 10;
	
	testset = testset + 3;
	
	flann_index_t index_id = flann_build_index(dataset, rows, cols, &speedup, &p);
	//flann_find_nearest_neighbors_index(index_id, testset, trows, result, dists, nn, &p);
	flann_radius_search(index_id, testset, result, dists, nn, 0.025, &p);
	//flann_find_nearest_neighbors(dataset, rows, cols, testset, trows, result, dists, nn, &p);
	
	printf("result: \n");
	int i, j;
	//for (i = 0; i < trows; i ++)
	{
		printf("POINT: {%8f, %8f, %8f} \n", *(testset),*(testset+1), *(testset+2)); 
		for (j = 0; j < nn; j++)
		{
			printf("row: %2d, dist: %8f : ", *result, *dists);
			printf("(%8f, %8f, %8f) \n", *(dataset+3*(*result)), *(dataset+3*(*result)+1), *(dataset+3*(*result)+2));
			result ++;
			dists ++;
			
		}
		testset=testset+3;
		printf("\n");
	}
}