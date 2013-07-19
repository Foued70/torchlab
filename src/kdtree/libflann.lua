-- FFI bindings to Flann:
local io = require 'io'
local ffi = require "ffi"
local ctorch = util.ctorch
ffi.cdef
[[
/* FROM flann/defines.h */

/* Nearest neighbour index algorithms */
enum flann_algorithm_t
{
    FLANN_INDEX_LINEAR 			= 0,
    FLANN_INDEX_KDTREE 			= 1,
    FLANN_INDEX_KMEANS 			= 2,
    FLANN_INDEX_COMPOSITE 		= 3,
    FLANN_INDEX_KDTREE_SINGLE 	= 4,
    FLANN_INDEX_HIERARCHICAL 	= 5,
    FLANN_INDEX_LSH 			= 6,
    FLANN_INDEX_KDTREE_CUDA 	= 7,
    FLANN_INDEX_SAVED 			= 254,
    FLANN_INDEX_AUTOTUNED 		= 255,
};

enum flann_centers_init_t
{
    FLANN_CENTERS_RANDOM = 0,
    FLANN_CENTERS_GONZALES = 1,
    FLANN_CENTERS_KMEANSPP = 2,
};

enum flann_log_level_t
{
    FLANN_LOG_NONE = 0,
    FLANN_LOG_FATAL = 1,
    FLANN_LOG_ERROR = 2,
    FLANN_LOG_WARN = 3,
    FLANN_LOG_INFO = 4,
    FLANN_LOG_DEBUG = 5
};

enum flann_distance_t
{
    FLANN_DIST_EUCLIDEAN 			= 1,
    FLANN_DIST_L2 					= 1,
    FLANN_DIST_MANHATTAN 			= 2,
    FLANN_DIST_L1 					= 2,
    FLANN_DIST_MINKOWSKI 			= 3,
    FLANN_DIST_MAX   				= 4,
    FLANN_DIST_HIST_INTERSECT  		= 5,
    FLANN_DIST_HELLINGER 			= 6,
    FLANN_DIST_CHI_SQUARE		 	= 7,
    FLANN_DIST_KULLBACK_LEIBLER  	= 8,
    FLANN_DIST_HAMMING         		= 9,
    FLANN_DIST_HAMMING_LUT			= 10,
    FLANN_DIST_HAMMING_POPCNT   	= 11,
    FLANN_DIST_L2_SIMPLE	   		= 12,
};

enum flann_datatype_t
{
    FLANN_NONE 		= -1,
    FLANN_INT8 		= 0,
    FLANN_INT16 	= 1,
    FLANN_INT32 	= 2,
    FLANN_INT64 	= 3,
    FLANN_UINT8 	= 4,
    FLANN_UINT16 	= 5,
    FLANN_UINT32 	= 6,
    FLANN_UINT64 	= 7,
    FLANN_FLOAT32 	= 8,
    FLANN_FLOAT64 	= 9
};

enum flann_checks_t {
    FLANN_CHECKS_UNLIMITED = -1,
    FLANN_CHECKS_AUTOTUNED = -2,
};

/* FROM flann/flann.h */

struct FLANNParameters
{
    enum flann_algorithm_t algorithm; /* the algorithm to use */

    /* search time parameters */
    int checks;                /* how many leafs (features) to check in one search */
    float eps;     /* eps parameter for eps-knn search */
    int sorted;     /* indicates if results returned by radius search should be sorted or not */
    int max_neighbors;  /* limits the maximum number of neighbors should be returned by radius search */
    int cores;      /* number of paralel cores to use for searching */

    /*  kdtree index parameters */
    int trees;                 /* number of randomized trees to use (for kdtree) */
    int leaf_max_size;

    /* kmeans index parameters */
    int branching;             /* branching factor (for kmeans tree) */
    int iterations;            /* max iterations to perform in one kmeans cluetering (kmeans tree) */
    enum flann_centers_init_t centers_init;  /* algorithm used for picking the initial cluster centers for kmeans tree */
    float cb_index;            /* cluster boundary index. Used when searching the kmeans tree */

    /* autotuned index parameters */
    float target_precision;    /* precision desired (used for autotuning, -1 otherwise) */
    float build_weight;        /* build tree time weighting factor */
    float memory_weight;       /* index memory weigthing factor */
    float sample_fraction;     /* what fraction of the dataset to use for autotuning */

    /* LSH parameters */
    unsigned int table_number_; /** The number of hash tables to use */
    unsigned int key_size_;     /** The length of the key in the hash tables */
    unsigned int multi_probe_level_; /** Number of levels to use in multi-probe LSH, 0 for standard LSH */

    /* other parameters */
    enum flann_log_level_t log_level;    /* determines the verbosity of each flann function */
    long random_seed;            /* random seed to use */
};


typedef void* FLANN_INDEX; /* deprecated */
typedef void* flann_index_t;

extern struct FLANNParameters DEFAULT_FLANN_PARAMETERS;

/**
   Sets the log level used for all flann functions (unless
   specified in FLANNParameters for each call

   Params:
    level = verbosity level
 */
void flann_log_verbosity(int level);


/**
 * Sets the distance type to use throughout FLANN.
 * If distance type specified is MINKOWSKI, the second argument
 * specifies which order the minkowski distance should have.
 */
void flann_set_distance_type(enum flann_distance_t distance_type, int order);


/**
   Builds and returns an index. It uses autotuning if the target_precision field of index_params
   is between 0 and 1, or the parameters specified if it's -1.

   Params:
    dataset = pointer to a data set stored in row major order
    rows = number of rows (features) in the dataset
    cols = number of columns in the dataset (feature dimensionality)
    speedup = speedup over linear search, estimated if using autotuning, output parameter
    index_params = index related parameters
    flann_params = generic flann parameters

   Returns: the newly created index or a number <0 for error
 */
flann_index_t flann_build_index(float* dataset,
                                             int rows,
                                             int cols,
                                             float* speedup,
                                             struct FLANNParameters* flann_params);


/**
 * Saves the index to a file. Only the index is saved into the file, the dataset corresponding to the index is not saved.
 *
 * @param index_id The index that should be saved
 * @param filename The filename the index should be saved to
 * @return Returns 0 on success, negative value on error.
 */
int flann_save_index(flann_index_t index_id,
                                  char* filename);


/**
 * Loads an index from a file.
 *
 * @param filename File to load the index from.
 * @param dataset The dataset corresponding to the index.
 * @param rows Dataset tors
 * @param cols Dataset columns
 * @return
 */
flann_index_t flann_load_index(char* filename,
                                            float* dataset,
                                            int rows,
                                            int cols);


/**
   Builds an index and uses it to find nearest neighbors.

   Params:
    dataset = pointer to a data set stored in row major order
    rows = number of rows (features) in the dataset
    cols = number of columns in the dataset (feature dimensionality)
    testset = pointer to a query set stored in row major order
    trows = number of rows (features) in the query dataset (same dimensionality as features in the dataset)
    indices = pointer to matrix for the indices of the nearest neighbors of the testset features in the dataset
            (must have trows number of rows and nn number of columns)
    nn = how many nearest neighbors to return
    flann_params = generic flann parameters

   Returns: zero or -1 for error
 */
int flann_find_nearest_neighbors(float* dataset,
                                              int rows,
                                              int cols,
                                              float* testset,
                                              int trows,
                                              int* indices,
                                              float* dists,
                                              int nn,
                                              struct FLANNParameters* flann_params);


/**
   Searches for nearest neighbors using the index provided

   Params:
    index_id = the index (constructed previously using flann_build_index).
    testset = pointer to a query set stored in row major order
    trows = number of rows (features) in the query dataset (same dimensionality as features in the dataset)
    indices = pointer to matrix for the indices of the nearest neighbors of the testset features in the dataset
            (must have trows number of rows and nn number of columns)
    dists = pointer to matrix for the distances of the nearest neighbors of the testset features in the dataset
            (must have trows number of rows and 1 column)
    nn = how many nearest neighbors to return
    flann_params = generic flann parameters

   Returns: zero or a number <0 for error
 */
int flann_find_nearest_neighbors_index(flann_index_t index_id,
                                                    float* testset,
                                                    int trows,
                                                    int* indices,
                                                    float* dists,
                                                    int nn,
                                                    struct FLANNParameters* flann_params);


/**
 * Performs an radius search using an already constructed index.
 *
 * In case of radius search, instead of always returning a predetermined
 * number of nearest neighbours (for example the 10 nearest neighbours), the
 * search will return all the neighbours found within a search radius
 * of the query point.
 *
 * The check parameter in the FLANNParameters below sets the level of approximation
 * for the search by only visiting "checks" number of features in the index
 * (the same way as for the KNN search). A lower value for checks will give
 * a higher search speedup at the cost of potentially not returning all the
 * neighbours in the specified radius.
 */
int flann_radius_search(flann_index_t index_ptr, /* the index */
                                     float* query, /* query point */
                                     int* indices, /* array for storing the indices found (will be modified) */
                                     float* dists, /* similar, but for storing distances */
                                     int max_nn,  /* size of arrays indices and dists */
                                     float radius, /* search radius (squared radius for euclidian metric) */
                                     struct FLANNParameters* flann_params);


/**
   Deletes an index and releases the memory used by it.

   Params:
    index_id = the index (constructed previously using flann_build_index).
    flann_params = generic flann parameters

   Returns: zero or a number <0 for error
 */
int flann_free_index(flann_index_t index_id,
                                  struct FLANNParameters* flann_params);

/**
   Clusters the features in the dataset using a hierarchical kmeans clustering approach.
   This is significantly faster than using a flat kmeans clustering for a large number
   of clusters.

   Params:
    dataset = pointer to a data set stored in row major order
    rows = number of rows (features) in the dataset
    cols = number of columns in the dataset (feature dimensionality)
    clusters = number of cluster to compute
    result = memory buffer where the output cluster centers are storred
    index_params = used to specify the kmeans tree parameters (branching factor, max number of iterations to use)
    flann_params = generic flann parameters

   Returns: number of clusters computed or a number <0 for error. This number can be different than the number of clusters requested, due to the
    way hierarchical clusters are computed. The number of clusters returned will be the highest number of the form
    (branch_size-1)*K+1 smaller than the number of clusters requested.
 */

int flann_compute_cluster_centers(float* dataset,
                                               int rows,
                                               int cols,
                                               int clusters,
                                               float* result,
                                               struct FLANNParameters* flann_params);

void * malloc(int size);

void free(void * ptr);

]]

-- Load lib:
local clib = util.ffi.load('libFlann')

local flann = {}
flann.clib = clib

function flann.read_xyz_file(fname)
	if util.fs.is_file(fname) and util.fs.extname(fname)=='.xyz' then
	
		local file = io.open(fname, 'r');
    	local count = 0;
    	while true do
      		local line = file:read();
      		if line == nil or line:len() < 5 then 
        		break 
      		end
      		count = count + 1
      	end
      	io.close(file)
      	local rows = count
      	local cols = 3
      	
	    local dataset = torch.Tensor(rows,cols);
    	count = 0;
    	
	    file = io.open(fname, 'r');
    	
    	while true do
      		local line = file:read();
		    if line == nil or line:len() < 5 then
        		break
		    end
      		count=count+1;
      		local begp = 1;
	      	local endp = line:find(' ', begp) - 1;
		    begp = endp + 2;
      		endp = line:find(' ', begp) - 1;
      		begp = endp + 2;
      		endp = line:find(' ', begp) - 1;

      		local x = tonumber(line:sub(begp,endp));
      		begp = endp + 2;
      		endp = line:find(' ', begp) - 1;
      		local y = tonumber(line:sub(begp,endp));
      		begp = endp + 2;
      		endp = line:find(' ', begp) - 1;
      		local z = tonumber(line:sub(begp,endp));
     
      		dataset[count] = torch.Tensor({x,y,z})
    	end
    	io.close(file)
    	
    	collectgarbage()
		return dataset
	end
end

local flann_parameter_list={'algorithm','checks','cb_index','trees','branching',
							'iterations','centers_init','target_precision','build_weight',
							'memory_weight','sample_fraction','table_number_','key_size_',
							'multi_probe_level_','log_level','random_seed'}

function flann.new_flann_params_pointer(flann_params_table)
	flann_params=clib.DEFAULT_FLANN_PARAMETERS
	for i = 1,#flann_parameter_list do
		local param = flann_parameter_list[i]
		if flann_params_table[param] then
			flann_params[param] = flann_params_table[param]
		end
	end
	collectgarbage()
	return ffi.cast('struct FLANNParameters *', flann_params)
end

function flann.build_index(dataset, flann_params)

	dataset = dataset:type('torch.FloatTensor')
	local speedup = ffi.gc(clib.malloc(ffi.sizeof('float')), clib.free)
	local dset = ffi.cast('float*',torch.data(dataset));
	local rows = dataset:size(1);
	local cols = dataset:size(2);
	
	collectgarbage()
	
	return clib.flann_build_index(dset, rows, cols, ffi.cast('float *', speedup), flann_params), 
				speedup
end

function flann.find_nearest_neighbors_index(index, testset, nn, flann_params)

	testset = testset:type('torch.FloatTensor')
	local tset = ffi.cast('float*', torch.data(testset));
	local trows = testset:size(1);
	
	local indices = torch.IntTensor(trows * nn);
	local dists = torch.FloatTensor(trows *nn);
	
	local indices_ptr = ffi.cast('int *', torch.data(indices))
	local dists_ptr = ffi.cast('float *', torch.data(dists))
	
	local ret = clib.flann_find_nearest_neighbors_index(index, tset, trows, indices_ptr, dists_ptr,
					nn, flann_params)
										
	assert(ret == 0, "ffnni: flann_find_nearest_neighbors_index error, returned "..ret)
	
	collectgarbage()
	
	return indices, dists
end

function flann.find_nearest_neighbors(dataset, testset, nn, flann_params)
	
	dataset = dataset:type('torch.FloatTensor')
	testset = testset:type('torch.FloatTensor')
	local dset = ffi.cast('float*',torch.data(dataset));
	local rows = dataset:size(1);
	local cols = dataset:size(2);
	local tset = ffi.cast('float*', torch.data(testset));
	local trows = testset:size(1);
	
	local indices = torch.IntTensor(trows * nn);
	local dists = torch.FloatTensor(trows *nn);
	
	local indices_ptr = ffi.cast('int *', torch.data(indices))
	local dists_ptr = ffi.cast('float *', torch.data(dists))
	
	local ret = clib.flann_find_nearest_neighbors(dset, rows, cols, tset, trows, 
					indices_ptr, dists_ptr, nn, flann_params)
	
	assert(ret == 0, "ffnn: flann_find_nearest_neighbors error, returned "..ret)
	
	collectgarbage()
	
	return indices, dists
	
end

function flann.radius_search(index, query, max_nn, radius, flann_params)
	
	query = query:type('torch.FloatTensor')
	local qset = ffi.cast('float *',torch.data(query));
	
	local indices = torch.IntTensor(max_nn);
	local dists = torch.FloatTensor(max_nn);
	
	local indices_ptr = ffi.cast('int *', torch.data(indices))
	local dists_ptr = ffi.cast('float *', torch.data(dists))
	
	local ret = clib.flann_radius_search(index, qset, indices_ptr, dist_ptr, max_nn, radius, flann_params)
	
	assert(ret == 0, "frs: flann_radius_search error, returned "..ret)
	
	collectgarbage()
	
	return indices, dists
	
end

flann.distance_types = {
	euclidean       =clib.FLANN_DIST_EUCLIDEAN,
	manhattan       =clib.FLANN_DIST_MANHATTAN,
	minkowski       =clib.FLANN_DIST_MINKOWSKI,
	hist_intersect  =clib.FLANN_DIST_HIST_INTERSECT,
	hellinger       =clib.FLANN_DIST_HELLINGER,
	chi_square      =clib.FLANN_DIST_CHI_SQUARE,
	kullback_leibler=clib.FLANN_DIST_KULLBACK_LEIBLER
}

function flann.set_distance_type(distance_type, order)
	return clib.flann_set_distance_type(distance_types[distance_type],order)
end

function flann.save_index(index, filename)
	return clib.flann_save_index(index, ffi.cast('char *', filename))
end

function flann.load_index(filename, rows, cols)
	local dataset = torch.FloatTensor(rows,cols)
	local dset = ffi.cast('float*',torch.data(dataset))
	local index = clib.flann_load_index(ffi.cast('char *', filename), dset, rows, cols)
	
	collectgarbage()
	
	return index, dataset
end

function flann.free_index(index, flann_params)
	return clib.flann_free_index(index, flann_params);
end

return flann;