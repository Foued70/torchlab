boost_ffi = require './boostgraph_ffi'

Class()

function shortest_path(tensor, index, result)
   tensor = tensor:contiguous();
   result = result or torch.Tensor();
   boost_ffi.get_graph_shortest_path(torch.cdata(tensor), index-1, torch.cdata(result))
   return result
end
