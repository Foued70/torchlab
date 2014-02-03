local imgraph = require '../imgraph'

cmd = torch.CmdLine()
cmd:text()
cmd:text()
cmd:text('test segmentation')
cmd:text()
cmd:text('Options')
cmd:option('-imgfile','/Users/lihui815/Documents/temporary-circle-6132/work/planes/002/blah.png')
cmd:option('-cc_thrsh', 0.1)
cmd:option('-cc_color', true)
cmd:option('-mst_thrsh', 3)
cmd:option('-mst_minsz', 20)
cmd:option('-minhgt', 0.10)
cmd:option('-connex', 8)

cmd:option('-non_interactive',false)

cmd:text()

-- parse input params
params = cmd:parse(process.argv)

function segment(inputimg, cc_thrsh, cc_color, mst_thrsh, mst_minsz, minhgt, connex)

	-- (1) build a graph on an input image
	local inputimgg = image.convolve(inputimg, image.gaussian(3), 'same')
	--local graph = imgraph.graph(inputimgg)

  --[[
	
	-- (2) compute its connected components, and mst segmentation
	local cc = imgraph.connectcomponents(graph, cc_thsh, cc_color)
	local mstsegm = imgraph.segmentmst(graph, mst_thrsh, mst_minsz)
	local mstsegmcolor = imgraph.colorize(mstsegm)

	-- (3) do a histogram pooling of the original image:
	local pool = imgraph.histpooling(inputimg, mstsegm)
	
	--[[]]

	-- (4) compute the watershed of the graph
	local graph = imgraph.graph(inputimgg, connex)
	local gradient = imgraph.graph2map(graph)
	local watershed = imgraph.watershed(gradient, minhgt, connex) 
	local watershedgraph = imgraph.graph(watershed, connex) 
	local watershedcc = imgraph.connectcomponents(watershedgraph, 0.5, true)

  --[[
  
	-- (5) compute the saliency of a graph
	local graph = imgraph.graph(inputimg)
	tree = imgraph.mergetree(graph)
	local hierarchy = imgraph.graph2map(imgraph.tree2graph(tree))
	imgraph.filtertree(tree, 'volume')
	local filteredhierarchy = imgraph.graph2map(imgraph.tree2graph(tree))

	-- (6) compute the merge tree of the last graph
	local mt = imgraph.mergetree(graph)

  --[[]]
  
	-- (7) display results
	image.display{image=inputimg, legend='input image'}
	--image.display{image=cc, legend='thresholded graph'}
	image.display{image=watershed, legend='watershed on the graph'}
	image.display{image=watershedcc, legend='components of watershed'}
	--image.display{image=mstsegmcolor, legend='segmented graph, using min-spanning tree'}
	--image.display{image=pool, legend='original imaged hist-pooled by segmentation'}
	--image.display{image=hierarchy, legend='raw edge-weighted graph watershed'}
	--image.display{image=filteredhierarchy, legend='filtered edge-weighted graph watershed'}
end

local inputimg
inputimg = image.load(params.imgfile)
segment(inputimg, params.cc_thrsh, params.cc_color, params.mst_thrsh, params.mst_minsz, params.minhgt, params.connex)

if params.non_interactive then
  process.exit()
end