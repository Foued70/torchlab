extractor = opencv.Extractor
keypoints = opencv.KeyPoint
mat = opencv.Mat
log = require '../util/log'

function get_sift_descriptor(use_img,use_msk)

  local h = use_img:size(2)
	local w = use_img:size(3)
	
	local crdw = torch.range(1,w):repeatTensor(h,1)[use_msk]
	local crdh = torch.range(1,h):repeatTensor(w,1):t()[use_msk]
	
	local minh = crdh:min()
	local maxh = crdh:max()
	
	use_msk:sub(minh, minh+4):fill(0)
	use_msk:sub(maxh-5, maxh):fill(0)
	for h = minh,maxh do
		local crdwh = crdw[crdh:eq(h)]:clone()
		local minw  = crdwh:min()
		local maxw  = crdwh:max()
		use_msk:sub(h, h, minw, minw+4):fill(0)
		use_msk:sub(h, h, maxw-5, maxw):fill(0)
		crdwh = nil
		collectgarbage()
	end
	
	local npts = use_msk:double():sum()
	
	crdh = nil
	crdw = nil
	collectgarbage()

	local k = torch.zeros(2,npts)
	k[1]    = torch.range(1,w):repeatTensor(h,1)[use_msk]:reshape(npts):clone():contiguous()
	k[2]    = torch.range(1,h):repeatTensor(w,1):t()[use_msk]:reshape(npts):clone():contiguous()
	k       = k:t()
	local kpts = keypoints.new(k:clone())
	local ext  = extractor.new('SIFT')

	k    = nil
	collectgarbage()

	local wnd = image.Wand.new(use_img)
	use_img   = wnd:toTensor('byte','G', 'HWD')
	m         = mat.new(use_img)
	local des = ext:compute(m, kpts.keypoints, kpts.npts)
	des       = des:toTensor():t():clone()
	local fts = torch.zeros(128,h,w):float()
	for c = 1,128 do
		fts[c][use_msk] = des[c]:clone()
	end
	
	wnd     = nil
	use_img = nil
	use_msk = nil
	des     = nil
	collectgarbage()
	
	return fts
	
end

div    = 5
while div > 3 do

	scale  = 1/math.pow(2,div)
	hfov_i = 1.3055
	vfov_i = 1.5290
	phi    = -0.05
	psi    = -0.0125
	imagefiles = util.fs.files_only('/Users/lihui815/Documents/precise-transit-6548/work/a_00/Images/001', '.tiff')
	imsw = stitcher.ImageSweep.new(imagefiles,hfov_i,vfov_i,scale,phi,psi)

	fts_tab = {}

	local max_scores = torch.zeros(128)

	for num = 1,imsw.num_images do

		print('finding features '..num)
		local dir = '00'..num
		util.fs.mkdir_p(dir)

		local use_img, outs, outl, use_msk = imsw:get_frame_output(num)
		use_msk    = use_msk[1]:eq(1)
		outs = nil
  	outl = nil
		
		local fts = get_sift_descriptor(use_img,use_msk)

		fts_tab[num] = fts:clone()
	
		local comp = torch.zeros(2,128)
		comp[1] = max_scores:clone()
		comp[2] = fts:max(2):max(3):squeeze()
		max_scores = comp:max(1):squeeze()
	
		comp    = nil
		use_img = nil
    use_msk = nil
		fts     = nil
		collectgarbage()

	end

	imsw = nil
	collectgarbage()

	for num = 1,#fts_tab do

		print('saving '..num)
		local dir = '00'..num
	
		local fts = fts_tab[num]:clone()
	
		torch.save(dir..'/fts_'..div..'.dat')
	
		--[[]]
		local h   = fts:size(2)
		local w   = fts:size(3)
	
		local i = 0
		while i < 3 do --128 do
			local img = torch.zeros(3,h,w)
		
			if i+3 <= 128 then
				img:copy(fts:sub(i+1,i+3):clone())
			else
				img:sub(1,128-i):copy(fts:sub(i+1,128):clone())
			end
		
			for c = 1,3 do
				img[c] = img[c]:div(max_scores[c])
			end
		
			image.save(dir..'/image_'..div..'.png',img)
			i = i+3
			img = nil
			collectgarbage()
		
		end
		--[[]]

		fts = nil
		collectgarbage()
	
	end

	div = div - 1
	max_scores     = nil
	fts_tab        = nil
	scores         = nil
	sorted_indices = nil
	collectgarbage()

end

collectgarbage()