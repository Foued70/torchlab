--Class()

dofile 'alignment_utils.lua'

--cmd structure
cmd = torch.CmdLine()
cmd:text()
cmd:text()
cmd:text('Compute image projections')
cmd:text()
cmd:text('Options')
cmd:option('-topdir', '/Users/lihui815/cloudlab/src/data/test/jet-pizza-6780/scanner371_job362006', 'project directory')
cmd:option('-outdir', 'output', 'directory to save images')
cmd:option('-alignto', '/Users/lihui815/cloudlab/src/data/test/jet-pizza-6780/jet-pizza-6780_a_00', 'matterport texture info')
cmd:option('-imagesdir', 'sweep_3/JPG', 'directory of particular sweep')
cmd:text()

if arg == nil then
  arg = ''
end

-- parse input params
print("  parse input params")
params = cmd:parse(arg)

projectdir = params.topdir
imagesdir  = projectdir..'/'..'/'..params.imagesdir
outdir = projectdir..'/'..params.outdir
swnum = tonumber(string.split(string.split(params.imagesdir,'/')[1], "_")[2])
mptex_num = swnum-1

print("  check directory validity")
--if outdir does not exists, setup outdir
if not util.fs.is_dir(projectdir) then
  error("-topdir %s is not a valid directory", params.topdir)
elseif not util.fs.is_dir(imagesdir) then
  error("-imagesdir %s is not a valid subdirectory of %s", params.imagesdir, params.topdir)
elseif not util.fs.is_dir(outdir) then
  outdir = projectdir
end

-- load images
print("  load images")
image_fnames,lab_images,rgb_images,smp_images,fnames=load_alignment_images(imagesdir)
img1 = rgb_images[1]
width_from = img1:size(3)
height_from = img1:size(2)
collectgarbage()

--load mp texture information
print("  load matterport texture information")
mptex_info_fname = util.fs.glob(params.alignto, "texture_info.txt");
pose = model.mp.load_poses(mptex_info_fname[1])[swnum];
mptex_rad_per_px_x = pose.degrees_per_px_x * d2r
mptex_rad_per_px_y = pose.degrees_per_px_y * d2r

--load mp texture
print("  load matterport texture")
mptex_fname = util.fs.glob(params.alignto, "00"..mptex_num..".jpg")[1];
mptex_img_orig    = image.load(mptex_fname):type('torch.DoubleTensor');
mptex_width_orig  = mptex_img_orig:size(3)
mptex_height_orig = mptex_img_orig:size(2)

mptex_hfov_orig = mptex_width_orig * mptex_rad_per_px_x
mptex_vfov_orig = mptex_height_orig * mptex_rad_per_px_y

extra_angle = mptex_hfov_orig - 2 * pi
extra_pixel = math.ceil(extra_angle / mptex_rad_per_px_x)

mptex_img_360 = mptex_img_orig:sub(1,3,1,mptex_height_orig,extra_pixel+1,mptex_width_orig)
mptex_width_360 = mptex_width_orig-extra_pixel
mptex_height_360=mptex_height_orig
mptex_hfov_360 = 2 * pi
mptex_vfov_360 = mptex_vfov_orig
--load precalculated
--d_off - angle offset of centers
--extra_angle - angle over 2pi of mp texture starting from left edge
--a_off - angle offset of stitched mounted pics projected onto 2pi hfov to mp texture with left extra cut off
--extraangle=0.52360994570551
--a_off=d_off-extraangle/2

print("  calculate initial guess for best_delta")
precalculated = load_precalculated_deltas(#image_fnames, #image_fnames)
collectgarbage()

print("  find the best delta partition")
--find best_delta

fbd={}

fbd.vfov_anchor = (97.0/180)*pi
fbd.vfov_wiggle = (1.0/180)*pi
fbd.vfov_quant = 0;

fbd.hfov_anchor = (74.8/180)*pi
fbd.hfov_wiggle = (0.5/180)*pi
fbd.hfov_quant = 0;

fbd.phi_anchor = 0;
fbd.phi_wiggle = (0.25/180)*pi
fbd.phi_quant = 0;

fbd.delta_anchor = precalculated:clone();
fbd.delta_wiggle = 0.25;
fbd.delta_quant = 50;

best_delta, best_vfov, best_hfov, best_phi = find_best_deltas(lab_images, smp_images, precalculated, fbd)
collectgarbage()

collectgarbage()

hfov = best_hfov;
vfov = best_vfov;
phi = best_phi;

print("  remap images to fit dimensions of matterport texture")
-- remap to 2pi hfov

ritp_1={}

ritp_1.lambda=0
ritp_1.phi = phi
ritp_1.sc = 1;
ritp_1.width_from = width_from
ritp_1.height_from = height_from
ritp_1.hfov_from = hfov
ritp_1.vfov_from = vfov
ritp_1.width_to = mptex_width_360
ritp_1.height_to = mptex_height_360
ritp_1.hfov_to = mptex_hfov_360
ritp_1.vfov_to = mptex_vfov_360

image_mount_360 = remap_images_to_panorama(rgb_images,best_delta, ritp_1)
image_mptex_360 = mptex_img_360:clone()
collectgarbage()

-- align to mptex

print("  align to matterport texture")
numrads=2500
numangs=1500
numbest=100

houghmount=get_hough(image_mount_360, numrads, numangs, numbest)
collectgarbage()
houghmptex=get_hough(image_mptex_360, numrads, numangs, numbest)
collectgarbage()

off = find_offset(houghmptex,houghmount) * mptex_width_360 / numrads
collectgarbage()

-- final remap
print("  final remap")

ritp_2={}

ritp_2.lambda=off*mptex_rad_per_px_x+extra_angle/2;
ritp_2.phi = phi
ritp_2.sc = 5;
ritp_2.width_from = width_from
ritp_2.height_from = height_from
ritp_2.hfov_from = hfov
ritp_2.vfov_from = vfov
ritp_2.width_to = mptex_width_orig
ritp_2.height_to = mptex_height_orig
ritp_2.hfov_to = mptex_hfov_orig
ritp_2.vfov_to = mptex_vfov_orig

image_mount_orig = remap_images_to_panorama(rgb_images,best_delta,ritp_2)
collectgarbage()

lambda = ritp_2.lambda
sc = ritp_2.sc

print(best_delta)
print(lambda)

image.save(outdir.."/"..swnum.."_big_img_single_"..swnum..".jpg", image_mount_orig)

img_mptex_big=image.scale(mptex_img_orig, mptex_width_orig*sc, mptex_height_orig*sc)
img_layer = image_mount_orig/2 + img_mptex_big/2
image.save(outdir.."/"..swnum.."_big_img_layer_"..swnum..".jpg", img_layer)
