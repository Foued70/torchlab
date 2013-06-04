loader= require 'util.loader'
LensSensor=data.LensSensor
projection=projection.util
p3p=require 'p3p'

creffname="cornerPoints_ref.txt";
cfname="cornerPoints.txt";
texfname="scanner371_job315000_texture_info.txt";
objfname="scanner371_job315000.obj"

function load_corners(cornerFile)
    local cornerPoints=dofile(cornerFile);
    local cp={}
    for i=1,#cornerPoints do
        cp[i]={}
        for j=1,#cornerPoints[i] do
            cp[i][j]=torch.Tensor({{cornerPoints[i][j][1],cornerPoints[i][j][2]}})
        end
    end
    return cp;
end

defTens=torch.Tensor({0,1,0});

function first_sweep(scanref,cp,cpref)

    local objref=scanref:get_model_data();
    local occref=retex.Occlusions.new(scanref);
    local positions={};
    local rotations={};
    local xyz={}
    
    local startRotV=torch.Tensor(3);
    local position=scanref.poses[1].position
    local rotation=scanref.poses[1].rotation
    geom.quaternion.rotate(startRotV, defTens, rotation)

    for i=1,#cp_ref do
        positions[i]={};
        rotations[i]={};
        xyz[i]={}
        for j=1,#cp_ref[i] do
            positions[i][j],rotations[i][j]=scanref:get_photos()[1]:localxy2globalray(cp_ref[i][j][1][1],cp_ref[i][j][1][2]);
            local rays=util.Ray.new(positions[i][j],rotations[i][j]);
            local dref,faceid=occref:get_occlusion_slow(rays,objref);
            xyz[i][j]=rays:endpoint(dref);
        end

        collectgarbage()
    
    end

    local mod={}
    mod.objref=objref
    mod.positions=positions
    mod.rotations=rotations
    mod.rays=rays
    mod.xyz=xyz
    mod.startRotV =startRotV
    mod.pose_position=position
    mod.pose_rotation=rotation
    return mod
end

scanref=util.mp.scan(texfname, objfname);
cp=load_corners(cfname);
cp_ref=load_corners(creffname);
mod=loader("tmplua_cache.t7",first_sweep,scanref,cp,cpref)

myLens=LensSensor.new("nikon_10p5mm_r2t_full")
img=image.load("/Users/lihui815/Downloads/damp-unicorn-2109_a_00/sweep_1/JPG/DSC_0008.jpg")
img:resize(img:size(1),img:size(2), img:size(3))
myLens:add_image(img)

cpuv={}

corduv={}
--cordpx={}

unitVectorsuv={}
--unitVectorspx={}

worldPoints={}

solutionsuv={}
--solutionspx={}

local pt_type_uv="uv"
local pt_type_px="pixel_space"
local out_type="uc"

for i=1,#mod.xyz do
    
    corduv[i]={}
    --cordpx[i]={}

    cpuv[i]={}
    
    unitVectorsuv[i]=torch.Tensor(3,3);
    --unitVectorspx[i]=torch.Tensor(3,3);
    
    worldPoints[i]=torch.Tensor(3,3);
    
    solutionsuv[i]=torch.Tensor(3,16);
    --solutionspx[i]=torch.Tensor(3,16);
    
    for j=1,#mod.xyz[i] do

        local a=cp[i][j]:clone()
        
        a[1][1]=2*a[1][1]/img:size()[3]-1
        a[1][2]=2*a[1][2]/img:size()[2]-1
        cpuv[i][j]=a
        corduv[i][j]=myLens:img_coords_to_world (cpuv[i][j], pt_type_uv, out_type)
        --cordpx[i][j]=myLens:img_coords_to_world (cp[i][j], pt_type_px, out_type)

        unitVectorsuv[i][j]=corduv[i][j][1]
        --unitVectorspx[i][j]=cordpx[i][j][1]

        worldPoints[i][j]=mod.xyz[i][j]

    end
    
    solutionsuv[i]=p3p.compute_poses(worldPoints[i],unitVectorsuv[i],1)
    --solutionspx[i]=p3p.compute_poses(worldPoints[i],unitVectorspx[i],1)

end
