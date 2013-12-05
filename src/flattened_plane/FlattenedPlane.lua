FlattenedPlane = Class()
FlattenedPlane.PLANE = "FlattenedPlanes"
local path = require 'path'
local pcl = PointCloud.PointCloud
local Sweep = align_floors_endtoend.Sweep
local io = require 'io'
function FlattenedPlane:__init(base_dir, sweep_name, pc, transf, index)
    if not(util.fs.is_dir(base_dir)) then
        error("expected base_dir to exist for initializer for Sweep")
    end
    self.sweep_name = sweep_name
    self.index = index
    self.base_dir = base_dir
    self.fsave_me = path.join(self.base_dir, FlattenedPlane.PLANE, "plane_".. sweep_name ..  "index_" .. index .. ".dat")

    self.resolution = 10

    self.transformation = transf or torch.eye(4,4)
    self:setupQuatToFlatten()
    self.thresh = .05
    self:addPlane(pc)
    self:saveMe()
end

function FlattenedPlane:get_xyz_coordinates()
    local minT, maxT = self:calculateMinAndMaxT()
    local flat, t, x_mat, y_mat = FlattenedPlane.flattened2Image(torch.zeros(1,2), minT, maxT)
    local eq_new = flattened_plane.PlaneIntersectionLine.getRotatedEquation(self:getPlaneEquation(self.index), self.quat)
    local x = x_mat:reshape(x_mat:size(1)*x_mat:size(2))*self.resolution --add min??
    local y = y_mat:reshape(y_mat:size(1)*y_mat:size(2))*self.resolution
    local z = (x*eq_new[1]+y*eq_new[2]+eq_new[4])/-eq_new[3]
    local unrotate_pts = geom.quaternion.rotate(geom.quaternion.inverse(self.quat), torch.cat(torch.cat(x,y ,2),z,2))
    local pts_test = PointCloud.PointCloud.get_global_from_2d_pts(unrotate_pts,torch.inverse(self.transformation)) --+center:reshape(1,3):repeatTensor(unrotate_pts:size(1),1)
    --local pts_test = PointCloud.PointCloud.get_global_from_2d_pts(self.plane:reshape(3,self.plane:size(2)*self.plane:size(3)):t(),torch.inverse(self.transformation)) --+center:reshape(1,3):repeatTensor(unrotate_pts:size(1),1)

    return pts_test
end
function FlattenedPlane:getBaseDir()
    return self.base_dir
end
function FlattenedPlane:getSaveLocation()
    return self.fsave_me
end

function FlattenedPlane:saveMe()
    torch.save(self.fsave_me, self)
end

function FlattenedPlane:setupQuatToFlatten()
    local eqn = self:getPlaneEquation(self.index)
    local angle_az = geom.util.unit_cartesian_to_spherical_angles(eqn)
    angle_az = angle_az:index(1,torch.LongTensor({2,1}))

    --intersect of plane with 0/0
    local vec_on_intersect = torch.Tensor({1,-eqn[1]/eqn[2],0})
    vec_on_intersect = vec_on_intersect/vec_on_intersect:norm()
    local quat1 = geom.quaternion.from_axis_angle(vec_on_intersect,math.pi/2-angle_az[1])
    angle_az[1] = 0
    self.quat = quat1
end
function FlattenedPlane.newOrLoad(base_dir, sweep_name, pc, transf, index)
    if(util.fs.is_file(path.join(base_dir, FlattenedPlane.PLANE, "plane_" ..sweep_name .. "index_" .. index .. ".dat"))) then
        print("loading plane from file " .. "plane_" ..sweep_name ..  "index_" .. index .. ".dat")
        return torch.load(path.join(base_dir, FlattenedPlane.PLANE, "plane_" ..sweep_name ..  "index_" .. index .. ".dat"))
    else
        return FlattenedPlane.new(base_dir, sweep_name, pc, transf, index)
    end
end

function FlattenedPlane:__write_keys()
    return {'base_dir','fsave_me', 'sweep_name', 'resolution', 'quat','plane','planed', 'plane_reald', 'index',
        'emptiesI', 'occupiedI',
        'iempties', 'iemptiesF', 'iemptiesFD', 'rempties','iemptiesFP',
        'ioccupied', 'ioccupiedF', 'ioccupiedFD', 'roccupied', 'ioccupiedFP',
        'minT', 'maxT', 'thresh','mask',
        'notnils', 'rnotnils','notnilsI', 'inilsF',
        'plane_interesection_data','iallD',
        'allIntersections', 'allIntersectionsInfo', 'transformation', 'occupiedOnLines'
}
end

function FlattenedPlane:getAllPlaneEquations()
    local location = util.fs.dirs_only(path.join(self.base_dir, "planes3", self.sweep_name),"saliency_base_9_scale_1.8_n_scale_5_thres_40_normthres_1.0472_minseed_81_minplane_900_slope_score_down_weight_pinned_center_normal_var")[1] --40
    --   local location = util.fs.dirs_only(path.join("/Users/stavbraun/Downloads/planes_sonia/", self.sweep_name),"40")[1]
    eqns = torch.load(path.join(location, "planes.t7"))
    table.sort(eqns, function(a,b) return a.n_pts>b.n_pts end)
    return eqns
end
function FlattenedPlane:getPlaneEquation(k)
    local eqn = self:getAllPlaneEquations(self.base_dir)[k].eqn
    local quat =  geom.quaternion.from_rotation_matrix(self.transformation:sub(1,3,1,3))
    local n1 = geom.quaternion.rotate(quat, eqn:sub(1,3))
    local pt1,t,vec1 = FlattenedPlane.findVector(eqn)
    local pt1r = geom.quaternion.rotate(quat, pt1)
    pt1r = pt1r+self.transformation:sub(1,3,4,4)
    local d_new = -pt1r*n1
   return torch.cat(n1, torch.Tensor({d_new}))
end

function FlattenedPlane.select3d(from, selectPts)
    local npts
    if(from:dim()==2) then
        if (selectPts:dim() == 1) then
            npts = selectPts:size(1)
        else
            npts = selectPts:size(1)*selectPts:size(2)
        end
        return from:index(1,torch.range(1,npts)[selectPts]:long())
    elseif(from:dim()==3) then
        npts = selectPts:size(1)*selectPts:size(2)
        from = from:reshape(3,npts):t()
        return from:index(1,torch.range(1,npts)[selectPts:reshape(npts)]:long())

    else
        error("wrong number of select pts")
    end

end

function FlattenedPlane:addPlane(pc)
    --to do check argument type
    local H = self.transformation
    local eqn = self:getPlaneEquation(self.index)

    local plane = self:getPlaneViewFromCenter(eqn, H, pc)
    local center = H:sub(1,3,4,4):squeeze()
    local planed = plane - center:reshape(3,1,1):repeatTensor(1,plane:size(2),plane:size(3))
    local plane_reald = pc:get_depth_map()
    pc:set_pose_from_rotation_matrix(H)
    local plane_norm = pc:get_global_normal_map_H(H)
    local index, maskI = pc:get_index_and_mask()
    local plane_norm_reshaped = plane_norm:reshape(3,plane_norm:size(2)*plane_norm:size(3)):t()
    --local good_norm = torch.lt(torch.acos(plane_norm_reshaped:clone()*eqn:sub(1,3):squeeze()),math.rad(60)):squeeze():reshape(plane_norm:size(2),plane_norm:size(3))
    local mask = torch.eq(torch.eq(plane_reald,0) + torch.eq(plane:clone():norm(2,1),0),0)


    local matcher = Plane.Matcher.new()
    local normals, temp, temp, temp, maskN = pc:get_normal_map()
    local normals = flattened_plane.FlattenedPlane.select3d(normals, maskN*-1+1)

    local pts = flattened_plane.FlattenedPlane.select3d(pc:get_xyz_map_no_mask(), maskN*-1+1)
    local score, indices = matcher:match(self:getAllPlaneEquations(), pts:t(), normals:t())
    score = score:squeeze()
    indices = indices:squeeze()
    local occupiedI = torch.zeros(maskN:size()):byte()
    occupiedI[maskN*-1+1] = torch.eq(torch.eq(indices,self.index)+torch.ge(score,.1),2)

    local temp = torch.zeros(maskN:size()):long()
    temp[maskN*-1+1] = indices

    --[[ if want to project onto plane, instead of assume position is on plane
    local pts  = flattened_plane.FlattenedPlane.select3d(pc:get_global_from_3d_pts(pc:get_xyz_map_no_mask(),H), maskN*-1+1)
    local indicesA = torch.eq(torch.eq(indices,self.index)+torch.ge(score,.1),2)
    local selected = flattened_plane.FlattenedPlane.select3d(pts, indicesA)
    t = selected*eqn:sub(1,3)
    local occupied = selected-t:reshape(t:size(1),1)*eqn:sub(1,3):reshape(1,3)-eqn:sub(4,4):squeeze()
    --]]
    --    local occupiedIT = torch.eq(torch.le(torch.abs(planed:clone():norm(2,1):squeeze()-plane_reald), self.thresh*5*1000)+mask+good_norm,3)
      local occupied = FlattenedPlane.select3d(plane:reshape(3,plane:size(2)*plane:size(3)):t(),occupiedI)

    --our hypothesized plane is more than 1 cm closer to us than the real point, it is going through empty!
    local emptiesI = torch.eq(torch.lt(planed:clone():norm(2,1):squeeze()-plane_reald, -self.thresh*1000)+mask,2):cmul(occupiedI*-1+1)
    local empties = FlattenedPlane.select3d(plane:reshape(3,planed:size(2)*planed:size(3)):t(),emptiesI)
    
    
    local notnilsI = maskI*-1+1
    local notnils = FlattenedPlane.select3d(plane:reshape(3,plane:size(2)*plane:size(3)):t(),notnilsI)
        
    self.pc =pc
    self.rempties = torch.zeros(0,0)
    self.roccupied = torch.zeros(0,0)
    self.plane = plane
    self.planed = planed
    self.plane_reald = plane_reald
    self.emptiesI = emptiesI
    self.occupiedI = occupiedI
    self.mask = mask
    if(empties:dim() >0) then
        self.rempties = geom.quaternion.rotate(self.quat,empties:contiguous())
    end
    if(occupied:dim() >0) then
        self.roccupied = geom.quaternion.rotate(self.quat,occupied:contiguous())
    end
    self.notnilsI = notnilsI
    if(notnils:dim()>0) then
        self.rnotnils = geom.quaternion.rotate(self.quat,notnils:contiguous())
    end
    pc:set_pose_from_rotation_matrix(torch.eye(4))
    
    self:addAllFrustrumsAndEmpties()
    self:saveMe()
    collectgarbage()
end

function FlattenedPlane:saveToXYZ(occI, fname)
    local xyz = self.pc:get_xyz_map_no_mask():reshape(3,self.occupiedI:size(1)*self.occupiedI:size(2)):t()
    xyz = xyz:index(1,torch.range(1,self.occupiedI:size(1)*self.occupiedI:size(2))[occI:reshape(self.occupiedI:size(1)*self.occupiedI:size(2))]:long())

    self.pc:save_any_points_to_xyz(fname, xyz)
    collectgarbage()
end

function FlattenedPlane.save(list, name)
    local file = io.open(name .. ".xyz", 'w')
    local tmpt = torch.range(1,list:size(1))                                                                                                                                
    tmpt:apply(function(ii) 
        file:write(''..list[ii][1]..' '..list[ii][2]..' '..list[ii][3] .. '\n')        
    end) 
    file:close()
end

function FlattenedPlane.inverseof3x3(a,b,c,d,e,f,g,h,i, x_1, y_1, z_1)
    local detM = (a:clone():cmul(e:clone():cmul(i)+f:clone():cmul(h:clone())*-1)+b:clone():cmul(d:clone():cmul(i)+f:clone():cmul(g)*-1)*-1+c:clone():cmul(d:clone():cmul(h)+e:clone():cmul(g)*-1)):pow(-1)
    local a_new = detM:clone():cmul(e:clone():cmul(i)+f:clone():cmul(h)*-1)
    local b_new = detM:clone():cmul(c:clone():cmul(h)+b:clone():cmul(i)*-1)
    local c_new = detM:clone():cmul(b:clone():cmul(f)+c:clone():cmul(e)*-1)
    local d_new = detM:clone():cmul(f:clone():cmul(g)+d:clone():cmul(i)*-1)
    local e_new = detM:clone():cmul(a:clone():cmul(i)+c:clone():cmul(g)*-1)
    local f_new = detM:clone():cmul(c:clone():cmul(d)+a:clone():cmul(f)*-1)
    local g_new = detM:clone():cmul(d:clone():cmul(h)+e:clone():cmul(g)*-1)
    local h_new = detM:clone():cmul(b:clone():cmul(g)+a:clone():cmul(h)*-1)
    local i_new = detM:clone():cmul(a:clone():cmul(e)+b:clone():cmul(d)*-1)

    local x_result = a_new:clone():cmul(x_1)+b_new:clone():cmul(y_1)+c_new:clone():cmul(z_1)
    local y_result = d_new:clone():cmul(x_1)+e_new:clone():cmul(y_1)+f_new:clone():cmul(z_1)
    local z_result = g_new:clone():cmul(x_1)+h_new:clone():cmul(y_1)+i_new:clone():cmul(z_1)
    return x_result, y_result, z_result
end
function FlattenedPlane:getPlaneViewFromCenter(eqn,H, pc)
    local center = torch.zeros(3)
    if(H) then
        center = H:sub(1,3,4,4):squeeze()
    end
    pc:set_pose_from_rotation_matrix(H or torch.eye(4))
    local pts_o = pc:get_global_from_3d_pts(pc:get_xyz_map_no_mask(),H)
    local pt1,t,vec1 = FlattenedPlane.findVector(eqn)
    local vec2 = torch.cross(vec1, eqn:sub(1,3))
    vec2 = vec2/vec2:norm()
    --plane = pt1 + vec1*u+vec2*v = line = center+ (pts-center)*t
    -- center -pt1 = [vec1; vec2; -pts;]*[u v t]
    local leftSide = (center-pt1):reshape(3,1)
    local transposedPts = pts_o:reshape(3,pts_o:size(2)*pts_o:size(3)):t()
    local tmp = torch.range(1,pts_o:size(2)*pts_o:size(3))
    local tmppts = torch.zeros(pts_o:size(2)*pts_o:size(3),3)

    local a = torch.ones(transposedPts:size(1)):fill(vec1[1])
    local d = torch.ones(transposedPts:size(1)):fill(vec1[2])
    local g = torch.ones(transposedPts:size(1)):fill(vec1[3])
    local b = torch.ones(transposedPts:size(1)):fill(vec2[1])
    local e = torch.ones(transposedPts:size(1)):fill(vec2[2])
    local h = torch.ones(transposedPts:size(1)):fill(vec2[3])

    local mvdCenter = transposedPts-center:reshape(1,3):repeatTensor(transposedPts:size(1),1)
    local c = mvdCenter:select(2,1)*-1
    local f = mvdCenter:select(2,2)*-1
    local i = mvdCenter:select(2,3)*-1

    local x_1 =torch.ones(transposedPts:size(1)):fill(leftSide[1][1])
    local y_1 =torch.ones(transposedPts:size(1)):fill(leftSide[2][1])
    local z_1 =torch.ones(transposedPts:size(1)):fill(leftSide[3][1])

    local x_new, y_new, z_new = FlattenedPlane.inverseof3x3(a,b,c,d,e,f,g,h,i, x_1, y_1, z_1)
    --means we are shooting ray forwards and not backwards
    local good_index = torch.gt(z_new, 0)
    local temp_pt = center:reshape(1,3):repeatTensor(transposedPts:size(1),1)+mvdCenter:clone():cmul(z_new:repeatTensor(3,1):t())
    local good_index2 = torch.ones(good_index:size()):byte() 
    good_index2 =torch.lt((temp_pt-center:reshape(1,3):repeatTensor(temp_pt:size(1),1)):norm(2,2):squeeze(), pc:get_max_radius()*5)
    local good_index3 = torch.eq(torch.lt(temp_pt:clone():sum(2), math.huge):double() + torch.gt(temp_pt:clone():sum(2),math.huge):double(),1):squeeze()
    local cum_good = torch.eq(good_index+good_index2+good_index3,3)
    temp_pt:select(2,1)[cum_good*-1+1]= 0
    temp_pt:select(2,2)[cum_good*-1+1] = 0
    temp_pt:select(2,3)[cum_good*-1+1] = 0

    local new_pts = temp_pt:t():reshape(3,pts_o:size(2), pts_o:size(3))
    return new_pts
end

--self = plane_t FlattenedPlane = align_floors_endtoend.FlattenedPlane
--[[ old relies on remap
function FlattenedPlane:getDistanceMap()
    self.pc:remap()
    local minT, maxT = self:calculateMinAndMaxT()
    local flat, t, x_mat, y_mat = FlattenedPlane.flattened2Image(torch.zeros(1,2), minT, maxT)
    local eq = self:getPlaneEquation(self.index)
    local n1 = geom.quaternion.rotate(self.quat, eq:sub(1,3))
    local p1,p2,v = FlattenedPlane.findVector(eq)
    p1 = geom.quaternion.rotate(self.quat,p1)
    local eq_new = torch.cat(n1:squeeze(),torch.Tensor({-p1*n1}),1)
    local x = x_mat:reshape(x_mat:size(1)*x_mat:size(2))*self.resolution --add min??
    local y = y_mat:reshape(y_mat:size(1)*y_mat:size(2))*self.resolution
    local z = (x*eq_new[1]+y*eq_new[2]+eq_new[4])/-eq_new[3]
    local unrotate_pts = geom.quaternion.rotate(geom.quaternion.inverse(self.quat), torch.cat(torch.cat(x,y ,2),z,2))
    --center = self.H:sub(1,3,4,4):squeeze()
    local pts_test = self.pc:get_global_from_2d_pts(unrotate_pts,torch.inverse(self.H)) --+center:reshape(1,3):repeatTensor(unrotate_pts:size(1),1)
    
    local xyz = self.pc:get_xyz_map_no_mask(true) --pc:get_global_from_3d_pts(pc:get_xyz_map_no_mask(),H)
    local depth = self.pc:get_depth_map(true)

    local index, mask = self.pc:get_index_and_mask()

    local pts = xyz:clone():reshape(3,xyz:size(2)*xyz:size(3)):t()

    local azimuth = torch.atan2(pts:select(2,2):clone(), pts:select(2,1):clone())

    local elevation = torch.acos(pts:select(2,3):clone():cdiv(pts:clone():norm(2,2):squeeze()+10^-6))

    local azimuth2d = azimuth:reshape(xyz:size(2), xyz:size(3)):cmul(mask:double()*-1+1)
    local elevation2d = elevation:reshape(xyz:size(2), xyz:size(3)):cmul(mask:double()*-1+1)
    
    local elev_range = (elevation2d:sum(2)):squeeze():cdiv((mask*-1+1):double():sum(2):squeeze())
    local azimuth_range = (azimuth2d:sum(1)):squeeze():cdiv((mask*-1+1):double():sum(1):squeeze())

    local step_size_elev = (elev_range[-1]-elev_range[1])/(elev_range:size(1)-1)
    local min_elev = elev_range[1]

    local step_size_az = (azimuth_range[-1]-azimuth_range[1])/(azimuth_range:size(1)-1)
    local min_az = azimuth_range[1]
    
    local azimuth_test = torch.atan2(pts_test:select(2,2):clone(), pts_test:select(2,1):clone())
    local elevation_test = torch.acos(pts_test:select(2,3):clone():cdiv(pts_test:clone():norm(2,2):squeeze()+10^-6))

    local pts_az = ((azimuth_test-min_az)/step_size_az+1)
    local pts_elev = ((elevation_test-min_elev)/step_size_elev+1)

    --to do don't need resize can do it myself...
    depth[mask] = math.huge
    local dis_resized = opencv.imgproc.resize(opencv.Mat.new(depth:clone()), 11):toTensor()
    dis_resized[torch.lt(dis_resized,math.huge)*-1+1] = -1
    --test this, want ot make sure this loops correctly
    if(pts_az[torch.eq(pts_az:floor(),0)]:dim()>0) then
    pts_az[torch.eq(pts_az:floor(),0)] = depth:size(1)+(1+pts_az[torch.eq(pts_az:floor(),0)]*-1)*-1 --get rid of looping around error
    end
    if(pts_az[torch.eq(pts_az:floor(),depth:size(1)+1)]:dim() >0) then
        pts_az[torch.eq(pts_az:floor(),depth:size(1)+1)] = (pts_az[torch.eq(pts_az:floor(),depth:size(1)+1)]-depth:size(1))+1 --get rid of looping around error
    end
    local resized_az = pts_az*11-5
    local resized_elev = pts_elev*11-5

    local good_pts = torch.eq(torch.le(resized_elev:floor(), dis_resized:size(1)) + torch.ge(resized_elev, 1) + 
                    torch.le(resized_az:floor(), dis_resized:size(2)) + torch.ge(resized_az, 1),4)

    --row*width+col, row# = elev=height, az = width
    local index = ((resized_elev:floor()-1)*dis_resized:size(2)+resized_az):long()

    local result = torch.ones(good_pts:size(1)):fill(-1)
    result[good_pts] = dis_resized:reshape(dis_resized:size(1)*dis_resized:size(2))[index[good_pts]]
    
--]]

    --local result = result:reshape(x_mat:size(1), x_mat:size(2))
   --[[ return result, pts_test:norm(2,2):squeeze():reshape(x_mat:size(1),x_mat:size(2)), depth--resultXYZ:norm(2,1):squeeze()

end
]]--
function FlattenedPlane.findVector(eqn)
    local pointonplane
    local n = eqn:sub(1,3)
    local d = eqn[4]
    if(n[3]~=0) then --ax+by+cz=d
        pointonplane = torch.Tensor({0,0,(-d/n[3])})
        if(n[2]~=0) then
            pointonplane2 = torch.Tensor({0,(-d/n[2]), 0})
        elseif(n[1]~=0) then
            pointonplane2 = torch.Tensor({(-d/n[1]),0,0})
        else
            pointonplane2 = torch.Tensor({0,0,0})
        end
    elseif(n[2]~=0) then
        pointonplane = torch.Tensor({0,(-d/n[2]), 0})
        if(n[3]~=0) then
            pointonplane2 = torch.Tensor({0,0,(-d/n[3])})
        elseif(n[1]~=0) then
            pointonplane2 = torch.Tensor({(-d/n[1]),0,0})
        else
            pointonplane2 = torch.Tensor({0,0,0})
        end
    elseif(n[1]~=0) then
        pointonplane = torch.Tensor({(-d/n[1]),0,0})
        if(n[3]~=0) then
            pointonplane2 = torch.Tensor({0,0,(-d/n[3])})
        elseif(n[2]~=0) then
            pointonplane2 = torch.Tensor({0,(-d/n[2]), 0})
        else
            pointonplane2 = torch.Tensor({0,0,0})
        end
    end
    local v = (pointonplane2+pointonplane*-1)
    v = v:clone():div(v:norm())
    return pointonplane, pointonplane2, v
end
function FlattenedPlane:getCornersSonia()
    plane = self.plane
    occupiedI = self.occupiedI
    --local corners = image.load(path.join("/Users/stavbraun/Downloads/planes_sonia/", self.sweep_name, "border", "plane_" .. string.format("%.3d.png",self.index)))
    
    local corners = image.load(path.join("/Users/stavbraun/Downloads/sweep_001", "extents2d", "plane_" .. string.format("%.3d.png",self.index)))
    return corners
end

--    local a,b,cornersI = self:getCornersSonia(self.plane, self.occupiedI) cmul(occupiedI)?
function FlattenedPlane:addArbitraryFrustum(cornersI, onlyOnThisOne)  

    if(cornersI:sum() > 0 ) then
        local minT, maxT = self:calculateMinAndMaxT()
        local t
        if(onlyOnThisOne) then
            t = torch.conv2(cornersI, torch.Tensor({0,1,0,1,1,1,0,1,0}):reshape(3,3):byte())        
        else
            t = torch.conv2(self.notnilsI, torch.Tensor({0,1,0,1,1,1,0,1,0}):reshape(3,3):byte())
        end
        local t_new = torch.zeros(t:size(1)+2,t:size(2)+2):byte()
        t_new[{{2,(-1)-1},{2,(-1)-1}}] = torch.eq(t,5)
        t_new:cmul(cornersI)
        local bottomSelectorN =torch.conv2(t_new, torch.Tensor({0,1,0,0,0,0,0,0,0}):reshape(3,3):byte(),'F'):sub(2,-2,2,-2)
        local topSelectorN =torch.conv2(t_new, torch.Tensor({0,0,0,0,0,0,0,1,0}):reshape(3,3):byte(),'F'):sub(2,-2,2,-2)
        local rightSelectorN = torch.conv2(t_new, torch.Tensor({0,0,0,1,0,0,0,0,0}):reshape(3,3):byte(),'F'):sub(2,-2,2,-2)
        local leftSelectorN = torch.conv2(t_new, torch.Tensor({0,0,0,0,0,1,0,0,0}):reshape(3,3):byte(), 'F'):sub(2,-2,2,-2)
        local plane = self.plane
        local occupied = FlattenedPlane.select3d(self.plane:reshape(3,plane:size(2)*plane:size(3)):t(),cornersI)
        local occupiedUs = FlattenedPlane.select3d(self.plane:reshape(3,plane:size(2)*plane:size(3)):t(),t_new)
        local occupiedL = FlattenedPlane.select3d(self.plane:reshape(3,plane:size(2)*plane:size(3)):t(),leftSelectorN)
        local occupiedR = FlattenedPlane.select3d(self.plane:reshape(3,plane:size(2)*plane:size(3)):t(),rightSelectorN)
        local occupiedT = FlattenedPlane.select3d(self.plane:reshape(3,plane:size(2)*plane:size(3)):t(),topSelectorN)
        local occupiedB = FlattenedPlane.select3d(self.plane:reshape(3,plane:size(2)*plane:size(3)):t(),bottomSelectorN)
        if(occupiedL:dim()> 0 and occupiedUs:size(1)>1) then
            local roccupiedUs= geom.quaternion.rotate(self.quat,occupiedUs:contiguous())
            local roccupied= geom.quaternion.rotate(self.quat,occupied:contiguous())
            local roccupiedL= geom.quaternion.rotate(self.quat,occupiedL:contiguous())
            local roccupiedR= geom.quaternion.rotate(self.quat,occupiedR:contiguous())
            local roccupiedT= geom.quaternion.rotate(self.quat,occupiedT:contiguous())
            local roccupiedB= geom.quaternion.rotate(self.quat,occupiedB:contiguous())
            --local coord_filter = torch.eq(torch.eq(occupiedL:sum(2):squeeze(),0)+torch.eq(occupiedR:sum(2):squeeze(),0)+torch.eq(occupiedT:sum(2):squeeze(),0)+torch.eq(occupiedB:sum(2):squeeze(),0),0)

            
            local dis_to_plane = self.planed:clone():norm(2,1):squeeze()-self.plane_reald
            local d = dis_to_plane[cornersI]
            local dUs = torch.eq(occupiedUs:clone():sum(2):squeeze(),0):double()--dis_to_plane[t_new]
            local dL = torch.eq(occupiedL:clone():sum(2):squeeze(),0):double()--dis_to_plane[leftSelectorN]
            local dR = torch.eq(occupiedR:clone():sum(2):squeeze(),0):double()--dis_to_plane[rightSelectorN]
            local dT = torch.eq(occupiedT:clone():sum(2):squeeze(),0):double()--dis_to_plane[topSelectorN]
            local dB = torch.eq(occupiedB:clone():sum(2):squeeze(),0):double()--dis_to_plane[bottomSelectorN]

            local disUs =dis_to_plane[t_new]
            local disL = dis_to_plane[leftSelectorN]
            local disR = dis_to_plane[rightSelectorN]
            local disT = dis_to_plane[topSelectorN]
            local disB = dis_to_plane[bottomSelectorN]
            local roccupiedUS = torch.cat(torch.cat(roccupiedUs:sub(1,-1,1,2)/self.resolution-torch.repeatTensor(minT, roccupiedUs:size(1), 1)+1,dUs:reshape(dUs:size(1),1),2), disUs,2)    
            local roccupiedL = torch.cat(torch.cat(roccupiedL:sub(1,-1,1,2)/self.resolution-torch.repeatTensor(minT, roccupiedL:size(1), 1)+1,dL:reshape(dL:size(1),1),2), disL, 2)
            local roccupiedR = torch.cat(torch.cat(roccupiedR:sub(1,-1,1,2)/self.resolution-torch.repeatTensor(minT, roccupiedR:size(1), 1)+1,dR:reshape(dR:size(1),1),2), disR, 2)    
            local roccupiedT = torch.cat(torch.cat(roccupiedT:sub(1,-1,1,2)/self.resolution-torch.repeatTensor(minT, roccupiedT:size(1), 1)+1, dT:reshape(dT:size(1),1),2), disT, 2)    
            local roccupiedB = torch.cat(torch.cat(roccupiedB:sub(1,-1,1,2)/self.resolution-torch.repeatTensor(minT, roccupiedB:size(1), 1)+1, dB:reshape(dB:size(1),1),2), disB, 2)    
            collectgarbage()
            local flat, flatD = FlattenedPlane.flattened2Image((roccupied:sub(1,-1,1,2)/self.resolution), minT, maxT, d)
            local coord = torch.cat(torch.cat(torch.cat(torch.cat(roccupiedL, roccupiedT, 2), roccupiedR,2), roccupiedB,2), roccupiedUS,2)
                               

            local shouldFrustrumCoords = torch.range(1,coord:size(1))
            local good_coords = coord:index(1,shouldFrustrumCoords:long())
            local cum = torch.zeros(flat:size())
            local emptiesF, emptiesFD= opencv.imgproc.fillQuadAllWithInterpolation(opencv.Mat.new(cum),coord)
            return flat, torch.gt(emptiesF:toTensor(),0), emptiesFD:toTensor()
        end
    end
    collectgarbage()
end

function FlattenedPlane:saveForMarco()
    collectgarbage()
        local base_dir = path.join(self.base_dir, FlattenedPlane.PLANE, "for_marco")

       
    local corners =self:getCornersSonia()
    local nils, distances = self:getNils()

    local rcornersI1, rcornersI1V = self:getArbitraryRotated(corners[1])
    local icornersI1, icornersI1V = self:concatArbitrary(rcornersI1, nil, rcornersI1V)
    print(torch.ne(corners[1],0):sum())
    local a,icorners1F,c = self:addArbitraryFrustum(torch.ne(corners[1],0))

    local rcornersI2, rcornersI2V = self:getArbitraryRotated(corners[2])
    local icornersI2, icornersI2V = self:concatArbitrary(rcornersI2, nil, rcornersI2V)
    local a,icorners2F,c = self:addArbitraryFrustum(torch.ne(corners[2],0))

    local rcornersI3, rcornersI3V = self:getArbitraryRotated(corners[3])
    local icornersI3, icornersI3V = self:concatArbitrary(rcornersI3, nil, rcornersI3V)
    local a,icorners3F,c = self:addArbitraryFrustum(torch.ne(corners[3],0))

    util.fs.mkdir_p(base_dir)

    torch.save(path.join(base_dir, "plane_sonia_" .. self.index .. ".dat"), {icornersI1, icornersI1V, icorners1F, 
                                                        icornersI2, icornersI2V, icorners2F,
                                                        icornersI3, icornersI3V, icorners3F})

    torch.save(path.join(base_dir,"plane_" .. self.index .. ".dat"), 
            {self.ioccupied, self.ioccupiedF, self.ioccupiedP, 
            self.iempties, self.iemptiesF, self.iemptiesP, nils, distances})


    local all_intersections = self:getAllIntersections()
    local all_intersections_weights =     self:getGoodIntersections()
    torch.save(path.join(base_dir,"plane_intersections_" .. self.index .. ".dat"), 
            {all_intersections, all_intersections_weights})
   

     collectgarbage()
end
--pc should already have transf set
function FlattenedPlane:getArbitraryRotated(corners)
    if(self.occupiedI:sum()>0) then
        local corners_coord = FlattenedPlane.select3d(self.plane:reshape(3,self.plane:size(2)*self.plane:size(3)):t(), torch.gt(corners,0):reshape(corners:size(1)*corners:size(2)):cmul(self.occupiedI))
        local rcornersV = corners[torch.gt(corners,0):reshape(corners:size(1)*corners:size(2)):cmul(self.occupiedI)]
        if(corners_coord:dim() > 0) then
         rcorners = geom.quaternion.rotate(self.quat, corners_coord)
        end
        return rcorners, rcornersV
    end
end

local function calculateMinAndMax(lua_table_of_vectors, minT, maxT)
    minT = minT or torch.Tensor({math.huge, math.huge, math.huge}):reshape(1,3)
    maxT = maxT or torch.Tensor({-math.huge, -math.huge, -math.huge}):reshape(1,3)
    local v = lua_table_of_vectors
    if(v:dim()~=0) then
        if(v:dim() == 1) then
            v = v:reshape(1,3)
        end
        minT = torch.min(torch.cat(minT, v,1),1)
        maxT = torch.max(torch.cat(maxT, v,1),1) 
    end
    return minT, maxT
end

function FlattenedPlane:addAllFrustrumsAndEmpties(redo)
    print("addAllFrustrumsAndEmpties")
    if(redo or not(self.ioccupied) or not(self.iempties)) then
        self.ioccupied, self.ioccupiedF, self.occupiedFD = self:addArbitraryFrustum(self.occupiedI, true)
        if(self.occupiedI:sum()>0) then
            local occupied_dis = opencv.imgproc.distanceTransform(opencv.Mat.new((self.ioccupied*1-1):byte())):toTensor():double()
            if(self.ioccupiedF) then
                self.ioccupiedFP = self.ioccupied:double() + torch.gt(self.ioccupiedF,0):double():clone():cmul((occupied_dis/5+1):pow(-1))
            end
        end
        self.iempties, self.iemptiesF, self.iemptiesFD = self:addArbitraryFrustum(self.emptiesI, true)
        if(self.emptiesI:sum()>0) then
            local empties_dis = opencv.imgproc.distanceTransform(opencv.Mat.new((self.iempties*1-1):byte())):toTensor():double()
            if(self.iemptiesF) then
                self.iemptiesFP = self.iempties:double() + torch.gt(self.iemptiesF,0):double():clone():cmul((empties_dis/5+1):pow(-1))        
            end
        end
    end
end
function FlattenedPlane:calculateMinAndMaxT()
    local minT,maxT = calculateMinAndMax(self.rempties)
    minT,maxT = calculateMinAndMax(self.roccupied,minT, maxT)
    minT = minT:sub(1,1,1,2)/self.resolution-200
    maxT =maxT:sub(1,1,1,2)/self.resolution+200
    self.minT = minT:floor()
    self.maxT = maxT:ceil()
    return self.minT, self.maxT
end

function FlattenedPlane:getNils(recalc)
    if(not(self.inilsF) or not(self.iallD) or recalc) then
        in_range, index = self:getInRange(self.rnotnils)
        in_range_i = self.notnilsI:clone()
        in_range_i[self.notnilsI] = index
        a,b,c = self:addArbitraryFrustum(in_range_i)
        self.inilsF = torch.gt(a+b,0)*-1+1
        self.iallD = c
        self:saveMe()
    end
    return self.inilsF, self.iallD 
end
function FlattenedPlane:getInRange(rcorners)
    local minT, maxT = self:calculateMinAndMaxT()
    rcorners = rcorners/self.resolution
    good_index = torch.le(rcorners:sub(1,-1,1,1), maxT[1][1])+torch.le(rcorners:sub(1,-1,2,2), maxT[1][2]) +
        torch.ge(rcorners:sub(1,-1,1,1), minT[1][1])+torch.ge(rcorners:sub(1,-1,2,2), minT[1][2])
    good_index = torch.eq(good_index, 4)
    rcorners = FlattenedPlane.select3d(rcorners, good_index:squeeze():byte())*self.resolution
    return rcorners, good_index:squeeze():byte()

end
function FlattenedPlane:concatArbitrary(rcorners, limitRange, rcornersV)
    local minT, maxT = self:calculateMinAndMaxT()
    if(limitRange) then
        rcorners = self:getInRange(rcorners)
    end
    if(rcorners:dim()~=0) then
        if(rcorners:dim()~=0) then
            icorners, icornersD = FlattenedPlane.flattened2Image(rcorners:sub(1,-1, 1,2)/self.resolution, minT, maxT, rcornersV) 
        end
    end
    return icorners, icornersD
end


function FlattenedPlane.get_ith_plane(plane_num, sweep_name, redo)
    local plane
    local sweep_name= sweep_name or "sweep_001"
    local plane_num=plane_num or 1
    local base_dir = "/Users/stavbraun/Desktop/play/motor-unicorn-0776_newsonia"
    if not(redo) and (util.fs.is_file(path.join(base_dir, FlattenedPlane.PLANE, "plane_" ..sweep_name .. "index_" .. plane_num .. ".dat"))) then
        plane= torch.load(path.join(base_dir, 
            FlattenedPlane.PLANE, "plane_" ..sweep_name ..  "index_" .. plane_num .. ".dat"))
    else
        local scan = align_floors_endtoend.Scan.new("/Users/stavbraun/Desktop/play/motor-unicorn-0776_newsonia")
        local forest =  scan:organize_in_trees()         
        local tree = forest.tree_list.sweep_001
        plane = FlattenedPlane.new(base_dir,sweep_name, forest.tree_list.sweep_001:getRoot():getSweep():getPC(), forest.tree_list.sweep_001:getRoot():getTransformationToRoot(), plane_num)
    end
    --image.save("iempties" .. plane_num .. ".png", plane.iempties:double())
    --image.save("ioccupied" .. plane_num .. ".png", plane.ioccupied:double())

    return plane
end

function FlattenedPlane:saveAllToWebsite(filename)
    local html_doc = sandbox.sbraun.CreateHtml.new(filename, "output from Plane.lua" .. self.sweep_name,"output for equation from sweep_001 and plane " .. self.sweep_name .. " ")
    html_doc:beginFile()
    html_doc:addImageFromTensor("empties", self.iempties, "images/empties.png", "empties")
    html_doc:addImageFromTensor("occupied", self.ioccupied, "images/occupied.png", "occupied")

    html_doc:endFile()
    html_doc:close_file()
    gnuplot.closeWindow(1)

end

function FlattenedPlane.flattened2Image(flattenedxy, minT, maxT, dis)

    flattenedxy = flattenedxy-torch.repeatTensor(minT, flattenedxy:size(1), 1)+1    
    local size_us = (maxT:ceil()-minT:floor()+1):reshape(2)
    local combined = torch.zeros(size_us[1]* size_us[2]):byte()
    local combinedD = torch.zeros(size_us[1]*size_us[2])
    flattenedxy:ceil()
    local indexV =  (((flattenedxy:t()[1]-1)*size_us[2])+flattenedxy:t()[2]):long()

    combined:indexCopy(1,indexV, torch.ones(indexV:size(1)):byte())
    combined= combined:reshape(size_us[1], size_us[2])
    if(dis) then --to do sort by dis, if we care about which one we get (prob want closest)
        combinedD:indexCopy(1,indexV, dis)
    end
    combinedD= combinedD:reshape(size_us[1],size_us[2])

    local x_vals = torch.range(minT[1][1],maxT[1][1]):repeatTensor(size_us[2],1):t()
    local y_vals = torch.range(minT[1][2],maxT[1][2]):repeatTensor(size_us[1],1)
    return combined, combinedD, x_vals, y_vals

end

function FlattenedPlane:getAllIntersections(recalc)
    if(not(self.allIntersections) or recalc) then
        combined = {}
        for j=1,#self:getAllPlaneEquations() do
            if (j~=self.index) then
                print("my j is", j)
                local intersect_j = flattened_plane.PlaneIntersectionLine.new(self.index,j)
                local flatIntersect, temp, temp, coords = intersect_j:findIntersectionImageOnPlane(1)
                if(flatIntersect) then
                    combined[j] = flattened_plane.Line.new(nil,coords[1],coords[2], coords[3], coords[4],{flatIntersect, flatIntersect:double()})

                end
            end
        end
        self.allIntersections = combined
        self:saveMe()
    end
    return self.allIntersections

end

function FlattenedPlane:getOccupiedOnLines(recalc)
    local base_dir = path.join(self.base_dir, FlattenedPlane.PLANE, "for_marco")
    util.fs.mkdir_p(base_dir)
    if(not(self.occupiedOnLines) or recalc) then
        range_min = 1
        range_max =  #self:getAllPlaneEquations()
        combined = {}
        for j=range_min, range_max do 
            combined[j] = {}
            if (j~=self.index) then
                print("my j is", j)
                local intersect_j = flattened_plane.PlaneIntersectionLine.new(self.index,j)
                local flatIntersect, flatOcc, flatEmpt, x_mat, y_mat = intersect_j:getRotatedIntersectionAndOccupiedEmpties(1)
                if flatIntersect then
                    local cumO = self:getOccupied(flatOcc, flatIntersect)
                    --local newCum, newCumD = self:combineSmartly(flatIntersect, cumO, cumE, cumUnknown,cumO2, cumE2, cumUnknown2, 1, intersect_j, x_mat, y_mat)
                    --do some smart combination
                    local cum = {cumO}
                    local newCum, newCumD = self:combineAll(flatIntersect, cum, 1, intersect_j, x_mat, y_mat) 
                    local flat1, flatd1,startX,startY, endX, endY = intersect_j:convertIntersectLineToPlane(1, 1, newCum[1], newCumD)
                    combined[j][1] = flattened_plane.Line.new(nil,startX,startY, endX, endY,{flat1, flatd1[1]})


                    local flatIntersect2, flatOcc2, flatEmpt2, x_mat, y_mat = intersect_j:getRotatedIntersectionAndOccupiedEmpties(2)
                    if flatIntersect2 then
                        local cumOP2 = self:getOccupied(flatOcc2,flatIntersect2)
                        cum = {cumOP2}
                        local newCum2, newCumD2 = self:combineAll(flatIntersect2, cum, 2, intersect_j, x_mat, y_mat) 
                        local flat1, flatd1, startX,startY, endX, endY = intersect_j:convertIntersectLineToPlane(1, 2, newCum2[1], newCumD2)

                        combined[j][2] = flattened_plane.Line.new(nil,startX,startY, endX, endY,{flat1, flatd1[1]})
                    end

                end
                collectgarbage()
            end
        end
        self.occupiedOnLines = combined
        self:saveMe()
        return self.occupiedOnLines
    end
    
end

function FlattenedPlane:getOccupied(flatOcc, flatIntersect)
    local temp,locOfRow = flatIntersect:sum(2):squeeze():max(1)
    locOfRow = locOfRow:squeeze()
    local cum = torch.sum(flatOcc:int(),1)
    return cum
end

function FlattenedPlane:getGoodIntersections(recalc)
    local base_dir = path.join(self.base_dir, FlattenedPlane.PLANE, "for_marco")
    util.fs.mkdir_p(base_dir)
    if(not(self.allIntersectionsInfo) or recalc) then
        range_min = 1
        range_max =  #self:getAllPlaneEquations()
        combined = {}
        for j=range_min, range_max do 
            combined[j] = {}
            if (j~=self.index) then
                print("my j is", j)
                local intersect_j = flattened_plane.PlaneIntersectionLine.new(self.index,j)
                local flatIntersect, flatOcc, flatEmpt, x_mat, y_mat = intersect_j:getRotatedIntersectionAndOccupiedEmpties(1)
                if flatIntersect then
                    local cumO, cumE, cumUnknown,cumO2, cumE2, cumUnknown2 = self:getOccupiedUnknown(flatOcc, flatEmpt,flatIntersect)
                    --local newCum, newCumD = self:combineSmartly(flatIntersect, cumO, cumE, cumUnknown,cumO2, cumE2, cumUnknown2, 1, intersect_j, x_mat, y_mat)
                    --do some smart combination
                    local cum = {cumO, cumE, cumUnknown,cumO2, cumE2, cumUnknown2}
                    local newCum, newCumD = self:combineAll(flatIntersect, cum, 1, intersect_j, x_mat, y_mat) 
                    local flat1, flatd1,startX,startY, endX, endY = intersect_j:convertIntersectLineToPlane(1, 1, newCum[1], newCumD)

                    for i=1,6 do
                        combined[j][i] = flattened_plane.Line.new(nil,startX,startY, endX, endY,{flat1, flatd1[i]})
                    end
                    local flatIntersect2, flatOcc2, flatEmpt2, x_mat, y_mat = intersect_j:getRotatedIntersectionAndOccupiedEmpties(2)
                    if flatIntersect2 then
                        local cumOP2, cumEP2, cumUnknownP2,cumO2P2, cumE2P2, cumUnknown2P2 = self:getOccupiedUnknown(flatOcc2, flatEmpt2,flatIntersect2)
                        cum = {cumOP2, cumEP2, cumUnknownP2,cumO2P2, cumE2P2, cumUnknown2P2}
                        local newCum2, newCumD2 = self:combineAll(flatIntersect2, cum, 2, intersect_j, x_mat, y_mat) 
                        local flat1, flatd1, startX,startY, endX, endY = intersect_j:convertIntersectLineToPlane(1, 2, newCum2[1], newCumD2)

                        for i=1,6 do                
                            if(flat1) then
                                combined[j][6+i] = flattened_plane.Line.new(nil,startX,startY, endX, endY,{flat1, flatd1[i]})
                            end
                        end            
                    end
                end
                collectgarbage()
            end
        end
        self.allIntersectionsInfo = combined
        self:saveMe()
    end
    return self.allIntersectionsInfo

end
function FlattenedPlane:combineAll(flatIntersect, cum, j, intersect_j, x_mat, y_mat)
    local temp,val = flatIntersect:sum(2):squeeze():max(1)
    local flatIntersectD_t = torch.zeros(flatIntersect:size())
    local flatIntersectD = {}
    for i =1, table.getn(cum) do
        temp = flatIntersectD_t:clone()
        temp[val:squeeze()]  = cum[i]
        flatIntersectD[i] = temp[flatIntersect:byte()]
    end
    local newCum, newCumD = intersect_j:sendRotateScoreBackToPlane(j, flatIntersect:byte(), flatIntersectD, x_mat, y_mat)
    return newCum, newCumD

end

function FlattenedPlane:combineSmartly(flatIntersect, cumO, cumE, cumUnknown,cumO2, cumE2, cumUnknown2, j, intersect_j, x_mat, y_mat)
    local temp,val = flatIntersect:sum(2):squeeze():max(1)
    local flatIntersectD = torch.zeros(flatIntersect:size())
    local combinedDir1= torch.ge(torch.ge(cumO,cumE+11)+torch.eq(torch.le(cumE,10) + torch.le(cumO,10),2),1):double() --not enough information
    local combinedDir2 = torch.ge(torch.ge(cumO2,cumE2+11)+torch.eq(torch.le(cumE2,10) + torch.le(cumO2,10),2),1):double() --not enough information
    flatIntersectD[val:squeeze()]  = torch.eq(combinedDir1+combinedDir2, 2):double()
    local newCum, newCumD = intersect_j:sendRotateScoreBackToPlane(j, flatIntersect:byte(), flatIntersectD[flatIntersect:byte()], x_mat, y_mat)
    return newCum, newCumD

end

--looks at intersections on plane and the empties/occupied relationships
function FlattenedPlane:getOccupiedUnknown(flatOcc, flatEmpt, flatIntersect)
    local temp,locOfRow = flatIntersect:sum(2):squeeze():max(1)
    locOfRow = locOfRow:squeeze()
    local cum = torch.zeros(flatOcc:size(2)):int()
    local cumE = torch.zeros(flatOcc:size(2)):int()
    local cumO = torch.zeros(flatOcc:size(2)):int()

    local sub_matrix_o = flatOcc:sub(locOfRow,math.min(locOfRow+500, flatOcc:size(1)))
    local sub_matrix_e = flatEmpt:sub(locOfRow,math.min(locOfRow+500, flatOcc:size(1)))
    local sub_matrix_i = flatIntersect:sub(locOfRow,math.min(locOfRow+500, flatOcc:size(1)))

    local occS = torch.cumsum(sub_matrix_o:int(),1)
    local empS = torch.cumsum(sub_matrix_e:int(),1)
    --torch.cumsum(sub_matrix_e:int(),1)
    local a,loc =     (torch.gt(occS,10)*-1+1):min(1)
    loc = loc:squeeze()
    loc[torch.eq(loc,1)] = occS:size(1)-1
    cum = loc:int()
    temp = torch.range(1,loc:size(1))
    temp:apply(function(v) cumO[v]= occS[loc[v]][v] cumE[v]= empS[loc[v]][v] end)

    sub_matrix_o = util.torch.flipTB(flatOcc:sub(1, locOfRow))
    sub_matrix_e = util.torch.flipTB(flatEmpt:sub(1, locOfRow))
    sub_matrix_i = util.torch.flipTB(flatIntersect:sub(1, locOfRow))

    occS = torch.cumsum(sub_matrix_o:int(),1)
    empS = torch.cumsum(sub_matrix_e:int(),1)

    --now the other side!
    local cum2 = torch.zeros(flatOcc:size(2)):int()
    local cumE2 = torch.zeros(flatOcc:size(2)):int()
    local cumO2 = torch.zeros(flatOcc:size(2)):int()
    for j =1, math.min(locOfRow-math.max(locOfRow-300, 1)+1+5,locOfRow) do

        local i = locOfRow-j+1
        --print(i,j,locOfRow)
        local t = occS[j]
        local t2 = empS[j]

        local good_pts = torch.eq(torch.gt(t,10) + torch.eq(cum2,0),2)
        cum2[good_pts] = j-1
        cumO2[good_pts] = t[good_pts]
        cumE2[good_pts] = t2[good_pts]
        if(j==locOfRow-math.max(locOfRow-300, 1)+1) then
            local good_pts =  torch.eq(cum2,0)
            cumO2[good_pts] = t[good_pts]
            cumE2[good_pts] = t2[good_pts]
            cum2[good_pts] = j-1
        end
    end
    local cumUnknown = cum-cumO-cumE
    local cumUnknown2 = cum2-cumO2-cumE2

    return cumO, cumE, cumUnknown, cumO2, cumE2, cumUnknown2
end

--just ideas
function FlattenedPlane:segment_floating_objets()
     empties = torch.gt(self.iempties + self.iemptiesF,0)
    occupied = torch.gt(self.ioccupied + self.ioccupiedF,0)

    cc, cca, mst, msta, mstsegmcolor = self:getConnectedComponentsRelationship((empties+occupied*-1):double())
    counter = 0
    for k,v in pairs(cca) do
        counter = counter+1
    end
    counter = 0

        
    occupiedBlobs = {}
    cca=msta
    cc=mst
    for k,v in pairs(cca) do
        print(k)
        totsE = torch.eq(cc,k):cmul(empties):sum()
        totsO = torch.eq(cc,k):cmul(occupied):sum()
        if(totsO >= totsE and totsO/torch.eq(cc,k):sum() > 0.1 and torch.eq(cc,k):sum()>100) then
            print(k, totsE, totsO, torch.eq(cc,k):sum())
            counter = counter + 1
            numE = 0
            total = 0
            for k2,v2 in pairs(v) do
                if (torch.eq(cc,k2):cmul(empties):sum()/torch.eq(cc,k2):sum()<.1) then
                    numE = numE+1
                end
                total = total+1
--                print(torch.eq(cc,k2):cmul(empties):sum(), torch.eq(cc,k2):cmul(occupied):sum())
            end    
            if numE/total > 0 then
                print("there is more than one neighbors with no empties", k, torch.eq(cc, k):sum())
                occupiedBlobs[table.getn(occupiedBlobs)+1] = {k, false}
            else
                occupiedBlobs[table.getn(occupiedBlobs)+1] = {k, true}
            end
        end

    end
    tAlone = torch.zeros(empties:size())
    tNoAlone = torch.zeros(empties:size())
    for i=1, table.getn(occupiedBlobs) do
        if(occupiedBlobs[i][2]) then
            tAlone = tAlone + torch.eq(cc, occupiedBlobs[i][1]):double()
        else
            tNoAlone = tNoAlone + torch.eq(cc, occupiedBlobs[i][1]):double()
        end
    end
    image.display(tAlone)
    image.display(tNoAlone)
    print(counter)
end

--just ideas
function FlattenedPlane:getBoundaryOptions()
     empties = torch.gt(self.iempties + self.iemptiesF,0)
    occupied = torch.gt(self.ioccupied + self.ioccupiedF,0)

    cc, cca, mst, msta, mstsegmcolor = self:getConnectedComponentsRelationship((empties+occupied*-1):double())
    counter = 0
    for k,v in pairs(cca) do
        counter = counter+1
    end
    print(counter)
    counter = 0
        
    bestSoFarV = 0
    bestSoFarK = 0
    for k,v in pairs(msta) do
        totsO = torch.eq(mst,k):cmul(occupied):sum()
        if(totsO > bestSoFarV) then
            bestSoFarV = totsO
            bestSoFarK = k
        end

    end
    biggestO =  torch.eq(mst,bestSoFarK):cmul(occupied)

    intersections = self:getAllIntersections()
    combinedI = torch.zeros(table.getn(intersections), self.ioccupied:size(1), self.ioccupied:size(2))
    good = torch.zeros(table.getn(intersections)):byte()
    for i =1,table.getn(intersections) do
        if(intersections[i]) then
            combinedI[i] = intersections[i]:getLineImage()
            good[i]=1
        end
    end


    intersections = self.wooohoo
    combinedI = torch.zeros(table.getn(intersections), self.ioccupied:size(1), self.ioccupied:size(2))
    good = torch.zeros(table.getn(intersections)):byte()
    for i =1,table.getn(intersections) do
        if(intersections[i][1]) then
            t1, t2 = intersections[i][2]:getLineImage()
            combinedI[i] = t2
            good[i]=1
        end
    end


    good_values = torch.range(1, good:size(1))[good]
        cc, cca, mst, msta, mstsegmcolor = self:getConnectedComponentsRelationship(torch.gt(combinedI:sum(1):squeeze(),0):double())

        combined = torch.zeros(self.ioccupied:size()):byte()
    for k,v in pairs(msta) do
        totsO = torch.eq(mst,k):sum()
        if(totsO > 1000) then
            print(k)
            combined = combined + torch.eq(mst, k)
        end

    end
end

function FlattenedPlane:growEmptiesFromCC()
    log.tic()
    local empties = torch.gt(self.iempties + self.iemptiesF,0)
    local occupied = torch.gt(self.ioccupied + self.ioccupiedF,0)

    local largest = self:getLargestCC()
    local disOccupied, disOccupiedLabels = opencv.imgproc.distanceTransformLabels(opencv.Mat.new(largest*-1+1))
    disOccupied = disOccupied:toTensor()
    disOccupiedLabels = disOccupiedLabels:toTensor()
    --local unique_vals = util.torch.unique(disOccupiedLabels:clone():cmul(largest:int()*-1+1):reshape(largest:size(1)*largest:size(2)))

    local good = largest:clone()
    local emptiesOnTheWay = torch.zeros(largest:size())
    local neigborEqual = torch.zeros(9, good:size(1), good:size(2)):byte()
    local closetsPoint = torch.zeros(9, good:size(1), good:size(2)):float()
    local kernel = torch.zeros(9,3,3)
    for i =1,9 do
        local toConvolve = torch.zeros(9)
        toConvolve[i]=1
        toConvolve = toConvolve:reshape(3,3)
        kernel[i] = toConvolve:clone()
        closetsPoint[i] =  torch.conv2(disOccupied, toConvolve:float(),'F'):sub(2,-2,2,-2)
        toConvolve = torch.zeros(9)
        toConvolve[i]=1
        toConvolve[5] = -1            
        toConvolve = toConvolve:reshape(3,3)
        neigborEqual[i] = torch.eq(torch.conv2(disOccupiedLabels, toConvolve:int(),'F'):sub(2,-2,2,-2),0)
    end
    closetsPoint[neigborEqual*-1+1]=math.huge
    local temp, closest_pt = closetsPoint:min(1)
    closest_pt = closest_pt:squeeze()
    local numEmpties = torch.zeros(9, good:size(1), good:size(2))
    local emptiesPic = torch.zeros(9, good:size(1), good:size(2)):byte()
    for i=1,9 do
        emptiesPic[i] = torch.eq(closest_pt, i)
    end
    local counter = 0
    print(log.toc())
    while(torch.eq(good,0):sum() ~=0) do
        print(torch.eq(good,0):sum())
        good = torch.gt(torch.conv2(good, torch.ones(3,3):byte(),'F'):sub(2,-2,2,-2),0)
        local numEmpties =  torch.conv3(kernel, emptiesOnTheWay:reshape(1,emptiesOnTheWay:size(1),emptiesOnTheWay:size(2)):double(),'F'):sub(1,-1,2,-2,2,-2)
        local emptiesFun = torch.zeros(numEmpties:size())
        emptiesFun[emptiesPic] = numEmpties[emptiesPic]
        emptiesFun = emptiesFun:sum(1):squeeze()
        emptiesOnTheWay[good:clone():cmul(largest*-1+1)] = (empties[good:clone():cmul(largest*-1+1)]:double()+emptiesFun[good:clone():cmul(largest*-1+1)]):double()
        if(counter%50 == 0) then
            collectgarbage()
        end
        counter = counter+1
    end
    print(log.toc())
    return emptiesOnTheWay
end
function FlattenedPlane:getLargestCC()
    local empties = torch.gt(self.iempties + self.iemptiesF,0)
    local occupied = torch.gt(self.ioccupied + self.ioccupiedF,0)

    local cc, cca, mst, msta, mstsegmcolor = self:getConnectedComponentsRelationship((empties+occupied*-1):double())        
    local bestSoFarV = 0
    local bestSoFarK = 0
    for k,v in pairs(msta) do
        local totsO = torch.eq(mst,k):cmul(occupied):sum()
        if(totsO > bestSoFarV) then
            bestSoFarV = totsO
            bestSoFarK = k
        end

    end
    return torch.eq(mst,bestSoFarK):cmul(occupied)
end

function FlattenedPlane:getConnectedComponentsRelationship(inputimg)
    local imgraph = require "../imgraph/init.lua"
    -- (1) build a graph on an input image
    local graph = imgraph.graph(inputimg,8)
    local cc = imgraph.connectcomponents(graph, 0.8, false)
    local mstsegm = imgraph.segmentmst(graph, 3, 20)
    local mstsegmcolor = imgraph.colorize(mstsegm)

    return cc, imgraph.adjacency(cc), mstsegm, imgraph.adjacency(mstsegm), mstsegmcolor

end
