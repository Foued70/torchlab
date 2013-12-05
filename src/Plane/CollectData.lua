local path = require 'path'
local pcl = PointCloud.PointCloud
local io = require 'io'

FlattenedPlane       = Class()
FlattenedPlane.PLANE = "FlattenedPlanes3"

function FlattenedPlane:__init(pc_pathOrObject, planes_pathOrObject, out_dir)
   -- Arg?
   if not pc_pathOrObject then 
      error("must pass a pointcloud object or path to pointcloud data")
   elseif type(pc_pathOrObject) == 'string' then
      -- Is a path:
      self.pc = pcl.new(pc_pathOrObject) 
   elseif type(pc_pathOrObject) == 'table' then
      -- Is a pointcloud object:
      self.pc = pc_pathOrObject 
   end
   -- Arg?
   if not planes_pathOrObject then 
      error("must pass a table of planes or path to serialized table of planes")
   elseif type(planes_pathOrObject) == 'string' then
      -- Is a path:
      self.eqns = torch.load(planes_pathOrObject) 
   elseif type(planes_pathOrObject) == 'table' then
      -- Is a planes list:
      self.eqns = planes_pathOrObject 
   end
   
   self.out_dir = out_dir or "output/"
   
   self.resolution = 10
   self.index = 1
   self.transformation = transf or torch.eye(4,4)
   self.thresh = .05
   self:update_plane(self.index)
end

function FlattenedPlane:update_plane(index)
   self.index = index
   self:setupQuatToFlatten()
   self:addPlane(true)
end

function FlattenedPlane:get_xyz_coordinates()
    local minT, maxT = self:calculateMinAndMaxT()
    local flat, t, x_mat, y_mat = FlattenedPlane.flattened2Image(torch.zeros(1,2), minT, maxT)
    local eq_new = flattened_plane.PlaneIntersectionLine.getRotatedEquation(self:getPlaneEquation(self.index), 
                                                                            self.quat)
    local x = x_mat:reshape(x_mat:size(1)*x_mat:size(2))*self.resolution --add min??
    local y = y_mat:reshape(y_mat:size(1)*y_mat:size(2))*self.resolution
    local z = (x*eq_new[1]+y*eq_new[2]+eq_new[4])/-eq_new[3]
    local unrotate_pts = geom.quaternion.rotate(geom.quaternion.inverse(self.quat), 
                                                torch.cat(torch.cat(x,y ,2),z,2))
    local pts_test = PointCloud.PointCloud.get_global_from_2d_pts(unrotate_pts,torch.inverse(self.transformation)) 
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
    --local quat2 = geom.quaternion.from_euler_angle(angle_az)
    self.quat = quat1 --geom.quaternion.product(quat2, quat1)
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
        'allIntersections', 'allIntersectionsInfo', 'transformation'
}
end

function FlattenedPlane:getAllPlaneEquations(plane_file)
   if (not self.eqns) or plane_file then
      eqns = torch.load(plane_file)
      table.sort(eqns, function(a,b) return a.n_pts>b.n_pts end)
      self.eqns = eqns
   end
   return self.eqns
end

function FlattenedPlane:getPlaneEquation(k)
   k = k or self.index
   local eqn = self:getAllPlaneEquations()[k].eqn
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

function FlattenedPlane:addPlane(redo)
   local pc = self.pc
   --to do check argument type
   local H = self.transformation
   local eqn = self:getPlaneEquation(self.index)
   
   local plane = self:getPlaneViewFromCenter(eqn, H)
   local center = H:sub(1,3,4,4):squeeze()
   local planed = plane - center:reshape(3,1,1):repeatTensor(1,plane:size(2),plane:size(3))
   local plane_reald = pc:get_depth_map()
   pc:set_pose_from_rotation_matrix(H)
   local plane_norm = pc:get_global_normal_map_H(H)
   local index, maskI = pc:get_index_and_mask()
   local plane_norm_reshaped = plane_norm:reshape(3,plane_norm:size(2)*plane_norm:size(3)):t()
   local good_norm = torch.lt(torch.acos(plane_norm_reshaped:clone()*eqn:sub(1,3):squeeze()),math.rad(45)):squeeze():reshape(plane_norm:size(2),plane_norm:size(3))
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
   
   temp = torch.zeros(maskN:size()):long()
   temp[maskN*-1+1] = indices
   --local occupiedI = torch.eq(torch.le(torch.abs(planed:clone():norm(2,1):squeeze()-plane_reald), self.thresh*1000)+mask+good_norm,3)
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
    
    self:addAllFrustrumsAndEmpties(redo)
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

function FlattenedPlane:getPlaneViewFromCenter(eqn,H)
   local pc = self.pc
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
function FlattenedPlane:addArbitraryFrustum(cornersI)  

    if(cornersI:sum() > 0 ) then
        local minT, maxT = self:calculateMinAndMaxT()
        local t = torch.conv2(self.notnilsI, torch.Tensor({0,1,0,1,1,1,0,1,0}):reshape(3,3):byte())
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
            local emptiesF, emptiesFD = 
               opencv.imgproc.fillQuadAllWithInterpolation(opencv.Mat.new(cum),coord)
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
        self.ioccupied, self.ioccupiedF, self.occupiedFD = self:addArbitraryFrustum(self.occupiedI)
        if(self.occupiedI:sum()>0) then
            local occupied_dis = opencv.imgproc.distanceTransform(opencv.Mat.new((self.ioccupied*1-1):byte())):toTensor():double()
            if(self.ioccupiedF) then
                self.ioccupiedFP = self.ioccupied:double() + torch.gt(self.ioccupiedF,0):double():clone():cmul((occupied_dis/5+1):pow(-1))
            end
        end
        self.iempties, self.iemptiesF, self.iemptiesFD = self:addArbitraryFrustum(self.emptiesI)
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
        
       -- plane:addAllFrustrumsAndEmpties()
        --a,b,c = plane:getCornersSonia()
        --plane:addArbitraryFrustum()
    end
    --image.save("iempties" .. plane_num .. ".png", plane.iempties:double())
    --image.save("ioccupied" .. plane_num .. ".png", plane.ioccupied:double())

    return plane
end

--[[
    scan = align_floors_endtoend.Scan.new("/Users/stavbraun/Desktop/play/motor-unicorn-0776_newsonia")
    forest =  scan:organize_in_trees()         
    tree = forest.tree_list.sweep_001
    k="sweep_001"
    plane_num=2 plane_t = align_floors_endtoend.FlattenedPlane.new("/Users/stavbraun/Desktop/play/motor-unicorn-0776_newsonia",k, forest.tree_list.sweep_001, plane_num)
    a, d_real, d_interpolated = plane_t:doFunStuff()


    j=20 image.display(torch.lt(torch.abs(d_interpolated-d_real),j):cmul(torch.ne(d_interpolated,-1)))
    FlattenedPlane = align_floors_endtoend.FlattenedPlane
    self = plane_t



    i=20 our_img = torch.lt(torch.abs(d_interpolated-d_real),i):cmul(torch.ne(d_interpolated,-1))
]]
--[[
function FlattenedPlane:getSegmentation()
    thresh = torch.Tensor({.005, .01, .03, .05, .07, .09, .11, .13, .15, .2})
    --for j=1,10  do
    self.thresh = thresh[4]
    self:addAllPlanes(self.sweep_node:getSweep(), self.sweep_node:getTransformationToRoot())
    d_interpolated, d_real = self:getDistanceMap(true)
    self:addAllFrustrumsAndEmpties(true)
    a,b,c = self:getCornersSonia()
    self:addArbitraryFrustum()

    probO = self.ioccupiedFP:clone()
    probE = self.iemptiesFP:clone()
    
    empties = torch.gt(self.iempties+ self.iemptiesF, 0)
    occupied = torch.gt(self.ioccupied+ self.ioccupiedF, 0)
    unknown = torch.eq(empties+occupied, 0)    
    probO[unknown] = .5
    probE[unknown] = .5

    t = torch.zeros(5,self.ioccupied:size(1), self.ioccupied:size(2))
    dmask = torch.eq(d_interpolated,-1)
    d_real[dmask] = -1
    disE  = d_interpolated-d_real
    disE[occupied] = dis[empties]:max()
    disE[unknown] = dis[empties]:min()

    disO  = d_interpolated-d_real
    disO[empties] = dis[occupied]:min()
    disO[occupied*-1+1] = 0

    --t[1] = disE/disE:max()
    --t[2] = disO/disO:max()

    t[2] = probO/10
    t[3] = probE/10
    t[1]=   torch.gt(self:lookAtIntersect(), 0)
    t[4] = (unknown+occupied*2):double()/2*10
    --t[5] = torch.gt(self.isonia+self.isoniaF, 0):double()
    mstsegmcolor, mstsegm, cc, imgraph = FlattenedPlane.getImgraphFun(t)
    image.display(cc)
    
    mt = imgraph.adjacency(mstsegm)
     d = d_interpolated-d_real
    k = 549423
    torch.eq(mstsegm, k):double():cmul(d)
    for k2,v in pairs(mt[k]) do
        print(k2)
        print(v)
    end
    counter = 0
    for k,v in pairs(mt) do
        counter = counter + 1
    end
    print(counter)
    for k,v in pairs(mt) do
        tots = torch.eq(mstsegm,k):sum()
        if(tots>25000) then
            print(k,tots)
        end
    end    
end

function FlattenedPlane.getImgraphFun(inputimg)
    local imgraph = require "../imgraph/init.lua"

    -- (1) build a graph on an input image
    local inputimgg = inputimg -- image.convolve(inputimg, image.gaussian(3), 'same')
    local graph = imgraph.graph(inputimgg)
    --image.display{image=inputimg, legend='input image'}

    -- (2) compute its connected components, and mst segmentation
    cc = imgraph.connectcomponents(graph, 0.1, true)
    mstsegm = imgraph.segmentmst(graph, 3, 20)
    mstsegmcolor = imgraph.colorize(mstsegm)

    return mstsegmcolor, mstsegm, cc, imgraph
end
]]--
function FlattenedPlane:saveAllToWebsite(filename)
    local s = "using sweeps "
    for k,v in pairs(self.rempties) do
        s = s .. k .. " and "
    end
    s = s:sub(1,#s-4)
    local html_doc = align_floors_endtoend.CreateHtml.new(filename, "output from Plane.lua" .. self.sweep_name,"output for equation from sweep_001 and plane " .. self.sweep_name .. " " .. s)
    html_doc:beginFile()
    self:concatAll()
    html_doc:addImageFromTensor("empties", self.iempties, "images/empties.png", "empties")
    html_doc:addImageFromTensor("occupied", self.ioccupied, "images/occupied.png", "occupied")
    html_doc:addImageFromTensor("corners", self.icorners, "images/corners.png", "occupied")
    html_doc:addImageFromTensor("connecs", self.iconnecs, "images/connecs.png", "occupied")

    html_doc:endFile()
    html_doc:close_file()
    gnuplot.closeWindow(1)

end

function FlattenedPlane.flattened2Image(flattenedxy, minT, maxT, dis)
    flattenedxy = flattenedxy-torch.repeatTensor(minT, flattenedxy:size(1), 1)+1    
    local size_us = (maxT:ceil()-minT:floor()+1):reshape(2)
    local combined = torch.zeros(size_us[1], size_us[2]):byte()
    local combinedD = torch.ones(size_us[1], size_us[2])*(math.huge)
    for i = 1, flattenedxy:size(1) do
        if(dis) then
            combinedD[flattenedxy[i][1]][flattenedxy[i][2]]=math.min(combinedD[flattenedxy[i][1]][flattenedxy[i][2]],dis[i])
        end
        combined[flattenedxy[i][1]][flattenedxy[i][2]]=1
    end
    local x_vals = torch.range(minT[1][1],maxT[1][1]):repeatTensor(size_us[2],1):t()
    local y_vals = torch.range(minT[1][2],maxT[1][2]):repeatTensor(size_us[1],1)
    combinedD_new = torch.zeros(combined:size())
    combinedD_new[combined] = combinedD[combined]
    return combined, combinedD_new, x_vals, y_vals
end

function FlattenedPlane:getAllIntersections()
    combined = {}
    for j=1,#self:getAllPlaneEquations() do
        if (j~=self.index) then
            print("my j is", j)
            local intersect_j = flattened_plane.PlaneIntersectionLine.new(self.index,j)
            local flatIntersect, temp, temp, coords = intersect_j:findIntersectionImageOnPlane(1)
            if(flatIntersect) then
                combined[j] = flattened_plane.Line.new(nil,coords[1],coords[2], coords[3], coords[4],{flatIntersect, flatIntersect})

            end
        end
    end
    self.allIntersections = combined
    self:saveMe()
    return self.allIntersections

end

function FlattenedPlane:getGoodIntersections(recalc)
    local base_dir = path.join(self.base_dir, FlattenedPlane.PLANE, "for_marco")
    util.fs.mkdir_p(base_dir)
    --if(not(self.plane_interesection_data) or recalc) then
        range_min = 1
        range_max =  #self:getAllPlaneEquations()
        combined = {}
        for j=range_min, range_max do --#self:getAllPlaneEquations() do
            combined[j] = {}
            --combined = torch.zeros(12, self.ioccupied:size(1), self.ioccupied:size(2))
            if (j~=self.index) then
                print("my j is", j)
                local intersect_j = flattened_plane.PlaneIntersectionLine.new(self.index,j)
                local flatIntersect, flatOcc, flatEmpt, x_mat, y_mat = intersect_j:getRotatedIntersectionAndOccupiedEmpties(1)
                if flatIntersect then
                    local cumO, cumE, cumUnknown,cumO2, cumE2, cumUnknown2 = self:temp(flatOcc, flatEmpt,flatIntersect)
                    --local newCum, newCumD = self:combineSmartly(flatIntersect, cumO, cumE, cumUnknown,cumO2, cumE2, cumUnknown2, 1, intersect_j, x_mat, y_mat)
                    --do some smart combination
                    local cum = {cumO, cumE, cumUnknown,cumO2, cumE2, cumUnknown2}
                    for i=1,6 do
                        local newCum, newCumD = self:combineOne(flatIntersect, cum[i], 1, intersect_j, x_mat, y_mat) 
                        local flat1, flatd1,startX,startY, endX, endY = intersect_j:convertIntersectLineToPlane(1, 1, newCum, newCumD)
                        combined[j][i] = flattened_plane.Line.new(nil,startX,startY, endX, endY,{flat1, flatd1})
                        
                        --torch.save("fun" .. j .. "_" .. i .. "_" .. self.index .. ".dat",flatd1 )
                    end
                    local flatIntersect2, flatOcc2, flatEmpt2, x_mat, y_mat = intersect_j:getRotatedIntersectionAndOccupiedEmpties(2)
                    if flatIntersect2 then
                        local cumOP2, cumEP2, cumUnknownP2,cumO2P2, cumE2P2, cumUnknown2P2 = self:temp(flatOcc2, flatEmpt2,flatIntersect2)
                        cum = {cumOP2, cumEP2, cumUnknownP2,cumO2P2, cumE2P2, cumUnknown2P2}
                        for i=1,6 do                
                            local newCum2, newCumD2 = self:combineOne(flatIntersect2, cum[i], 2, intersect_j, x_mat, y_mat) 
                            local flat1, flatd1, startX,startY, endX, endY = intersect_j:convertIntersectLineToPlane(1, 2, newCum2, newCumD2)
                            if(flat1) then
                                combined[j][6+i] = flattened_plane.Line.new(nil,startX,startY, endX, endY,{flat1, flatd1})
                            end
                            --torch.save("fun" .. j .. "_" .. 6+i .. "_" .. self.index .. ".dat",flatd1 )

                        end            

    --
      --                  local newCum2, newCumD2 = self:combineSmartly(flatIntersect2, cumOP2, cumEP2, cumUnknownP2,cumO2P2, cumE2P2, cumUnknown2P2, 2, intersect_j, x_mat, y_mat)
        --                local flat, flatd = intersect_j:convertIntersectLineToPlane(1, 2, newCum2, newCumD2)
          --              flatd1 = torch.eq(flatd1 + flatd,2)
                    end
                end
                --torch.save(path.join(base_dir, "intersection_of_plane_" .. self.index .. "_with_plane_" .. j .. "_occ_empty_unknown_info.dat"),combined )
                collectgarbage()
            end
        end
            self.allIntersectionsInfo = combined
            self:saveMe()
        return combined
        --self.plane_interesection_data = combined
        --self:saveMe()
    --end
    
    --return self.plane_interesection_data

end
function FlattenedPlane:combineOne(flatIntersect, cum, j, intersect_j, x_mat, y_mat)
    local temp,val = flatIntersect:sum(2):squeeze():max(1)
    local flatIntersectD = torch.zeros(flatIntersect:size())
    flatIntersectD[val:squeeze()]  = cum
    local newCum, newCumD = intersect_j:sendRotateScoreBackToPlane(j, flatIntersect:byte(), flatIntersectD[flatIntersect:byte()], x_mat, y_mat)
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
function FlattenedPlane:temp(flatOcc, flatEmpt, flatIntersect)

    local temp,locOfRow = flatIntersect:sum(2):squeeze():max(1)
    locOfRow = locOfRow:squeeze()
    local cum = torch.zeros(flatOcc:size(2)):int()
    local cumE = torch.zeros(flatOcc:size(2)):int()
    local cumO = torch.zeros(flatOcc:size(2)):int()

    for i =locOfRow+5,math.min(locOfRow+500, flatOcc:size(1)) do
        local t = flatOcc:sub(locOfRow, i, 1,-1):int():sum(1):squeeze()
        local t2 = flatEmpt:sub(locOfRow, i, 1,-1):int():sum(1):squeeze()

        local good_pts = torch.eq(torch.gt(t,10) + torch.eq(cum,0),2)
        cum[good_pts] = i-locOfRow
        cumO[good_pts] = t[good_pts]
        cumE[good_pts] = t2[good_pts]

        if (i==math.min(locOfRow+500, flatOcc:size(1)) ) then --set all the other values
            local good_pts =  torch.eq(cum,0)
            cumO[good_pts] = t[good_pts]
            cumE[good_pts] = t2[good_pts]
            cum[good_pts] = i-locOfRow
        end
    end

    --now the other side!
    local cum2 = torch.zeros(flatOcc:size(2)):int()
    local cumE2 = torch.zeros(flatOcc:size(2)):int()
    local cumO2 = torch.zeros(flatOcc:size(2)):int()
    for j =1, math.min(locOfRow-math.max(locOfRow-300, 1)+1+5,locOfRow) do

        local i = locOfRow-j+1
        --print(i,j,locOfRow)
        local t = flatOcc:sub(i, locOfRow, 1,-1):int():sum(1):squeeze()
        local t2 = flatEmpt:sub(i, locOfRow, 1,-1):int():sum(1):squeeze()

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

--size must be odd
function FlattenedPlane.getKernelWithSlope(slope, ksize, flip)
    local middle = (ksize-1)/2+1
    local kernel = torch.zeros(ksize,ksize)
    local combined
    local negSlope = false
    if(slope < 0) then
        slope = -slope
        negSlope = true
    end

        --lower right corner
    if(slope>=1) then
        y_linspace = torch.linspace(middle, ksize, middle)
        x_linspace = torch.linspace(middle, middle+(ksize-middle)*1/slope, middle)
    else
        x_linspace = torch.linspace(middle, ksize, middle)
        y_linspace = torch.linspace(middle, middle+(ksize-middle)*slope, middle)
    end

    if(negSlope) then --flip y over axis
        y_linspacer = torch.range(1,x_linspace:size(1))
        y_linspacer:apply(function(v) 
            x_linspace[v] = ksize - x_linspace[v]+1
        end)
    end
    if(flip) then
        y_linspacer = torch.range(1,x_linspace:size(1))
        y_linspacer:apply(function(v) 
            y_linspace[v] = ksize - y_linspace[v]+1
        end)
        x_linspacer = torch.range(1,x_linspace:size(1))
        x_linspacer:apply(function(v) 
            x_linspace[v] = ksize - x_linspace[v]+1
        end)
    end
    combined = torch.cat(x_linspace,y_linspace,2) 
    temp = torch.range(1,combined:size(1))
    for counter = 1,2 do
        temp:apply(function(i) if(combined[i][counter]+.5>=math.ceil(combined[i][counter])) then combined[i][counter] = math.ceil(combined[i][counter]) else  combined[i][counter] = math.floor(combined[i][counter]) end end)
    end
    kernel = FlattenedPlane.flattened2Image(combined, torch.Tensor({1,1}):reshape(1,2),torch.Tensor({ksize,ksize}):reshape(1,2))
    return kernel
end

function FlattenedPlane:getSegmentationOnOccupied()
    local empties, emptiesD, occupied, occupiedD
    local counter=1
    for k,v in pairs(self.iempties) do
        if(self.iempties[k]) then
            self.iemptiesF[k] = self.iemptiesF[k] or torch.zeros(self.iempties[k]:size())           
            if not(empties) then
                empties = torch.gt(self.iempties[k] + self.iemptiesF[k], 0)
            else
                empties = torch.cat(empties, torch.gt(self.iempties[k]+self.iemptiesF[k],0),3)
            end

        end
        if(self.ioccupiedF[k]) then
            self.ioccupiedF[k] = self.ioccupiedF[k] or torch.zeros(self.ioccupied[k]:size())           

            if not(occupied) then
                occupied = torch.gt(self.ioccupied[k]+self.ioccupiedF[k],0)
            else
                occupied = torch.cat(occupied, torch.gt(self.ioccupied[k]+self.ioccupiedF[k],0), 3)
            end
        end
    end

    local occupiedI = torch.ge(occupied:clone():sum(3):squeeze(),empties:clone():sum(3):squeeze()):cmul(torch.gt(occupied:clone():sum(3):squeeze(),0)):double()
    local emptiesI = torch.lt(occupied:clone():sum(3):squeeze(),empties:clone():sum(3):squeeze()):cmul(torch.gt(empties:clone():sum(3):squeeze(),0)):double()

    local inputimg = occupiedI-emptiesI

    local t = torch.zeros(3,inputimg:size(1), inputimg:size(2))
    inputimg = inputimg - inputimg:min()
    inputimg = inputimg/inputimg:max()

    t[1] = inputimg
    t[2] = torch.gt(self:getIntersectionLines(),0):double()/5
    --t[2]= torch.gt(empties_dis,occupied_dis):double()*2--:max()
    --t[3] = empties_dis/empties_dis:max()
    local mstsegmcolor, watershed, mstsegm, watershedgraph, watershedcc = Plane.getImgraphFun(t)
--    image.display(mstsegm)
--    image.display(watershedgraph)
--    image.display(watershed+inputimg*-1)

    local all_zeros = torch.zeros(mstsegm:max())
    all_zeros[mstsegm:reshape(mstsegm:numel()):long()]=1
    local uniquei = torch.range(1,all_zeros:numel())[all_zeros:byte()]

    local tempO = torch.zeros(mstsegm:size())
    local tempE = torch.zeros(mstsegm:size())
    print("num segments looking at is " .. uniquei:numel())
    for u=1,uniquei:numel() do
        mySegment = torch.eq(mstsegm, uniquei[u])
        numOccupied = mySegment:double():cmul(occupiedI):sum()
        numEmpties = mySegment:double():cmul(emptiesI):sum()

        print(numOccupied, numEmpties, numOccupied/(numOccupied+numEmpties))
        if(numOccupied/(numOccupied+numEmpties) > .6) then
            if(numOccupied/mySegment:sum() > .1) then
                print("good occupied")
                tempO = tempO + mySegment:double()
            end
        elseif (numEmpties/(numOccupied+numEmpties) > .6) then
                            print("good empties")

            if(numEmpties/mySegment:sum() > .05) then
                tempE = tempE + mySegment:double()
            end

        end
            collectgarbage()
    end
    
    local mstsegmcolor2 = Plane.getImgraphFun(t[1])
    return tempO, tempE, mstsegmcolor, emptiesI, occupiedI, t[1], mstsegmcolor2
end

function FlattenedPlane:getSegmentationOnDistanceToPlane()
        k= "sweep_001"
        local emptiesI = self.emptiesI[k]
        local dis_to_plane = self.planed[k]:clone():norm(2,1):squeeze()-self.plane_reald[k]
        local emptiesD = dis_to_plane[emptiesI] 
        local occupiedD = dis_to_plane[self.occupiedI[k]]

        local inputimg = dis_to_plane:clone() --(planed:clone():norm(2,1):squeeze()-plane_reald)
        mask = torch.eq(torch.eq(self.plane_reald[k],0) + torch.eq(self.planed[k]:clone():norm(2,1),0),0)
        mask_new = torch.eq(mask+torch.gt(dis_to_plane, -.2*1000) + 
        torch.lt(dis_to_plane, .2*1000),3)
        inputimg = inputimg:cmul(mask_new:double())

        inputimg = inputimg - inputimg:min()
        inputimg = inputimg/inputimg:max()

        inputimg = inputimg:cmul(mask_new:double())
        mst, watershed = Plane.getImgraphFun(inputimg)
        return mst, watershed
end

function FlattenedPlane:getSegmentationOnDistanceToPlaneAndEmpties()
    local emptiesD, occupiedD
    for k,v in pairs(self.emptiesI) do
        if(v:sum()~=0) then
            local dis_to_plane = self.planed[k]:clone():norm(2,1):squeeze()-self.plane_reald[k]
            if not(emptiesD) then
                emptiesD = dis_to_plane[v]
            else
                emptiesD = torch.cat(emptiesD,dis_to_plane[v],1)
            end
        end
    end

    for k,v in pairs(self.occupiedI) do
        if(v:sum()~=0) then
            local dis_to_plane = self.planed[k]:clone():norm(2,1):squeeze()-self.plane_reald[k]
            if not(occupiedD) then
                occupiedD = dis_to_plane[v]
            else
                occupiedD = torch.cat(occupiedD,dis_to_plane[v],1)
            end
        end
    end

    local occupiedxyt = self.occupiedxy:sub(1,self.occupiedxy:size(1), 1,2)/self.resolution
    local emptiesxyt = self.emptiesxy:sub(1,self.emptiesxy:size(1), 1,2)/self.resolution

    local a,b,c,d,x_mat,y_mat, minT, maxT = self:concatAll()     
    local occupied, x_mat, y_mat = FlattenedPlane.flattened2Image(occupiedxyt, minT, maxT, occupiedD)
    local empties = FlattenedPlane.flattened2Image(emptiesxyt, minT, maxT)
    t = torch.zeros(3,self.ioccupied:size(1), self.ioccupied:size(2))
    t[1] = occupied
    t[2] = empties
    t[3] = self:getIntersectionLines()
    local inputimg = t:clone() --(planed:clone():norm(2,1):squeeze()-plane_reald)
    inputimg = inputimg - inputimg:min()
    inputimg = inputimg/inputimg:max()
    inputimg[1][torch.eq(torch.eq(occupied,0)+torch.eq(empties,0),2)]=.5
    inputimg[2][torch.eq(torch.eq(occupied,0)+torch.eq(empties,0),2)]=.5

    mstc, watershed, mst = Plane.getImgraphFun(inputimg)
    return mst, mstc, watershed
end

function FlattenedPlane:doFunStuff()
    require "gnuplot"
    local a=torch.zeros(50)
   local pts_explained = torch.zeros(50)

   -- local d_interpolated, d_real = self:getDistanceMap(true)
   -- local dmask = torch.eq(d_interpolated,-1)

   local d_interpolated = self.planed:clone():norm(2,1):squeeze()
   local d_real = self.plane_reald
    local mask = self.occupiedI
   local o
   imgraph = require "../imgraph/init.lua"

--[[
    for j=1,50 do
            d = d_interpolated-d_real
            d[torch.eq(self.plane:clone():norm(2,1):squeeze(),0)] = -j
        emptiesI = torch.lt(d, -j)
        occupiedI = torch.lt(torch.abs(d), j)
        empties = emptiesI:double():cmul(torch.abs(d))
        occupied = occupiedI:double():cmul(torch.abs(d))
                    empties[torch.eq(self.plane:clone():norm(2,1):squeeze(),0)] = 0
                    occupied[torch.eq(self.plane:clone():norm(2,1):squeeze(),0)] = 0

        we = emptiesI:sum()/(emptiesI:sum()+occupiedI:sum())
        wo = occupiedI:sum()/(emptiesI:sum()+occupiedI:sum())
        ue = empties:mean()
        uo = occupied:mean()
        score = we*wo*(uo-ue)^2
        print(j,score,we,wo,ue,uo)
    end
    --]]
    saliency = require "../image/saliency.lua"
    for i=1,a:size(1) do 
        if(i%10 == 0) then
            print(i)
        end
        local j=i 
        local our_img = torch.lt(torch.abs(d_interpolated-d_real),j):cmul(torch.ne(d_interpolated,-1)):cmul(mask)
        our_img[torch.eq(self.plane:clone():norm(2,1):squeeze(),0)] = 0
        --salient, sc = saliency.high_entropy_features{img=our_img:double(), kr=5, kc=5, nscale=1}

        --graph = imgraph.graph(our_img:double(), 8)

        
        --gradient = imgraph.graph2map(graph)
        --watershed = imgraph.watershed(gradient, 0.01, 8)
        --[[
        watershedgraph = imgraph.graph(watershed, 8)
        watershedcc = imgraph.connectcomponents(graph, 0.5, false)
        mt = imgraph.adjacency(watershedcc)
        local counter = 0
        local size_of_seg = {}
        for k,v in pairs(mt)  do
            counter = counter + 1
            --size_of_seg[counter] = torch.eq(watershedcc, k):sum()
        end--]]

        --[[
        local mat_test = (opencv.imgproc.CannyDetectEdges(opencv.Mat.new(our_img:clone()),1,1)) 
        local mat_test_Tensor = (mat_test:toTensor():double()):clone():contiguous()
        local testO = opencv.Mat.new(mat_test_Tensor)
        local test = opencv.imgproc.DFT(testO):toTensor() 
        mstsegmcolor, watershed, mstsegm, watershedgraph, watershedcc, imgraph = FlattenedPlane.getImgraphFun(our_img)

        a[i] =test:reshape(test:size(1)*test:size(2)):sort(true):sub(10,90):mean()
        ]]
        --salient, sc = saliency.high_entropy_features{img=our_img:double(), kr=2, kc=2, nscale=1}
       --edges = (opencv.imgproc.CannyDetectEdges(opencv.Mat.new(our_img:clone()),1,1)) 
       
       --[[
        test = opencv.imgproc.DFT(edges):toTensor() 

        a[i] = test:reshape(test:size(1)*test:size(2)):sort(true):sub(1,10):mean()
       --]]

       conv = torch.conv2(our_img,torch.ones(3,3):byte())
       a[i] = torch.eq(conv,1):sum()*5 +torch.eq(conv,2):sum()*5 +torch.eq(conv,3):sum()*5+torch.eq(conv,4):sum()+torch.eq(conv,5):sum()+torch.eq(conv,6):sum()


       --(opencv.imgproc.detectCornerHarris(opencv.Mat.new(our_img), 2,1,.04):toTensor()/255.0):sum() -- math.log(torch.gt(salient,1):sum())
        pts_explained[i] = our_img:sum()
        collectgarbage()
    end
    
    gnuplot.plot(a)
    gnuplot.figure()
    gnuplot.plot(pts_explained)

    return a, d_real, d_interpolated
end
-- for i =1,10 do plane_1 = flattened_plane.FlattenedPlane.get_ith_plane(i) end

function FlattenedPlane:segment_floating_objets()
     empties = torch.gt(self.iempties + self.iemptiesF,0)
    occupied = torch.gt(self.ioccupied + self.ioccupiedF,0)

    cc, cca, mst, msta, mstsegmcolor = self:getConnectedComponentsRelationship((empties+occupied*-1):double())
    counter = 0
    for k,v in pairs(cca) do
        counter = counter+1
    end
    print(counter)
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

function FlattenedPlane:getConnectedComponentsRelationship(inputimg)
    local imgraph = require "../imgraph/init.lua"
    -- (1) build a graph on an input image
    local graph = imgraph.graph(inputimg,8)
    local cc = imgraph.connectcomponents(graph, 0.8, false)
    local mstsegm = imgraph.segmentmst(graph, 3, 20)
    local mstsegmcolor = imgraph.colorize(mstsegm)

    return cc, imgraph.adjacency(cc), mstsegm, imgraph.adjacency(mstsegm), mstsegmcolor

end
