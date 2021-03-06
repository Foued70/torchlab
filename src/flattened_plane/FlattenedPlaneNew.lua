FlattenedPlaneNew = Class()
FlattenedPlaneNew.PLANE = "FlattenedPlaneNew"
local path = require 'path'
local pcl = pointcloud.pointcloud
local Sweep = align_floors_endtoend.Sweep
local io = require 'io'
function FlattenedPlaneNew:__init(pc, eqn, mask, transf)
    if(not(pc) or not(eqn)) then
        error("need pc and equation as input")
    end
    self.resolution = 10
    self.transformations = transf or {torch.eye(4), torch.eye(4)}
    self.maskP = mask or torch.ones(pc:get_depth_map():size())
    self.eqn = eqn

    self:setup_quat_to_flatten()
    self.thresh = .05
    self:add_plane(pc)
end

function FlattenedPlaneNew:get_xyz_coordinates()
    local minT, maxT = self:calculate_min_and_max_t()
    local flat, t, x_mat, y_mat = FlattenedPlaneNew.flattened2Image(torch.zeros(1,2), minT, maxT)
    local eq_new = flattened_plane.PlaneIntersectionLine.getRotatedEquation(self:get_our_plane_equation(self.index), self.quat)
    local x = x_mat:reshape(x_mat:size(1)*x_mat:size(2))*self.resolution --add min??
    local y = y_mat:reshape(y_mat:size(1)*y_mat:size(2))*self.resolution
    local z = (x*eq_new[1]+y*eq_new[2]+eq_new[4])/-eq_new[3]
    local unrotate_pts = geom.quaternion.rotate(geom.quaternion.inverse(self.quat), torch.cat(torch.cat(x,y ,2),z,2))
    local pts_test = (self.transformations[self.index]*torch.cat(unrotate_pts,torch.ones(points:size(1),1)):t()):t():sub(1,-1,1,3)
    return pts_test
end

function FlattenedPlaneNew:setup_quat_to_flatten()
    local eqn = self:get_our_plane_equation(self.index)
    local angle_az = geom.util.unit_cartesian_to_spherical_angles(eqn)
    angle_az = angle_az:index(1,torch.LongTensor({2,1}))

    --intersect of plane with 0/0
    local vec_on_intersect = torch.Tensor({1,-eqn[1]/eqn[2],0})
    vec_on_intersect = vec_on_intersect/vec_on_intersect:norm()
    local quat1 = geom.quaternion.from_axis_angle(vec_on_intersect,math.pi/2-angle_az[1])
    angle_az[1] = 0
    self.quat = quat1
end

function FlattenedPlaneNew.get_plane_equation(eqn, H)
    local quat =  geom.quaternion.from_rotation_matrix(H:sub(1,3,1,3))
    local n1 = geom.quaternion.rotate(quat, eqn:sub(1,3))
    local pt1,t,vec1 = FlattenedPlaneNew.find_vector(eqn)
    local pt1r = geom.quaternion.rotate(quat, pt1)
    pt1r = pt1r+H:sub(1,3,4,4)
    local d_new = -pt1r*n1
   return torch.cat(n1, torch.Tensor({d_new}))
end

function FlattenedPlaneNew:get_our_plane_equation()
    return FlattenedPlaneNew.get_plane_equation(self.eqn, self.transformations[2])
end
function FlattenedPlaneNew:add_plane(pc)
    --to do check argument type
    local H = torch.eye(4)
    H[{{1,3},{1,3}}]= geom.quaternion.to_rotation_matrix(geom.quaternion.from_rotation_matrix(self.transformations[1]))
    H[{{1,3},{4}}] = self.transformations[1]:sub(1,3,4,4)
    local eqn  = self:get_our_plane_equation()
    local plane, mask = FlattenedPlaneNew.get_plane_view_from_center(eqn, H, pc)
    local occupiedI = self.maskP
    local center = H:sub(1,3,4,4):squeeze()
    local planed = plane - center:reshape(3,1,1):repeatTensor(1,plane:size(2),plane:size(3))
    local dmsk, cmsk, imsk, nmsk = pc:get_valid_masks()
    nmsk = nmsk:cmul(mask)
    --local good_norm = torch.lt(torch.acos(plane_norm_reshaped:clone()*eqn:sub(1,3):squeeze()),math.rad(60)):squeeze():reshape(plane_norm:size(2),plane_norm:size(3))
    local plane_reald = pc:get_depth_list(torch.ones(nmsk:size()):byte())
    local points = pc:get_xyz_list_transformed(torch.ones(nmsk:size()):byte(), H)
    local plane_norms = pc:get_normal_list_transformed(torch.ones(nmsk:size()):byte(), H)
    occupiedI[nmsk*-1+1] = 0

    occupiedI = occupiedI:cmul(torch.lt(torch.abs(points*eqn:sub(1,3)+eqn:sub(4,4):squeeze()), self.thresh*1000):reshape(nmsk:size()))

    local occupied  = torch.Tensor()
    if(occupiedI:sum()>0) then
        occupied = util.torch.select3d(points,occupiedI)
    end
    --our hypothesized plane is more than 5 cm closer to us than the real point, it is going through empty!
    --[[ if want to project onto plane, instead of assume position is on plane ]]
    local emptiesI = torch.gt(points*eqn:sub(1,3)+eqn:sub(4,4):squeeze(), self.thresh*1000):reshape(nmsk:size()):cmul(occupiedI*-1+1):cmul(nmsk)
  --if want distance along ray we are shooting from camera
--    local emptiesI = torch.lt(planed:clone():norm(2,1):squeeze()-plane_reald, -self.thresh*1000):cmul(occupiedI*-1+1):cmul(nmsk)
    
    local empties  = torch.Tensor()
    if(emptiesI:sum()>0) then
        empties = util.torch.select3d(points,emptiesI)
    end
    
    
    local notnilsI = nmsk*-1+1
    local notnils = util.torch.select3d(plane,notnilsI)

    self.pc =pc
    self.rempties = torch.zeros(0,0)
    self.roccupied = torch.zeros(0,0)
    self.plane = plane
    self.planed = planed
    self.plane_reald = plane_reald
    self.points = points
    self.emptiesI = emptiesI
    self.occupiedI = occupiedI
    self.mask = nmsk
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
    
    --self:add_all_frustrum_and_empties()
    collectgarbage()
end

function FlattenedPlaneNew:saveToXYZ(occI, fname)
    self.pc:save_to_xyz_file(fname, nil, nil, nil, nil, nil, occI)
    collectgarbage()
end

function FlattenedPlaneNew.save(list, name)
    local file = io.open(name .. ".xyz", 'w')
    local tmpt = torch.range(1,list:size(1))                                                                                                                                
    tmpt:apply(function(ii) 
        file:write(''..list[ii][1]..' '..list[ii][2]..' '..list[ii][3] .. '\n')        
    end) 
    file:close()
end

function FlattenedPlaneNew.inverseof3x3(a,b,c,d,e,f,g,h,i, x_1, y_1, z_1)
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
function FlattenedPlaneNew.get_plane_view_from_center(eqn,H, pc)
    local center = torch.zeros(3)
    if(H) then
        center = H:sub(1,3,4,4):squeeze()
    end
    local vmsk       = pc:get_valid_masks()
    local transposedPts = pc:get_xyz_list_transformed(torch.ones(vmsk:size()):byte(),H)
    local pt1,t,vec1 = FlattenedPlaneNew.find_vector(eqn)
    local vec2 = torch.cross(vec1, eqn:sub(1,3))
    vec2 = vec2/vec2:norm()
    local leftSide = (center-pt1):reshape(3,1)
    local tmp = torch.range(1,vmsk:size(1)*vmsk:size(2))
    local tmppts = torch.zeros(vmsk:size(1)*vmsk:size(2),3)

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

    local x_new, y_new, z_new = FlattenedPlaneNew.inverseof3x3(a,b,c,d,e,f,g,h,i, x_1, y_1, z_1)
    --means we are shooting ray forwards and not backwards
    local good_index = torch.ge(z_new, 0)
    local temp_pt = center:reshape(1,3):repeatTensor(transposedPts:size(1),1)+mvdCenter:clone():cmul(z_new:repeatTensor(3,1):t())
    local good_index2 = torch.ones(good_index:size()):byte() 
    local xyz_minval, xyz_maxval, xyz_radius = pc:get_xyz_stats()
    good_index2 =torch.lt((temp_pt-center:reshape(1,3):repeatTensor(temp_pt:size(1),1)):norm(2,2):squeeze(), (xyz_radius:clone():norm()*2))
    local good_index3 = torch.eq(torch.lt(temp_pt:clone():sum(2), math.huge):double() + torch.gt(temp_pt:clone():sum(2),math.huge):double(),1):squeeze()
    local cum_good = torch.eq(good_index+good_index2+good_index3+vmsk,4)
    cum_good = (cum_good*-1+1)
    temp_pt:select(2,1)[cum_good]= 0
    temp_pt:select(2,2)[cum_good] = 0
    temp_pt:select(2,3)[cum_good] = 0
    local new_pts = temp_pt:t():reshape(3,vmsk:size(1), vmsk:size(2))
    return new_pts, cum_good*-1+1
end

function FlattenedPlaneNew.find_vector(eqn)
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

function FlattenedPlaneNew:add_arbitrary_frustrum(cornersI, onlyOnThisOne)  

    if(cornersI:sum() > 0 ) then
        local minT, maxT = self:calculate_min_and_max_t()
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
        local plane = self.points
        local eqn  = self:get_our_plane_equation()
        local dis_to_plane = plane*eqn:sub(1,3)+eqn:sub(4,4):squeeze()
        plane = plane:t():reshape(self.plane:size())
        local occupied = util.torch.select3d(plane:reshape(3,plane:size(2)*plane:size(3)):t(),cornersI)
        local roccupied= geom.quaternion.rotate(self.quat,occupied:contiguous())
        local d = dis_to_plane[cornersI]
        if(t_new:sum()==0) then
            local flat, flatD = FlattenedPlaneNew.flattened2Image((roccupied:sub(1,-1,1,2)/self.resolution), minT, maxT, d)
            return flat, flat, flatD, flatD
        end
        local occupiedUs = util.torch.select3d(plane:reshape(3,plane:size(2)*plane:size(3)):t(),t_new)
        local occupiedL = util.torch.select3d(plane:reshape(3,plane:size(2)*plane:size(3)):t(),leftSelectorN)
        local occupiedR = util.torch.select3d(plane:reshape(3,plane:size(2)*plane:size(3)):t(),rightSelectorN)
        local occupiedT = util.torch.select3d(plane:reshape(3,plane:size(2)*plane:size(3)):t(),topSelectorN)
        local occupiedB = util.torch.select3d(plane:reshape(3,plane:size(2)*plane:size(3)):t(),bottomSelectorN)
        if(occupiedL:dim()> 0 and occupiedUs:size(1)>1) then
            local roccupiedUs= geom.quaternion.rotate(self.quat,occupiedUs:contiguous())
            local roccupied= geom.quaternion.rotate(self.quat,occupied:contiguous())
            local roccupiedL= geom.quaternion.rotate(self.quat,occupiedL:contiguous())
            local roccupiedR= geom.quaternion.rotate(self.quat,occupiedR:contiguous())
            local roccupiedT= geom.quaternion.rotate(self.quat,occupiedT:contiguous())
            local roccupiedB= geom.quaternion.rotate(self.quat,occupiedB:contiguous())
            --local coord_filter = torch.eq(torch.eq(occupiedL:sum(2):squeeze(),0)+torch.eq(occupiedR:sum(2):squeeze(),0)+torch.eq(occupiedT:sum(2):squeeze(),0)+torch.eq(occupiedB:sum(2):squeeze(),0),0)

            
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
            local flat, flatD = FlattenedPlaneNew.flattened2Image((roccupied:sub(1,-1,1,2)/self.resolution), minT, maxT, d)
            local coord = torch.cat(torch.cat(torch.cat(torch.cat(roccupiedL, roccupiedT, 2), roccupiedR,2), roccupiedB,2), roccupiedUS,2)
                               

            local shouldFrustrumCoords = torch.range(1,coord:size(1))
            local good_coords = coord:index(1,shouldFrustrumCoords:long())
            local cum = torch.zeros(flat:size())
            local emptiesF, emptiesFD= opencv.imgproc.fillQuadAllWithInterpolation(opencv.Mat.new(cum),coord)
            return flat, torch.gt(emptiesF:toTensor(),0), emptiesFD:toTensor(), flatD
        end
    end
    collectgarbage()
end

function FlattenedPlaneNew.calculate_min_and_max(lua_table_of_vectors, minT, maxT)
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

function FlattenedPlaneNew:add_all_frustrum_and_empties(redo)
    print("add_all_frustrum_and_empties")
    if(redo or not(self.ioccupied) or not(self.iempties)) then
        self.ioccupied, self.ioccupiedF, self.ioccupiedFD, self.ioccupiedD = self:add_arbitrary_frustrum(self.occupiedI, true)
        if(self.occupiedI:sum()>0) then
            local occupied_dis = opencv.imgproc.distanceTransform(opencv.Mat.new((self.ioccupied*1-1):byte())):toTensor():double()
            if(self.ioccupiedF) then
                self.ioccupiedFP = self.ioccupied:double() + torch.gt(self.ioccupiedF,0):double():clone():cmul((occupied_dis/5+1):pow(-1))
            end
        end
        self.iempties, self.iemptiesF, self.iemptiesFD = self:add_arbitrary_frustrum(self.emptiesI, true)
        if(self.emptiesI:sum()>0) then
            local empties_dis = opencv.imgproc.distanceTransform(opencv.Mat.new((self.iempties*1-1):byte())):toTensor():double()
            if(self.iemptiesF) then
                self.iemptiesFP = self.iempties:double() + torch.gt(self.iemptiesF,0):double():clone():cmul((empties_dis/5+1):pow(-1))        
            end
        end
    end
end
function FlattenedPlaneNew:set_min_and_max(minV,maxV)
    self.minV =minV
    self.maxV = maxV
end

function FlattenedPlaneNew:calculate_min_and_max_t()
    if(not(self.minV)) then
        local minT,maxT = FlattenedPlaneNew.calculate_min_and_max(self.rempties)
        minT,maxT = FlattenedPlaneNew.calculate_min_and_max(self.roccupied,minT, maxT)
        
        minT = minT:sub(1,1,1,2)/self.resolution-200
        maxT =maxT:sub(1,1,1,2)/self.resolution+200
        self.minT = minT:floor()
        self.maxT = maxT:ceil()
        return self.minT, self.maxT
    else
        return self.minV, self.maxV
    end
end

function FlattenedPlaneNew:get_nils(recalc)
    if(not(self.inilsF) or not(self.iallD) or recalc) then
        in_range, index = self:get_in_range(self.rnotnils)
        in_range_i = self.notnilsI:clone()
        in_range_i[self.notnilsI] = index
        a,b,c = self:add_arbitrary_frustrum(in_range_i)
        self.inilsF = torch.gt(a+b,0)*-1+1
        self.iallD = c
        self:save_me()
    end
    return self.inilsF, self.iallD 
end
function FlattenedPlaneNew:get_in_range(rcorners)
    local minT, maxT = self:calculate_min_and_max_t()
    rcorners = rcorners/self.resolution
    good_index = torch.le(rcorners:sub(1,-1,1,1), maxT[1][1])+torch.le(rcorners:sub(1,-1,2,2), maxT[1][2]) +
        torch.ge(rcorners:sub(1,-1,1,1), minT[1][1])+torch.ge(rcorners:sub(1,-1,2,2), minT[1][2])
    good_index = torch.eq(good_index, 4)
    rcorners = util.torch.select3d(rcorners, good_index:squeeze():byte())*self.resolution
    return rcorners, good_index:squeeze():byte()

end

function FlattenedPlaneNew.flattened2Image(flattenedxy, minT, maxT, dis)
    flattenedxy = flattenedxy-torch.repeatTensor(minT, flattenedxy:size(1), 1)+1   
    local size_us = (maxT:ceil()-minT:floor()+1):reshape(2)
    local combined = torch.zeros(size_us[1]* size_us[2]):byte()
    local combinedD = torch.zeros(size_us[1]*size_us[2])
    flattenedxy:ceil()
    if(dis) then
        dis, order = dis:sort(true) --largest to smallest 
        flattenedxy = flattenedxy:index(1,order)
    end
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

function FlattenedPlaneNew:get_intersection_with(plane)
    local intersect_j = flattened_plane.PlaneIntersectionLine.new(self, plane)
    local flatIntersect, temp, temp, coords = intersect_j:findIntersectionImageOnPlane(1)
    local combined = flattened_plane.Line.new(nil,coords[1],coords[2], coords[3], coords[4],{flatIntersect, flatIntersect:double()})
    return combined
end

function FlattenedPlaneNew:get_occupied_on_lines(recalc)
    if(not(self.occupiedOnLines) or recalc) then
        range_min = 1
        range_max =  #self.planes
        combined = {}
        for j=range_min, range_max do 
            combined[j] = {}
            if (j~=self.index) then
                print("my j is", j)
                local intersect_j = flattened_plane.PlaneIntersectionLine.new(self.index,j)
                local flatIntersect, flatOcc, flatEmpt, x_mat, y_mat = intersect_j:getRotatedIntersectionAndOccupiedEmpties(1)
                if flatIntersect then
                    local cumO = self:get_occupied(flatOcc, flatIntersect)
                    --local newCum, newCumD = self:combine_smartly(flatIntersect, cumO, cumE, cumUnknown,cumO2, cumE2, cumUnknown2, 1, intersect_j, x_mat, y_mat)
                    --do some smart combination
                    local cum = {cumO}
                    local newCum, newCumD = self:combine_all(flatIntersect, cum, 1, intersect_j, x_mat, y_mat) 
                    local flat1, flatd1,startX,startY, endX, endY = intersect_j:convertIntersectLineToPlane(1, 1, newCum[1], newCumD)
                    combined[j][1] = flattened_plane.Line.new(nil,startX,startY, endX, endY,{flat1, flatd1[1]})


                    local flatIntersect2, flatOcc2, flatEmpt2, x_mat, y_mat = intersect_j:getRotatedIntersectionAndOccupiedEmpties(2)
                    if flatIntersect2 then
                        local cumOP2 = self:get_occupied(flatOcc2,flatIntersect2)
                        cum = {cumOP2}
                        local newCum2, newCumD2 = self:combine_all(flatIntersect2, cum, 2, intersect_j, x_mat, y_mat) 
                        local flat1, flatd1, startX,startY, endX, endY = intersect_j:convertIntersectLineToPlane(1, 2, newCum2[1], newCumD2)

                        combined[j][2] = flattened_plane.Line.new(nil,startX,startY, endX, endY,{flat1, flatd1[1]})
                    end

                end
                collectgarbage()
            end
        end
        self.occupiedOnLines = combined
        self:save_me()
    end
        return self.occupiedOnLines
    
end

function FlattenedPlaneNew:get_occupied(flatOcc, flatIntersect)
    local temp,locOfRow = flatIntersect:sum(2):squeeze():max(1)
    locOfRow = locOfRow:squeeze()
    local cum = torch.sum(flatOcc:int(),1)
    return cum
end

function FlattenedPlaneNew:get_good_intersections(save_dir, recalc)
    local base_dir = path.join(save_dir, FlattenedPlaneNew.PLANE, "for_marco")
    util.fs.mkdir_p(base_dir)
    if(not(self.allIntersectionsInfo) or recalc) then
        range_min = 1
        range_max =  #self.planes
        combined = {}
        for j=range_min, range_max do 
            combined[j] = {}
            if (j~=self.index) then
                print("my j is", j)
                local intersect_j = flattened_plane.PlaneIntersectionLine.new(self.index,j)
                local flatIntersect, flatOcc, flatEmpt, x_mat, y_mat = intersect_j:getRotatedIntersectionAndOccupiedEmpties(1)
                if flatIntersect then
                    local cumO, cumE, cumUnknown,cumO2, cumE2, cumUnknown2 = self:get_occupied_unknown(flatOcc, flatEmpt,flatIntersect)
                    --local newCum, newCumD = self:combineSmartly(flatIntersect, cumO, cumE, cumUnknown,cumO2, cumE2, cumUnknown2, 1, intersect_j, x_mat, y_mat)
                    --do some smart combination
                    local cum = {cumO, cumE, cumUnknown,cumO2, cumE2, cumUnknown2}
                    local newCum, newCumD = self:combine_all(flatIntersect, cum, 1, intersect_j, x_mat, y_mat) 
                    local flat1, flatd1,startX,startY, endX, endY = intersect_j:convertIntersectLineToPlane(1, 1, newCum[1], newCumD)

                    for i=1,6 do
                        combined[j][i] = flattened_plane.Line.new(nil,startX,startY, endX, endY,{flat1, flatd1[i]})
                    end
                    local flatIntersect2, flatOcc2, flatEmpt2, x_mat, y_mat = intersect_j:getRotatedIntersectionAndOccupiedEmpties(2)
                    if flatIntersect2 then
                        local cumOP2, cumEP2, cumUnknownP2,cumO2P2, cumE2P2, cumUnknown2P2 = self:get_occupied_unknown(flatOcc2, flatEmpt2,flatIntersect2)
                        cum = {cumOP2, cumEP2, cumUnknownP2,cumO2P2, cumE2P2, cumUnknown2P2}
                        local newCum2, newCumD2 = self:combine_all(flatIntersect2, cum, 2, intersect_j, x_mat, y_mat) 
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
        self:save_me()
    end
    return self.allIntersectionsInfo

end
function FlattenedPlaneNew:combine_all(flatIntersect, cum, j, intersect_j, x_mat, y_mat)
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

function FlattenedPlaneNew:combine_smartly(flatIntersect, cumO, cumE, cumUnknown,cumO2, cumE2, cumUnknown2, j, intersect_j, x_mat, y_mat)
    local temp,val = flatIntersect:sum(2):squeeze():max(1)
    local flatIntersectD = torch.zeros(flatIntersect:size())
    local combinedDir1= torch.ge(torch.ge(cumO,cumE+11)+torch.eq(torch.le(cumE,10) + torch.le(cumO,10),2),1):double() --not enough information
    local combinedDir2 = torch.ge(torch.ge(cumO2,cumE2+11)+torch.eq(torch.le(cumE2,10) + torch.le(cumO2,10),2),1):double() --not enough information
    flatIntersectD[val:squeeze()]  = torch.eq(combinedDir1+combinedDir2, 2):double()
    local newCum, newCumD = intersect_j:sendRotateScoreBackToPlane(j, flatIntersect:byte(), flatIntersectD[flatIntersect:byte()], x_mat, y_mat)
    return newCum, newCumD

end

--looks at intersections on plane and the empties/occupied relationships
function FlattenedPlaneNew:get_occupied_unknown(flatOcc, flatEmpt, flatIntersect)
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

