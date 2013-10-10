local path = require 'path'
local pcl = PointCloud.PointCloud
local Sweep = align_floors_endtoend.Sweep
local SweepPair = align_floors_endtoend.SweepPair

local Scan = Class()

Scan.XYZ = "XYZ"


function Scan:__init(base_dir, complete_loop)
    self.base_dir = base_dir
    self.xyz_dir = path.join(base_dir,Scan.XYZ)
    self.first_sweep = 1
    self.complete_loop = false
    if complete_loop then
      self.complete_loop = true
    end
end

function Scan:set_first_sweep(num)
    self.first_sweep = num
end

function Scan:get_first_sweep()
    return self.first_sweep
end

function Scan:load_and_save_all_sweeps()
    if util.fs.is_dir(self.xyz_dir) then
        local all_files = util.fs.files_only(self.xyz_dir,'.xyz')
        for i = 1,#all_files do
            local fname = all_files[i]
            local bname = path.basename(fname,'.xyz')
            local aSweep = Sweep.new(self.base_dir,bname)
            local pc = aSweep:getPC()
            pc = nil
            aSweep = nil
        end
        collectgarbage()
    end
end

--TODO: parameters at top level ie scale
function Scan:find_forward_and_backward_transformations()
    if util.fs.is_dir(self.xyz_dir) then
        
        local all_files = util.fs.files_only(self.xyz_dir,'.xyz')
        
        if self.first_sweep < 1 or self.first_sweep > #all_files then
            self.first_sweep = 1
        end
        
        local numfiles = #all_files
        
        --[[]]
        print('looping through forward and back pairs')
        for i = 1,#all_files-1 do
        
            local numcur
            local numnex
            local fnamecur
            local bnamecur
            local sweepcur
            local fnamenex
            local bnamenex
            local sweepnex
            local sweepPair_curr_to_nex
            
            for j = 0,1 do
                
                if self.complete_loop or j == 0 then
                
	                local global = true
    	            local forward = true
                
        	        if j == 0 then
            	        -- forwards
                	    numcur = (self.first_sweep + (i-1) + (numfiles-1)) % numfiles + 1
                    	numnex = (numcur + 1 + (numfiles-1)) % numfiles + 1
	                else
    	                --backwards
        	            numcur = (self.first_sweep - (i-1) + (numfiles-1)) % numfiles + 1
            	        numnex = (numcur - 1 + (numfiles-1)) % numfiles + 1
                	    forward = false
	                end
                
    	            print('aligning '..numcur..' '..numnex)
        	        
            	    fnamecur = all_files[numcur]
                	bnamecur = path.basename(fnamecur,'.xyz')
	                sweepcur = Sweep.new(self.base_dir,bnamecur)
            
    	            fnamenex = all_files[numnex]
        	        bnamenex = path.basename(fnamenex,'.xyz')
            	    sweepnex = Sweep.new(self.base_dir,bnamenex)
                
                	if i == 1 and j == 0 then
                
                    	print('axis align the first sweep')
	                    sweepcur:axisAlign()
    	                local corners, flattenedxy, flattenedv = sweepcur:flattenAndCorners(global,forward)
        	            sweepcur:transformIn3D()
            	    end
            
                	print('create new sweep pair')
	                sweepPair_curr_to_nex = SweepPair.new(self.base_dir, sweepcur, sweepnex, i, forward)
    	            sweepPair_curr_to_nex :getAllTransformations()
        	        --sweepPair_curr_to_nex:setInlierTransformation(1)
        	        sweepPair_curr_to_nex:setBestTransformation()
        	        
        	        print('saving stuff')
        	        sweepcur:getPC():write(sweepcur.fod)
        	        sweepnex:getPC():write(sweepnex.fod)
                
            	    print()
            	    
            	    numcur = nil
                    numnex = nil
                    fnamecur = nil
                    bnamecur = nil
                    sweepcur = nil
                    fnamenex = nil
                    bnamenex = nil
                    sweepnex = nil
                    sweepPair_curr_to_nex = nil
                
	            end
    	        collectgarbage()
    	    end
        end
        print('done looping through forward and back pairs')
        print()
        collectgarbage()
        
        --[[]]
        for i = 1,#all_files do
        
            local numcur = (self.first_sweep + (i-1) + (numfiles-1)) % numfiles + 1
            local fnamecur = all_files[numcur]
            local bnamecur = path.basename(fnamecur,'.xyz')
            local sweepcur = Sweep.new(self.base_dir,bnamecur)
                
            if i > 1 then
            
                local transf = sweepcur:getTransformation(true).H
                local transavg
                if self.complete_loop then
	                local transb = sweepcur:getTransformation(false).H
	                transavg = transf:clone():mul(sweepcur.steps_B):add(transb:clone():mul(sweepcur.steps_F)):div(sweepcur.steps_F+sweepcur.steps_B)
                else
                	transavg = sweepcur:getTransformation(true).H
                end
            
                local transfull = geom.Homography.new(0,torch.Tensor({0,0}))
                transfull.H = transavg
                
                sweepcur:setAlignmentTransformation(transfull)
            
            	print('forward, backward, and full transforms')
                print(transf)
                print(transb)
                print(transavg)
                print(transfull.H)
            end
            
            collectgarbage()
            
            sweepcur:transformIn3D()
            sweepcur:getPC():save_downsampled_global_to_xyz(0.01, path.join(self.base_dir,"DOWNSAMPLEDXYZ",bnamecur..'.xyz'))
            sweepcur:getPC():write(sweepcur.fod)
            print()

        end
        --[[]]
        
    end
end
