require 'torch'
require 'sys'
local geom = require "util/geom"

local objops = {}

-- see: http://en.wikipedia.org/wiki/Wavefront_.obj_file
function objops.load(...)
   local file,maxvertsperface
   local args = {...}
   local nargs = #args
   if nargs == 2 then
      file = args[1]
      maxvertsperface = args[2]
   elseif nargs == 1 then
      file = args[1]
      maxvertsperface = 3
   else
      print(dok.usage('obj.load',
                      'load an obj file',
                      '> returns: lua table with faces, verts etc.',
                      {type='string', help='obj filepath', req=true},
                      {type='number', help='max vertices per face', default=3}))
      dok.error('incorrect arguments','obj.load')
   end
   
   sys.tic()
   local nverts = tonumber(io.popen(string.format("grep -c '^v ' %s",file)):read())
   local nfaces = tonumber(io.popen(string.format("grep -c '^f ' %s",file)):read())

   print(string.format("Found nverts: %d nfaces: %d in %2.2f", nverts, nfaces, sys.toc()))

   sys.tic()

   -- could be a table as we don't know max number of vertices per face
   local verts = torch.Tensor(nverts,4):fill(1)
   local faces = torch.IntTensor(nfaces,maxvertsperface):fill(-1)
   local nverts_per_face = torch.IntTensor(nfaces)

   local objFile = io.open(file)
   local fc = 1
   local vc = 1
   for line in objFile:lines() do
      if (line:match("^v ")) then
         -- vertices
         -- v -1.968321 0.137992 -1.227969
         local vs = line:gsub("^v ", "")
         local k = 1
         for n in vs:gmatch("[-.%d]+") do
            verts[vc][k] = tonumber(n)
            k = k + 1
         end
         vc = vc + 1
      elseif (line:match("^f ")) then
         -- faces
         -- 1) position
         -- f 1 2 3
         -- 2) position/texture
         -- f 44/12 51/13 1/14
         -- 3) position/texture/normal
         -- f 44/12/1 51/13/2 1/14/2
         local vs = line:gsub("^f ", "")
         -- FIXME for now discard texture and normal
         vs = vs:gsub("/%d*","")
         local k = 1
         -- can have faces with different number of verts gmatch
         -- returns an iterator so we can't get the length before hand
         -- and have to check against maxverts in the loop
         for n in vs:gmatch("[-.%d]+") do 
            if (k > maxvertsperface) then
               print(string.format("Warning face %d has more verts than max: %d", 
                                   fc, maxvertsperface))
               break
            else
               faces[fc][k] = tonumber(n)
            end
            k = k + 1
         end
         nverts_per_face[fc] = k - 1
         fc = fc + 1
      end
   end
   print(string.format("Loaded %d verts %d faces in %2.2fs", vc-1, fc-1, sys.toc()))


   sys.tic()
   local face_verts  = torch.Tensor(nfaces,maxvertsperface,3)
   local normals     = torch.Tensor(nfaces,3)
   local d           = torch.Tensor(nfaces) 
   local centers     = torch.Tensor(nfaces,3)
   local face_bboxes = torch.Tensor(nfaces,6) -- xmin,ymin,zmin,xmax,ymax,zmax
   local bbox        = torch.Tensor(6)
   for fid = 1,nfaces do

      local nverts = nverts_per_face[fid]
      local fverts = face_verts[fid]:narrow(1,1,nverts)
      local face   = faces[fid]

      -- a) build face_verts (copy all the data)
      for j = 1,nverts do
         fverts[j] = verts[ {face[j],{1,3}} ]
      end

      -- b) compute object centers
      centers[fid] = fverts:mean(1):squeeze()

      -- c) compute plane normal and d distance from origin for plane eq.
      normals[fid] = geom.compute_normal(fverts)
      d[fid]       = - torch.dot(normals[fid],centers[fid])

      -- d) compute bbox
      local thisbb   = face_bboxes[fid]
      thisbb:narrow(1,1,3):copy(fverts:min(1):squeeze())
      thisbb:narrow(1,4,3):copy(fverts:max(1):squeeze())
      
   end

   bbox:narrow(1,1,3):copy(face_bboxes:narrow(2,1,3):min(1):squeeze())
   bbox:narrow(1,4,3):copy(face_bboxes:narrow(2,4,3):max(1):squeeze())

   print(string.format("Processed face_verts, centers and normals in %2.2fs", sys.toc()))

   local obj = {}
   obj.nverts          = nverts
   obj.nfaces          = nfaces
   obj.verts           = verts
   obj.faces           = faces
   obj.nverts_per_face = nverts_per_face
   obj.face_verts      = face_verts
   obj.normals         = normals
   obj.d               = d
   obj.centers         = centers
   obj.face_bboxes     = face_bboxes
   obj.bbox            = bbox
   
   return obj
end

-- save a retextured object to obj
function objops.save(obj,objfname,mtlfname,imgbasename)
   if not objfname then 
      objfname = "retexture.obj" 
   end
   local objf = assert(io.open(objfname, "w"))

   if not mtlfname then
      mtlfname = "retexture.mtl"
   end
   local mtlf = assert(io.open(mtlfname, "w"))
   objf:write(string.format("mtllib %s\n\n", mtlfname))

   if not imgbasename then 
      imgbasename = "texture"
   end
   local nvpf = obj.nverts_per_face

   -- print vertices
   local verts = obj.verts
   for vid = 1,verts:size(1) do 
      local v = verts[vid]
      objf:write(string.format("v %f %f %f %f\n",v[1],v[2],v[3],v[4]))
   end

   -- uvs
   local uv = obj.uv
   objf:write("\n")
   for uid = 1,uv:size(1) do 
      for vid = 1,nvpf[uid] do 
         objf:write(string.format("vt %f %f\n",
                                  uv[uid][vid][1],uv[uid][vid][2]))
      end
   end

   -- faces
   local faces = obj.faces
   local vti = 1
   for fid = 1,obj.nfaces do

      -- save texture to an image file
      local iname = string.format("%s_face%05d.png",imgbasename,fid)
      image.save(iname,obj.textures[fid])

      -- store path to image in mtlfile
      mtlf:write(string.format("newmtl %s\n", iname))
      mtlf:write(string.format("map_Ka %s\n", iname))
      mtlf:write(string.format("map_Kd %s\n", iname))
      mtlf:write("\n")

      -- store face and mtl info to obj
      objf:write("\n")
      objf:write(string.format("o face%05d\n",fid))
      objf:write(string.format("usemtl %s\n",iname))
      str = "f "
      for vid = 1,nvpf[fid] do 
         str = str .. string.format("%d/%d ",faces[fid][vid],vti)
         vti = vti + 1
      end
      objf:write(str .. "\n")
   end
   
   mtlf:close()
   objf:close()
end


return objops
