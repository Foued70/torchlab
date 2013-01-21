require 'torch'
require 'sys'

util.obj = {}
local objops = util.obj

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
   local obj = {}
   obj.nverts = tonumber(io.popen(string.format("grep -c '^v ' %s",file)):read())
   obj.nfaces = tonumber(io.popen(string.format("grep -c '^f ' %s",file)):read())

   print(string.format("Found nverts: %d nfaces: %d in %2.2f", obj.nverts, obj.nfaces, sys.toc()))

   sys.tic()
   -- could be a table as we don't know max number of vertices per face
   obj.verts = torch.Tensor(obj.nverts,4):fill(1)
   obj.faces = torch.IntTensor(obj.nfaces,maxvertsperface):fill(-1)
   obj.nverts_per_face = torch.IntTensor(obj.nfaces)

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
            obj.verts[vc][k] = tonumber(n)
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
               obj.faces[fc][k] = tonumber(n)
            end
            k = k + 1
         end
         obj.nverts_per_face[fc] = k - 1
         fc = fc + 1
      end
   end
   print(string.format("Loaded %d verts %d faces in %2.2fs", vc-1, fc-1, sys.toc()))

   sys.tic()
   obj.face_verts = torch.Tensor(obj.nfaces,maxvertsperface,3)
   obj.normals    = torch.Tensor(obj.nfaces,3)
   obj.d          = torch.Tensor(obj.nfaces) 
   obj.centers    = torch.Tensor(obj.nfaces,3)

   for i = 1,obj.nfaces do
      for j = 1,obj.nverts_per_face[i] do
         for k = 1,3 do
            obj.face_verts[i][j][k] = obj.verts[obj.faces[i][j]][k]
         end
      end
      for j = 1,3 do
         obj.centers[i][j] = obj.face_verts[{i,{},j}]:mean()
      end
      obj.normals[i]:copy(util.geom.compute_normal(obj.face_verts[i]))
      obj.d[i] = - torch.dot(obj.normals[i],obj.centers[i])
   end
   print(string.format("Processed face_verts, centers and normals in %2.2fs", sys.toc()))
   
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
