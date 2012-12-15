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
         fc = fc + 1
      end
   end
   print(string.format("Loaded %d verts %d faces in %2.2fs", vc-1, fc-1, sys.toc()))

   sys.tic()
   obj.face_verts = torch.Tensor(obj.nfaces,3,3)
   obj.normals    = torch.Tensor(obj.nfaces,3)
   obj.centers    = torch.Tensor(obj.nfaces,3)

   for i = 1,obj.nfaces do
      for j = 1,3 do
         for k = 1,3 do
            obj.face_verts[i][j][k] = obj.verts[obj.faces[i][j]][k]
         end
      end
      for j = 1,3 do
         obj.centers[i][j] = obj.face_verts[{i,{},j}]:mean()
      end
      -- assume right handed points
      local n = torch.cross(obj.face_verts[i][2] - obj.face_verts[i][1],
                            obj.face_verts[i][3] - obj.face_verts[i][2])
      -- norm of 1
      n = n * 1 / n:norm()
      obj.normals[i]:copy(n)
   end
   print(string.format("Processed face_verts, centers and normals in %2.2fs", sys.toc()))
   
   return obj
end
