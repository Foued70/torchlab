require 'sys'

-- see: http://en.wikipedia.org/wiki/Wavefront_.obj_file
function loadObj(file,maxvertsperface)
   if (not file) then
      file = "data/scan.obj"
   end
   if (not maxvertsperface) then
      maxvertsperface = 10
   end
   sys.tic()
   obj = {}
   obj.nverts = tonumber(io.popen(string.format("grep -c '^v ' %s",file)):read())
   obj.nfaces = tonumber(io.popen(string.format("grep -c '^f ' %s",file)):read())

   print(string.format("Found nverts: %d nfaces: %d in %2.2f", obj.nverts, obj.nfaces, sys.toc()))

   sys.tic()
   -- could be a table as we don't know max number of vertices per face
   obj.verts = torch.Tensor(obj.nverts,4):fill(1)
   obj.faces = torch.IntTensor(obj.nfaces,maxvertsperface):fill(-1)

   objFile = io.open(file)
   fc = 1
   vc = 1
   for line in objFile:lines() do
      if (line:match("^v ")) then
         -- vertices
         -- v -1.968321 0.137992 -1.227969
         vs = line:gsub("^v ", "")
         k = 1
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
         vs = line:gsub("^f ", "")
         -- for now just look at position
         vs = vs:gsub("/%d*","")
         k = 1
         for n in vs:gmatch("[-.%d]+") do
            if (k > maxvertsperface) then
               print(string.format("Warning face has more verts than max: %d", maxvertsperface))
            end
            obj.faces[fc][k] = tonumber(n)
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
      n = torch.cross(obj.face_verts[i][2] - obj.face_verts[i][1],
                      obj.face_verts[i][3] - obj.face_verts[i][2])
      -- norm of 1
      n = n * 1 / n:norm()
      obj.normals[i]:copy(n)
   end
   print(string.format("Processed face_verts, centers and normals in %2.2fs", sys.toc()))

   return obj
end