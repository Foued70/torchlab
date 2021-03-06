-- Obj can load and save a .obj file.
-- Viewer can show n-gon as triangulated mesh. materials will be wack.
-- TODO: n-gon support -- connect tri to associated ngon so that viewer can show "ngon"
-- TODO: n-gon support -- do submeshes and uvs need to be updated?

local path = require 'path'
local fs = require 'fs'
local os = require 'os'
local io = require 'io'
local geom  = geom.util

local _t = os.time()
local function tic(msg)
  local lapse = os.time() - _t
  _t = os.time()
  if msg then
    print(lapse, msg)
  end
end

local Obj = Class()

function Obj:__init(filename)
  if filename and fs.existsSync(filename) then    
    self:load(filename)
    self:add_derived_data()
  end
end

local function parse_color(line)
  local r, g, b = line:match('[^%s]+%s+([^%s]+)%s+([^%s]+)%s+([^%s]+)')
  return {tonumber(r), tonumber(g), tonumber(b), 1}
end

local function trim(s)
  return s:match'^%s*(.*%S)' or ''
end

local function get_counts(filename)
  local n_faces = 0;
  local n_verts = 0;
  local n_uvs = 0;
  local n_gon = 0;
  local n_verts_per_face = {}
    
  for line in io.lines(filename) do
    if line:match('^f ') then
      n_faces = n_faces + 1
      new_line, face_count = trim(line):gsub("%s+", "")
      if face_count > n_gon then n_gon = face_count end
      table.insert(n_verts_per_face, face_count)
    elseif line:match('^v ') then
      n_verts = n_verts + 1
    elseif line:match('^vt ') then
      n_uvs = n_uvs + 1;
    end
  end
  -- better to store data in Tensor
  return n_faces, n_verts, n_uvs, n_gon, torch.LongTensor(n_verts_per_face)
end

function Obj:add_derived_data()
  tic()
  local n_faces             = self.n_faces
  local n_verts_per_face    = self.n_verts_per_face
  local face_verts          = self.face_verts
  
  local face_planes         = torch.Tensor(n_faces, 4)
  local face_centers        = torch.ones(n_faces, 4)
  local face_bboxes         = torch.Tensor(n_faces, 6) -- xmin,ymin,zmin,xmax,ymax,zmax
  local bbox                = torch.Tensor(6)

  -- TODO compute w/out for loop
  for face_idx = 1,n_faces do 
    local nverts = n_verts_per_face[face_idx]
    local fverts = face_verts[face_idx]:narrow(1, 1, nverts)
    -- a) compute face centers
    face_centers[face_idx] = fverts:mean(1):squeeze()
 
    fverts = fverts:narrow(2,1,3)

    -- b) compute plane normal and face_center_dists distance from origin for plane eq.
    local face_norm = face_planes[{face_idx,{1,3}}] 

    face_norm:copy(geom.compute_normal(fverts))
    face_planes[face_idx][4] = - torch.dot(face_norm, face_centers[face_idx]:narrow(1,1,3))
    
    -- c) compute bbox
    local thisbb   = face_bboxes[face_idx]
    thisbb:narrow(1,1,3):copy(fverts:min(1):squeeze())
    thisbb:narrow(1,4,3):copy(fverts:max(1):squeeze())
  end 
  
  bbox:narrow(1,1,3):copy(face_bboxes:narrow(2,1,3):min(1):squeeze())
  bbox:narrow(1,4,3):copy(face_bboxes:narrow(2,4,3):max(1):squeeze())
  
  self.face_planes  = face_planes
  self.face_normals = face_planes:narrow(2,1,3)
  self.face_centers = face_centers
  self.face_bboxes  = face_bboxes
  self.bbox = bbox

  tic('derive')
  
end

local function default_material()
  local material = {}
  material.name = "default"
  material.diffuse = {0.5, 0.5, 0.5, 1}
  material.ambient = {0, 0, 0, 1}
  return material
end

local function load_materials(pathname, filename)
  filename = path.join(pathname, filename)
  
  local materials = {}
  local mtl_name_index_map = {}
  local mtl
  
  if fs.existsSync(filename) then 

    for line in io.lines(filename) do
      if line:match('^newmtl ') then
        mtl = {name = trim(line:sub(8))}
        table.insert(materials, mtl)

        mtl_name_index_map[mtl.name] = #materials
      elseif line:match('^Ns ') then
        mtl.shininess = tonumber(line:sub(4))
      elseif line:match('^Ka ') then
        mtl.ambient = parse_color(line)
      elseif line:match('^Kd ') then
        mtl.diffuse = parse_color(line)
      elseif line:match('^Ks ') then
        mtl.specular = parse_color(line)
      elseif line:match('^d ')  then
        mtl.alpha = tonumber(line:sub(3))
      elseif line:match('^Tr ') then
        mtl.alpha = tonumber(line:sub(4))
      elseif line:match('^illum ') then
        mtl.illumType = tonumber(line:sub(7))
      elseif line:match('^map_Kd ') then
        mtl.diffuse_tex_path = path.join(pathname, trim(line:sub(8)))
      end
    end
    
  end

  return materials, mtl_name_index_map
end

function Obj:load(filename)
  tic()
  local n_faces, n_verts, n_uvs, n_gon, n_verts_per_face = get_counts(filename);
  tic('counts')  
  
  local verts         = torch.ones(n_verts, 4)
  local uvs           = torch.ones(n_uvs, 2)
  local faces         = torch.LongTensor(n_faces, n_gon, 3):fill(-1) 
  local face_verts    = torch.ones(n_faces, n_gon, 4)
  local unified_verts = torch.ones(n_faces*n_gon, 9) -- n_faces*n_gon is max. we'll trim later.  
  tic('alloc')
  
  local materials, mtl_name_index_map
  local submeshes = {}
  local vert_cache = {}
  local vert_idx = 1
  local face_idx = 1
  local uv_idx = 1
  local unified_verts_idx = 1
  
  for line in io.lines(filename) do 
    if line:match('^v ') then
      -- vertices: v -1.968321 0.137992 -1.227969
      local x, y, z = line:match('%s*([^%s]+)%s*([^%s]+)%s*([^%s]+)', 3)
      verts[{vert_idx, 1}] = tonumber(x)
      verts[{vert_idx, 2}] = tonumber(y)
      verts[{vert_idx, 3}] = tonumber(z)
      vert_idx = vert_idx + 1      
    elseif line:match('^vt ') then
      -- uvs: vt 0.442180 0.660924
      local u, v = line:match('%s*([^%s]+)%s*([^%s]+)', 4)
      uvs[{uv_idx, 1}] = tonumber(u)
      uvs[{uv_idx, 2}] = tonumber(v)
      uv_idx = uv_idx + 1;      
    elseif line:match('^f ') then
      -- faces: 
      -- f 944 945 942
      -- f 944/1572 945/1569 942/1570
      -- f 944/1572/944 945/1569/945 942/1570/942
      -- f 944//1572 945//1572 942//942
      -- we don't care about normals, we will calc them later
      local face_vert_idx = 1
      for face_vert in line:gmatch("%d+[/%d+]*") do                       
        local idx = vert_cache[face_vert]        
        local vert_pos_idx = face_vert:match("%d+")
        
        local vert_pos_idx = tonumber(face_vert:match("%d+")) -- first set of digits        
        local vert_uv_idx = tonumber(face_vert:match("[^/]/(%d+)")) -- second set of digits (might not exist)
        
        local vert = verts[vert_pos_idx]:narrow(1, 1, 3)        
        
        if not idx then
          idx = unified_verts_idx                
                    
          unified_verts[{idx, {1, 3}}] = vert
          if vert_uv_idx then unified_verts[{idx, {5, 6}}] = uvs[vert_uv_idx] end
          vert_cache[face_vert] = idx
          unified_verts_idx = unified_verts_idx + 1
        end              
        
        face_verts[{face_idx, face_vert_idx, {1, 3}}] = vert -- shortcut to the vert
        
        faces[{face_idx, face_vert_idx, 1}] = idx          -- unified_vert index
        faces[{face_idx, face_vert_idx, 2}] = vert_pos_idx -- vert indx
        if vert_uv_idx then faces[{face_idx, face_vert_idx, 3}] = vert_uv_idx end -- uv index
                
        face_vert_idx = face_vert_idx + 1
      end
      
      face_idx = face_idx + 1           
    elseif line:match('^usemtl ') then
      local mtl_name = trim(line:sub(8))
      local mtl_id = mtl_name_index_map[mtl_name]

      if #submeshes > 0 then
        submeshes[#submeshes][2] = face_idx - 1
      end

      table.insert(submeshes, {face_idx, 0, mtl_id})
    elseif line:match('^mtllib ') then
      local mtllib = trim(line:sub(8))    
      materials, mtl_name_index_map = load_materials(path.dirname(filename), mtllib)
    end
  end
  if #submeshes > 0 then
    submeshes[#submeshes][2] = face_idx - 1
  elseif #submeshes == 0 then
    -- default submesh is all faces pointing to default material
    materials = {default_material()}
    submeshes[1] = {1, n_faces, 1} 
  end  
  tic('parse file')
  
  unified_verts_count = unified_verts_idx - 1
  local trimmed_unified_verts = torch.Tensor(unified_verts_count, 9)
  trimmed_unified_verts[{{1, unified_verts_count}}] = unified_verts[{{1, unified_verts_count}}]
  tic('trim')
    
  self.n_faces            = n_faces -- number of faces
  self.n_verts            = n_verts -- number of verts
  self.n_uvs              = n_uvs -- number of uvs
  self.n_gon              = n_gon -- max number of verts for any given face
  self.n_verts_per_face   = n_verts_per_face
  
  self.verts              = verts -- all unique verts. n_verts x 4
  self.uvs                = uvs -- all unique uvs. n_uvs x 2
  self.faces              = faces -- all faces with indexes into unified verts, verts, and uvs. n_faces x n_gon x 3
  self.face_verts         = face_verts -- all faces with copy of vert data. n_faces x n_gon x 3 (retexture)
  self.unified_verts      = trimmed_unified_verts -- unique pos/texture pairs. ?x9. 4 position + 2 uv + 3 normal (viewer)
  self.submeshes          = torch.LongTensor(submeshes) -- indexes of faces in  a submesh and index into materials. 
  self.materials          = materials -- table of material properties (viewer)
end

local function write_mtl_prop(mtlf, mtl, mtl_prop, file_prop)
  mtl_prop = mtl[mtl_prop]
  
  if mtl_prop then 
    local str = file_prop.." "
    
    if type(mtl_prop) == 'table' then
      for i=1, #mtl_prop do
        str = str..mtl_prop[i].." "
      end
    else
      str = str..mtl_prop
    end
    mtlf:write(str.."\n") 
  end
end

function Obj:get_tris()
  if not self.tris then 
    if self.n_gon == 3 then
      self.tris = self.faces[{{}, {}, 1}]
    else
      log.trace('triangulating')
      local max_tris = (self.n_gon - 2) * self.n_faces
      local tris =  torch.LongTensor(max_tris, 3):fill(1)
      local n_verts_per_face = self.n_verts_per_face
      local faces = self.faces
      local tris_idx = 0    
      
      -- input face can be a tri, quad, or poly but output face must be a tri      
      for face_idx=1, self.n_faces do
        local n_verts = n_verts_per_face[face_idx]
        local face = faces[face_idx]
        -- Rough Fix to deal with quads and polys there may be better methods        
        for vert_idx=1, n_verts-2 do -- number of triangles to make up the face          
          tris_idx = tris_idx+1
          tris[tris_idx][1] = face[1][1]
          tris[tris_idx][2] = face[vert_idx+1][1]
          tris[tris_idx][3] = face[vert_idx+2][1]
        end
      end      
      local trimmed_tris = torch.LongTensor(tris_idx, 3)
      trimmed_tris[{{1, tris_idx}}] = tris[{{1, tris_idx}}]
      self.tris = trimmed_tris      
    end
  end
  
  return self.tris
end

function Obj:save(filename, mtlname)
  filename = filename or "scan.obj"
  mtlname = mtlname or filename:gsub("obj","mtl")
  
  local objf = assert(io.open(filename, "w"))  
  objf:write(string.format("mtllib %s\n\n", path.basename(mtlname)))
  
  -- print vertices
  local verts = self.verts
  if self.n_verts and verts then 
     log.trace('writing verts', self.n_verts)  
     for vert_idx = 1,self.n_verts do 
        local vert = verts[vert_idx]
        objf:write(string.format("v %f %f %f\n",vert[1],vert[2],vert[3]))
     end
  else
     log.trace("no verts to write")
  end
  
  -- print uvs
  local uvs = self.uvs
  if uvs then 
     log.trace('writing uvs', self.n_uvs)  
     objf:write("\n")
     for uv_idx = 1,self.n_uvs do 
        local uv = uvs[uv_idx]
        objf:write(string.format("vt %f %f\n", uv[1], uv[2]))
     end
  else 
     log.trace('no uvs to write')
  end

  -- faces
  local faces = self.faces
  local materials = self.materials
  local submeshes = self.submeshes
  local n_verts_per_face = self.n_verts_per_face
     
  if self.faces then 
     log.trace('writing faces', self.n_faces)    
     if submeshes then
        for submesh_idx= 1,#submeshes do
           local submesh = submeshes[submesh_idx]
           objf:write("\n")
           objf:write(string.format("g face%05d\n",submesh_idx))
           local material = materials[submesh[3]]      
           if material then
              objf:write(string.format("usemtl %s\n", material.name))
           end
           
           for face_idx = submesh[1], submesh[2] do                        
              local str = "f "
              for face_vert_idx=1, n_verts_per_face[face_idx] do
                 local vert_idx = faces[face_idx][face_vert_idx][2];
                 local uv_idx = faces[face_idx][face_vert_idx][3];    
                 if uv_idx == -1 then
                    str = str..string.format("%d ", vert_idx)    
                 else
                    str = str..string.format("%d/%d ", vert_idx, uv_idx)          
                 end          
              end
              objf:write(str.."\n")
           end
        end   
     else
        material = materials[1]
        objf:write(string.format("usemtl %s\n", material.name))
        for face_idx = 1, self.n_faces do                        
              local str = "f "
              for face_vert_idx=1, n_verts_per_face[face_idx] do
                 local vert_idx = faces[face_idx][face_vert_idx][2];
                 local uv_idx = faces[face_idx][face_vert_idx][3];    
                 if uv_idx == -1 then
                    str = str..string.format("%d ", vert_idx)    
                 else
                    str = str..string.format("%d/%d ", vert_idx, uv_idx)          
                 end          
              end
              objf:write(str.."\n")
           end
     end
     if not materials then
        materials = {default_material()}
     end
     objf:write("\n")        
     
  end

  objf:close()

  if materials then 
     local mtlf = assert(io.open(mtlname, "w"))  
     log.trace('writing materials', #materials)  
     for i=1, #materials do
        local material = materials[i]
        write_mtl_prop(mtlf, material, 'name', 'newmtl')
        write_mtl_prop(mtlf, material, 'shininess', 'Ns')
        write_mtl_prop(mtlf, material, 'ambient', 'Ka')
        write_mtl_prop(mtlf, material, 'diffuse', 'Kd')
        write_mtl_prop(mtlf, material, 'specular', 'Ks')
        write_mtl_prop(mtlf, material, 'alpha', 'd')    
        write_mtl_prop(mtlf, material, 'illumType', 'illum')
        
        if material.diffuse_tex_path then      
           mtlf:write(string.format("map_Kd %s\n", path.basename(material.diffuse_tex_path)))
        end   
    
        mtlf:write("\n")
     end
     mtlf:close()
  end

  log.trace('obj saved', filename)
end
