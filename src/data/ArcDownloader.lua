local ArcDownloader = Class()

local fs = require 'fs'
local path = require 'path'
local log = require '../util/log'
local ufs = util.fs

local function add_arcfile_to_table(retTable, t, pth, ext)
  for k,v in pairs(t) do
    if type(v) == 'table' then
      add_arcfile_to_table(retTable,v,pth,ext)
    elseif (type(v) == 'string' and k == 'filename') then
      if ext == nil or path.extname(v) == ext then
        local download = true
        if pth then
          local b,e = string.find(v,pth)
          if not b then
            download = false
          end
        end
        if download then
          table.insert(retTable,t)
        end
      end
    end
  end
end

local function find_all_arcfiles(arc,pth,ext)
  local retTable = {}
  add_arcfile_to_table(retTable,arc,pth,ext)
  return retTable
end

local function downloadRecursive(i, arcFileTable)
  if i > 0 and i <= #arcFileTable then
    print('downloading', i, arcFileTable[i].path)
    arcFileTable[i]:download(function(err, arcfile)
                           if err then
                             print("logging error")
				             log.error(err)
				           end
				           collectgarbage()
                           downloadRecursive(i+1,arcFileTable)
                         end)
  end
  collectgarbage()
end

local function downloadAllFunction(err, arc, pth, ext)
  if err then
    print('logging error')
    log.error(err)
  end
  
  local topdir = arc:dirname()
  local arcFileTable = find_all_arcfiles(arc,pth,ext)
  downloadRecursive(1,arcFileTable)
  arcFileTable = nil
  collectgarbage()
end

local function saveRecursive(t,num,pth,ext)

  t:import_all()
  for k,v in pairs(t) do
    if k ~= '' and type(v) == 'table' then
      if v.filename then
      
        if (not ext) or path.extname(k) == ext then
        
          local save = true
          if pth then
            local b,e = string.find(v.path,pth)
            if not b then
              save = false
            end
          end
        
          if save then
            print(v.path)
            v:save()
        
            num = num+1
            local wait_time = 500
            if num >= 10 then
              wait_time = 5000
              num = 0
            end      
            local toc = log.toc()
            while log.toc()-toc < wait_time do
            end
          end
        
        end
  
      else
        v:import_all()
        num = saveRecursive(v,num,pth,ext)
      end
    end
  end
  
  return num
  
end

local function saveAllFunction(err,arc,pth,ext)

  if err then
    print('logging error')
    log.error(err)
  end
  
  local toc = log.toc()
  while log.toc()-toc < 5000 do
  end
  
  local num = 0
  
  arc:import_all()
  saveRecursive(arc,num,pth,ext)
  
  toc = log.toc()
  while log.toc()-toc < 5000 do
  end
  
  collectgarbage()
end

function ArcDownloader:__init(arc_path)
  self.path = arc_path
end

function ArcDownloader:download_all(pth,ext)
  local arc = data.Arc.get(self.path, function(err,arc) downloadAllFunction(err,arc,pth,ext) end)
  arc = nil
  collectgarbage()
end

function ArcDownloader:get_arc()
  local arc = data.Arc.get(self.path, function(err,arc) end)
  return arc
end

function ArcDownloader:save_all(pth,ext)
  local arc = self:get_arc(saveAllFunction(nil,arc,pth,ext))
  arc = nil
  collectgarbage()
end
  