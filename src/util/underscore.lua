-- http://documentcloud.github.com/underscore/

local _ = {}

function _.each(tbl, fn) 
  if not tbl then return end
  
  for k, v in pairs(tbl) do
    fn(v, k)
  end
end

local each = _.each

function _.map(tbl, fn)
  local results = {}
  local function map(v, k)
    table.insert(results, fn(v, k))
  end
  each(tbl, map)
  return results
end

function _.identity(val)
  return val
end

function _.any(tbl, fn)
  fn = fn or _.identity
  local result = false
  if not tbl then return result end
  for k, v in pairs(tbl) do
    result = fn(v, k)
    if result then break end
  end
  return result
end

local any = _.any

function _.find(tbl, fn)
  local result
  for k, v in pairs(tbl) do
    if fn(v, k) then 
      result = v
      break
    end
  end
  return result
end

function _.filter(tbl, fn)
  local results = {}
  if not tbl then return end
  local function filter(v, k)
    if fn(v, k) then table.insert(results, v) end
  end
  each(tbl, filter)
  return results
end

function _.extend(tbl, source)
  if source then
    for k, v in pairs(source) do
      tbl[k] = v
    end
  end
  
  return tbl
end

function _.keys(tbl)
  local ktbl = {}
  for k, v in pairs(tbl) do
    table.insert(ktbl, k)
  end
  return ktbl
end

function _.values(tbl)
  local vtbl = {}
  for k, v in pairs(tbl) do
    table.insert(vtbl, v)
  end
  return vtbl
end



return _