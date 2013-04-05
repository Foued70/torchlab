require "paths"

local AssetManager = Class()

local ASSET_DIR = paths.concat(CLOUDLAB_SRC, 'assets')
local MODEL_EXTENSION = 'obj'

models = {}

function AssetManager:__init()
end

function get_model(model_name)
  if models[model_name] ~= nil then return models[model_name] end

  local model_path = paths.concat(ASSET_DIR, model_name..'.'..MODEL_EXTENSION)

  if not paths.filep(model_path) then
    log.trace("Model file ".."\""..model_path.."\"".." does not exist.")
    return nil 
  end

  models[model_name] = util.Obj.new(model_path)
  return models[model_name]
end

