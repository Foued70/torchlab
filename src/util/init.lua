require 'torch'
require 'dok'
require 'paths'

local utils = {}

-- variation on torch.include which also searches the local dir
-- and pulls in the submodule files
function utils.include (...)
   local torchPackageName,file,localPackage
   local args = {...}
   local nargs = #args
   if nargs > 2 then
      localPackage     = args[3]
   end
   if nargs > 1 then
      torchPackageName = args[1]
      file             = args[2]
   else
      print(dok.usage('include',
                      'include with parameters a module file that returns its table',
                      '> returns: loaded module to add to global namespace',
                      {type='string', help='torch package name', req=true},
                      {type='string', help='installed file to load', req=true},
                      {type='table',  help='locally loaded module to call in submodules'}
                ))
      dok.error('incorrect arguements', 'include')
   end
   local pname = torch.packageLuaPath(torchPackageName)
   local fpath
   if pname and paths.filep(pname .. '/' .. file) then
      fpath = pname .. '/' .. file
   else
      fpath = file
   end
   -- method to pass args to file begin loaded
   local foo = assert(loadfile(fpath))
   return foo(localPackage)
end

-- add files to the local utils table

utils.geom = utils.include('utils','geom.lua')
utils.pose = utils.include('utils','pose.lua')
utils.obj  = utils.include('utils','obj.lua')

function utils.run_tests()
   utils.test = {}
   utils.test.geom = utils.include('utils','geom-test.lua', utils)
   utils.test.geom.all()
end

return utils