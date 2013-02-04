-- use paths lib
require 'paths'

-- help/args
help = 
[=[Cloudlab

Usage: torch [options] [script [args]]

General options:
  -e string        execute string
  -l lib           require lib
  -i               enter interactive mode after executing script [false]
  -m|-import       import torch and gnuplot packages into global namespace
  -v|-version      show version information [false]
  -h|-help         this help [false]

Qt options:
  -nographics|-ng  disable all the graphical capabilities [false]
  -ide             enable IDE (graphical console) [false]
  -onethread       run lua in the main thread (might be safer) [false] ]=]

-- default lua: qlua
lua = 'torch-qlua'

-- preload torch environment
env = ' -e "require \'exe/cloudlab-env\'" '

local pwd = paths.cwd()
local end_i = 1
while true do
  local s, e = pwd:find('src', end_i, true)
  if not e then break end
  end_i = e
end

if (end_i > 1) then
  local src_dir = pwd:sub(1, end_i)
  local src_path = paths.concat(src_dir, '?.lua') .. ';' .. paths.concat(src_dir, '?', 'init.lua') .. ';'
  env = ' -e "package.path = \'' .. src_path ..'\' .. package.path"' .. env
end

-- by default, be interactive
interactive = true

-- parse some arguments
for i,a in ipairs(arg) do
   --  no graphics mode?
   if a == '-nographics' or a == '-ng' then
      lua = 'torch-lua'
      arg[i] = ''
   end
   -- help?
   if a == '-help' or a == '--help' or a == '-h' then
      print(help)
      os.exit()
   end
   -- version?
   if a == '-v' or a == '-version' then
      print('Torch 7.0  Copyright (C) 2001-2012 Idiap, NEC Labs, NYU')
      os.execute(paths.concat(paths.install_bin,lua) .. ' -v')
      os.exit()
   end
   -- use import
   if a == '-m' or a == '-import' then
      env = ' -e "loadwithimport=true"' .. env
      -- we don't pass this to qlua
      arg[i] = ' '
   end
   -- autostart interactive sessions if no user script:
   if a:find('%.lua$') and paths.filep(a) then
      interactive = false
      -- do not consider further arguments
      break
   end
end

-- interactive?
if interactive then
   env = env .. ' -i '
end

-- re-pack arguments
for i,a in ipairs(arg) do
   if (a:find('[^-=+.%w]')) then
      arg[i] = '"' .. string.gsub(arg[i],'[$`"\\]','\\%0') .. '"'
   end
end
args = table.concat(arg, ' ')

-- finally execute main thread, with proper options
os.execute(paths.concat(paths.install_bin,lua) .. env .. args)
