local search = require("luarocks.search")
local cfg = require("luarocks.cfg")
local command_line = require("luarocks.command_line")
local path = require("luarocks.path")

commands = {}
commands.install = require("luarocks.install")

local installed_rocks = {}
local query = search.make_query("", nil)
query.exact_name = false
local trees = cfg.rocks_trees
for _, tree in ipairs(trees) do
  search.manifest_search(installed_rocks, path.rocks_dir(tree), query)
end

local rocks_file = 'deps/rocks.lua'

for line in io.lines(rocks_file) do 
  rock_name, version = line:match('^(%w[^%s]*)%s*([^%s]+)')
  if rock_name then 
    -- install rock at version
    installed = false
    installed_versions = installed_rocks[rock_name]
    if installed_versions then
      for ver, _ in pairs(installed_versions) do
        if ver == version then
          installed = true
          break
        end
      end
    end

    if not installed then
      print('INSTALLING '..rock_name..' at '..version)
      command_line.run_command('install', rock_name, version)
    else
      print(("uptodate     %-16s  %s"):format(rock_name, version))
    end
  end
end