local path = require 'path'
local io = require 'io'

local fix_newline = Class()

function fix_newline.fix_newline(fname)
    local dname = path.dirname(fname)
    local ename = path.extname(fname)
    local bname = path.basename(fname,ename)
	local fname_tmp = path.join(dname,bname..'_tmp'..ename)
	
	local file_in = io.open(fname)
	local file_tp = io.open(fname_tmp,'w')
	local ct = 0
	local blc = 0
	local flag = true
	
	for block in file_in:lines() do
	  
	  if blc == 0 then
	    local numtok = 0
  	    for token in string.gmatch(block, "[^\r]+") do
  	      numtok = numtok + 1
  	      if numtok > 1 then
  	        flag = false
  	        break
  	      end
  	    end
  	    collectgarbage()
  	  end
  	  
  	  if flag then
  	    break
  	  end
  	  
	  for token in string.gmatch(block, "[^\r]+") do
	    if ct > 0 then
  	      file_tp:write('\n'..token)
  	    else
  	      file_tp:write(token)
  	    end
  	    ct = ct + 1
	  end
	  
	  collectgarbage()
	  blc = blc + 1
	  
	end
	
	if not flag then
	  util.fs.exec('cp '..fname_tmp..' '..fname)
	end
	  util.fs.exec('rm '..fname_tmp)
	
	collectgarbage()
	file_in:close()
	file_tp:close()
	collectgarbage()
end