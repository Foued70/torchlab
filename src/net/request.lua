local fs = require'fs'
local path = require'path'
local http = require'http'
local url_lib = require'url'

local string = require'string'
local math = require'math'
local table = require'table'

local request = {}

local BOUNDARY = 'BOUNDARY-USPTCRITLOCBZSEZ-BOUNDARY'


function request.get(url, headers, callback)
  request.request('GET', url, nil, nil, headers, callback)
end

function request.post(url, data, files, headers, callback)
  request.request('POST', url, data, files, headers, callback)
end

function request.put(url, data, files, headers, callback)
  request.request('PUT', url, data, files, headers, callback)
end

function request.request(method, url, data, files, headers, callback)
  log.info(method, url)
  local url_parts = url_lib.parse(url)

  local content_length, req_body, content_type
  if files then
    content_type = 'multipart/form-data; boundary='..BOUNDARY
    content_length, req_body = multipart(files, data)
  elseif data then
    content_type = 'application/x-www-form-urlencoded'
    req_body = net.qs.stringify(data)
    content_length = #req_body
  else
    content_type = ''
    req_body = ''
    content_length = 0
  end

  -- An object of options to indicate where to post to
  headers = headers or {}
  headers['Content-Type'] = content_type
  headers['Content-Length'] = content_length

  local sink = headers.sink
  headers.sink = nil
 
  local req_options = {
    host = url_parts.hostname,
    port = url_parts.port,
    path = url_parts.pathname,
    method = method,
    headers = headers
  }

  -- Set up the request
  local req = http.request(req_options)

  req:on('response', function(res)
    local body = ''
    res:on('data', function (chunk)
      if sink then
        sink:write(chunk, function(err) if(err) then print(err) end end) --util.fs.writeAll(sink, 1, chunk)
      else
        body = body..chunk
      end
    end)

    res:on('end', function()
      callback(nil, res, body)
    end)

    res:on('error', function(err)
      log.error(err)
      callback(err)
    end)
  end)

  req:on('error', function(err)
    log.error(err)
    callback(err)
  end)

  if type(req_body) == 'string' then
    req:write(req_body)
    req:done()
  else
    write_parts(req, req_body, 1)
  end

end


function write_parts(req, req_body, start_i)
  for i = start_i,#req_body do
    start_i = i
    local part = req_body[i]
    if type(part) == 'string' then
      req:write(part)
    else
      -- this is a file
      local filename = part[1]
      local fstream = fs.createReadStream(filename)

      fstream:on('data', function(chunk) 
        req:write(chunk)
      end)

      fstream:on('end', function() 
        write_parts(req, req_body, start_i+1)
      end)

      break
    end
  end

  if start_i >= #req_body then
    req:done()
  end
end



function multipart(files, data)  
  local body = setmetatable({}, {__index=table})
  local len = 0

  if data then
    for k, v in pairs(data) do
      local field_part = string.format('--%s\r\ncontent-disposition: form-data; name="%s"\r\n\r\n%s\r\n', BOUNDARY, k, v)
      body:insert(field_part)
      len = len + #field_part
    end
  end
  
  for k, v in pairs(files) do
    if type(v) == 'string' and fs.existsSync(v) then
      local field_part = string.format('--%s\r\ncontent-disposition: form-data; name="%s"; filename="$s"\r\ncontent-type: application/octet-stream\r\n\r\n', BOUNDARY, k, path.basename(v))
      body:insert(field_part)
      len = len + #field_part

      -- add the file
      local stats = fs.statSync(v)
      body:insert({v})
      len = len + stats.size
      
      local str_end = '\r\n'
      body:insert(str_end)
      len = len + #str_end
    end
  end
  
  if #body > 0 then
    local str_end = string.format("--%s--\r\n",BOUNDARY)
    body:insert(str_end)
    len = len + #str_end
  end

  return len, body
end

return request

