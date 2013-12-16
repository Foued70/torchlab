  CreateHTML = Class()


  local io = require "io"
  local path = require "path"
  require "gnuplot"
  function CreateHTML:__init(filepath, title, subtitle)
    self.base_dir = path.dirname(filepath)
    self.file = io.open(filepath, 'w')
    self.title = title or ""
    self.subtitle = subtitle or ""
  end

function CreateHTML:beginFile()
    self.file:write([[
      <html>
      <head><TITLE>]])
    self.file:write(self.title)
    self.file:write([[</TITLE></head>
      <body BGCOLOR="#FFFFFF"><h1>]])
    self.file:write(self.subtitle)
    self.file:write([[</h1>
      <BR>
      <style type="text/css">
    .left{float:left;}
  </style>

  <style>
    td
    {
    height:50px;
    padding:15px;
  }
</style>

<table>
    ]])
  end
  function CreateHTML:fwrite (...)
    return self.file:write(string.format(...))
  end

function CreateHTML:addMultipleLines(n)
  if(n) then
  s = ''
  for i =1,n do
    s = s .. [[<br/>]]
  end

  self.file:write(s)
end
end
function CreateHTML:newRow()
  return self.file:write([[<tr>]])
end
function CreateHTML:endRow()
  return self.file:write([[</tr>]])
end
function CreateHTML:addImageInNewRow(title, name, description)
  self.file:write(string.format("<tr>\n<td>\n<h2>\n%s</h2><img src=\"%s\" /><pre>\n%s\n</pre>\n", title, name,description))
  self:addMultipleLines(num_new_lines)
  self.file:write("</td>\n</tr>\n")
end

function CreateHTML:addImageInNewColumn(title, name, description)
  self.file:write(string.format("<td>\n<h2>\n%s</h2><img src=\"%s\" /><pre>\n%s\n</pre>\n", title, name,description))
  self:addMultipleLines(num_new_lines)
  self.file:write("</td>\n")
end

--save location should be relative to html file and folder should exist
function CreateHTML:addImageFromTensor(title, tensor, save_location, description, new_row, num_new_new_lines)
    --gnuplot.imagesc(tensor)
    --gnuplot.title(description)
    --gnuplot.figprint(path.join(self.base_dir,save_location))
    --image.display(tensor)
    image.save(path.join(self.base_dir,save_location), tensor:clone()/tensor:clone():max())
    if(new_row) then
      self:addImageInNewRow(title, save_location, description, num_new_new_lines)
    else
      self:addImageInNewColumn(title, save_location, description, num_new_new_lines)
    end
end

function CreateHTML:endFile()
  self.file:write('</table></body></html>\n')
end
function CreateHTML:close_file()
  self.file:close()
end

function CreateHTML:doAllTest()
  self:beginFile()
  self:addImageFromTensor("image 1", torch.range(1,1000):reshape(100,10), "image1.png", "this is the second test i am doing", false)
  self:addImageFromTensor("image 2", torch.range(1,10000):reshape(100,100), "image2.png", "this is the second test i am doing", false)
  self:addImageFromTensor("image 3", torch.range(1,1000):reshape(100,10), "image3.png", "this is the second test i am doing", false)
  self:newRow()
  self:addImageFromTensor("fourth image", torch.range(1,10000):reshape(100,100), "image4.png", "this is the second test i am doing", false)

  self:endRow()
  self:newRow()
  self:addImageFromTensor("fourth image", torch.range(1,10000):reshape(100,100), "image4.png", "this is the second test i am doing", false)
  self:addImageFromTensor("fourth image", torch.range(1,10000):reshape(100,100), "image4.png", "this is the second test i am doing", false)

  self:endRow()

  --self:addImagesSideBySide("image1.png", "image2.png")
  self:endFile()
  self:close_file()
  gnuplot.closeWindow(1)
end

return CreateHTML

