local MatrixStack = Class()

function MatrixStack:__init()
  -- we use transposed matrices because opengl wants column major in memory and torch is row major
  self.projection_matrix            = torch.FloatTensor(4,4):t()
  self.model_view_matrix            = torch.FloatTensor(4,4):t()
  self.normal_matrix                = torch.FloatTensor(3,3)
  self.model_view_projection_matrix = torch.FloatTensor(4,4):t()

  self.stack = {}

  -- scratch
  self.scale_matrix = torch.FloatTensor(4,4):eye(4,4)
  self.rotation_matrix = torch.FloatTensor(4,4):eye(4,4)
  self.translation_matrix = torch.FloatTensor(4,4):eye(4,4)
  self.model_matrix = torch.FloatTensor(4,4):t()
end

function MatrixStack:push()
  table.insert(self.stack, self.model_view_matrix)
  self.model_view_matrix = self.model_view_matrix:clone() -- the :t() stride gets cloned
end

function MatrixStack:pop()
  self.model_view_matrix = table.remove(self.stack)
end

function MatrixStack:scale(scale)
  self.scale_matrix[{1,1}] = scale[1]
  self.scale_matrix[{2,2}] = scale[2]
  self.scale_matrix[{3,3}] = scale[3]

  -- we have to make a copy here because torch can't do *= with matices, prolly overwrites the data it needs as it goes
  -- we use this method to maintain the :t() on the model_view_matrix
  self.model_view_matrix:mm(self.model_view_matrix:clone(), self.scale_matrix)
end

function MatrixStack:translate(translation)
  self.translation_matrix[{{1,3},4}] = translation

  -- we have to make a copy here because torch can't do *= with matices, prolly overwrites the data it needs as it goes
  -- we use this method to maintain the :t() on the model_view_matrix
  self.model_view_matrix:mm(self.model_view_matrix:clone(), self.translation_matrix)
end

function MatrixStack:set_model(quaternion, translation)
  geom.quaternion.to_rotation_matrix(quaternion, self.rotation_matrix:sub(1,3,1,3))

  self.translation_matrix[{{1,3},4}] = translation

  self.model_matrix:mm(self.translation_matrix, self.rotation_matrix) 
  self.model_view_matrix:mm(self.model_view_matrix:clone(), self.model_matrix)
end

function MatrixStack:set_projection(projection_matix)
  self.projection_matrix:copy(projection_matix:float())
end

function MatrixStack:for_gl()
  -- extract normal matrix
  torch.inverse(self.normal_matrix, self.model_view_matrix[{{1, 3}, {1, 3}}])
  self.normal_matrix = self.normal_matrix:t()

  -- model_view_projection_matrix = projection_matrix * model_view_matrix
  self.model_view_projection_matrix:mm(self.projection_matrix, self.model_view_matrix)

  return self.projection_matrix, 
    self.model_view_matrix, 
    self.normal_matrix:t():contiguous(), 
    self.model_view_projection_matrix 
end

