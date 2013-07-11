local uv = require 'uv_native'

local Timer = Class()

function Timer:__init()
  self.previous = nil
  self.current = nil
  self.delta = nil
  self.MAX_TIME_STEP = 1000/30  
end

function Timer:reset()
	self.previous = nil
  	self.current = nil
  	self.delta = nil
end

function Timer:start()
	self.previous = nil
	self.current = self:get_time()
	self.delta = nil
end

function Timer:tick()
	self.previous = self.current
	self.current = self:get_time()
	self.delta = self.current - self.previous
end

function Timer:get_time()
	return uv.hrtime()
end

