local libui = require 'libui'

local Timer = Class()

function Timer:__init()
  self.previous = nil
  self.current = nil
  self.delta = nil
  self.MAX_TIME_STEP = 1/30  
end

function Timer:reset()
	self.previous = nil
  	self.current = nil
  	self.delta = nil
end

function Timer:start()
	self.previous = nil
	self.current = self:get_time_seconds()
	self.delta = nil
end

function Timer:tick()
	self.previous = self.current
	self.current = self:get_time_seconds()
	self.delta = self.current - self.previous

	--Limiting timestep to stop simulations from spiraling out of control during performance spikes
	if self.MAX_TIME_STEP < self.delta then
		self.delta = self.MAX_TIME_STEP
	end
end

function Timer:get_time_seconds()
	return sys.clock()
end

