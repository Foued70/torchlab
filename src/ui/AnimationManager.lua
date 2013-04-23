local libui = require 'libui'

local AnimationManager = Class()

BEZIER_START_BEHAVIORS = {
	["LINEAR"]	= torch.Tensor({0.5, 0.5}),
	["SLOW"]	= torch.Tensor({0.5, 0.0}),
	["FAST"]	= torch.Tensor({0.0, 0.5})
}

BEZIER_END_BEHAVIORS = {
	["LINEAR"]	= torch.Tensor({0.5, 0.5}),
	["SLOW"]	= torch.Tensor({0.5, 1.0}),
	["FAST"]	= torch.Tensor({1.0, 0.5})
}

function AnimationManager:__init()
  self.timer = ui.Timer.new()
  self.animations = {}
end

function AnimationManager:add(animating_value, end_value, transition_time, bezier_start_behavior, bezier_end_behavior )
	local animation = {}
	animation.start_value 		= animating_value:clone()
	animation.end_value 		= end_value
	animation.transition_time 	= transition_time
	animation.start_handle 		= bezier_start_behavior
	animation.end_handle 		= bezier_end_behavior
	animation.progress			= 0

	self.animations[animating_value] = animation

	return true
end

function AnimationManager:remove(animating_value)
	if self.animations[animating_value] then
		log.trace("Can't remove")
		log.trace(animating_value)
		log.trace("It's not being animated!")
		return nil
	end

	self.animations[animating_value] = nil

	return true
end

function AnimationManager:tick_animation(animating_value, delta_time)
	self.animations[animating_value].progress = self.animations[animating_value].progress + delta_time / self.animations[animating_value].transition_time
	if self.animations[animating_value].progress > 1 then
		self.animations[animating_value].progress = 1
	end

	local normalized_value = self:calculate_point_on_bezier_curve(	self.animations[animating_value].progress, 
																	self.animations[animating_value].start_handle,
																	self.animations[animating_value].end_handle)

		for s = 1, animating_value:size(1) do
			animating_value[s] 		= normalized_value *
									(self.animations[animating_value].end_value[s] - self.animations[animating_value].start_value[s]) +
									self.animations[animating_value].start_value[s]

		end
end

function AnimationManager:calculate_point_on_bezier_curve(progress, start_handle, end_handle)
	local point_on_curve = torch.Tensor(2)

	local temp_a = 3*(1-progress)*(1-progress)*progress
	local temp_b = 3*(1-progress)*progress*progress
	local temp_c = progress*progress*progress

	torch.mul(point_on_curve, start_handle, temp_a)
	torch.add(point_on_curve, point_on_curve, torch.mul(end_handle, temp_b))
	torch.add(point_on_curve, point_on_curve, temp_c)

	return point_on_curve[2]
end

function AnimationManager:tick_all()
	if self.timer.delta == nil then
		self.timer:start()
	end
	self.timer:tick()

	for key,value in pairs(self.animations) do
		self:tick_animation(key, self.timer.delta)
		if self.animations[key].progress >= 1 then
			self.animations[key] = nil
		end
	end

	if self:needsAnimating() == false then
		self.timer:reset()
	end
end

function AnimationManager:needsAnimating()
	if next(self.animations) then
		return true
	end

	return false 
end

