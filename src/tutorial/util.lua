Class()
io = require 'io'
repl = require 'repl'

function print_slide(start)
   slide_no = start or 1
   return function () 
      print("-- ##### Slide: ".. slide_no .. " ##### --") 
      slide_no = slide_no + 1
   end
end

-- little function to pause execution, and request user input
function next(slide)
   local answer = nil
   while answer ~= '' and answer ~= 'y' and answer ~= 'Y' and neverstall ~= true do
      io.write("continue ([y]/n/!)? ")
      io.flush()
      answer=io.read()
      if answer == '!' then
         neverstall = true
      end
      if answer == 'n' then
         print('to quit type ctrl^D')
--          os.exit()
      end
      if slide then slide() end
   end
   print ''
end

-- an eval which prints a string then evaluates it.
function eval (str)
   print("> " .. str)
   repl.evaluateLine(str)
end
