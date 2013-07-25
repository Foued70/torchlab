slide = tutorial.util.print_slide()
next = tutorial.util.next
eval = tutorial.util.eval

slide()

print [[ A tensor is a view into a storage.  You can think of it as a
souped up pointer to a bag of data which is the storage. The tensor
holds information about dimensions, types and strides which allow you
to work quickly on the same data without reallocating memory. You can
select different dimensions and narrow along a dimension to operate on
the same underlying data. It is difficult to program in torch without
understanding this important concept.  ]]

eval("t = torch.range(1,12)")
eval("t")

next(slide) 

print [[ "t" is a torch double tensor of 1 dimension.  We can change
the view, but the underlying data stays the same.  ]]

eval("t:resize(3,4)")
eval("t:resize(3,2,2)")
next(slide)

print [[ We can make a new name for the same tensor.  ]]

eval("s = t")
eval("s")
eval("t:resize(3,4)")
print [[ "s" points to the same tensor ]]
eval("s")
next(slide)

print [[ We can create new tensors (new views) of the same underlying data.
 
"narrow" narrows a dimension creating a new tensor with different
offsets on the same underlying data.  ]]

eval("nrw = t:narrow(2,2,2)")
eval("nrw")

print [[ "select" removes a dimension creating a new tensor with
different offsets on the same underlying data.  ]]

eval("slct = t:select(2,2)")
eval("slct")
next(slide)

print [[
The underlying data is still the same.
]]
eval("t[3] = 100")
eval("t")
eval("nrw")
eval("slct")
next(slide)

print [[ The underlying data is in another torch object called a
storage. As storage is a 1 dimensional array representing data in
memory, or in some cases on disk.  ]]

eval("stor = t:storage()")
eval("stor")
next(slide)

print [[
The storages of all the tensors we have created are all the same:
]]
eval("nrw_stor = nrw:storage()")
eval("nrw_stor == stor")
eval("nrw_stor")
next(slide)

eval("slct_stor = slct:storage()")
eval("slct_stor == stor")
eval("slct_stor")
next(slide)

print [[
Check out the manual for more details on manipulating tensors.
http://www.torch.ch/manual/torch/tensor
]]
