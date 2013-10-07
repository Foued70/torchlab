Class()
-- optimize <func()> with respect to <start_param>, and current best
-- result <best_result> by testing +,- <wiggle> and repeating binary
-- search by halving wiggle until wiggle is smaller that <stop>.
-- Arguments can be passed to <func()> with the args in {...}
function convex_binary_search(wiggle,stop,start_param,best_result,func,...)
   print("Optimizing")
   count = 0
   current_best_result = best_result
   best_vals           = nil
   best_param          = start_param
   while (wiggle >= stop) do 
      printf(" - [%d] testing wiggle %2.4f (>%2.4f) window (%2.4f,%2.4f) around %2.4f",
             count,wiggle,stop,best_param-wiggle,best_param+wiggle, best_param)
      log.tic()
      for _,wig in pairs({ -wiggle,wiggle}) do
         param = best_param + wig
         test_val,vals = func(param,...)
         if test_val < current_best_result then
            best_param          = param
            current_best_result = test_val;
            best_vals           = vals
            printf(" -     new best found %2.4f %2.4f", best_param, current_best_result);
         end
      end
      printf(" -     in %2.2fs best: param: %2.4f, score: %2.4f",
             log.toc()/1000, best_param, current_best_result)
      wiggle = wiggle / 2
      count = count + 1
   end
   return best_param, current_best_result, best_vals
end
