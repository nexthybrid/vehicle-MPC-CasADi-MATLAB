function [f,g]=obj_casadi(f_function,x,p)
  tic
  [f,g] = f_function(x,p);
  f = full(f);
  g = full(g);
  disp('objective computed')
  toc
end
