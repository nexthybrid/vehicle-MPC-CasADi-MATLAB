function [c,ceq,gradc,gradceq]=nonlin_casadi(g_function,x,p)
  tic
  % Morph the constraints into c<=0, ceq==0
  %
  % CasADi way: lbg <= g <= ubg
  [g,J,lbg,ubg] = g_function(x,p);
  g = full(g);
  lbg = full(lbg);
  ubg = full(ubg);
  J = sparse(J);
  
  % Classify into eq/ineq
  eq = lbg==ubg;
  ineq = lbg~=ubg;
  
  % Classify into free/fixed
  lbg_fixed = lbg~=-inf;
  ubg_fixed = ubg~=inf;
  
  % Constraint vector
  c = [lbg(ineq&lbg_fixed)-g(ineq&lbg_fixed);g(ineq&ubg_fixed)-ubg(ineq&ubg_fixed)];
  ceq = g(eq)-lbg(eq);
  
  % Constraint Jacobian tranposed
  gradc = [-J(ineq&lbg_fixed,:);J(ineq&ubg_fixed,:)]';
  gradceq = J(eq,:)';
  disp('constraints computed')
  toc
end
