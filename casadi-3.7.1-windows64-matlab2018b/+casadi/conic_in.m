function varargout = conic_in(varargin)
    %CONIC_IN [INTERNAL] 
    %
    %  {char} = CONIC_IN()
    %  char = CONIC_IN(int ind)
    %
    %Get QP solver input scheme name by index.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1eg
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/main/casadi/core/conic.hpp#L73
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/main/casadi/core/conic.cpp#L73-L90
    %
    %
    %.......
    %
    %::
    %
    %  CONIC_IN()
    %
    %
    %
    %[INTERNAL] 
    %Get input scheme of QP solvers.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1ee
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/main/casadi/core/conic.hpp#L61
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/main/casadi/core/conic.cpp#L61-L65
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  CONIC_IN(int ind)
    %
    %
    %
    %[INTERNAL] 
    %Get QP solver input scheme name by index.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1eg
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/main/casadi/core/conic.hpp#L73
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/main/casadi/core/conic.cpp#L73-L90
    %
    %
    %.............
    %
    %
  [varargout{1:nargout}] = casadiMEX(852, varargin{:});
end
