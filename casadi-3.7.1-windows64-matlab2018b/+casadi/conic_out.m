function varargout = conic_out(varargin)
    %CONIC_OUT [INTERNAL] 
    %
    %  {char} = CONIC_OUT()
    %  char = CONIC_OUT(int ind)
    %
    %Get output scheme name by index.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1eh
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/main/casadi/core/conic.hpp#L92
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/main/casadi/core/conic.cpp#L92-L101
    %
    %
    %.......
    %
    %::
    %
    %  CONIC_OUT(int ind)
    %
    %
    %
    %[INTERNAL] 
    %Get output scheme name by index.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1eh
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/main/casadi/core/conic.hpp#L92
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/main/casadi/core/conic.cpp#L92-L101
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  CONIC_OUT()
    %
    %
    %
    %[INTERNAL] 
    %Get QP solver output scheme of QP solvers.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1ef
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/main/casadi/core/conic.hpp#L67
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/main/casadi/core/conic.cpp#L67-L71
    %
    %
    %.............
    %
    %
  [varargout{1:nargout}] = casadiMEX(853, varargin{:});
end
