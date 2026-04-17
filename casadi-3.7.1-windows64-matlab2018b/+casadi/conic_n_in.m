function varargout = conic_n_in(varargin)
    %CONIC_N_IN [INTERNAL] 
    %
    %  int = CONIC_N_IN()
    %
    %Get the number of QP solver inputs.
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1ei
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/main/casadi/core/conic.hpp#L103
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/main/casadi/core/conic.cpp#L103-L105
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(854, varargin{:});
end
