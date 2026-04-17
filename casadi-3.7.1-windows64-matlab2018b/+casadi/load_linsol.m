function varargout = load_linsol(varargin)
    %LOAD_LINSOL [INTERNAL] 
    %
    %  LOAD_LINSOL(char name)
    %
    %Explicitly load a plugin dynamically.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/main/casadi/core/linsol.hpp#L210
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/main/casadi/core/linsol.cpp#L210-L212
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(901, varargin{:});
end
