function varargout = doc_linsol(varargin)
    %DOC_LINSOL [INTERNAL] 
    %
    %  char = DOC_LINSOL(char name)
    %
    %Get the documentation string for a plugin.
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/main/casadi/core/linsol.hpp#L214
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/main/casadi/core/linsol.cpp#L214-L216
    %
    %
    %
  [varargout{1:nargout}] = casadiMEX(902, varargin{:});
end
