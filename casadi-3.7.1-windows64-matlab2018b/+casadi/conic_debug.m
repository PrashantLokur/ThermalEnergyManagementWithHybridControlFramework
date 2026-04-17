function varargout = conic_debug(varargin)
    %CONIC_DEBUG [INTERNAL] 
    %
    %  CONIC_DEBUG(Function f, std::ostream & file)
    %  CONIC_DEBUG(Function f, char filename)
    %
    %Generate native code in the interfaced language for debugging
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/main/casadi/core/conic.hpp#L55
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/main/casadi/core/conic.cpp#L55-L59
    %
    %
    %.......
    %
    %::
    %
    %  CONIC_DEBUG(Function f, std::ostream & file)
    %
    %
    %
    %[INTERNAL] 
    %Generate native code in the interfaced language for debugging
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/main/casadi/core/conic.hpp#L55
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/main/casadi/core/conic.cpp#L55-L59
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  CONIC_DEBUG(Function f, char filename)
    %
    %
    %
    %[INTERNAL] 
    %Generate native code in the interfaced language for debugging
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/main/casadi/core/conic.hpp#L49
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/main/casadi/core/conic.cpp#L49-L53
    %
    %
    %.............
    %
    %
  [varargout{1:nargout}] = casadiMEX(862, varargin{:});
end
