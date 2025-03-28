classdef  OptiSol < casadi.PrintableCommon
    %OPTISOL [INTERNAL] 
    %
    %
    %A simplified interface for NLP modeling/solving.
    %
    %This class offers a view with solution retrieval facilities The API is
    % 
    %guaranteed to be stable.
    %
    %Joris Gillis, Erik Lambrechts
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1v
    %
    %C++ includes: optistack.hpp
    %
    %
  methods
    function varargout = type_name(self,varargin)
    %TYPE_NAME [INTERNAL] 
    %
    %  char = TYPE_NAME(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1307, self, varargin{:});
    end
    function varargout = disp(self,varargin)
    %DISP [INTERNAL] 
    %
    %  std::ostream & = DISP(self, bool more)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1308, self, varargin{:});
    end
    function varargout = str(self,varargin)
    %STR [INTERNAL] 
    %
    %  char = STR(self, bool more)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1309, self, varargin{:});
    end
    function varargout = value(self,varargin)
    %VALUE [INTERNAL] 
    %
    %  double = VALUE(self, DM x, {MX} values)
    %  double = VALUE(self, SX x, {MX} values)
    %  double = VALUE(self, MX x, {MX} values)
    %
    %Obtain value of expression at the current value
    %
    %In regular mode, teh current value is the converged solution In debug 
    %mode,
    % the value can be non-converged
    %
    %Parameters:
    %-----------
    %
    %values: 
    %Optional assignment expressions (e.g. x==3) to overrule the current
    % 
    %value
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L645
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L768-L770
    %
    %
    %
    %.......
    %
    %::
    %
    %  VALUE(self, DM x, {MX} values)
    %
    %
    %
    %[INTERNAL] 
    %Obtain value of expression at the current value
    %
    %In regular mode, teh current value is the converged solution In debug 
    %mode,
    % the value can be non-converged
    %
    %Parameters:
    %-----------
    %
    %values: 
    %Optional assignment expressions (e.g. x==3) to overrule the current
    % 
    %value
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L644
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L765-L767
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  VALUE(self, SX x, {MX} values)
    %
    %
    %
    %[INTERNAL] 
    %Obtain value of expression at the current value
    %
    %In regular mode, teh current value is the converged solution In debug 
    %mode,
    % the value can be non-converged
    %
    %Parameters:
    %-----------
    %
    %values: 
    %Optional assignment expressions (e.g. x==3) to overrule the current
    % 
    %value
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L645
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L768-L770
    %
    %
    %
    %.............
    %
    %
    %.......
    %
    %::
    %
    %  VALUE(self, MX x, {MX} values)
    %
    %
    %
    %[INTERNAL] 
    %Obtain value of expression at the current value
    %
    %In regular mode, teh current value is the converged solution In debug 
    %mode,
    % the value can be non-converged
    %
    %Parameters:
    %-----------
    %
    %values: 
    %Optional assignment expressions (e.g. x==3) to overrule the current
    % 
    %value
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L643
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L762-L764
    %
    %
    %
    %.............
    %
    %
      [varargout{1:nargout}] = casadiMEX(1310, self, varargin{:});
    end
    function varargout = value_variables(self,varargin)
    %VALUE_VARIABLES [INTERNAL] 
    %
    %  {MX} = VALUE_VARIABLES(self)
    %
    %get assignment expressions for the optimal solution
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L649
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L772-L774
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1311, self, varargin{:});
    end
    function varargout = value_parameters(self,varargin)
    %VALUE_PARAMETERS [INTERNAL] 
    %
    %  {MX} = VALUE_PARAMETERS(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1312, self, varargin{:});
    end
    function varargout = stats(self,varargin)
    %STATS [INTERNAL] 
    %
    %  struct = STATS(self)
    %
    %Get statistics.
    %
    %nlpsol stats are passed as-is. No stability can be guaranteed about 
    %this 
    %part of the API
    %
    %Extra doc: https://github.com/casadi/casadi/wiki/L_1w
    %
    %Doc source: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.hpp#L658
    %
    %Implementation: 
    %https://github.com/casadi/casadi/blob/develop/casadi/core/optistack.cpp#L780-L782
    %
    %
    %
      [varargout{1:nargout}] = casadiMEX(1313, self, varargin{:});
    end
    function varargout = opti(self,varargin)
    %OPTI [INTERNAL] 
    %
    %  Opti = OPTI(self)
    %
    %
      [varargout{1:nargout}] = casadiMEX(1314, self, varargin{:});
    end
    function self = OptiSol(varargin)
    %OPTISOL 
    %
    %  new_obj = OPTISOL()
    %
    %
      self@casadi.PrintableCommon(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = casadiMEX(1315, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
        if self.swigPtr
          casadiMEX(1316, self);
          self.SwigClear();
        end
    end
  end
  methods(Static)
  end
end
