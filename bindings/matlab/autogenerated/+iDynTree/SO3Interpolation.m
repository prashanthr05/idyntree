classdef SO3Interpolation < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = SO3Interpolation(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(864, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = setData(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(865, self, varargin{:});
    end
    function varargout = setInitialConditions(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(866, self, varargin{:});
    end
    function varargout = evaluatePoint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(867, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(868, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
