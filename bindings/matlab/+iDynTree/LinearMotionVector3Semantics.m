classdef LinearMotionVector3Semantics < iDynTree.GeomVector3Semantics__LinearMotionVector3Semantics
  methods
    function self = LinearMotionVector3Semantics(varargin)
      self@iDynTree.GeomVector3Semantics__LinearMotionVector3Semantics(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(408, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function varargout = changePoint(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(409, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(411, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
    function varargout = compose(varargin)
     [varargout{1:nargout}] = iDynTreeMEX(410, varargin{:});
    end
  end
end
