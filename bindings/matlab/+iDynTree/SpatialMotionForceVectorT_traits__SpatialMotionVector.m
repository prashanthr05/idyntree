classdef SpatialMotionForceVectorT_traits__SpatialMotionVector < SwigRef
  methods
    function self = SpatialMotionForceVectorT_traits__SpatialMotionVector(varargin)
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(228, varargin{:});
        tmp = iDynTreeMATLAB_wrap(228, varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(229, self);
        self.swigInd=uint64(0);
      end
    end
  end
  methods(Static)
  end
end
