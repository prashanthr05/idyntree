classdef MotionDerivativeOf_AngularMotionVector3_AngularMotionVector3 < SwigRef
  methods
    function self = MotionDerivativeOf_AngularMotionVector3_AngularMotionVector3(varargin)
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(198, varargin{:});
        tmp = iDynTreeMATLAB_wrap(198, varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(199, self);
        self.swigInd=uint64(0);
      end
    end
  end
  methods(Static)
  end
end
