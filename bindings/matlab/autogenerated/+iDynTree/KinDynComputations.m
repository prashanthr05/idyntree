classdef KinDynComputations < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = KinDynComputations(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1788, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1789, self);
        self.SwigClear();
      end
    end
    function varargout = loadRobotModel(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1790, self, varargin{:});
    end
    function varargout = loadRobotModelFromFile(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1791, self, varargin{:});
    end
    function varargout = loadRobotModelFromString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1792, self, varargin{:});
    end
    function varargout = isValid(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1793, self, varargin{:});
    end
    function varargout = setFrameVelocityRepresentation(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1794, self, varargin{:});
    end
    function varargout = getFrameVelocityRepresentation(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1795, self, varargin{:});
    end
    function varargout = getNrOfDegreesOfFreedom(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1796, self, varargin{:});
    end
    function varargout = getDescriptionOfDegreeOfFreedom(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1797, self, varargin{:});
    end
    function varargout = getDescriptionOfDegreesOfFreedom(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1798, self, varargin{:});
    end
    function varargout = getNrOfLinks(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1799, self, varargin{:});
    end
    function varargout = getNrOfFrames(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1800, self, varargin{:});
    end
    function varargout = getFloatingBase(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1801, self, varargin{:});
    end
    function varargout = setFloatingBase(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1802, self, varargin{:});
    end
    function varargout = model(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1803, self, varargin{:});
    end
    function varargout = getRobotModel(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1804, self, varargin{:});
    end
    function varargout = getRelativeJacobianSparsityPattern(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1805, self, varargin{:});
    end
    function varargout = getFrameFreeFloatingJacobianSparsityPattern(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1806, self, varargin{:});
    end
    function varargout = setJointPos(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1807, self, varargin{:});
    end
    function varargout = setRobotState(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1808, self, varargin{:});
    end
    function varargout = getRobotState(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1809, self, varargin{:});
    end
    function varargout = getWorldBaseTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1810, self, varargin{:});
    end
    function varargout = getBaseTwist(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1811, self, varargin{:});
    end
    function varargout = getJointPos(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1812, self, varargin{:});
    end
    function varargout = getJointVel(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1813, self, varargin{:});
    end
    function varargout = getModelVel(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1814, self, varargin{:});
    end
    function varargout = getFrameIndex(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1815, self, varargin{:});
    end
    function varargout = getFrameName(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1816, self, varargin{:});
    end
    function varargout = getWorldTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1817, self, varargin{:});
    end
    function varargout = getRelativeTransformExplicit(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1818, self, varargin{:});
    end
    function varargout = getRelativeTransform(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1819, self, varargin{:});
    end
    function varargout = getFrameVel(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1820, self, varargin{:});
    end
    function varargout = getFrameAcc(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1821, self, varargin{:});
    end
    function varargout = getFrameFreeFloatingJacobian(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1822, self, varargin{:});
    end
    function varargout = getRelativeJacobian(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1823, self, varargin{:});
    end
    function varargout = getRelativeJacobianExplicit(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1824, self, varargin{:});
    end
    function varargout = getFrameBiasAcc(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1825, self, varargin{:});
    end
    function varargout = getCenterOfMassPosition(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1826, self, varargin{:});
    end
    function varargout = getCenterOfMassVelocity(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1827, self, varargin{:});
    end
    function varargout = getCenterOfMassJacobian(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1828, self, varargin{:});
    end
    function varargout = getCenterOfMassBiasAcc(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1829, self, varargin{:});
    end
    function varargout = getAverageVelocity(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1830, self, varargin{:});
    end
    function varargout = getAverageVelocityJacobian(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1831, self, varargin{:});
    end
    function varargout = getCentroidalAverageVelocity(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1832, self, varargin{:});
    end
    function varargout = getCentroidalAverageVelocityJacobian(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1833, self, varargin{:});
    end
    function varargout = getLinearAngularMomentum(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1834, self, varargin{:});
    end
    function varargout = getLinearAngularMomentumJacobian(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1835, self, varargin{:});
    end
    function varargout = getCentroidalTotalMomentum(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1836, self, varargin{:});
    end
    function varargout = getFreeFloatingMassMatrix(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1837, self, varargin{:});
    end
    function varargout = inverseDynamics(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1838, self, varargin{:});
    end
    function varargout = generalizedBiasForces(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1839, self, varargin{:});
    end
    function varargout = generalizedGravityForces(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1840, self, varargin{:});
    end
    function varargout = generalizedExternalForces(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1841, self, varargin{:});
    end
    function varargout = inverseDynamicsInertialParametersRegressor(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1842, self, varargin{:});
    end
  end
  methods(Static)
  end
end
