classdef LinkUnknownWrenchContacts < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = LinkUnknownWrenchContacts(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1478, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = clear(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1479, self, varargin{:});
    end
    function varargout = resize(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1480, self, varargin{:});
    end
    function varargout = getNrOfContactsForLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1481, self, varargin{:});
    end
    function varargout = setNrOfContactsForLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1482, self, varargin{:});
    end
    function varargout = addNewContactForLink(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1483, self, varargin{:});
    end
    function varargout = addNewContactInFrame(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1484, self, varargin{:});
    end
    function varargout = addNewUnknownFullWrenchInFrameOrigin(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1485, self, varargin{:});
    end
    function varargout = contactWrench(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1486, self, varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1487, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1488, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
