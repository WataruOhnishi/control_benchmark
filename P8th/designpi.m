function varargout = designpi(varargin)
if length(varargin) == 3
    % Kpid = designpid(d,k,Kt,wp)
    % G = Kt/(s^2 + d*s + k)
    d = varargin{1};
    Kt = varargin{2};
    wp = varargin{3};
else
    % Kpid = designpid(Gn,wp)
    [num,den] = tfdata(varargin{1},'v');
    
    if length(num) == length(den) % matlab
        if ~(num(1) == 0 || num(2) == 0), error('not implemented'); end
        d = den(2)/den(1);
        Kt = num(2)/den(1);
        wp = varargin{2};
    else % octave
        d = den(2)/den(1);
        Kt = num(1)/den(1);
        wp = varargin{2};
    end
end

s = tf('s');

Mpi = [1 0 0;
    d Kt 0;
    0 0 Kt;];

wpi = [1;
    2*wp;
    wp^2;];

cpi = inv(Mpi)*wpi;

kp = cpi(2);
ki = cpi(3);

Kpi = (kp*s + ki)/s;

if nargout == 1
    varargout{1} = Kpi;
else
    varargout{1} = kp;
    varargout{2} = ki;
end

end
