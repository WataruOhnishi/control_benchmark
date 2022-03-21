function varargout = designpid(varargin)
if length(varargin) == 4
    % Kpid = designpid(d,k,Kt,wp)
    % G = Kt/(s^2 + d*s + k)
    d = varargin{1};
    k = varargin{2};
    Kt = varargin{3};
    wp = varargin{4};
else
    % Kpid = designpid(Gn,wp)
    [num,den] = tfdata(varargin{1},'v');
    
    if length(num) == length(den) % matlab
        if ~(num(1) == 0 || num(2) == 0), error('not implemented'); end
        d = den(2)/den(1);
        k = den(3)/den(1);
        Kt = num(3)/den(1);
        wp = varargin{2};
    else % octave
        d = den(2)/den(1);
        k = den(3)/den(1);
        Kt = num(1)/den(1);
        wp = varargin{2};
    end
end

s = tf('s');

Mpid = [1 0 0 0 0;
    d 1 0 0 0;
    k d Kt 0 0;
    0 k 0 Kt 0;
    0 0 0 0 Kt;];

wpid = [1;
    4*wp;
    6*wp^2;
    4*wp^3;
    wp^4];

cpid = inv(Mpid)*wpid;

a1 = cpid(2);
b2 = cpid(3);
b1 = cpid(4);
b0 = cpid(5);

Kpid = (b2*s^2 + b1*s + b0)/(s^2 + a1*s);

if nargout == 1
    varargout{1} = Kpid;
else
    tau = 1/a1;
    ki = b0*tau;
    kp = (b1-ki)*tau;
    kd = (b2-kp)*tau;
    
    varargout{1} = kp;
    varargout{2} = ki;
    varargout{3} = kd;
    varargout{4} = tau;
end

end
