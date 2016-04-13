classdef TvLQRTree < HybridDrakeSystem

%constructor with trajectories
     properties
    zero_mode_num=1;
    tilqr_mode_num=2;
    num_tvlqr_modes=0;
    xtraj;
    utraj;
    xG
    uG
    Vf
    
    Vtv={};
    
    % precompute some values to optimize speed of guards and transitions
    S1=[];
    s2=[];
    s3=[];
    x0=[];
    t0=[];
    m=[];
     end
  
  methods
    function obj=TvLQRTree(x_intraj,u_intraj,tvlqr,Vtv)
      
      
%       obj=obj@HybridDrakeSystem(tilqr.getNumInputs(),tilqr.getNumOutputs());
      obj=obj@HybridDrakeSystem(tvlqr.getNumInputs(),tvlqr.getNumOutputs());    %will tvlqr give the same out put as tilqr  
      obj.xtraj=x_intraj;
      obj.utraj=u_intraj;
      typecheck(tvlqr,'AffineSystem');
      typecheck(Vtv,'QuadraticLyapunovFunction');
      
      N = obj.num_tvlqr_modes;
      [obj,mode_num] = addMode(obj,tvlqr);
       obj.Vtv=Vtv;
       
       nX=obj.getNumInputs();
      ts=Vtv.S.getBreaks();
      for i=1:length(ts)
        n=length(obj.t0);
        obj.S1(nX*n+(1:nX),nX*n+(1:nX))=Vtv.S.eval(ts(i));
        obj.s2(n+1,nX*n+(1:nX))=Vtv.s1.eval(ts(i))';
        obj.s3(n+1,1)=Vtv.s2.eval(ts(i));
        obj.t0(n+1)=ts(i);
        obj.x0(:,n+1)=xtraj.eval(ts(i));
        obj.m(n+1)=mode_num;
      end
      %take catre of this later
      
      %       obj = obj.setInputFrame(tilqr.getInputFrame());
%       obj = obj.setOutputFrame(tilqr.getOutputFrame());
%             
%       typecheck(xG,'Point');
%       typecheck(uG,'Point');
%       obj.xG = xG;
%       obj.uG = uG;
%       
%       % add a "do nothing" controller (aka zero controller)
%       c0=ConstOrPassthroughSystem(zeros(obj.num_y,1),obj.num_u);
%       c0=c0.setInputFrame(tilqr.getInputFrame());
%       c0=c0.setOutputFrame(tilqr.getOutputFrame());
%       [obj,obj.zero_mode_num] = addMode(obj,c0);
%       
%       % switch from zero control to any controller
%       obj = addTransition(obj,obj.zero_mode_num,@inAnyFunnelGuard,@transitionIntoAnyFunnel,true,true);
% 
%       % add the ti controller
%       [obj,obj.tilqr_mode_num] = addMode(obj,tilqr);
% 
%       % switch to object coordinates
%       V = V.inFrame(obj.getInputFrame);
%       
%       % switch from ti controller to any controller
%       obj = addTransition(obj,obj.tilqr_mode_num,@(obj,t,~,x)1-eval(V,t,x),@transitionIntoAnyFunnel,true,true);
%       
%       %% precompute quadratic forms
%       if ~isTI(V) error('i''ve assumed so far that the initial controller / lyapunov pair is time invariant'); end
%       V = V.extractQuadraticLyapunovFunction();
%       obj.S1=V.S;
%       obj.s2=V.s1';
%       obj.s3=V.s2;
%       obj.t0=0;
%       obj.x0=double(obj.xG.inFrame(obj.getInputFrame));
%       obj.m=obj.tilqr_mode_num;
    end
    
       
