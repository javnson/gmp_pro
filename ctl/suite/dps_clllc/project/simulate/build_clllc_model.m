function build_clllc_model
%BUILD_CLLLC_MODEL Create the self-contained CLLLC/DAB switching SIL model.
root = fileparts(mfilename('fullpath'));
repo = fullfile(root,'..','..','..','..','..');
addpath(genpath(fullfile(repo,'slib','install_path','R2024b')));
fsbb = fullfile(repo,'ctl','suite','dps_fsbb','project','simulate','MCS_STD_FSBB_MODEL.slx');
load_system(fsbb); % source of the verified GMP ADC sensor blocks
load_system('gmp_sil_core_pack');
libs = {'spsUniversalBridgeLib','spsSeriesRLCBranchLib', ...
        'spsLinearTransformerLib','spsDCVoltageSourceLib','spsGroundLib','spspowerguiLib'};
for k=1:numel(libs), load_system(libs{k}); end

mdl = 'GMP_STD_CLLLC_MODEL';
if bdIsLoaded(mdl), close_system(mdl,0); end
new_system(mdl); open_system(mdl);
set_param(mdl,'SolverType','Fixed-step','Solver','FixedStepDiscrete', ...
    'FixedStep','2e-8','StopTime','0.02','SignalLogging','on');

panel = add_block(['gmp_sil_core_pack/GMP Stanrdard Digital Power Controller ' ...
    'Panel (SIL) Pro Edition'],[mdl '/GMP SIL Controller'], ...
    'Position',[70 120 300 270]);
plant = add_block('simulink/Ports & Subsystems/Subsystem',[mdl '/GMP STD CLLLC Module'], ...
    'Position',[430 90 700 310]);
Simulink.SubSystem.deleteContents(plant);
build_module(plant,fsbb);
add_block('spspowerguiLib/powergui',[mdl '/powergui'], ...
    'SimulationMode','Discrete','SampleTime','2e-8','Position',[760 310 850 350]);

z16=add_block('simulink/Sources/Constant',[mdl '/Panel Commands'], ...
    'Value','zeros(16,1)','Position',[75 300 155 330]);
z8=add_block('simulink/Sources/Constant',[mdl '/Digital Inputs'], ...
    'Value','zeros(8,1)','Position',[75 350 155 380]);
phP=get_param(panel,'PortHandles'); phM=get_param(plant,'PortHandles');
add_line(mdl,phM.Outport(1),phP.Inport(1),'autorouting','on');
add_line(mdl,get_param(z16,'PortHandles').Outport,phP.Inport(2),'autorouting','on');
add_line(mdl,get_param(z8,'PortHandles').Outport,phP.Inport(3),'autorouting','on');
add_line(mdl,phM.Outport(2),phP.Inport(4),'autorouting','on');
memE=add_block('simulink/Discrete/Memory',[mdl '/Enable ZOH'],'InitialCondition','0','Position',[340 120 375 145]);
memP=add_block('simulink/Discrete/Memory',[mdl '/PWM ZOH'],'InitialCondition','zeros(8,1)','Position',[340 165 375 190]);
memM=add_block('simulink/Discrete/Memory',[mdl '/Monitor ZOH'],'InitialCondition','zeros(16,1)','Position',[340 215 375 240]);
add_line(mdl,phP.Outport(1),get_param(memE,'PortHandles').Inport,'autorouting','on');
add_line(mdl,phP.Outport(2),get_param(memP,'PortHandles').Inport,'autorouting','on');
add_line(mdl,phP.Outport(4),get_param(memM,'PortHandles').Inport,'autorouting','on');
add_line(mdl,get_param(memE,'PortHandles').Outport,phM.Inport(1),'autorouting','on');
add_line(mdl,get_param(memP,'PortHandles').Outport,phM.Inport(2),'autorouting','on');
add_line(mdl,get_param(memM,'PortHandles').Outport,phM.Inport(3),'autorouting','on');

mon=add_block('simulink/Sinks/Scope',[mdl '/Controller Monitor'], ...
    'Position',[765 115 805 155]);
adc=add_block('simulink/Sinks/Scope',[mdl '/ADC Codes'], ...
    'Position',[765 200 805 240]);
add_line(mdl,phP.Outport(4),get_param(mon,'PortHandles').Inport,'autorouting','on');
add_line(mdl,phM.Outport(1),get_param(adc,'PortHandles').Inport,'autorouting','on');
tom=add_block('simulink/Sinks/To Workspace',[mdl '/Log Monitor'], ...
    'VariableName','clllc_monitor','SaveFormat','Timeseries','Position',[760 165 850 190]);
toa=add_block('simulink/Sinks/To Workspace',[mdl '/Log ADC'], ...
    'VariableName','clllc_adc','SaveFormat','Timeseries','Position',[760 250 850 275]);
add_line(mdl,phP.Outport(4),get_param(tom,'PortHandles').Inport,'autorouting','on');
add_line(mdl,phM.Outport(1),get_param(toa,'PortHandles').Inport,'autorouting','on');

a=Simulink.Annotation(mdl,['CLLLC: 1:1, Lm=120 uH, Lr1=Lr2=20 uH, ' ...
    'Cr1=Cr2=120 nF, 75--150 kHz. PWM monitor fields carry period and dead time.']);
a.Position=[420 430 900 470];
save_system(mdl,fullfile(root,[mdl '.slx']));
close_system(mdl,0); close_system('MCS_STD_FSBB_MODEL',0);
fprintf('Created %s\n',fullfile(root,[mdl '.slx']));
end

function build_module(sub,fsbb)
sub=getfullname(sub);
inE=add_block('simulink/Ports & Subsystems/In1',[sub '/Enable'],'Port','1','Position',[25 40 55 60]);
inP=add_block('simulink/Ports & Subsystems/In1',[sub '/PWM Compare and Phase'],'Port','2','Position',[25 85 55 105]);
inM=add_block('simulink/Ports & Subsystems/In1',[sub '/Monitor'],'Port','3','Position',[25 130 55 150]);
outA=add_block('simulink/Ports & Subsystems/Out1',[sub '/ADC Result'],'Port','1','Position',[1130 70 1160 90]);
outT=add_block('simulink/Ports & Subsystems/Out1',[sub '/Sample Trigger'],'Port','2','Position',[1130 120 1160 140]);

epwm=add_block('simulink/Ports & Subsystems/Subsystem',[sub '/Advanced ePWM'], ...
    'Position',[130 35 360 165]); Simulink.SubSystem.deleteContents(epwm);
build_epwm(epwm);
pe=get_param(epwm,'PortHandles');
add_line(sub,get_param(inE,'PortHandles').Outport,pe.Inport(1));
add_line(sub,get_param(inP,'PortHandles').Outport,pe.Inport(2));
add_line(sub,get_param(inM,'PortHandles').Outport,pe.Inport(3));
add_line(sub,pe.Outport(2),get_param(outT,'PortHandles').Inport,'autorouting','on');

pri=add_block('spsUniversalBridgeLib/Universal Bridge',[sub '/Primary Full Bridge'], ...
    'Arms','2','Device','MOSFET / Diodes','Ron','0.01', ...
    'SnubberResistance','1e6','Position',[430 210 530 330]);
sec=add_block('spsUniversalBridgeLib/Universal Bridge',[sub '/Secondary Full Bridge'], ...
    'Arms','2','Device','MOSFET / Diodes','Ron','0.01', ...
    'SnubberResistance','1e6','Position',[880 210 980 330]);
gate_demux=add_block('simulink/Signal Routing/Demux',[sub '/Bridge Gate Split'], ...
    'Outputs','[4 4]','Position',[390 40 420 100]);
hd=get_param(gate_demux,'PortHandles');
add_line(sub,pe.Outport(1),hd.Inport);
add_line(sub,hd.Outport(1),get_param(pri,'PortHandles').Inport);
add_line(sub,hd.Outport(2),get_param(sec,'PortHandles').Inport,'autorouting','on');

src=add_block('spsDCVoltageSourceLib/DC Voltage Source',[sub '/48 V Input'], ...
    'Amplitude','48','Position',[400 390 450 460]);
tr=add_block('spsLinearTransformerLib/Linear Transformer',[sub '/1 to 1 Transformer'], ...
    'UNITS','SI','NominalPower','[1000 100e3]','winding1','[48 0.01 1e-9]', ...
    'winding2','[48 0.01 1e-9]','ThreeWindings','off','RmLm','[1e6 120e-6]', ...
    'Position',[680 225 780 315]);
t1=add_block('spsSeriesRLCBranchLib/Series RLC Branch',[sub '/Primary Lr Cr'], ...
    'BranchType','RLC','Resistance','0.05','Inductance','20e-6', ...
    'Capacitance','120e-9','Position',[575 215 640 255]);
t2=add_block('spsSeriesRLCBranchLib/Series RLC Branch',[sub '/Secondary Lr Cr'], ...
    'BranchType','RLC','Resistance','0.05','Inductance','20e-6', ...
    'Capacitance','120e-9','Position',[800 215 865 255]);
cout=add_block('spsSeriesRLCBranchLib/Series RLC Branch',[sub '/Output Capacitor'], ...
    'BranchType','C','Capacitance','440e-6','Position',[1025 245 1075 295]);
loadb=add_block('spsSeriesRLCBranchLib/Series RLC Branch',[sub '/10 Ohm Load'], ...
    'BranchType','R','Resistance','10','Position',[1025 335 1075 385]);
gnd=add_block('spsGroundLib/Ground',[sub '/Ground'],'Position',[420 500 450 530]);

base='MCS_STD_FSBB_MODEL/GMP STD FSBB Module/';
vpri=copy_sensor([base 'Input Voltage Measurement'],[sub '/Primary Voltage ADC'],[370 555 525 625]);
vsec=copy_sensor([base 'Output Voltage Measurement'],[sub '/Secondary Voltage ADC'],[930 555 1085 625]);
ipri=copy_sensor([base 'Output Current Measurement'],[sub '/Primary Current ADC'],[535 345 690 415]);
ires=copy_sensor([base 'Inductor Current Measurement'],[sub '/Resonant Current ADC'],[535 455 695 525]);
isec=copy_sensor([base 'Output Current Measurement'],[sub '/Secondary Current ADC'],[825 345 980 415]);
set_sensor(vpri,'0.02705','12','3.3','0','0'); set_sensor(vsec,'0.02705','12','3.3','0','0');
set_sensor(ipri,'0.15','12','3.3','1.65','2048'); set_sensor(ires,'0.15','12','3.3','1.65','2048');
set_sensor(isec,'0.15','12','3.3','1.65','2048');
% ADC order must match xplt: Vpri, Ipri, Vsec, Isec, Iresonant.
sens={vpri,ipri,vsec,isec,ires};
for k=1:numel(sens), add_line(sub,pe.Outport(2),get_param(sens{k},'PortHandles').Inport(1),'autorouting','on'); end

% Electrical network.
hS=get_param(src,'PortHandles'); hP=get_param(pri,'PortHandles'); hQ=get_param(sec,'PortHandles');
hT=get_param(tr,'PortHandles'); h1=get_param(t1,'PortHandles'); h2=get_param(t2,'PortHandles');
hVP=get_param(vpri,'PortHandles'); hVS=get_param(vsec,'PortHandles');
hIP=get_param(ipri,'PortHandles'); hIR=get_param(ires,'PortHandles'); hIS=get_param(isec,'PortHandles');
hC=get_param(cout,'PortHandles'); hL=get_param(loadb,'PortHandles'); hG=get_param(gnd,'PortHandles');
add_line(sub,hS.RConn(1),hP.RConn(1)); add_line(sub,hS.LConn(1),hP.RConn(2)); add_line(sub,hS.LConn(1),hG.LConn(1));
add_line(sub,hS.RConn(1),hVP.LConn(1)); add_line(sub,hS.LConn(1),hVP.LConn(2));
add_line(sub,hP.LConn(1),hIP.LConn(1)); add_line(sub,hIP.RConn(1),hIR.LConn(1));
add_line(sub,hIR.RConn(1),h1.LConn(1)); add_line(sub,h1.RConn(1),hT.LConn(1)); add_line(sub,hT.LConn(2),hP.LConn(2));
add_line(sub,hT.RConn(1),h2.LConn(1)); add_line(sub,h2.RConn(1),hIS.LConn(1));
add_line(sub,hIS.RConn(1),hQ.LConn(1)); add_line(sub,hT.RConn(2),hQ.LConn(2));
add_line(sub,hQ.RConn(1),hC.LConn(1)); add_line(sub,hQ.RConn(1),hL.LConn(1)); add_line(sub,hQ.RConn(1),hVS.LConn(1));
add_line(sub,hQ.RConn(2),hC.RConn(1)); add_line(sub,hQ.RConn(2),hL.RConn(1)); add_line(sub,hQ.RConn(2),hVS.LConn(2));

mux=add_block('simulink/Signal Routing/Mux',[sub '/24 Channel ADC Mux'], ...
    'Inputs','6','Position',[1040 55 1065 175]);
for k=1:5, add_line(sub,get_param(sens{k},'PortHandles').Outport(1),get_param(mux,'PortHandles').Inport(k),'autorouting','on'); end
zero=add_block('simulink/Sources/Constant',[sub '/Unused ADC Channels'], ...
    'Value','zeros(19,1)','OutDataTypeStr','uint32','Position',[930 105 1010 135]);
add_line(sub,get_param(zero,'PortHandles').Outport,get_param(mux,'PortHandles').Inport(6));
add_line(sub,get_param(mux,'PortHandles').Outport,get_param(outA,'PortHandles').Inport);

mask=Simulink.Mask.create(sub); mask.Display='disp(''GMP CLLLC / DAB'')';
g1=mask.addDialogControl('group','TankGroup'); g1.Prompt='Resonant Tank (SDPE synchronized)';
g2=mask.addDialogControl('group','SensorGroup'); g2.Prompt='ADC Sensors';
g3=mask.addDialogControl('group','PwmGroup'); g3.Prompt='PWM and Sampling';
end

function build_epwm(sub)
sub=getfullname(sub);
in1=add_block('simulink/Ports & Subsystems/In1',[sub '/Enable'],'Port','1','Position',[20 30 50 50]);
in2=add_block('simulink/Ports & Subsystems/In1',[sub '/PWM'],'Port','2','Position',[20 70 50 90]);
in3=add_block('simulink/Ports & Subsystems/In1',[sub '/Monitor'],'Port','3','Position',[20 110 50 130]);
clk=add_block('simulink/Sources/Clock',[sub '/Clock'],'Position',[20 155 50 175]);
adcN=add_block('simulink/Sources/Constant',[sub '/ADC Cycles'],'Value','AdcCyclesPerIsr','Position',[20 195 85 215]);
timerHz=add_block('simulink/Sources/Constant',[sub '/Timer Clock'],'Value','TimerClockHz','Position',[20 230 85 250]);
nomTicks=add_block('simulink/Sources/Constant',[sub '/Nominal Ticks'],'Value','NominalPeriodTicks','Position',[20 265 85 285]);
deadtime=add_block('simulink/Sources/Constant',[sub '/Deadtime'],'Value','DeadtimeSeconds','Position',[20 300 85 320]);
fn=add_block('simulink/User-Defined Functions/MATLAB Function',[sub '/ePWM Timebase'], ...
    'Position',[145 40 340 320]);
o1=add_block('simulink/Ports & Subsystems/Out1',[sub '/Eight Gate Signals'],'Port','1','Position',[380 70 410 90]);
o2=add_block('simulink/Ports & Subsystems/Out1',[sub '/ADC ISR Trigger'],'Port','2','Position',[380 130 410 150]);
rt=sfroot; chart=rt.find('-isa','Stateflow.EMChart','Path',[sub '/ePWM Timebase']);
chart.Script = sprintf([ ...
'function [gate,trigger] = fcn(enable,pwm,monitor,t,adcCycles,timerClock,nominalTicks,deadtimeSeconds)\n' ...
'%% Variable-frequency, phase-shifted four-leg ePWM with complementary dead time.\n' ...
'persistent previous_bucket\n' ...
'if isempty(previous_bucket), previous_bucket = -1; end\n' ...
'period_pu=max(monitor(1),0.5); period=period_pu*nominalTicks/timerClock;\n' ...
'period_ticks=nominalTicks*period_pu; dead=max(monitor(2),deadtimeSeconds/period);\n' ...
'upper=false(4,1); lower=false(4,1);\n' ...
'for leg=1:4\n' ...
' duty=min(max(double(pwm(leg))/period_ticks,0),1);\n' ...
' phase=double(pwm(leg+4))/period_ticks; x=mod(t/period-phase,1);\n' ...
' upper(leg)=enable>0.5 && x>=dead && x<max(duty-dead,0);\n' ...
' lower(leg)=enable>0.5 && x>=min(duty+dead,1) && x<1-dead;\n' ...
'end\n' ...
'%% Universal Bridge gate order is [Aupper Bupper Alower Blower].\n' ...
'gate=[upper(1:2);lower(1:2);upper(3:4);lower(3:4)];\n' ...
'bucket=floor(t/(period*max(adcCycles,1))); trigger=double(bucket~=previous_bucket); previous_bucket=bucket;\n' ...
'end\n']);
pf=get_param(fn,'PortHandles');
add_line(sub,get_param(in1,'PortHandles').Outport,pf.Inport(1));
add_line(sub,get_param(in2,'PortHandles').Outport,pf.Inport(2));
add_line(sub,get_param(in3,'PortHandles').Outport,pf.Inport(3));
add_line(sub,get_param(clk,'PortHandles').Outport,pf.Inport(4));
add_line(sub,get_param(adcN,'PortHandles').Outport,pf.Inport(5));
add_line(sub,get_param(timerHz,'PortHandles').Outport,pf.Inport(6));
add_line(sub,get_param(nomTicks,'PortHandles').Outport,pf.Inport(7));
add_line(sub,get_param(deadtime,'PortHandles').Outport,pf.Inport(8));
add_line(sub,pf.Outport(1),get_param(o1,'PortHandles').Inport);
add_line(sub,pf.Outport(2),get_param(o2,'PortHandles').Inport);
mask=Simulink.Mask.create(sub); mask.Display='disp(''Advanced ePWM\nF/Phase/Duty/Deadtime'')';
g1=mask.addDialogControl('group','Timebase'); g1.Prompt='Timebase and Frequency';
g2=mask.addDialogControl('group','Deadband'); g2.Prompt='Complementary Dead Band';
g3=mask.addDialogControl('group','Sampling'); g3.Prompt='ADC / ISR Trigger';
p=mask.addParameter('Type','edit','Name','TimerClockHz','Prompt','Timer clock (Hz)','Value','100e6'); p.Container='Timebase';
p=mask.addParameter('Type','edit','Name','NominalPeriodTicks','Prompt','Nominal period ticks','Value','1000'); p.Container='Timebase';
p=mask.addParameter('Type','edit','Name','DeadtimeSeconds','Prompt','Nominal dead time (s)','Value','200e-9'); p.Container='Deadband';
p=mask.addParameter('Type','edit','Name','AdcCyclesPerIsr','Prompt','PWM cycles per ADC/ISR','Value','2'); p.Container='Sampling';
end

function b=copy_sensor(src,dst,pos)
b=add_block(src,dst,'Position',pos,'CopyOption','nolink');
end
function set_sensor(b,gain,bits,vref,bias,init)
set_param(b,'MaskValues',{gain,bits,vref,bias,init});
end
