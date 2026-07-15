function metrics=run_clllc_validation(stop_time)
%RUN_CLLLC_VALIDATION Run SIL and save a commissioning waveform/metrics set.
if nargin<1, stop_time=0.01; end
root=fileparts(mfilename('fullpath')); out=run_clllc_cosim(stop_time);
mon=out.get('clllc_monitor'); adc=out.get('clllc_adc');
folder=fullfile(root,'validation'); if ~isfolder(folder), mkdir(folder); end
t=mon.Monitor_CH3.Time;
data=[mon.Monitor_CH3.Data mon.Monitor_CH4.Data mon.Monitor_CH5.Data ...
      mon.Monitor_CH6.Data mon.Monitor_CH7.Data];
metrics.stop_time_s=stop_time;
metrics.final_primary_voltage_v=data(end,1);
metrics.final_primary_current_a=data(end,2);
metrics.final_secondary_voltage_v=data(end,3);
metrics.final_resonant_current_a=data(end,4);
metrics.final_modulation_command=data(end,5);
fid=fopen(fullfile(folder,'build_level_1_metrics.json'),'w');
fprintf(fid,'%s',jsonencode(metrics,'PrettyPrint',true)); fclose(fid);
f=figure('Visible','off','Color','w','Position',[100 100 1100 720]);
tiledlayout(3,1,'TileSpacing','compact');
nexttile; plot(t,data(:,1),t,data(:,3),'LineWidth',1); grid on; ylabel('Voltage (V)'); legend('Primary','Secondary');
nexttile; plot(t,data(:,2),t,data(:,4),'LineWidth',1); grid on; ylabel('Current (A)'); legend('Primary','Resonant');
nexttile; plot(t,data(:,5),'LineWidth',1); grid on; ylabel('Command (pu)'); xlabel('Time (s)');
exportgraphics(f,fullfile(folder,'build_level_1_waveforms.png'),'Resolution',160); close(f);
save(fullfile(folder,'build_level_1_results.mat'),'metrics','mon','adc');
end
