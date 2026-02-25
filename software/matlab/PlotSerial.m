% script to read from COM3 and then plot vs time (angle and angular velocity)

% create serial object
s = serialport("COM3", 115200);  % COM3 and baud rate 115200
configureTerminator(s, "CR/LF");  % match '\r\n' from STM32
flush(s);                         % clear buffer

% initialize plot
figure;
h1 = animatedline('Color', 'b', 'DisplayName', 'AngVel (deg)');
hold on;
ax = gca;
ax.YLabel.String = 'Value';
ax.XLabel.String = 'Time (s)';
ax.XLim = [0 10];  % initial 10 s window
legend show
startTime = datetime('now');

% read and plot continuously
while true
    if s.NumBytesAvailable > 0
        line = readline(s);                  % reads until CR/LF
        value = str2double(regexp(line, '[-+]?\d*\.?\d+', 'match', 'once'));

            
        if ~isempty(value) && isfinite(value)

            % get elapsed time
            t = seconds(datetime('now') - startTime);
        
            addpoints(h1, t, value);
            drawnow limitrate;
        
            % update sliding window
            ax.XLim = [max(0,t-10) max(10,t)];

        end
    end
end