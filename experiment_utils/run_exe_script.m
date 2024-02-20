if run_exe
    exe_path = sprintf("%s%s",pwd,"/centralized_plan/Release");
    fileInfo = fullfile(exe_path,'iros2024_matlab.exe');
    cmd = sprintf('cd "%s" & iros2024_matlab.exe &',exe_path);
    system(cmd);
    % Wait for the process to start
    pause(1);  % Adjust the pause duration as needed
    % Run the PowerShell command to get the process ID
    [status, result] = system('powershell.exe -Command "Get-Process -Name ''iros2024_matlab'' | Select-Object -ExpandProperty Id"');
    if status == 0
        % Extract the process ID using regular expressions
        pidPattern = '\d+'; % Regular expression pattern to match one or more digits
        matches = regexp(result, pidPattern, 'match');
        if ~isempty(matches)
            pid = str2double(matches{1}); % Extract the first match and convert it to a numeric value
            if ~isnan(pid)
                disp(['The process ID is: ', num2str(pid)]);
                % Continue checking the process status until it's not running
                processRunning = true;
                while processRunning
                    % Check if the process is still running
                    [~, result] = system(['powershell.exe Get-Process -Id ', num2str(pid)]);
                    running = contains(result, 'Handles');
                    if running
                        disp('The process is still running.');
                        % Add a short pause before checking again
                        pause(1);  % Adjust the pause duration as needed
                    else
                        disp('The process has completed.');
                        processRunning = false;  % Exit the loop if the process has completed
                    end
                end
            else
                error('Unable to retrieve valid process ID.');
            end
        else
            error('Process ID not found in the result.');
        end
    else
        error('Error running PowerShell command to get process ID.');
    end
    disp('EXE complete')
end

