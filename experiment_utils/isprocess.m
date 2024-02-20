function varargout = isprocess(pname)
%ISPROCESS checks if the input process name is running on the system
%The function returns True/False (0/1) along with the number of instances of the
%process and the process ID (PID) numbers.
%
% Syntax/Usage:  [result] = isprocess('fire*')
%                [result pid_total] = isprocess('firefox')
%                [result pid_total pid_nums] = isprocess('firefox')
%
% See also endtask

% Distribution Version 2.0
% Written by Varun Gandhi 13-May-2009

if ispc
    if (strcmp(pname(end),'*'))
    pname=pname(1:(end-1));
    end
    str=sprintf('tasklist /SVC /NH /FI "IMAGENAME eq %s*"',pname);
    [sys_status,sys_out] = system(str);
    if (sys_status == 0)
        [matches, start_idx, end_idx, extents, tokens, names, splits] = regexp(sys_out,pname,'ignorecase', 'match');
    else
        disp(sys_out);
        error('Unable to access system processes');
        
    end
    
    
    
    if (numel(matches) == 0)
        
        varargout{1} = false;
        varargout{2}=0;
        varargout{3}='No Matching Process-IDs';
    else
        
        for i=1:numel(matches)
            match_pid(i) = regexp(splits(i+1), '\d+', 'match', 'once');
            pid_nums{i}=str2double(match_pid{i});
            varargout{1}=true;
        end
        varargout{2}=numel(matches);
        varargout{3}=pid_nums;
    end
    
end

if isunix
    [sys_status , sys_out] = system('ps -A -o cmd');
    expr = ['\w*' pname '\w*'];
    if (sys_status == 0)
        matches = regexp(sys_out,expr,'ignorecase', 'match');
    else
        disp(sys_out);
        error('Unable to access system processes');
    end
    
    if (numel(matches) == 0)
        varargout{1} = false;
        varargout{2}=0;
        varargout{3}='No Matching Process-IDs';
    else
        
        pgrep_cmd = ['pgrep ' matches{1}];
        [sys_status , sys_out] = system(pgrep_cmd);
        pid_nums =regexp(sys_out,'\d+','ignorecase', 'match');
        varargout{1}=true;
        varargout{2}=numel(pid_nums);
        varargout{3}=pid_nums;
    end
    
end



