function RDP_GUI
% This is a demo of the Ramer-Douglas-Peucker algorithm. The function calls
% DouglasPeucker.m, which performs the algorithm. Input
% data is obtained by drawing with your mouse.
%
% Jan/11/2015, hanligong@gmail.com

% Drawing Pad
h_f = figure('name','RDP GUI Demo',...
    'color',[1 1 1],'menubar','none','numbertitle','off',...
    'position',[50 150 600 450],'resize','off','closerequestfcn',@my_closereq);
h_a = axes('parent',h_f,'xlim',[0 4],'ylim',[0 3],...
    'box','on','dataaspectratio',[1 1 1],'xtick',[],'ytick',[],...
    'position',[0,0,1,1],'xcolor',[1 1 1],'ycolor',[1 1 1]);
h_tg = uicontrol(h_f,'style','pushbutton','position',[520 5 75 25],...
    'fontsize',12,'callback',@clear_lines,'string','Clear');
% Data Reporting
h_f2 = figure('name','Ramer-Douglas-Peucker algorithm',...
    'color',[1 1 1],'menubar','none','numbertitle','off',...
    'position',[700 150 600 450],'resize','off','closerequestfcn',@my_closereq);
h_a2 = axes('parent',h_f2,'xlim',[0 4],'ylim',[0 3],...
    'box','on','dataaspectratio',[1 1 1],'xcolor',[1 1 1],'ycolor',[1 1 1],...
    'position',[0,0,1,1]);
% Original data
l_o = line(NaN,NaN,'parent',h_a2,...
    'color',[1 0.5 0],'linestyle','-','linewidth',2,'marker','o','markersize',6);
% Reduced data
l_r = line(NaN,NaN,'parent',h_a2,...
    'color',[0 0 1],'linestyle','-','linewidth',2.5,'marker','o','markersize',6.5);
n = 0;
x = nan;
y = nan;
nk = 1;
h_l = line(NaN,NaN,'parent',h_a,'linewidth',3,'color','r');
set(h_f,'windowbuttonupfcn',@stopdragfcn)
set(h_a,'buttondownfcn',@startdragfcn)
% Display instruction
h_msg = uicontrol('style','text','parent',h_f,...
    'backgroundcolor',[1 1 1],'position',[100,180,400,100],...
    'string','Draw lines in this window with your mouse.',...
    'fontsize',24,'HorizontalAlignment','center',...
    'fontweight','bold','foregroundcolor',[.1 .25 1]);
for k = 1:3
    set(h_msg,'visible','on')
    pause(0.75)
    set(h_msg,'visible','off')
    pause(0.25)
end

    function startdragfcn(varargin)
        set(h_f,'windowbuttonmotionfcn',@draggingfcn);
        nk = n+1;
    end

    function stopdragfcn(varargin)
        set(h_f,'windowbuttonmotionfcn','');
        n = n+1;
        x(n) = NaN;
        y(n) = NaN;
        % Call RDP Fcn
        ptList = DouglasPeucker([x(nk:n-1);y(nk:n-1)],0.05,false);
        set(l_o,'xdata',x(nk:n-1),'ydata',y(nk:n-1));
        set(l_r,'xdata',ptList(:,1),'ydata',ptList(:,2));
    end

    function draggingfcn(varargin)
        n = n+1;
        pt = get(h_a,'currentpoint');
        x(n) = pt(1,1);
        y(n) = pt(1,2);
        set(h_l,'xdata',x(1:n),'ydata',y(1:n))
    end

    function my_closereq(varargin)
        % User-defined close request function
        % to display a question dialog box
        selection = questdlg('Do you want to quit?','Close Figure',...
            'Yes','No','Yes');
        switch selection,
            case 'Yes',
                delete(h_f)
                delete(h_f2)
            case 'No'
                return
        end
    end

    function clear_lines(varargin)
        n = 0;
        x = nan;
        y = nan;
        nk = 1;
        set(h_l,'xdata',nan,'ydata',nan);
    end
end
