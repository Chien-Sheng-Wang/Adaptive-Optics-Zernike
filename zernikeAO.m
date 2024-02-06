function zernikeAO(src,event,varargin)

 %Retains data between function calls
persistent   slmDisplay hFig plotData Table Piston TiltX TiltY Power AstigX AstigY ComaX ComaY PrimarySpherical TrefoilX TrefoilY SecondaryAstigX SecondaryAstigY SecondaryComaX SecondaryComaY SecondarySpherical TetrafoilX TetrafoilY TertiarySpherical QuaternarySpherical iteration

hSI = src.hSI; % get the handle to the ScanImage zerModel
range=10;   % chart starts to scroll once this many points have been acquired
stepSize=0.5;
changeNumber=2*range/stepSize+1;
%Image = libpointer('uint8Ptr', zeros(width*height,1));

switch event.EventName

    case 'focusStart'
        addpath('C:\Program Files\Meadowlark Optics\Blink OverDrive Plus\SDK','-begin');
        % Iteratively optimize image intensity by Zernike

        % Instructions:
        %
        % * Add the directory containing this function to your path
        % * In ScanImage go to Settings > User Functions and
        %   assign "ZernikeAO" to the events:
        %   focusStart
        %   frameAcquired
        %
        % * Ensure all are enabled
        % * Set up to acquire data on channel 1
        % * Press Focus
        % * Try pressing Abort then focus again. See how the figure is cleared
        %   first before data are added.
        %



        %% load the SDK of SLM
        % Load the DLL
        % Blink_C_wrapper.dll, Blink_SDK.dll, ImageGen.dll, FreeImage.dll and wdapi1021.dll
        % should all be located in the same directory as the program referencing the
        % library
        if ~libisloaded('Blink_C_wrapper')
            loadlibrary('Blink_C_wrapper.dll', 'Blink_C_wrapper.h');
        end

        % This loads the image generation functions
        if ~libisloaded('ImageGen')
            loadlibrary('ImageGen.dll', 'ImageGen.h');
        end


        % Basic parameters for calling Create_SDK
        bit_depth = 12; %bit depth = 8 for small 512, 12 for 1920
        num_boards_found = libpointer('uint32Ptr', 0);
        constructed_okay = libpointer('int32Ptr', 0);
        is_nematic_type = 1;
        RAM_write_enable = 1;
        use_GPU = 1;
        max_transients = 10;
        wait_For_Trigger = 0; % This feature is user-settable; use 1 for 'on' or 0 for 'off'
        timeout_ms = 5000;

        %Both pulse options can be false, but only one can be true. You either generate a pulse when the new image begins loading to the SLM
        %or every 1.184 ms on SLM refresh boundaries, or if both are false no output pulse is generated.
        OutputPulseImageFlip = 0;
        OutputPulseImageRefresh = 0; %only supported on 1920x1152, FW rev 1.8.


        % - This regional LUT file is only used with Overdrive Plus, otherwise it should always be a null string
        reg_lut = libpointer('string');

        % Call the constructor
        calllib('Blink_C_wrapper', 'Create_SDK', bit_depth, num_boards_found, constructed_okay, is_nematic_type, RAM_write_enable, use_GPU, max_transients, reg_lut);


       

        % if isempty(iteration)
        %     iteration=1;
        % else
        %     iteration=1+iteration;
        % end



        % constructed okay return of 0 is success, nonzero integer is an error
        % if constructed_okay.value ~= 0
        %     disp('Blink SDK was not successfully constructed');
        %     disp(calllib('Blink_C_wrapper', 'Get_last_error_message'));
        %     calllib('Blink_C_wrapper', 'Delete_SDK');
        % else
        board_number = 1;
        % disp('Blink SDK was successfully constructed');
        % fprintf('Found %u SLM controller(s)\n', num_boards_found.value);

        height = calllib('Blink_C_wrapper', 'Get_image_height', board_number);
        width = calllib('Blink_C_wrapper', 'Get_image_width', board_number);

        %allocate arrays for our images
        Image = libpointer('uint8Ptr', zeros(width*height,1));
        %WFC = libpointer('uint8Ptr', zeros(width*height,1));

        %***you should replace *bit_linear.LUT with your custom LUT file***
        %but for now open a generic LUT that linearly maps input graylevels to output voltages
        %***Using *bit_linear.LUT does NOT give a linear phase response***
        if width == 512
            lut_file = 'C:\\Program Files\\Meadowlark Optics\\Blink OverDrive Plus\\LUT Files\\8bit_linear.LUT';
        else
            lut_file = 'C:\\Program Files\\Meadowlark Optics\\Blink OverDrive Plus\\LUT Files\\12bit_linear.LUT';
        end
        calllib('Blink_C_wrapper', 'Load_LUT_file', board_number, lut_file);


        % Generate a blank wavefront correction image, you should load your
        % custom wavefront correction that was shipped with your SLM.
        % PixelValue = 0;
        % calllib('ImageGen', 'Generate_Solid', WFC, width, height, PixelValue);
        % WFC = reshape(WFC.Value, [width,height]);

        % Start the SLM with a blank image
        % calllib('Blink_C_wrapper', 'Write_image', board_number, WFC, width*height, wait_For_Trigger, OutputPulseImageFlip, OutputPulseImageRefresh, timeout_ms);
        % calllib('Blink_C_wrapper', 'ImageWriteComplete', board_number, timeout_ms);




        %% iteratively change the image of SLM and get data from scanimage


        iteration=1;
        [file,path] = uigetfile('*.xlsx*','select GRIN lens location file');
        Table = readtable(fullfile(path,file));
        % Look for a figure window that contains data from a previous
        % run of meanFrame. Make one if it doesn't exist, wipe it if it
        % does.

        hFig = findobj(0,'Name','meanFramePlot');
        if isempty(hFig)
            hFig = figure;
            hFig.Name='meanFramePlot';
        end
        figure(hFig)
        tmpC = cla;
        plotData=plot(tmpC, nan, 'r-o', 'LineWidth', 2); %Plot a nan
        title('zernike mode 1 : Piston')
        grid off
        xlim([-range,range])


        pupilimg=uint8(zeros(height,width));
        for pupilnumber=1:size(Table,1)
            % Generate a zernike
            CenterX=Table.CxP(pupilnumber);
            CenterY=Table.CyP(pupilnumber);
            Radius=Table.Rp(pupilnumber);
            Piston=-range;
            TiltX=0;
            TiltY=0;
            Power=0;
            AstigX=0;
            AstigY=0;
            ComaX=0;
            ComaY=0;
            PrimarySpherical=0;
            TrefoilX=0;
            TrefoilY=0;
            SecondaryAstigX=0;
            SecondaryAstigY=0;
            SecondaryComaX=0;
            SecondaryComaY=0;
            SecondarySpherical=0;
            TetrafoilX=0;
            TetrafoilY=0;
            TertiarySpherical=0;
            QuaternarySpherical=0;
            calllib('ImageGen','Generate_Zernike',Image,width,height,CenterX,CenterY,Radius,Piston,TiltX,TiltY,Power,AstigX,AstigY,ComaX,ComaY,PrimarySpherical,TrefoilX,TrefoilY,SecondaryAstigX,SecondaryAstigY,SecondaryComaX,SecondaryComaY, SecondarySpherical,TetrafoilX,TetrafoilY,TertiarySpherical,QuaternarySpherical)
            img= reshape(Image.Value, [width,height]);
            %Image = rot90(mod(Image+WFC, 256));
            img = flipud(rot90(mod(img,256)));

            %create a pupil mask
            pupilMask=poly2mask(CenterX+Radius*cos(0:0.1:2*pi),CenterY+Radius*sin(0:0.1:2*pi),height,width);
            pupilMask=uint8(pupilMask);
            pupilimg=img.*pupilMask+pupilimg;
        end

        %write image returns on DMA complete, ImageWriteComplete returns when the hardware
        %image buffer is ready to receive the next image. Breaking this into two functions is
        %useful for external triggers. It is safe to apply a trigger when Write_image is complete
        %and it is safe to write a new image when ImageWriteComplete returns
        calllib('Blink_C_wrapper', 'Write_image', board_number, pupilimg, width*height, wait_For_Trigger, OutputPulseImageFlip, OutputPulseImageRefresh, timeout_ms);
        calllib('Blink_C_wrapper', 'ImageWriteComplete', board_number, timeout_ms);
        pause(0) % This is in seconds - IF USING EXTERNAL TRIGGERS, SET THIS TO 0
        %figure(),imshow(pupilimg)

        slmDisplay=findobj('Name','SLM display');
        if isempty(slmDisplay)
            slmDisplay=figure;
            slmDisplay.Name='SLM display';
            slmDisplay.Units='normalized';
            slmDisplay.Position=[0.4 0.4 0.45 0.45];
        end
        figure(slmDisplay),imshow(pupilimg,'InitialMagnification', 'fit')



    case 'frameAcquired'

        board_number = 1;
        height = calllib('Blink_C_wrapper', 'Get_image_height', board_number);
        width = calllib('Blink_C_wrapper', 'Get_image_width', board_number);
        Image = libpointer('uint8Ptr', zeros(width*height,1));
        % Basic parameters for calling Create_SDK
        % bit_depth = 12; %bit depth = 8 for small 512, 12 for 1920
        % num_boards_found = libpointer('uint32Ptr', 0);
        % constructed_okay = libpointer('int32Ptr', 0);
        % is_nematic_type = 1;
        % RAM_write_enable = 1;
        % use_GPU = 1;
        %max_transients = 10;
        wait_For_Trigger = 0; % This feature is user-settable; use 1 for 'on' or 0 for 'off'
        timeout_ms = 5000;
        %Both pulse options can be false, but only one can be true. You either generate a pulse when the new image begins loading to the SLM
        %or every 1.184 ms on SLM refresh boundaries, or if both are false no output pulse is generated.
        OutputPulseImageFlip = 0;
        OutputPulseImageRefresh = 0; %only supported on 1920x1152, FW rev 1.8.
        % - This regional LUT file is only used with Overdrive Plus, otherwise it should always be a null string
        %reg_lut = libpointer('string');

        zerMode=fix((iteration-1)/changeNumber)+1;
        iteration=1+iteration;

        % Pull in data from the first depth of the first channel
        lastFrame = hSI.hDisplay.stripeDataBuffer{1}.roiData{1}.imageData{1}{1};

        if mod(iteration-1,changeNumber)~=0
            plotData.XData(mod(iteration-1,changeNumber))=-range+mod(iteration-2,changeNumber).*stepSize;
            plotData.YData(mod(iteration-1,changeNumber))=mean(lastFrame(:));

            shg
        else
            plotData.XData(end+1)=-range+mod(iteration-2,changeNumber).*stepSize;
            plotData.YData(end+1)=mean(lastFrame(:));


            shg
        end

        pupilimg=uint8(zeros(height,width));
        for pupilnumber=1:size(Table,1)
            CenterX=Table.CxP(pupilnumber);
            CenterY=Table.CyP(pupilnumber);
            Radius=Table.Rp(pupilnumber);

            if pupilnumber==1
                switch zerMode
                    case 1
                        Piston=stepSize+Piston;
                        if length(plotData.YData)==changeNumber
                            a=find(plotData.YData==max(plotData.YData(:)));
                            Piston=-range+stepSize*a(1);
                            TiltX=-range;
                            figure(hFig)
                            plotData=plot(cla, nan, 'r-o', 'LineWidth', 2); %Plot a nan
                            title('zernike mode 2 : TiltX')

                            grid off
                            xlim([-range,range])
                        end

                    case 2
                        TiltX=stepSize+TiltX;
                        if length(plotData.YData)==changeNumber
                            a=find(plotData.YData==max(plotData.YData(:)));
                            TiltX=-range+stepSize*a(1);
                            TiltY=-range;
                            figure(hFig)
                            plotData=plot(cla, nan, 'r-o', 'LineWidth', 2); %Plot a nan
                            title('zernike mode 3 : TiltY')
                            grid off
                            xlim([-range,range])
                        end



                    case 3
                        TiltY=stepSize+TiltY;
                        if length(plotData.YData)==changeNumber
                            a=find(plotData.YData==max(plotData.YData(:)));
                            TiltY=-range+stepSize*a(1);
                            Power=-range;
                            figure(hFig)
                            plotData=plot(cla, nan, 'r-o', 'LineWidth', 2); %Plot a nan
                            title('zernike mode 4 : Power')
                            grid off
                            xlim([-range,range])
                        end

                    case 4
                        Power=stepSize+Power;
                        if length(plotData.YData)==changeNumber
                            a=find(plotData.YData==max(plotData.YData(:)));
                            Power=-range+stepSize*a(1);
                            AstigX=-range;
                            figure(hFig)
                            plotData=plot(cla, nan, 'r-o', 'LineWidth', 2); %Plot a nan
                            title('zernike mode 5 : AstigX')
                            grid off
                            xlim([-range,range])
                        end

                    case 5
                        AstigX=stepSize+AstigX;
                        if length(plotData.YData)==changeNumber
                            a=find(plotData.YData==max(plotData.YData(:)));
                            AstigX=-range+stepSize*a(1);
                            AstigY=-range;
                            figure(hFig)
                            plotData=plot(cla, nan, 'r-o', 'LineWidth', 2); %Plot a nan
                            title('zernike mode 6 : AstigY')
                            grid off
                            xlim([-range,range])
                        end

                    case 6
                        AstigY=stepSize+ AstigY;
                        if length(plotData.YData)==changeNumber
                            a=find(plotData.YData==max(plotData.YData(:)));
                            AstigY=-range+stepSize*a(1);
                            ComaX=-range;
                            figure(hFig)
                            plotData=plot(cla, nan, 'r-o', 'LineWidth', 2); %Plot a nan
                            title('zernike mode 7 : ComaX')
                            grid off
                            xlim([-range,range])
                        end
                    case 7
                        ComaX=stepSize+ComaX;
                        if length(plotData.YData)==changeNumber
                            a=find(plotData.YData==max(plotData.YData(:)));
                            ComaX=-range+stepSize*a(1);
                            ComaY=-range;
                            figure(hFig)
                            plotData=plot(cla, nan, 'r-o', 'LineWidth', 2); %Plot a nan
                            title('zernike mode 8 : ComaY')
                            grid off
                            xlim([-range,range])
                        end

                    case 8
                        ComaY=stepSize+ComaY;
                        if length(plotData.YData)==changeNumber
                            a=find(plotData.YData==max(plotData.YData(:)));
                            ComaY=-range+stepSize*a(1);
                            PrimarySpherical=-range;
                            figure(hFig)
                            plotData=plot(cla, nan, 'r-o', 'LineWidth', 2); %Plot a nan
                            title('zernike mode 9 : PrimarySpherical')
                            grid off
                            xlim([-range,range])
                        end

                    case 9
                        PrimarySpherical=stepSize+PrimarySpherical;
                        if length(plotData.YData)==changeNumber
                            a=find(plotData.YData==max(plotData.YData(:)));
                            PrimarySpherical=-range+stepSize*a(1);
                            TrefoilX=-range;
                            figure(hFig)
                            plotData=plot(cla, nan, 'r-o', 'LineWidth', 2); %Plot a nan
                            title('zernike mode 10 : TrefoilX')
                            grid off
                            xlim([-range,range])
                        end


                    case 10
                        TrefoilX=stepSize+TrefoilX;
                        if length(plotData.YData)==changeNumber
                            a=find(plotData.YData==max(plotData.YData(:)));
                            TrefoilX=-range+stepSize*a(1);
                            TrefoilY=-range;
                            figure(hFig)
                            plotData=plot(cla, nan, 'r-o', 'LineWidth', 2); %Plot a nan
                            title('zernike mode 11 : TrefoilY')
                            grid off
                            xlim([-range,range])
                        end

                    case 11
                        TrefoilY=stepSize+TrefoilY;
                        if length(plotData.YData)==changeNumber
                            a=find(plotData.YData==max(plotData.YData(:)));
                            TrefoilY=-range+stepSize*a(1);
                            SecondaryAstigX=-range;
                            figure(hFig)
                            plotData=plot(cla, nan, 'r-o', 'LineWidth', 2); %Plot a nan
                            title('zernike mode 12 : SecondaryAstigX')
                            grid off
                            xlim([-range,range])
                        end

                    case 12
                        SecondaryAstigX=stepSize+SecondaryAstigX;
                        if length(plotData.YData)==changeNumber
                            a=find(plotData.YData==max(plotData.YData(:)));
                            SecondaryAstigX=-range+stepSize*a(1);
                            SecondaryAstigY=-range;
                            figure(hFig)
                            plotData=plot(cla, nan, 'r-o', 'LineWidth', 2); %Plot a nan
                            title('zernike mode 13 : SecondaryAstigY')
                            grid off
                            xlim([-range,range])
                        end

                    case 13
                        SecondaryAstigY=stepSize+SecondaryAstigY;
                        if length(plotData.YData)==changeNumber
                            a=find(plotData.YData==max(plotData.YData(:)));
                            SecondaryAstigY=-range+stepSize*a(1);
                            SecondaryComaX=-range;
                            figure(hFig)
                            plotData=plot(cla, nan, 'r-o', 'LineWidth', 2); %Plot a nan
                            title('zernike mode 14 : SecondaryComaX')
                            grid off
                            xlim([-range,range])
                        end

                    case 14
                        SecondaryComaX=stepSize+SecondaryComaX;
                        if length(plotData.YData)==changeNumber
                            a=find(plotData.YData==max(plotData.YData(:)));
                            SecondaryComaX=-range+stepSize*a(1);
                            SecondaryComaY=-range;
                            figure(hFig)
                            plotData=plot(cla, nan, 'r-o', 'LineWidth', 2); %Plot a nan
                            title('zernike mode 15 : SecondaryComaY')
                            grid off
                            xlim([-range,range])
                        end

                    case 15
                        SecondaryComaY=stepSize+SecondaryComaY;
                        if length(plotData.YData)==changeNumber
                            a=find(plotData.YData==max(plotData.YData(:)));
                            SecondaryComaY=-range+stepSize*a(1);
                            SecondarySpherical=-range;
                            figure(hFig)
                            plotData=plot(cla, nan, 'r-o', 'LineWidth', 2); %Plot a nan
                            title('zernike mode 16 : SecondarySpherical')
                            grid off
                            xlim([-range,range])
                        end

                    case 16
                        SecondarySpherical=stepSize+SecondarySpherical;
                        if length(plotData.YData)==changeNumber
                            a=find(plotData.YData==max(plotData.YData(:)));
                            SecondarySpherical=-range+stepSize*a(1);
                            TetrafoilX=-range;
                            figure(hFig)
                            plotData=plot(cla, nan, 'r-o', 'LineWidth', 2); %Plot a nan
                            title('zernike mode 17 : TetrafoilX')
                            grid off
                            xlim([-range,range])
                        end

                    case 17
                        TetrafoilX=stepSize+TetrafoilX;
                        if length(plotData.YData)==changeNumber
                            a=find(plotData.YData==max(plotData.YData(:)));
                            TetrafoilX=-range+stepSize*a(1);
                            TetrafoilY=-range;
                            figure(hFig)
                            plotData=plot(cla, nan, 'r-o', 'LineWidth', 2); %Plot a nan
                            title('zernike mode 18 : TetrafoilY')
                            grid off
                            xlim([-range,range])
                        end


                    case 18
                        TetrafoilY=stepSize+TetrafoilY;
                        if length(plotData.YData)==changeNumber
                            a=find(plotData.YData==max(plotData.YData(:)));
                            TetrafoilY=-range+stepSize*a(1);
                            TertiarySpherical=-range;
                            figure(hFig)
                            plotData=plot(cla, nan, 'r-o', 'LineWidth', 2); %Plot a nan
                            title('zernike mode 19 : TertiarySpherical')
                            grid off
                            xlim([-range,range])
                        end

                    case 19
                        TertiarySpherical=stepSize+TertiarySpherical;
                        if length(plotData.YData)==changeNumber
                            a=find(plotData.YData==max(plotData.YData(:)));
                            TertiarySpherical=-range+stepSize*a(1);
                            QuaternarySpherical=-range;
                            figure(hFig)
                            plotData=plot(cla, nan, 'r-o', 'LineWidth', 2); %Plot a nan
                            title('zernike mode 20 : QuaternarySpherical')
                            grid off
                            xlim([-range,range])
                        end


                    case 20
                        QuaternarySpherical=stepSize+QuaternarySpherical;
                        if length(plotData.YData)==changeNumber
                            a=find(plotData.YData==max(plotData.YData(:)));
                            QuaternarySpherical=-range+stepSize*a(1);
                            hSI.abort();
                            disp('Scan aborted.');
                            % Always call Delete_SDK before exiting
                            calllib('Blink_C_wrapper', 'Delete_SDK');

                            %destruct
                            if libisloaded('Blink_C_wrapper')
                                unloadlibrary('Blink_C_wrapper');
                            end

                            if libisloaded('ImageGen')
                                unloadlibrary('ImageGen');
                            end
                            break;

                        end


                end
            end
            calllib('ImageGen','Generate_Zernike',Image,width,height,CenterX,CenterY,Radius,Piston,TiltX,TiltY,Power,AstigX,AstigY,ComaX,ComaY,PrimarySpherical,TrefoilX,TrefoilY,SecondaryAstigX,SecondaryAstigY,SecondaryComaX,SecondaryComaY, SecondarySpherical,TetrafoilX,TetrafoilY,TertiarySpherical,QuaternarySpherical)
            img= reshape(Image.Value, [width,height]);
            %Image = rot90(mod(Image+WFC, 256));
            img = flipud(rot90(mod(img,256)));

            %create a pupil mask
            pupilMask=poly2mask(CenterX+Radius*cos(0:0.1:2*pi),CenterY+Radius*sin(0:0.1:2*pi),height,width);
            pupilMask=uint8(pupilMask);
            pupilimg=img.*pupilMask+pupilimg;
        end
        if ~(zerMode==20 && length(plotData.YData)==changeNumber)
            calllib('Blink_C_wrapper', 'Write_image', board_number, pupilimg, width*height, wait_For_Trigger, OutputPulseImageFlip, OutputPulseImageRefresh, timeout_ms);
            calllib('Blink_C_wrapper', 'ImageWriteComplete', board_number, timeout_ms);
            pause(0) % This is in seconds - IF USING EXTERNAL TRIGGERS, SET THIS TO 0
            %figure(),imshow(pupilimg)
            figure(slmDisplay),imshow(pupilimg,'InitialMagnification', 'fit')
        end


end % switch

%end


end
