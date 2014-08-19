function generateFPArduLog


%============================
%function generateFPArduLog
%
%14/08/2014
%Paul Hill
%University of Bristol
%============================

% Analyse input logs from the arducopter logger and create google earth
% first person fly through
%
% see http://copter.ardupilot.com/wiki/downloading-and-analyzing-data-logs-in-mission-planner/
% for information on format of the logs
%GPS: Status,TimeMS,Week,NSats,HDop,Lat,Lng,RelAlt,Alt,Spd,GCrs,VZ,TimeMS
%IMU: TimeMS, GyrX, GyrY, GyrZ, AccX, AccY, AccZ
%INAV: BAlt, IAlt, IClb, ACorrX. ACorrY, ACorrZ, GLat, GLon, ILat, ILng
%ATT: RollIn, Roll, PitchIn, Pitch, YawIn, Yaw, NavYaw
%MAG: MagX, MagY, MagZ, OfsX, OfsY, OfsZ, MOfsX, MOfsY, MOfsZ
%CTUN: ThrIn,SonAlt,BarAlt,WPAlt,DesSonAlt,AngBst,CRate,ThrOut,DCRate
clear all;    

File = 'C:\Program Files (x86)\Mission Planner\logs\2014-08-08 08-57-29.log'
outputnamestart = 'seq1/08-57-29';

data1 = fopen(File);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                      EXTRACT DATA LINE BY LINE                          %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
isCAM = 0;
beginTime = -1; % Must find the first CAM message
doloop =1 ;
inc = 0;
GPSInd = 0;CAMInd=0;IMUInd = 0;INAVInd = 0;ATTInd=0; MAGInd=0; CTUNInd=0;
while doloop == 1
 
 inc = inc+1;
    tline = fgetl(data1);
    if tline == -1
        doloop =0;
        break;
    end
    if length(tline)>0
        C = textscan(tline, '%s', ',');
        MSGString = C{1}{1};
        switch MSGString(1:end-1)
            case 'CAM'
                CAMInd = CAMInd+1;
                if (CAMInd==1)
                    beginTime = str2double(C{1}(2));
                    thisTime = 0; 
                end
            case 'GPS'
                if beginTime > -1
                    GPSInd = GPSInd+1;
                    %GPS: Status,TimeMS,Week,NSats,HDop,Lat,Lng,RelAlt,Alt,Spd,GCrs,VZ,APTimeMS
                    lenC = length(C{1});
                    for indC = 2: lenC
                        GPS(GPSInd,indC) = str2double(C{1}(indC));
                    end
                    thisTime = GPS(GPSInd,end)-beginTime;
                    GPS(GPSInd,1) = thisTime;
                end
            case {'IMU'}
                if beginTime > -1
                    IMUInd = IMUInd+1;
                    %IMU: TimeMS, GyrX, GyrY, GyrZ, AccX, AccY, AccZ
                    lenC = length(C{1});
                    for indC = 2: lenC
                        IMU(IMUInd,indC) = str2double(C{1}(indC));
                    end
                    thisTime = IMU(IMUInd,2)-beginTime;
                    IMU(IMUInd,1) = thisTime;
                end
            case {'INAV'}
                if beginTime > -1
                    INAVInd = INAVInd+1;      
                    %INAV: BAlt, IAlt, IClb, ACorrX. ACorrY, ACorrZ, GLat, GLon, ILat, ILng
                    lenC = length(C{1});
                    for indC = 2: lenC
                        INAV(INAVInd,indC) = str2double(C{1}(indC));
                    end
                    INAV(INAVInd,1) = thisTime;
                end
            case {'ATT'}
                if beginTime > -1
                    ATTInd = ATTInd+1;
                    %ATT: RollIn, Roll, PitchIn, Pitch, YawIn, Yaw, NavYaw
                    lenC = length(C{1});
                    for indC = 2: lenC
                        ATT(ATTInd,indC) = str2double(C{1}(indC));
                    end
                    ATT(ATTInd,1) = thisTime;
                end
            case {'MAG'}
                if beginTime > -1
                    MAGInd = MAGInd+1;
                    %MAG: MagX, MagY, MagZ, OfsX, OfsY, OfsZ, MOfsX, MOfsY, MOfsZ
                    lenC = length(C{1});
                    for indC = 2: lenC
                        MAG(MAGInd,indC) = str2double(C{1}(indC));
                    end
                    MAG(MAGInd,1) = thisTime;
                end
            case {'CTUN'}
                if beginTime > -1
                    CTUNInd = CTUNInd+1;
                    %CTUN: ThrIn,SonAlt,BarAlt,WPAlt,DesSonAlt,AngBst,CRate,ThrOut,DCRate
                    lenC = length(C{1});
                    for indC = 2: lenC
                        CTUN(CTUNInd,indC) = str2double(C{1}(indC));
                    end
                    CTUN(CTUNInd,1) = thisTime;
                end   
        end





    end
end

% Various offsets to make the data match the video better (better config required)
LONG = GPS(:,8); LAT = GPS(:,7); ALT = GPS(:,9); ALT(ALT<0) = 0; ALT = ALT+95;
ROLL = -ATT(:,3); TILT = 95+ATT(:,5); HEADING = ATT(:,7)-25;

% Interpolation of the GPS and ATT information so all are aligned at 25Hz
GPST5 = GPS(:,1);
GPST25 = interp(GPS(:,1),5);
LONG25 = interp1(GPST5,LONG,GPST25);
LAT25 = interp1(GPST5,LAT,GPST25);
ALT25 = interp1(GPST5,ALT,GPST25);
ATTT = ATT(:,1);
ROLL25 = interp1(ATTT,ROLL,GPST25);
TILT25 = interp1(ATTT,TILT,GPST25);
HEADING25 = interp1(ATTT,HEADING,GPST25);

robo = java.awt.Robot;
t = java.awt.Toolkit.getDefaultToolkit();
rectangle = java.awt.Rectangle(288, 159, 800, 450);
image = robo.createScreenCapture(rectangle);
filehandle = java.io.File('screencapture.png');
javax.imageio.ImageIO.write(image,'png',filehandle);

ge = GEserver;

% Change start index according to trigger (should be within a few frames,
% but in the first example it was way off at 1501
startInd = 1501;
for ind = startInd:length(GPST25)
    generateKML(ge, LAT25(ind), LONG25(ind), ALT25(ind), HEADING25(ind), TILT25(ind), ROLL25(ind));
        
        pause(0.2);
        image = robo.createScreenCapture(rectangle);
        outname = [outputnamestart sprintf('%04d.png',ind-startInd)];
        filehandle = java.io.File(outname);
        javax.imageio.ImageIO.write(image,'png',filehandle);
        thisim = imread('screencapture.png');
end
fclose(data1);


function generateKML(ge, lat, lon, alt, head, tilt, roll)

h = ['<kml xmlns="http://www.opengis.net/kml/2.2">',10,...
    '<Document>',10,...
    '<Placemark>',10,...
    '<name>100m looking east</name>',10,...
    '<Camera>',10,...
    '<longitude>',num2str(lon,10),'</longitude>',10,...
    '<latitude>',num2str(lat,10),'</latitude>',10,...
    '<altitude>',num2str(alt),'</altitude>',10,...
    '<roll>',num2str(roll),'</roll>',10,...
    '<tilt>',num2str(tilt),'</tilt>',10,...
    '<heading>',num2str(head),'</heading>',10,...
    '<altitudeMode>absolute</altitudeMode>',10,...
    '</Camera>',10,...
    '</Placemark>',10,...
    '</Document>',10,...
    '</kml>'];
%ge.LoadKmlData(h);
fid = fopen( 'g:\output.kml', 'wt');
fprintf(fid,'%s',h);
fclose(fid);

ge.OpenKmlFile('g:\output.kml',1);

%struct log_GPS pkt = {
%   LOG_PACKET_HEADER_INIT(LOG_GPS_MSG),
% 	status        : (uint8_t)gps->status(),
% 	gps_week_ms   : gps->time_week_ms,
% 	gps_week      : gps->time_week,
%   num_sats      : gps->num_sats,
%   hdop          : gps->hdop,
%   latitude      : gps->latitude,
%   longitude     : gps->longitude,
%   rel_altitude  : relative_alt,
%   altitude      : gps->altitude_cm,
%   ground_speed  : gps->ground_speed_cm,
%   ground_course : gps->ground_course_cd,
%   vel_z         : gps->velocity_down(),
%   apm_time      : hal.scheduler->millis()
%}

%struct log_IMU pkt = {
%    LOG_PACKET_HEADER_INIT(LOG_IMU_MSG),
%    timestamp : hal.scheduler->millis(),
%    gyro_x  : gyro.x,
%    gyro_y  : gyro.y,
%    gyro_z  : gyro.z,
%    accel_x : accel.x,
%    accel_y : accel.y,
%    accel_z : accel.z
%};

% struct log_INAV pkt = {
%     LOG_PACKET_HEADER_INIT(LOG_INAV_MSG),
%     baro_alt            : (int16_t)baro_alt,                        // 1 barometer altitude
%     inav_alt            : (int16_t)inertial_nav.get_altitude(),     // 2 accel + baro filtered altitude
%     inav_climb_rate     : (int16_t)inertial_nav.get_velocity_z(),   // 3 accel + baro based climb rate
%     accel_corr_x        : accel_corr.x,                             // 4 accel correction x-axis
%     accel_corr_y        : accel_corr.y,                             // 5 accel correction y-axis
%     accel_corr_z        : accel_corr.z,                             // 6 accel correction z-axis
%     gps_lat_from_home   : g_gps->latitude-home.lat,                 // 7 lat from home
%     gps_lon_from_home   : g_gps->longitude-home.lng,                // 8 lon from home
%     inav_lat_from_home  : inertial_nav.get_latitude_diff(),         // 9 accel based lat from home
%     inav_lon_from_home  : inertial_nav.get_longitude_diff()        // 10 accel based lon from home
% };

% struct log_Attitude pkt = {
%     LOG_PACKET_HEADER_INIT(LOG_ATTITUDE_MSG),
%     roll_in     : (int16_t)control_roll,
%     roll        : (int16_t)ahrs.roll_sensor,
%     pitch_in    : (int16_t)control_pitch,
%     pitch       : (int16_t)ahrs.pitch_sensor,
%     yaw_in      : (int16_t)g.rc_4.control_in,
%     yaw         : (uint16_t)ahrs.yaw_sensor,
%     nav_yaw     : (uint16_t)nav_yaw
% };

% struct log_Compass pkt = {
%     LOG_PACKET_HEADER_INIT(LOG_COMPASS_MSG),
%     mag_x           : compass.mag_x,
%     mag_y           : compass.mag_y,
%     mag_z           : compass.mag_z,
%     offset_x        : (int16_t)mag_offsets.x,
%     offset_y        : (int16_t)mag_offsets.y,
%     offset_z        : (int16_t)mag_offsets.z,
%     motor_offset_x  : (int16_t)mag_motor_offsets.x,
%     motor_offset_y  : (int16_t)mag_motor_offsets.y,
%     motor_offset_z  : (int16_t)mag_motor_offsets.z
% };

% struct log_Control_Tuning pkt = {
%      LOG_PACKET_HEADER_INIT(LOG_CONTROL_TUNING_MSG),
%      throttle_in         : g.rc_3.control_in,
%      sonar_alt           : sonar_alt,
%      baro_alt            : baro_alt,
%      next_wp_alt         : get_target_alt_for_reporting() / 100.0f,
%      desired_sonar_alt   : (int16_t)target_sonar_alt,
%      angle_boost         : angle_boost,
%      climb_rate          : climb_rate,
%      throttle_out        : g.rc_3.servo_out,
%      desired_climb_rate  : desired_climb_rat
%  };

% end