clear;
close all force;

%% User Constants
file = uigetfile('*.csv', '*.xlsx');
Data = readtable(file,"NumHeaderLines",0);

mAthte = 89; %KiloGrams
cirWheelRear = 2.127; % Meters
cirWheelFront = 2.127; % Meters
rWheelRear = (cirWheelFront/(pi*2)); 
rWheelFront = (cirWheelRear/(pi*2));

CRR = 0.00441;
ACDA = 0.1747;
rPushRim = 0.5;

%% Smoothing Parameters
Window = 100;
A = 2;
B = (1/Window)*ones(1,Window);

%% Datalog data skim
time = Data.Time;
gChairX = Data.AccelX;
gChairY = Data.AccelY;
gChairZ = Data.AccelZ;

dnChairx = Data.GyroX;
dnChairY = Data.GyroY;
dnChairZ = Data.GyroZ;

TAirOut = Data.Temp+287;
PAirOut = Data.Pressure*100;
DAirOut = (PAirOut./(287.*TAirOut));

vChairX = Data.Speed./3.6;
fWheelRPS = (vChairX)./cirWheelRear;

Delta_P_Raw = (Data.DeltaP);
DP_PSI = ((Delta_P_Raw - 8192) ./ 16384) .* 15;
Delta_P = -1.*(Delta_P_Raw-Delta_P_Raw(1));



powermeter = Data.Power;
powermeter_3s = filter(B,A,powermeter);

%% Array setups and Zero Functions
gChairX_Filtered = filter(B,A,gChairX);
gChairZ_Filtered = filter(B,A,gChairZ);
Pitot_DP_Filtered = -filter(B,A,Delta_P);
XZeroed = gChairX_Filtered(1);
ZZeroeed = gChairZ_Filtered(1); 

systemZeroAngle = atan(XZeroed/ZZeroeed);

%% Loop calculations
for i = 2:length(time)
    modgChairX(i) = (gChairX(i) - XZeroed); 
    modgChairZ(i) = gChairZ(i) - ZZeroeed;
    timeZero(i)=time(i)-time(1);

    if modgChairZ(i) > 0
        signZ = -1;
    else 
        signZ = 1;
    end
    if modgChairX(i) > 0
            signX = -1;
    else
        signX = 1;
    end

    delta_vChairX(i) = vChairX(i-1)-vChairX(i);

    if delta_vChairX > 0.5
        vChairX(i) = vChairX(i-1);
    end

    time_Step(i) = time(i)-time(i-1);

    Windy(i) = (sqrt((2*(Pitot_DP_Filtered(i)))/(DAirOut(i)))/2);
    
    SquareCenter(i) = (signX*(modgChairX(i)^2))+(signX*(modgChairZ(i)^2));
    Force_Drag(i) = 0.5*DAirOut(i)*ACDA*(Windy(i)^2);   
    Force_Rolling(i) = 2*CRR*vChairX(i)*(mAthte/2)*9.81;
 
    x(i) = sign(SquareCenter(i)); %Positive Negative signing,
    gLong(i) = x(i)*sqrt(abs(SquareCenter(i)));
    FTireRear(i) = (modgChairX(i)*mAthte)+Force_Drag(i)+Force_Rolling(i);

    TWheelRear(i) = FTireRear(i)*rWheelRear;
    POWER(i) = (fWheelRPS(i)/2*pi()) * TWheelRear(i);

    vChairX_KMH(i) = vChairX(i)*3.6;

    dV(i) = ((vChairX(i)-vChairX(i-1))/time_Step(i));

    if vChairX(i) <= 0
        POWER(i) = 0;
    end
   

end

Window2 = 300;
A2 = 1;
B2 = (1/Window2)*ones(1,Window2);

WindowPow = 300;
APow = 1;
BPow = (1/WindowPow)*ones(1,WindowPow);

WindowVel = 300;
AVel = 1;
BVel = (1/WindowPow)*ones(1,WindowPow);

POWER_Filter = filter(BPow,APow,POWER);
POWER_Filter_Meter = filter(B2,A2,powermeter);
Vel_Filter = filter(BVel,AVel,vChairX);

%% Plot

tiledlayout(4,4)
nexttile([1 4])
plot(timeZero, POWER_Filter, 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Power (Torque Basis) (W)');
grid on;
hold on;
plot(timeZero, POWER_Filter_Meter , 'LineWidth', 2)
legend('Power','Bike Powermeter')
hold off;

nexttile([1 4])
plot(timeZero,Windy, 'LineWidth',2);
hold on
plot(timeZero, Vel_Filter, 'LineWidth',2)
xlabel('Time (s)');
ylabel('Windy (m/s)');
legend('Wind Speed','Vchair','Velocity Chair KMH')
hold off

nexttile([1 2])
plot(timeZero, Force_Drag, 'LineWidth',2)
hold on
plot(timeZero, Force_Rolling, 'LineWidth',2)
xlabel('Time (s)');
ylabel('Force (N)');
legend('Force Drag','Force Roll')
hold off

nexttile([1 1])
plot(timeZero, -Pitot_DP_Filtered, 'LineWidth',2)
hold on
xlabel('Time (s)');
ylabel('Force (N)');
legend('Impulse Force')
hold off

nexttile([1 1])
plot(timeZero, dV, 'LineWidth',2)
hold on
xlabel('Time (s)');
ylabel('DV (m/s)');
legend('DV')
hold off

nexttile([1 4])
plot(timeZero, modgChairX, 'LineWidth',2)
hold on
plot(timeZero, modgChairZ, 'LineWidth',2)

xlabel('Time (s)');
ylabel('Accell');
legend('Accell X','Accell Z')
hold off


 
   