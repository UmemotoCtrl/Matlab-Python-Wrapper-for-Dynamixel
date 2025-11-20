clear;

py.logging.basicConfig(pyargs('level', py.logging.INFO));  % To see packet error messages

Dynamixel = py.importlib.import_module('Dynamixel');
% py.importlib.reload(Dynamixel);
Dynamixel.Dynamixel.print_list_ports();
ports_cell = cell(py.serial.tools.list_ports.comports());
ports = cell(length(  ports_cell  ), 2);
for ii=1:length(ports_cell)
    ports{ii,1} = ports_cell{ii}.device.char;
    ports{ii,2} = ports_cell{ii}.serial_number.char;
end
serial_number = ports{length(ports),2};
port_name = string(Dynamixel.Dynamixel.device_name_from_serial_number(serial_number));
dyn = PyDynamixel(port_name, 3e6);

DXL_IDs = [1; 2];
dyn.setRecommendedValue(DXL_IDs);
dyn.writeMaxPositionLimit(DXL_IDs, [3945; 2969]);
dyn.writeMinPositionLimit(DXL_IDs, [1999; 1097]);

testReadWritePosition(dyn, DXL_IDs);
% testReadWriteVelocity(dyn, DXL_IDs);
% testReadWriteCurrent(dyn, DXL_IDs);
% testReadSettings(dyn, DXL_IDs);

dyn.delete();

function testReadWritePosition(dyn, DXL_IDs)
    len = length(DXL_IDs);
    dyn.writeTorqueEnable(DXL_IDs, 0*ones(len,1));
    dyn.writeOperatingMode(DXL_IDs, 3*ones(len,1));
    dyn.writeTorqueEnable(DXL_IDs, 1*ones(len,1));
    
    py.logging.info( "Position: %s", dyn.readPresentPosition(DXL_IDs) );
    dyn.writeGoalPosition( DXL_IDs, dyn.readMaxPositionLimit(DXL_IDs) );
    pause(1.0);
    py.logging.info( "Position: %s", dyn.readPresentPosition(DXL_IDs) );
    dyn.writeGoalPosition( DXL_IDs, dyn.readMinPositionLimit(DXL_IDs) );
    pause(1.0);
    py.logging.info( "Position: %s", dyn.readPresentPosition(DXL_IDs) );

    num = 100;
    t1 = tic;
    for ii=1:num
        dyn.readPresentPosition(DXL_IDs);
        dyn.readPresentVelocity(DXL_IDs);
    end
    py.logging.info("%s msec (%d reads)", toc(t1)*1000, num);
    py.logging.info( "Position: %s", dyn.readPresentPosition(DXL_IDs) );
    py.logging.info( "Velocity: %s", dyn.readPresentVelocity(DXL_IDs) );

    dyn.writeTorqueEnable(DXL_IDs, 0*ones(len,1));


    % num = 100
    % for i in range(3):
    %     start = time.time()
    %     for i in range(num):
    %         dyn.readPresentPosition(DXL_IDs)
    %         dyn.readPresentVelocity(DXL_IDs)
    %     logging.info("elapsed_time(%s read):%s[msec]", num, time.time()*1000 - start*1000)
    %     logging.info( "Position: %s", dyn.readPresentPosition(DXL_IDs) )
    %     logging.info( "Velocity: %s", dyn.readPresentVelocity(DXL_IDs) )
    
end

function testReadWriteVelocity(dyn, DXL_IDs)
    len = length(DXL_IDs);
    dyn.writeTorqueEnable(DXL_IDs, 0*ones(len,1));
    dyn.writeOperatingMode(DXL_IDs, 1*ones(len,1));
    dyn.writeTorqueEnable(DXL_IDs, 1*ones(len,1));
    
    dyn.writeGoalVelocity(DXL_IDs, 10*ones(len,1));
    pause(1.0);
    tic;
    a = dyn.readPresentVelocity(DXL_IDs);
    fprintf('read: %.3f(ms)\n', toc*1e3);
    disp(a);
    tic;
    dyn.writeGoalVelocity(DXL_IDs, -10*ones(len,1));
    fprintf('write: %.3f(ms)\n', toc*1e3);
    pause(1.0);
    tic;
    a = dyn.readPresentVelocity(DXL_IDs);
    fprintf('read: %.3f(ms)\n', toc*1e3);
    disp(a);
    
    dyn.writeTorqueEnable(DXL_IDs, zeros(len,1));
end

function testReadWriteCurrent(dyn, DXL_IDs)
    len = length(DXL_IDs);
    dyn.writeTorqueEnable(DXL_IDs, 0*ones(len,1));
    dyn.writeOperatingMode(DXL_IDs, 0*ones(len,1));
    dyn.writeTorqueEnable(DXL_IDs, 1*ones(len,1));
    
    dyn.writeGoalCurrent(DXL_IDs, -5*ones(len,1));
    pause(1.0);
    tic;
    a = dyn.readPresentCurrent(DXL_IDs);
    fprintf('read: %.3f(ms)\n', toc*1e3);
    disp(a);
    tic;
    dyn.writeGoalCurrent(DXL_IDs, 3*ones(len,1));
    fprintf('write: %.3f(ms)\n', toc*1e3);
    pause(1.0);
    tic;
    a = dyn.readPresentCurrent(DXL_IDs);
    fprintf('read: %.3f(ms)\n', toc*1e3);
    disp(a);
    
    dyn.writeTorqueEnable(DXL_IDs, zeros(len,1));
end

function testReadSettings(dyn, DXL_IDs)
    disp(dyn.readReturnDelayTime(DXL_IDs)');
    disp(dyn.readDriveMode(DXL_IDs)');
    disp(dyn.readOperatingMode(DXL_IDs)');
    disp(dyn.readTorqueEnable(DXL_IDs)');
    disp(dyn.readLED(DXL_IDs)');
    disp(dyn.readStatusReturnLevel(DXL_IDs)');
end