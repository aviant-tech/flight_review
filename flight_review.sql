CREATE TABLE IF NOT EXISTS Logs (
        Id TEXT,  -- log id (part of the file name)
        ULogId INT,  -- reference to ULog
        Title TEXT, 
        Description TEXT, 
        OriginalFilename TEXT, 
        Date TIMESTAMP,  -- date & time when uploaded
        AllowForAnalysis INTEGER,  -- if 1 allow for statistical analysis
        Obfuscated INTEGER, 
        Source TEXT,  -- where it comes from: 'webui', 'CI', 'QGroundControl'
        Email TEXT,  -- email (may be empty)
        WindSpeed INT,  -- Wind speed in beaufort scale
        Rating TEXT,  -- how the flight was rated
        Feedback TEXT,  -- additional feedback
        Type TEXT,  -- upload type: 'personal' (or '') or 'flightreport'
        VideoUrl TEXT, 
        ErrorLabels TEXT,  -- the type of error (if any) that occurred during flight
        Public INT,  -- if 1 this log can be publicly listed
        Token TEXT,  -- Security token (currently used to delete the entry)
        CONSTRAINT Id_PK PRIMARY KEY (Id)
);

CREATE TABLE IF NOT EXISTS LogsGenerated(
        Id TEXT,  -- log id
        Duration INT,  -- logging duration in [s]
        MavType TEXT,  -- vehicle type
        Estimator TEXT, 
        AutostartId INT,  -- airframe config
        Hardware TEXT,  -- board
        Software TEXT,  -- software (git tag)
        NumLoggedErrors INT,  -- number of logged error messages (or more severe)
        NumLoggedWarnings INT, 
        FlightModes TEXT,  -- all flight modes as comma-separated ints
        SoftwareVersion TEXT,  -- release version
        UUID TEXT,  -- vehicle UUID (sys_uuid in log)
        FlightModeDurations TEXT,  -- comma-separated list of <flight_mode_int>:<duration_sec>
        StartTime INT,  --UTC Timestap from GPS log (useful when uploading multiple logs)
        CONSTRAINT Id_PK PRIMARY KEY (Id)
);

CREATE TABLE IF NOT EXISTS Vehicle (
        UUID TEXT,  -- vehicle UUID (sys_uuid in log)
        LatestLogId TEXT,  -- log id of latest uploaded log file
        Name TEXT,  -- vehicle Name (as provided by the uploader)
        FlightTime INTEGER,  -- latest flight time in seconds
        CONSTRAINT UUID_PK PRIMARY KEY (UUID)
);
