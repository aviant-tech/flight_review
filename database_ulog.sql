CREATE TABLE IF NOT EXISTS ULog (
        Id TEXT PRIMARY KEY,
        FileVersion INT,
        StartTimestamp REAL,
        LastTimestamp REAL,
        CompatFlags TEXT,
        IncompatFlags TEXT
);

CREATE TABLE IF NOT EXISTS ULogDataset (
        Id INTEGER PRIMARY KEY AUTOINCREMENT,
        DatasetName TEXT,
        MultiId INT,
        TimestampIndex INT,
        ULogId TEXT REFERENCES ULog (Id),
        UNIQUE (ULogId, DatasetName, MultiId)
);

CREATE TABLE IF NOT EXISTS ULogField (
        Id INTEGER PRIMARY KEY AUTOINCREMENT,
        TopicName TEXT,
        DataType TEXT,
        ValueArray BLOB,

        DatasetId INTEGER REFERENCES ULogDataset (Id)
);
CREATE INDEX IF NOT EXISTS btree_ulogfield_datasetid ON ULogField(DatasetId);

CREATE TABLE IF NOT EXISTS ULogMessageDropout (
        Timestamp REAL,
        Duration FLOAT,
        ULogId TEXT REFERENCES ULog (Id)
);

CREATE TABLE IF NOT EXISTS ULogMessageFormat (
        Id INTEGER PRIMARY KEY AUTOINCREMENT,
        Name TEXT,
        ULogId TEXT REFERENCES ULog (Id)
);

CREATE TABLE IF NOT EXISTS ULogMessageFormatField (
        FieldType TEXT,
        ArraySize INT,
        Name TEXT,
        MessageId INT REFERENCES ULogMessageFormat (Id)
);

CREATE TABLE IF NOT EXISTS ULogMessageLogging (
        LogLevel INT,
        Timestamp REAL,
        Message TEXT,
        ULogId TEXT REFERENCES ULog (Id)
);

CREATE TABLE IF NOT EXISTS ULogMessageLoggingTagged (
        LogLevel INT,
        Timestamp REAL,
        Tag TEXT,
        Message TEXT,
        ULogId TEXT REFERENCES ULog (Id)
);

CREATE TABLE IF NOT EXISTS ULogMessageInfo (
        Key TEXT,
        Value BLOB,

        ULogId TEXT REFERENCES ULog (Id)
);

CREATE TABLE IF NOT EXISTS ULogMessageInfoMultiple (
        Id INTEGER PRIMARY KEY AUTOINCREMENT,
        Key TEXT,
        ULogId TEXT REFERENCES ULog (Id)
);
CREATE TABLE IF NOT EXISTS ULogMessageInfoMultipleList (
        Id INTEGER PRIMARY KEY AUTOINCREMENT,
        SeriesIndex INTEGER,
        MessageId TEXT REFERENCES ULogMessageInfoMultiple (Id)
);
CREATE TABLE IF NOT EXISTS ULogMessageInfoMultipleListElement (
        Id INTEGER PRIMARY KEY AUTOINCREMENT,
        SeriesIndex INTEGER,
        Value TEXT,
        ListId TEXT REFERENCES ULogMessageInfoMultipleList (Id)
);

CREATE TABLE IF NOT EXISTS ULogInitialParameter (
        Key TEXT,
        Value BLOB,

        ULogId TEXT REFERENCES ULog (Id)
);
