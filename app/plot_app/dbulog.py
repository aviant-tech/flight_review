from pyulog import ULog
import struct
import numpy as np
import sqlite3
from datetime import datetime

class DatabaseULog(ULog):

    '''
    This class should do nothing except override the constructor and setting
    appropriate values from a connection and log id. The choice of whether to
    load ULog from file or DatabaseULog from DB should be done outside in the
    load_ulog_file function.
    '''

    def __init__(self, log_id, db_handle, ulog=None,
            disable_str_exceptions=True, message_name_filter_list=None):
        self._log_id = log_id
        self._db = db_handle

        attrs = [
            ('_debug', False),
            ('_file_corrupt', False),
            ('_start_timestamp', 0),
            ('_last_timestamp', 0),
            ('_msg_info_dict', {}),
            ('_msg_info_multiple_dict', {}),
            ('_initial_parameters', {}),
            ('_default_parameters', {}),
            ('_changed_parameters', []),
            ('_message_formats', {}),
            ('_logged_messages', []),
            ('_logged_messages_tagged', {}),
            ('_dropouts', []),
            ('_data_list', []),
            ('_subscriptions', {}),
            ('_filtered_message_ids', set()),
            ('_missing_message_ids', set()),
            ('_file_version', 0),
            ('_compat_flags', [0] * 8),
            ('_incompat_flags', [0] * 8),
            ('_appended_offsets', []),
            ('_has_sync', True),
            ('_sync_seq_cnt', 0),
        ]
        ULog._disable_str_exceptions = disable_str_exceptions

        if ulog is not None:
            for attr_name, _ in attrs:
                setattr(self, attr_name, getattr(ulog, attr_name))
        else:
            for attr_name, default_value in attrs:
                setattr(self, attr_name, default_value)
            self.load()

    @property
    def db(self):
        return self._db

    def load(self):
        '''
        Load all necessary data from the database, except for the data series themselves.
        '''
        with self.db() as con:
            cur = con.cursor()

            # Check for existence
            cur.execute('''
                SELECT COUNT(*)
                FROM ULog
                WHERE Id = ?
                ''', (self._log_id,))
            count, = cur.fetchone()
            if count == 0:
                raise KeyError(f'No ULog in database with Id={self._log_id}')

            # ULog metadata
            cur.execute('''
                SELECT FileVersion, StartTimestamp, LastTimestamp, CompatFlags, IncompatFlags
                FROM ULog
                WHERE Id = ?
                ''', (self._log_id,))
            ulog_result = cur.fetchone()
            self._file_version = ulog_result[0]
            self._start_timestamp = ulog_result[1]
            self._last_timestamp = ulog_result[2]
            self._compat_flags = [ord(c) for c in ulog_result[3]]
            self._incompat_flags = [ord(c) for c in ulog_result[4]]

            # data_list
            self._data_list = []
            cur.execute('''
                SELECT Id, DatasetName, MultiId, TimestampIndex
                FROM ULogDataset
                WHERE ULogId = ?
                ''', (self._log_id,))
            dataset_results = cur.fetchall()
            for dataset_id, dataset_name, multi_id, timestamp_idx in dataset_results:
                cur.execute('''
                    SELECT Id, TopicName, DataType
                    FROM ULogField
                    WHERE DatasetId = ?
                    ''', (dataset_id,))
                field_results = cur.fetchall()
                fields = []
                #data = {}
                for field_id, field_name, data_type in field_results:
                    fields.append(DatabaseULog._FieldData(field_name=field_name,type_str=data_type))
                    dtype = DatabaseULog._UNPACK_TYPES[data_type][2]
                    #data[field_name] = np.frombuffer(value_bytes, dtype=dtype)

                dataset = DatabaseULog.DatabaseData(
                    name=dataset_name,
                    multi_id=multi_id,
                    timestamp_idx=timestamp_idx,
                    field_data=fields,
                    #data=data,
                )
                self._data_list.append(dataset)

            # dropouts
            cur.execute('''
                SELECT Timestamp, Duration
                FROM ULogMessageDropout
                WHERE ULogId = ?
                ''', (self._log_id,))
            for timestamp, duration in cur.fetchall():
                self._dropouts.append(
                    DatabaseULog.DatabaseMessageDropout(
                        timestamp=timestamp,
                        duration=duration,
                    )
                )

            # logged_messages
            cur.execute('''
                SELECT LogLevel, Timestamp, Message
                FROM ULogMessageLogging
                WHERE ULogId = ?
                ''', (self._log_id,))
            for log_level, timestamp, message in cur.fetchall():
                self._logged_messages.append(
                    DatabaseULog.DatabaseMessageLogging(
                        log_level=log_level,
                        timestamp=timestamp,
                        message=message,
                    )
                )

            # logged_messages_tagged
            cur.execute('''
                SELECT LogLevel, Tag, Timestamp, Message
                FROM ULogMessageLoggingTagged
                WHERE ULogId = ?
                ''', (self._log_id,))
            for log_level, tag, timestamp, message in cur.fetchall():
                self._logged_messages_tagged.append(
                    DatabaseULog.DatabaseMessageLoggingTagged(
                        log_level=log_level,
                        tag=tag,
                        timestamp=timestamp,
                        message=message,
                    )
                )

            # message_formats
            cur.execute('''
                SELECT Id, Name
                FROM ULogMessageFormat
                WHERE ULogId = ?
                ''', (self._log_id,))
            for message_id, name in cur.fetchall():
                cur.execute('''
                    SELECT FieldType, ArraySize, Name
                    FROM ULogMessageFormatField
                    WHERE MessageId = ?
                    ''', (message_id,))
                self._message_formats[name] = DatabaseULog.DatabaseMessageFormat(name=name, fields=cur.fetchall())

            # msg_info_dict
            cur.execute('''
                SELECT Key, Value
                FROM ULogMessageInfo
                WHERE ULogId = ?
                ''', (self._log_id,))
            for key, value in cur.fetchall():
                self._msg_info_dict[key] = value

            # msg_info_multiple_dict
            cur.execute('''
                SELECT Id, Key
                FROM ULogMessageInfoMultiple
                WHERE ULogId = ?
                ''', (self._log_id,))
            for message_id, key in cur.fetchall():
                self._msg_info_multiple_dict[key] = list()
                cur.execute('''
                    SELECT Id
                    FROM ULogMessageInfoMultipleList
                    WHERE MessageId = ?
                    ORDER BY SeriesIndex
                    ''', (message_id,))
                for (list_id,) in cur.fetchall():
                    cur.execute('''
                        SELECT Value
                        FROM ULogMessageInfoMultipleListElement
                        WHERE ListId = ?
                        ORDER BY SeriesIndex
                        ''', (list_id,))
                    self._msg_info_multiple_dict[key].append([value for (value,) in cur.fetchall()])

            # initial_parameters
            cur.execute('''
                SELECT Key, Value
                FROM ULogInitialParameter
                WHERE ULogId = ?
                ''', (self._log_id,))
            for key, value in cur.fetchall():
                self._initial_parameters[key] = value

            cur.close()

    def get_dataset(self, name, multi_instance=0):
        '''
        name and multi_instance parameters are named the same way as the
        original ULog file for compatibility
        '''
        with self.db() as con:
            cur = con.cursor()
            cur.execute('''
                SELECT Id, TimestampIndex
                FROM ULogDataset
                WHERE DatasetName = ? AND MultiId = ? AND ULogId = ?
                ''', (name, multi_instance, self._log_id))
            dataset_result = cur.fetchone()
            if dataset_result is None:
                raise KeyError(f'Dataset with name {name} and multi id {multi_instance} not found.')
            dataset_id, timestamp_idx = dataset_result
            cur.execute('''
                SELECT TopicName, DataType, ValueArray
                FROM ULogField
                WHERE DatasetId = ?
                ''', (dataset_id,))
            field_results = cur.fetchall()
            fields = []
            data = {}
            for field_name, data_type, value_bytes in field_results:
                fields.append(DatabaseULog._FieldData(field_name=field_name,type_str=data_type))
                dtype = DatabaseULog._UNPACK_TYPES[data_type][2]
                data[field_name] = np.frombuffer(value_bytes, dtype=dtype)

            dataset = DatabaseULog.DatabaseData(
                name=name,
                multi_id=multi_instance,
                timestamp_idx=timestamp_idx,
                field_data=fields,
                data=data,
            )
        return dataset

    def save(self):
        with self.db() as con:
            cur = con.cursor()

            # ULog metadata
            cur.execute('''
                INSERT INTO ULog
                (Id, FileVersion, StartTimestamp, LastTimestamp, CompatFlags, IncompatFlags)
                VALUES
                (?, ?, ?, ?, ?, ?)
            ''', (
                    self._log_id,
                    self._file_version,
                    self._start_timestamp,
                    self._last_timestamp,
                    ''.join([chr(n) for n in self._compat_flags]),
                    ''.join([chr(n) for n in self._incompat_flags]),
                )
            )

            # data_list
            for dataset in self.data_list:
                cur.execute('''
                    INSERT INTO ULogDataSet
                    (DatasetName, MultiId, TimestampIndex, ULogId)
                    VALUES
                    (?, ?, ?, ?)
                    ''', (
                        dataset.name,
                        dataset.multi_id,
                        dataset.timestamp_idx,
                        self._log_id,
                    )
                )
                dataset_id = cur.lastrowid
                for field in dataset.field_data:
                    cur.execute('''
                        INSERT INTO ULogField
                        (TopicName, DataType, ValueArray, DatasetId)
                        VALUES
                        (?, ?, ?, ?)
                        ''', (
                            field.field_name,
                            field.type_str,
                            dataset.data[field.field_name].tobytes(),
                            dataset_id,
                        )
                    )

            # dropouts
            cur.executemany('''
                INSERT INTO ULogMessageDropout
                (Timestamp, Duration, ULogId)
                VALUES
                (?, ?, ?)
                ''', [(
                    dropout.timestamp,
                    dropout.duration,
                    self._log_id,
                ) for dropout in self._dropouts])

            # logged_messages
            cur.executemany('''
                INSERT INTO ULogMessageLogging
                (LogLevel, Timestamp, Message, ULogId)
                VALUES
                (?, ?, ?, ?)
                ''', [(
                    message.log_level,
                    message.timestamp,
                    message.message,
                    self._log_id,
                ) for message in self._logged_messages])

            # logged_messages_tagged
            cur.executemany('''
                INSERT INTO ULogMessageLoggingTagged
                (LogLevel, Timestamp, Tag, Message, ULogId)
                VALUES
                (?, ?, ?, ?, ?)
                ''', [(
                    message.log_level,
                    message.timestamp,
                    message.tag,
                    message.message,
                    self._log_id,
                ) for message in self._logged_messages_tagged])

            # message_formats
            for name, format in self._message_formats.items():
                cur.execute('''
                    INSERT INTO ULogMessageFormat
                    (Name, ULogId)
                    VALUES
                    (?, ?)
                    ''', (name, self._log_id))
                format_id = cur.lastrowid
                cur.executemany('''
                    INSERT INTO ULogMessageFormatField
                    (FieldType, ArraySize, Name, MessageId)
                    VALUES
                    (?, ?, ?, ?)
                    ''', [(*field, format_id) for field in format.fields])
            # msg_info_dict
            cur.executemany('''
                INSERT INTO ULogMessageInfo
                (Key, Value, ULogId)
                VALUES
                (?, ?, ?)
                ''', [(
                    key,
                    value,
                    self._log_id,
                ) for key, value in self.msg_info_dict.items()])

            # msg_info_multiple_dict
            for key, lists in self.msg_info_multiple_dict.items():
                cur.execute('''
                    INSERT INTO ULogMessageInfoMultiple
                    (Key, ULogId)
                    VALUES
                    (?, ?)
                    ''', (key, self._log_id))
                message_id = cur.lastrowid
                for list_index, message_list in enumerate(lists):
                    cur.execute('''
                        INSERT INTO ULogMessageInfoMultipleList
                        (SeriesIndex, MessageId)
                        VALUES
                        (?, ?)
                        ''', (list_index, message_id))
                    list_id = cur.lastrowid
                    cur.executemany('''
                        INSERT INTO ULogMessageInfoMultipleListElement
                        (SeriesIndex, Value, ListId)
                        VALUES
                        (?, ?, ?)
                        ''', [(
                            series_index,
                            value,
                            list_id,
                        ) for series_index, value in enumerate(message_list)])

            # initial_parameters
            cur.executemany('''
                INSERT INTO ULogInitialParameter
                (Key, Value, ULogId)
                VALUES
                (?, ?, ?)
                ''', [(
                    key,
                    value,
                    self._log_id,
                ) for key, value in self.initial_parameters.items()])

            cur.close()

    class DatabaseData(ULog.Data):
        '''
        Overrides the ULog.Data class since its constructor only
        reads ULog.MessageLogAdded objects, and we want to specify
        the fields directly instead for simplicity.
        '''
        def __init__(self, name, multi_id, timestamp_idx, field_data, data=None):
            self.name = name
            self.multi_id = multi_id
            self.timestamp_idx = timestamp_idx
            self.field_data = field_data
            self.data = data

    class DatabaseMessageDropout(ULog.MessageDropout):
        '''
        Overrides the ULog.MessageDropout class since its
        constructor is not suitable for out purpose.
        '''
        def __init__(self, timestamp, duration):
            self.timestamp = timestamp
            self.duration = duration

    class DatabaseMessageFormat(ULog.MessageFormat):
        '''
        Overrides the ULog.MessageFormat class since its
        constructor is not suitable for out purpose.
        '''
        def __init__(self, name, fields):
            self.name = name
            self.fields = fields

    class DatabaseMessageLogging(ULog.MessageLogging):
        '''
        Overrides the ULog.MessageLogging class since its
        constructor is not suitable for out purpose.
        '''
        def __init__(self, log_level, timestamp, message):
            self.log_level = log_level
            self.timestamp = timestamp
            self.message = message

    class DatabaseMessageLoggingTagged(ULog.MessageLoggingTagged):
        '''
        Overrides the ULog.MessageLoggingTagged class since its
        constructor is not suitable for out purpose.
        '''
        def __init__(self, log_level, timestamp, message):
            self.log_level = log_level
            self.timestamp = timestamp
            self.message = message
