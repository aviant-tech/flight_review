import unittest
import os
import numpy as np
from functools import partial

from pyulog import ULog
from dbulog import DatabaseULog
import sqlite3

'''
The following attributes are not tested with the current suite, since they
are not contained in the tested ulogs:
    - has_data_appended
    - file_corruption
    - _default_parameters
    - _changed_parameters
    - logged_messages_tagged
'''

class TestDatabaseULog(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        '''
        Sets up the test database.
        '''
        base_dir = os.path.dirname(os.path.realpath(os.path.join(__file__, '..')))
        db_setup_script = os.path.join(base_dir, 'database_ulog.sql')
        TestDatabaseULog.log_dir = os.path.join(base_dir, 'tests/data/log_files')
        TestDatabaseULog.db_path = os.path.join(base_dir, 'tests/data/logs.sqlite3')
        def db_handle():
            return sqlite3.connect(
                TestDatabaseULog.db_path,
                detect_types=sqlite3.PARSE_DECLTYPES|sqlite3.PARSE_COLNAMES
            )
        TestDatabaseULog.db_handle = db_handle

        with open(db_setup_script, 'r') as f:
            setup_sql = f.read()
        with TestDatabaseULog.db_handle() as con:
            cur = con.cursor()
            cur.executescript(setup_sql)
            cur.close()

    @classmethod
    def tearDownClass(cls):
        '''
        Removes the test database after use.
        '''
        os.remove(TestDatabaseULog.db_path)

    def _load_ulog(self, log_id):
        '''
        Takes a log_id, which has to correspond to a .ulg file, and stores it
        into the database. Then it cleans the variable and loads it fresh from
        the database again, ready for testing.
        '''
        log_path = os.path.join(TestDatabaseULog.log_dir, f'{log_id}.ulg')

        self.ulog = ULog(log_path)
        dbulog = DatabaseULog(log_id, TestDatabaseULog.db_handle, ulog=self.ulog)
        dbulog.save()
        del dbulog
        self.dbulog = DatabaseULog(log_id, TestDatabaseULog.db_handle)

    def _compare_lists(self, elem1, elem2, list_name, props, sort_func, callback=None):
        '''
        Deep compares the lists of 'elem1' and 'elem2' specified by the
        'list_name' attribute, by comparing their values according to the
        properties 'props', after first sorting with 'sort_func'. For each
        element pair, one or more callback functions are also called to allow
        for even deeper comparison.
        '''
        self.assertTrue(hasattr(elem1, list_name), list_name)
        self.assertTrue(hasattr(elem2, list_name), list_name)
        list1 = getattr(elem1, list_name)
        list2 = getattr(elem2, list_name)

        # Sort the dataset lists so we can fairly _compare them.
        list1.sort(key=sort_func)
        list2.sort(key=sort_func)
        # Check that their lengths are equal, which zip can hide
        self.assertEqual(len(list1), len(list2))
        for list1_elem, list2_elem in zip(list1, list2):
            for prop in props:
                self.assertTrue(hasattr(list1_elem, prop), prop)
                self.assertTrue(hasattr(list2_elem, prop), prop)
                self.assertEqual(getattr(list1_elem, prop), getattr(list2_elem, prop), prop)
            if callback is not None:
                callback(list1_elem, list2_elem)

    def _compare_dicts(self, elem1, elem2, dict_name, props, callback=None):
        '''
        Deep compares the dicts of 'elem1' and 'elem2' specified by the
        'dict_name' attribute, and compares their values according to the
        properties 'props'. For each element pair, one or more callback
        functions are also called to allow for even deeper comparison.
        '''
        self.assertTrue(hasattr(elem1, dict_name), dict_name)
        self.assertTrue(hasattr(elem2, dict_name), dict_name)
        dict1 = getattr(elem1, dict_name)
        dict2 = getattr(elem2, dict_name)

        # Sort the dataset dicts so we can fairly _compare them.
        dict1_keys = list(dict1.keys())
        dict2_keys = list(dict2.keys())
        dict1_keys.sort()
        dict2_keys.sort()
        # Check that their lengths are equal, which zip can hide
        self.assertEqual(len(dict1_keys), len(dict2_keys))
        for dict1_key, dict2_key in zip(dict1_keys, dict2_keys):
            dict1_elem = dict1[dict1_key]
            dict2_elem = dict2[dict2_key]
            for prop in props:
                self.assertTrue(hasattr(dict1_elem, prop), prop)
                self.assertTrue(hasattr(dict2_elem, prop), prop)
                self.assertEqual(getattr(dict1_elem, prop), getattr(dict2_elem, prop), prop)
            if callback is not None:
                callback(dict1_elem, dict2_elem)

    def _compare_attrs(self, elem1, elem2, attr_name):
        '''
        Compares the attributes 'attr_name' of 'elem1' and 'elem2'.
        '''
        self.assertTrue(hasattr(elem1, attr_name), attr_name)
        self.assertTrue(hasattr(elem2, attr_name), attr_name)

        attr1 = getattr(elem1, attr_name)
        attr2 = getattr(elem2, attr_name)
        self.assertEqual(attr1, attr2, attr_name)

    def _test_replication(self):
        '''
        Test that the DatabaseULog is identical to the ULog object in the
        attributes/methods that matter.
        '''
        self._compare_attrs(self.dbulog, self.ulog, '_file_version'),
        self._compare_attrs(self.dbulog, self.ulog, '_start_timestamp'),
        self._compare_attrs(self.dbulog, self.ulog, '_last_timestamp'),
        self._compare_attrs(self.dbulog, self.ulog, '_compat_flags'),
        self._compare_attrs(self.dbulog, self.ulog, '_incompat_flags'),
        self._compare_attrs(self.dbulog, self.ulog, 'start_timestamp'),
        self._compare_attrs(self.dbulog, self.ulog, 'last_timestamp'),
        self._compare_attrs(self.dbulog, self.ulog, 'has_default_parameters'),
        self._compare_attrs(self.dbulog, self.ulog, 'has_data_appended'),
        self._compare_attrs(self.dbulog, self.ulog, 'file_corruption'),

        self._compare_attrs(self.dbulog, self.ulog, 'msg_info_dict')
        self._compare_attrs(self.dbulog, self.ulog, 'msg_info_multiple_dict')

        self._compare_attrs(self.dbulog, self.ulog, '_default_parameters')
        self._compare_attrs(self.dbulog, self.ulog, '_initial_parameters')
        self._compare_attrs(self.dbulog, self.ulog, '_changed_parameters')

        self._compare_lists(
            self.dbulog, self.ulog,
            list_name='data_list', 
            props=('name', 'multi_id', 'timestamp_idx'),
            sort_func=lambda dataset: (dataset.name, dataset.multi_id),
            callback=partial(
                self._compare_lists,
                list_name='field_data',
                props=('field_name', 'type_str'),
                sort_func=lambda field: field.field_name,
            ),
        )
        self._compare_lists(
            self.dbulog, self.ulog,
            list_name='dropouts', 
            props=('timestamp', 'duration'),
            sort_func=lambda dropout: dropout.timestamp,
        )
        self._compare_dicts(
            self.dbulog, self.ulog,
            'message_formats', 
            ('name', 'fields'),
        )
        self._compare_lists(
            self.dbulog, self.ulog,
            'logged_messages', 
            ('log_level', 'timestamp', 'message'),
            sort_func=lambda message: message.timestamp,
        )
        self._compare_dicts(
            self.dbulog, self.ulog,
            'logged_messages_tagged', 
            ('log_level', 'timestamp', 'message', 'tag'),
        )

    def _test_datasets(self):
        for dataset in self.ulog.data_list:
            db_dataset = self.dbulog.get_dataset(dataset.name, multi_instance=dataset.multi_id)
            ulog_dataset = self.ulog.get_dataset(dataset.name, multi_instance=dataset.multi_id)

            self._compare_lists(
                db_dataset, ulog_dataset,
                list_name='field_data',
                props=('field_name', 'type_str'),
                sort_func=lambda field: field.field_name,
            )
            self._compare_dicts(
                db_dataset, ulog_dataset,
                dict_name='data',
                props=(),
                callback=lambda array1, array2:
                    self.assertTrue(np.array_equal(array1, array2, equal_nan=True))
            )

    def test_medium_log(self):
        ''' Test a normal sized log file.'''
        self._load_ulog('279a4171-7217-4028-b5c7-c217e9d2de33')
        self._test_replication()
        self._test_datasets()

    def test_short_log(self):
        ''' Test a small log file.'''
        self._load_ulog('86b8daec-b6a2-4f28-9239-69e7a11a342f')
        self._test_replication()
        self._test_datasets()
