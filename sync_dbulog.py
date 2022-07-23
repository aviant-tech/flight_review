#! /usr/bin/env python3

# Script to sync all ULog files to database

import sqlite3
import os
import argparse
import logging
import re

from pyulog import ULog
from pyulog.db import DatabaseULog
from plot_app.config import get_db_filename, get_log_filepath


parser = argparse.ArgumentParser(description='Sync ULog files to database')
parser.add_argument('--verbose',
                    action='store_true',
                    help='Print verbose messages.')
parser.add_argument('--unregister',
                    action='store_true',
                    help='Unregister Logs rows with no corresponding log file nor ULog row')
parser.add_argument('--register',
                    action='store_true',
                    help='Register and generate log files with no corresponding Logs row')
args = parser.parse_args()

if args.verbose:
    logging.basicConfig(level=logging.DEBUG)
else:
    logging.basicConfig(level=logging.INFO)

db_handle = DatabaseULog.get_db_handle(get_db_filename())

log_ids_in_db = set()
log_ids_with_dbulog = set()
with db_handle() as db:
    cur = db.cursor()
    cur.execute('SELECT Id, ULogId FROM Logs', [])
    for log_id, dbulog_pk in cur.fetchall():
        log_ids_in_db.add(log_id)

        if dbulog_pk is None:
            logging.debug('Log with Id=%s has no ULogId', log_id)
        elif not DatabaseULog.exists_in_db(db_handle, dbulog_pk):
            logging.warning('Log with Id=%s has a ULogId, but it does not point to a ULog row', log_id)
        else:
            log_ids_with_dbulog.add(log_id)
            logging.debug('Log with Id=%s has valid ULogId=%d', log_id, dbulog_pk)
            continue

dbulog_pks_in_db = set()
with db_handle() as db:
    cur = db.cursor()
    cur.execute('SELECT Id FROM ULog', [])
    for dbulog_pk, in cur.fetchall():
        dbulog_pks_in_db.add(dbulog_pk)
        logging.debug('Found ULog row with Id %d', dbulog_pk)

    for dbulog_pk in dbulog_pks_in_db:
        cur.execute('SELECT Id FROM Logs WHERE ULogId=?', [dbulog_pk])
        if cur.fetchone() is None:
            logging.warning('ULog with Id %d has no corresponding Logs row', dbulog_pk)

log_ids_in_files = set()
log_dir_path = get_log_filepath()
for filename in os.listdir(log_dir_path):
    match = re.match(r'([a-z0-9-]+)\.ulg$', filename)
    if match:
        log_id = match.group(1)
        log_ids_in_files.add(log_id)
        logging.debug('Found log file %s', match.group(0))
    else:
        logging.debug('Invalid file in log directory %s: %s', log_dir_path, filename)

unregistered_log_ids = log_ids_in_files - log_ids_in_db
no_data_log_ids = log_ids_in_db.difference(log_ids_in_files).difference(log_ids_with_dbulog)

for log_id in unregistered_log_ids:
    logging.debug('Log file %s.ulg has no corresponding Logs entry.', log_id)
for log_id in no_data_log_ids:
    logging.debug('Log row with Id=%s has no corresponding data.', log_id)

logging.info('Files in log directory: %d', len(log_ids_in_files))
logging.info('Logs in database: %d', len(log_ids_in_db))
logging.info('ULogs in database: %d', len(dbulog_pks_in_db))
logging.info('Logs to register: %d', len(unregistered_log_ids))
logging.info('Logs to unregister: %d', len(no_data_log_ids))

if args.register:
    for i, log_id in enumerate(unregistered_log_ids, start=1):
        logging.info('(%d/%d) Registering Logs row with Id=%s',
                      i, len(unregistered_log_ids), log_id)
        filename = os.path.join(log_dir_path, f'{log_id}.ulg')
        ulog = ULog(filename)
        dbulog = DatabaseULog(db_handle, ulog=ulog)
        dbulog.save()

        with db_handle() as db:
            cur = db.cursor()
            cur.execute('''
                INSERT INTO Logs (Id, ULogId)
                VALUES (?, ?)
            ''', [log_id, dbulog.primary_key])
            cur.close()

if args.unregister:
    for i, log_id in enumerate(no_data_log_ids, start=1):
        logging.info('(%d/%d) Unregistering Logs row with Id=%s',
                      i, len(no_data_log_ids), log_id)
        with db_handle() as db:
            cur = db.cursor()
            cur.execute('''
                DELETE FROM Logs
                WHERE Id=?
            ''', [log_id])
            cur.close()

