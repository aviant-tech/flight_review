""" some helper methods that don't fit in elsewhere """
import json
from timeit import default_timer as timer
import time
import tornado
import re
import os
import traceback
import sys
from functools import lru_cache
from urllib.request import urlretrieve
import xml.etree.ElementTree # airframe parsing
import shutil
import uuid
import numpy as np

from pyulog import *
from pyulog.db import DatabaseULog
from pyulog.px4 import *

from config_tables import *
from config import get_log_filepath, get_airframes_filename, get_airframes_url, \
                   get_parameters_filename, get_parameters_url, \
                   get_log_cache_size, debug_print_timing, \
                   get_releases_filename, get_db_filename, get_ulogdb_filename

#pylint: disable=line-too-long, global-variable-not-assigned,invalid-name,global-statement

def print_timing(name, start_time):
    """ for debugging: print elapsed time, with start_time = timer(). """
    if debug_print_timing():
        print(name + " took: {:.3} s".format(timer() - start_time))


# the following is for using the plotting app locally
__log_id_is_filename = {'enable': False}
def set_log_id_is_filename(enable=False):
    """ treat the log_id as filename instead of id if enable=True.

    WARNING: this disables log id validation, so that all log id's are valid.
    Don't use it on a live server! """

    __log_id_is_filename['enable'] = enable

def _check_log_id_is_filename():
    return __log_id_is_filename['enable']

def is_running_locally():
    """
    Check if we run locally.
    Avoid using this if possible as it makes testing more difficult
    """
    # this is an approximation: it's actually only True if a log is displayed
    # directly via ./serve.py -f <file.ulg>
    return __log_id_is_filename['enable']


def validate_log_id(log_id):
    """ Check whether the log_id has a valid form (not whether it actually
    exists) """
    if _check_log_id_is_filename():
        return True
    # we are a bit less restrictive than the actual format
    if re.match(r'^[0-9a-zA-Z_-]+$', log_id):
        return True
    return False

def get_log_filename(log_id):
    """ return the ulog file name from a log id in the form:
        xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx
    """
    if _check_log_id_is_filename():
        return log_id
    return os.path.join(get_log_filepath(), log_id + '.ulg')


__last_failed_downloads = {} # dict with key=file name and a timestamp of last failed download

def download_file_maybe(filename, url):
    """ download an url to filename if it does not exist or it's older than a day.
        returns True if the file can be used
    """
    need_download = False
    if os.path.exists(filename):
        elapsed_sec = time.time() - os.path.getmtime(filename)
        if elapsed_sec / 3600 > 24:
            need_download = True
            os.unlink(filename)
    else:
        need_download = True
    if need_download:
        if filename in __last_failed_downloads:
            if time.time() < __last_failed_downloads[filename] + 30:
                # don't try to download too often
                return False
        print("Downloading "+url)
        try:
            # download to a temporary random file, then move to avoid race
            # conditions
            temp_file_name = filename+'.'+str(uuid.uuid4())
            urlretrieve(url, temp_file_name)
            shutil.move(temp_file_name, filename)
        except Exception as e:
            print("Download error: "+str(e))
            __last_failed_downloads[filename] = time.time()
            return False
    return True


@lru_cache(maxsize=128)
def __get_airframe_data(airframe_id):
    """ cached version of get_airframe_data()
    """
    airframe_xml = get_airframes_filename()
    if download_file_maybe(airframe_xml, get_airframes_url()):
        try:
            e = xml.etree.ElementTree.parse(airframe_xml).getroot()
            for airframe_group in e.findall('airframe_group'):
                for airframe in airframe_group.findall('airframe'):
                    if str(airframe_id) == airframe.get('id'):
                        ret = {'name': airframe.get('name')}
                        try:
                            ret['type'] = airframe.find('type').text
                        except:
                            pass
                        return ret
        except:
            pass
    return None

__last_airframe_cache_clear_timestamp = 0
def get_airframe_data(airframe_id):
    """ return a dict of aiframe data ('name' & 'type') from an autostart id.
    Downloads aiframes if necessary. Returns None on error
    """
    global __last_airframe_cache_clear_timestamp
    current_time = time.time()
    if current_time > __last_airframe_cache_clear_timestamp + 3600:
        __last_airframe_cache_clear_timestamp = current_time
        __get_airframe_data.cache_clear()
    return __get_airframe_data(airframe_id)

def get_sw_releases():
    """ return a JSON object of public releases.
    Downloads releases from github if necessary. Returns None on error
    """

    releases_json = get_releases_filename()
    if download_file_maybe(releases_json, 'https://api.github.com/repos/PX4/Firmware/releases'):
        with open(releases_json) as data_file:
            return json.load(data_file)
    return None

def get_default_parameters():
    """ get the default parameters

        :return: dict with params (key is param name, value is a dict with
                 'default', 'min', 'max', ...)
    """
    parameters_xml = get_parameters_filename()
    param_dict = {}
    if download_file_maybe(parameters_xml, get_parameters_url()):
        try:
            e = xml.etree.ElementTree.parse(parameters_xml).getroot()
            for group in e.findall('group'):
                group_name = group.get('name')
                try:
                    for param in group.findall('parameter'):
                        param_name = param.get('name')
                        param_type = param.get('type')
                        param_default = param.get('default')
                        cur_param_dict = {
                            'default': param_default,
                            'type': param_type,
                            'group_name': group_name,
                            }
                        try:
                            cur_param_dict['min'] = param.find('min').text
                        except:
                            pass
                        try:
                            cur_param_dict['max'] = param.find('max').text
                        except:
                            pass
                        try:
                            cur_param_dict['short_desc'] = param.find('short_desc').text
                        except:
                            pass
                        try:
                            cur_param_dict['long_desc'] = param.find('long_desc').text
                        except:
                            pass
                        try:
                            cur_param_dict['decimal'] = param.find('decimal').text
                        except:
                            pass
                        param_dict[param_name] = cur_param_dict
                except:
                    pass
        except:
            pass
    return param_dict

def WGS84_to_mercator(lon, lat):
    """ Convert lon, lat in [deg] to Mercator projection """
# alternative that relies on the pyproj library:
# import pyproj # GPS coordinate transformations
#    wgs84 = pyproj.Proj('+proj=longlat +ellps=WGS84 +datum=WGS84 +no_defs')
#    mercator = pyproj.Proj('+proj=merc +a=6378137 +b=6378137 +lat_ts=0.0 ' +
#       '+lon_0=0.0 +x_0=0.0 +y_0=0 +units=m +k=1.0 +nadgrids=@null +no_defs')
#    return pyproj.transform(wgs84, mercator, lon, lat)

    semimajor_axis = 6378137.0  # WGS84 spheriod semimajor axis
    east = lon * 0.017453292519943295
    north = lat * 0.017453292519943295
    northing = 3189068.5 * np.log((1.0 + np.sin(north)) / (1.0 - np.sin(north)))
    easting = semimajor_axis * east

    return easting, northing

def map_projection(lat, lon, anchor_lat, anchor_lon):
    """ convert lat, lon in [rad] to x, y in [m] with an anchor position """
    sin_lat = np.sin(lat)
    cos_lat = np.cos(lat)
    cos_d_lon = np.cos(lon - anchor_lon)
    sin_anchor_lat = np.sin(anchor_lat)
    cos_anchor_lat = np.cos(anchor_lat)

    arg = sin_anchor_lat * sin_lat + cos_anchor_lat * cos_lat * cos_d_lon
    arg[arg > 1] = 1
    arg[arg < -1] = -1

    np.set_printoptions(threshold=sys.maxsize)
    c = np.arccos(arg)
    k = np.copy(lat)
    for i in range(len(lat)):
        if np.abs(c[i]) < np.finfo(float).eps:
            k[i] = 1
        else:
            k[i] = c[i] / np.sin(c[i])

    CONSTANTS_RADIUS_OF_EARTH = 6371000
    x = k * (cos_anchor_lat * sin_lat - sin_anchor_lat * cos_lat * cos_d_lon) * \
        CONSTANTS_RADIUS_OF_EARTH
    y = k * cos_lat * np.sin(lon - anchor_lon) * CONSTANTS_RADIUS_OF_EARTH

    return x, y


def html_long_word_force_break(text, max_length=15):
    """
    force line breaks for text that contains long words, suitable for HTML
    display
    """
    ret = ''
    for d in text.split(' '):
        while len(d) > max_length:
            ret += d[:max_length]+'<wbr />'
            d = d[max_length:]
        ret += d+' '

    if len(ret) > 0:
        return ret[:-1]
    return ret

def validate_url(url):
    """
    check if valid url provided

    :return: True if valid, False otherwise
    """
    # source: http://stackoverflow.com/questions/7160737/python-how-to-validate-a-url-in-python-malformed-or-not
    regex = re.compile(
        r'^(?:http|ftp)s?://' # http:// or https://
        r'(?:(?:[A-Z0-9](?:[A-Z0-9-]{0,61}[A-Z0-9])?\.)+(?:[A-Z]{2,6}\.?|[A-Z0-9-]{2,}\.?)|' #domain...
        r'localhost|' #localhost...
        r'\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3})' # ...or ip
        r'(?::\d+)?' # optional port
        r'(?:/?|[/?]\S+)$', re.IGNORECASE)
    return regex.match(url) is not None

class ULogException(Exception):
    """
    Exception to indicate an ULog parsing error. It is most likely a corrupt log
    file, but could also be a bug in the parser.
    """
    pass

def load_ulog(sha256sum):
    """
    Loads a ULog with the given SHA256sum, or throw a 404. """
    ulogdb_handle = DatabaseULog.get_db_handle(get_ulogdb_filename())
    print(f'Loading log with with {sha256sum=} from ULog database {get_ulogdb_filename()}')
    dbulog_pk = DatabaseULog.primary_key_from_sha256sum(ulogdb_handle, sha256sum)
    if dbulog_pk is None:
        raise tornado.web.HTTPError(
            status_code=404,
            reason=f'Could not find ULog with {sha256sum=}'
        )
    ulog = DatabaseULog(ulogdb_handle, primary_key=dbulog_pk)
    return ulog

def get_airframe_name(ulog, multi_line=False):
    """
    get the airframe name and autostart ID.
    :return: tuple (airframe name & type (str), autostart ID (str)) or None if no
             autostart ID
    """

    if 'SYS_AUTOSTART' in ulog.initial_parameters:
        sys_autostart = ulog.initial_parameters['SYS_AUTOSTART']
        airframe_data = get_airframe_data(sys_autostart)

        if airframe_data is None:
            return ("", str(sys_autostart))

        airframe_type = ''
        if multi_line:
            separator = '<br>'
        else:
            separator = ', '
        if 'type' in airframe_data:
            airframe_type = separator+airframe_data['type']
        return (airframe_data.get('name')+ airframe_type, str(sys_autostart))
    return None


def get_total_flight_time(ulog):
    """
    get the total flight time from an ulog in seconds
    :return: integer or None if not set
    """
    if ('LND_FLIGHT_T_HI' in ulog.initial_parameters and
            'LND_FLIGHT_T_LO' in ulog.initial_parameters):
        high = ulog.initial_parameters['LND_FLIGHT_T_HI']
        if high < 0: # both are signed int32
            high += 2**32
        low = ulog.initial_parameters['LND_FLIGHT_T_LO']
        if low < 0:
            low += 2**32
        flight_time_s = ((high << 32) | low) / 1e6
        return flight_time_s
    return None

def get_tecs_mode_changes(ulog):
    """
    get a list of tecs mode changes
    :return: list of (timestamp, int mode) tuples, the last is the last log
    timestamp and mode = -1.
    """
    try:
        cur_dataset = ulog.get_dataset('tecs_status')
        tecs_mode_changes = cur_dataset.list_value_changes('mode')
        tecs_mode_changes.append((ulog.last_timestamp, -1))
    except (KeyError, IndexError) as error:
        tecs_mode_changes = []
    return tecs_mode_changes

def get_flight_mode_changes(ulog):
    """
    get a list of flight mode changes
    :return: list of (timestamp, int mode) tuples, the last is the last log
    timestamp and mode = -1.
    """
    try:
        cur_dataset = ulog.get_dataset('vehicle_status')
        flight_mode_changes = cur_dataset.list_value_changes('nav_state')
        flight_mode_changes.append((ulog.last_timestamp, -1))
    except (KeyError, IndexError) as error:
        flight_mode_changes = []
    return flight_mode_changes

def validate_error_ids(err_ids):
    """
    validate the err_ids
    """

    for err_id in err_ids:
        if err_id not in error_labels_table:
            return False

    return True


def lpf(values, timestamps, time_const, initial=None):
    """
    Use a discrete low pass filter to filter the values.

    :param values: numpy array of values to filter
    :param timestamps: numpy array of timestamps in us
    :param time_const: time constant in seconds
    :param initial: initial value (optional). If None, the first value is used
    :return: numpy array of filtered values, length is the same as input values
    """

    lpf_values = np.zeros(values.shape)
    if initial is not None:
        lpf_values[0] = initial
    else:
        lpf_values[0] = values[0]

    for i in range(1, len(values)):
        dt = (timestamps[i] - timestamps[i-1])/1e6
        alpha = dt/(dt + time_const)  # Add dt to allow time_const=0
        lpf_values[i] = (1-alpha)*lpf_values[i-1] + alpha*values[i]
    return lpf_values
