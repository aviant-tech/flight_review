"""
Tornado handler for the upload page
"""

from __future__ import print_function
import os
from html import escape
import sys
import uuid
import binascii
import tornado.web
import json
from tornado.ioloop import IOLoop

from pyulog import ULog
from pyulog.px4 import PX4ULog
from pyulog.db import DatabaseULog

# this is needed for the following imports
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../plot_app'))
from db_entry import DBVehicleData, DBData
from config import get_ulogdb_filename, get_http_protocol, get_domain_name, \
    email_notifications_config
from helper import get_total_flight_time, validate_url, get_log_filename, \
    get_airframe_name, ULogException
from overview_generator import generate_overview_img_from_id

#pylint: disable=relative-beyond-top-level
from .common import get_jinja_env, CustomHTTPError, generate_db_data_from_log_file, \
    TornadoRequestHandlerBase
from .send_email import send_notification_email, send_flightreport_email
from .multipart_streamer import MultiPartStreamer
import logging
logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger("default")


UPLOAD_TEMPLATE = 'upload.html'


#pylint: disable=attribute-defined-outside-init,too-many-statements, unused-argument


def update_vehicle_db_entry(cur, ulog, log_id, vehicle_name):
    """
    Update the Vehicle DB entry
    :param cur: DB cursor
    :param ulog: ULog object
    :param vehicle_name: new vehicle name or '' if not updated
    :return vehicle_data: DBVehicleData object
    """

    vehicle_data = DBVehicleData()
    if 'sys_uuid' in ulog.msg_info_dict:
        vehicle_data.uuid = escape(ulog.msg_info_dict['sys_uuid'])

        if vehicle_name == '':
            cur.execute('select Name '
                        'from Vehicle where UUID = ?', [vehicle_data.uuid])
            db_tuple = cur.fetchone()
            if db_tuple is not None:
                vehicle_data.name = db_tuple[0]
            logger.info('reading vehicle name from db:'+vehicle_data.name)
        else:
            vehicle_data.name = vehicle_name
            logger.info('vehicle name from uploader:'+vehicle_data.name)

        vehicle_data.log_id = log_id
        flight_time = get_total_flight_time(ulog)
        if flight_time is not None:
            vehicle_data.flight_time = flight_time

        # update or insert the DB entry
        cur.execute('insert or replace into Vehicle (UUID, LatestLogId, Name, FlightTime)'
                    'values (?, ?, ?, ?)',
                    [vehicle_data.uuid, vehicle_data.log_id, vehicle_data.name,
                     vehicle_data.flight_time])
    return vehicle_data


@tornado.web.stream_request_body
class UploadHandler(TornadoRequestHandlerBase):
    """ Upload log file Tornado request handler: handles page requests and POST
    data """

    def initialize(self):
        """ initialize the instance """
        self.multipart_streamer = None

    def prepare(self):
        """ called before a new request """
        if self.request.method.upper() == 'POST':
            if 'expected_size' in self.request.arguments:
                self.request.connection.set_max_body_size(
                    int(self.get_argument('expected_size')))
            else:
                # Set max upload size to 10GB
                self.request.connection.set_max_body_size(10*1024*1024*1024)

            try:
                total = int(self.request.headers.get("Content-Length", "0"))
            except KeyError:
                total = 0
            self.multipart_streamer = MultiPartStreamer(total)

    def data_received(self, chunk):
        """ called whenever new data is received """
        if self.multipart_streamer:
            self.multipart_streamer.data_received(chunk)

    def get(self, *args, **kwargs):
        """ GET request callback """
        template = get_jinja_env().get_template(UPLOAD_TEMPLATE)
        self.write(template.render())

    def post(self, *args, **kwargs):
        """ POST request callback """
        logger.info("UploadHandler POST")
        if self.multipart_streamer:
            try:
                self.multipart_streamer.data_complete()
                form_data = self.multipart_streamer.get_values(
                    ['description', 'email',
                     'allowForAnalysis', 'obfuscated', 'source', 'type',
                     'feedback', 'windSpeed', 'rating', 'videoUrl', 'public',
                     'vehicleName'])
                description = escape(form_data['description'].decode("utf-8"))
                email = form_data['email'].decode("utf-8")
                upload_type = 'personal'
                if 'type' in form_data:
                    upload_type = form_data['type'].decode("utf-8")
                source = 'webui'
                title = '' # may be used in future...
                if 'source' in form_data:
                    source = form_data['source'].decode("utf-8")
                obfuscated = 0
                if 'obfuscated' in form_data:
                    if form_data['obfuscated'].decode("utf-8") == 'true':
                        obfuscated = 1
                allow_for_analysis = 0
                if 'allowForAnalysis' in form_data:
                    if form_data['allowForAnalysis'].decode("utf-8") == 'true':
                        allow_for_analysis = 1
                feedback = ''
                if 'feedback' in form_data:
                    feedback = escape(form_data['feedback'].decode("utf-8"))
                wind_speed = -1
                rating = ''
                stored_email = ''
                video_url = ''
                is_public = 0
                vehicle_name = ''
                error_labels = ''

                if upload_type == 'flightreport':
                    try:
                        wind_speed = int(escape(form_data['windSpeed'].decode("utf-8")))
                    except ValueError:
                        wind_speed = -1
                    rating = escape(form_data['rating'].decode("utf-8"))
                    if rating == 'notset': rating = ''
                    stored_email = email
                    # get video url & check if valid
                    video_url = escape(form_data['videoUrl'].decode("utf-8"), quote=True)
                    if not validate_url(video_url):
                        video_url = ''
                    if 'vehicleName' in form_data:
                        vehicle_name = escape(form_data['vehicleName'].decode("utf-8"))

                    # always allow for statistical analysis
                    allow_for_analysis = 1
                    if 'public' in form_data:
                        if form_data['public'].decode("utf-8") == 'true':
                            is_public = 1

                file_obj = self.multipart_streamer.get_parts_by_name('filearg')[0]
                upload_file_name = file_obj.get_filename()

                while True:
                    log_id = str(uuid.uuid4())
                    new_file_name = get_log_filename(log_id)
                    if not os.path.exists(new_file_name):
                        break

                # read file header & check if really an ULog file
                header_len = len(ULog.HEADER_BYTES)
                if (file_obj.get_payload_partial(header_len) !=
                        ULog.HEADER_BYTES):
                    if upload_file_name[-7:].lower() == '.px4log':
                        raise CustomHTTPError(
                            400,
                            'Invalid File. This seems to be a px4log file. '
                            'Upload it to <a href="http://logs.uaventure.com" '
                            'target="_blank">logs.uaventure.com</a>.')
                    raise CustomHTTPError(400, 'Invalid File')

                logger.info(f'Moving uploaded file to {str(new_file_name)}')
                file_obj.move(new_file_name)

                if obfuscated == 1:
                    # TODO: randomize gps data, ...
                    pass

                # generate a token: secure random string (url-safe)
                token = str(binascii.hexlify(os.urandom(16)), 'ascii')

                filename = get_log_filename(log_id)
                logger.info(f'Loading log with {filename=}')
                ulog = ULog(filename)

                ulogdb_handle = DatabaseULog.get_db_handle(get_ulogdb_filename())
                digest = DatabaseULog.calc_sha256sum(filename)
                dbulog_pk = DatabaseULog.primary_key_from_sha256sum(ulogdb_handle, digest)
                if dbulog_pk is None:
                    logger.info('Saving DatabaseULog to database.')
                    dbulog = DatabaseULog(ulogdb_handle, log_file=filename)
                    dbulog.save()
                    dbulog_pk = dbulog.primary_key
                else:
                    logger.info(f'Found DatabaseULog with hash {digest} in database.')
                    logger.info(f'Loading DatabaseULog with primary key {dbulog_pk} from database.')
                    dbulog = DatabaseULog(ulogdb_handle, primary_key=dbulog_pk)

                url = '/plot_app?log='+digest
                full_plot_url = get_http_protocol()+'://'+get_domain_name()+url
                logger.info(full_plot_url)

                delete_url = get_http_protocol()+'://'+get_domain_name()+ \
                    '/edit_entry?action=delete&log='+log_id+'&token='+token

                # information for the notification email
                info = {}
                info['description'] = description
                info['feedback'] = feedback
                info['upload_filename'] = upload_file_name
                info['type'] = ''
                info['airframe'] = ''
                info['hardware'] = ''
                info['uuid'] = ''
                info['software'] = ''
                info['rating'] = rating
                if len(vehicle_name) > 0:
                    info['vehicle_name'] = vehicle_name

                if ulog is not None:
                    px4_ulog = PX4ULog(ulog)
                    info['type'] = px4_ulog.get_mav_type()
                    airframe_name_tuple = get_airframe_name(ulog)
                    if airframe_name_tuple is not None:
                        airframe_name, airframe_id = airframe_name_tuple
                        if len(airframe_name) == 0:
                            info['airframe'] = airframe_id
                        else:
                            info['airframe'] = airframe_name
                    sys_hardware = ''
                    if 'ver_hw' in ulog.msg_info_dict:
                        sys_hardware = escape(ulog.msg_info_dict['ver_hw'])
                        info['hardware'] = sys_hardware
                    if 'sys_uuid' in ulog.msg_info_dict and sys_hardware != 'SITL':
                        info['uuid'] = escape(ulog.msg_info_dict['sys_uuid'])
                    branch_info = ''
                    if 'ver_sw_branch' in ulog.msg_info_dict:
                        branch_info = ' (branch: '+ulog.msg_info_dict['ver_sw_branch']+')'
                    if 'ver_sw' in ulog.msg_info_dict:
                        ver_sw = escape(ulog.msg_info_dict['ver_sw'])
                        info['software'] = ver_sw + branch_info


                if upload_type == 'flightreport' and is_public and source != 'CI':
                    destinations = set(email_notifications_config['public_flightreport'])
                    if rating in ['unsatisfactory', 'crash_sw_hw', 'crash_pilot']:
                        destinations = destinations | \
                            set(email_notifications_config['public_flightreport_bad'])
                    send_flightreport_email(
                        list(destinations),
                        full_plot_url,
                        DBData.rating_str_static(rating),
                        DBData.wind_speed_str_static(wind_speed), delete_url,
                        stored_email, info)

                    # also generate the additional DB entry
                    # (we may have the log already loaded in 'ulog', however the
                    # lru cache will make it very quick to load it again)
                    log_id = log_id
                    # also generate the preview image
                    IOLoop.instance().add_callback(generate_overview_img_from_id, log_id)



                # send notification emails
                send_notification_email(email, full_plot_url, delete_url, info)

                # do not redirect for QGC
                if source == 'aviant_fms':
                    gps_data = ulog.get_dataset('vehicle_gps_position')
                    start_timestamp = 0
                    end_timestamp = 0

                    for timestamp in gps_data.data['time_utc_usec']:
                        if timestamp > 0:
                            start_timestamp = int(timestamp/1e6)
                            break
                    end_timestamp = int(gps_data.data['time_utc_usec'][-1]/1e6)

                    utc_offset_min = ulog.initial_parameters.get('SDLOG_UTC_OFFSET', 0)
                    start_timestamp += utc_offset_min * 60
                    end_timestamp += utc_offset_min * 60

                    self.write(json.dumps({
                        'log_id': log_id,
                        'vehicle_uuid': info['uuid'],
                        'start_timestamp': start_timestamp,
                        'end_timestamp': end_timestamp,
                    }))
                elif source != 'QGroundControl':
                    self.redirect(url)

            except CustomHTTPError:
                raise

            except ULogException:
                raise CustomHTTPError(
                    400,
                    'Failed to parse the file. It is most likely corrupt.')
            except Exception as e:
                logger.info(f'Error when handling POST data: {sys.exc_info()[0]} - {sys.exc_info()[1]}, {e}')
                raise CustomHTTPError(500)

            finally:
                self.multipart_streamer.release_parts()
