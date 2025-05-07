""" handler that is called by Tornado when someone visits /plot_app """

from timeit import default_timer as timer


import tornado
from pyulog import ULog
from pyulog.db import DatabaseULog
from pyulog.px4 import PX4ULog
from bokeh.io import curdoc
from bokeh.layouts import column

from helper import ( # pylint: disable=import-error
    print_timing,
    validate_log_id,
    get_log_filename,
    flight_modes_table,
    vtol_modes_table,
    load_ulog,
)
from config import get_db_filename, get_mapbox_api_access_token # pylint: disable=import-error
from colors import HTML_color_to_RGB # pylint: disable=import-error
from db_entry import DBData # pylint: disable=import-error
from configured_plots import generate_plots # pylint: disable=import-error
from pid_analysis_plots import get_pid_analysis_plots # pylint: disable=import-error
import logging

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger("default")

GET_arguments = curdoc().session_context.request.arguments
if GET_arguments is None or 'log' not in GET_arguments:
    raise tornado.web.HTTPError(
        status_code=404,
        reason='You must supply "log=<log_id>" as GET argument.'
    )

plots_page = 'default'
if 'plots' in GET_arguments:
    plots_args = GET_arguments['plots']
    if len(plots_args) == 1:
        plots_page = str(plots_args[0], 'utf-8')

# show the plots of a single log

start_time = timer()

log_args = GET_arguments['log']
if len(log_args) != 1:
    raise tornado.web.HTTPError(
        status_code=401,
        reason=f'Bad log id: {log_args}'
    )

log_id = str(log_args[0], 'utf-8')
if not validate_log_id(log_id):
    raise tornado.web.HTTPError(
        status_code=401,
        reason=f'Invalid log id: {log_id}'
    )
logger.info(f'Loading log with {log_id=}')
ulog = load_ulog(log_id)

logger.info('Retrieving PX4 specific data.')
px4_ulog = PX4ULog(ulog)
logger.info('Retrieved PX4 specific data.')

print_timing("Data Loading", start_time)
start_time = timer()

# read the data from DB (now ignored)
db_data = DBData()
vehicle_data = None

# check which plots to show
if plots_page == 'pid_analysis':
    link_to_main_plots = '?log='+log_id
    plots = get_pid_analysis_plots(ulog, px4_ulog, db_data,
                                   link_to_main_plots)
elif plots_page == 'default':
    curdoc().template_variables['mapbox_api_access_token'] = get_mapbox_api_access_token()
    curdoc().template_variables['is_plot_page'] = True
    curdoc().template_variables['log_id'] = log_id
    flight_modes = [
        {'name': 'Manual', 'color': HTML_color_to_RGB(flight_modes_table[0][1])},
        {'name': 'Altitude Control', 'color': HTML_color_to_RGB(flight_modes_table[1][1])},
        {'name': 'Position Control', 'color': HTML_color_to_RGB(flight_modes_table[2][1])},
        {'name': 'Acro', 'color': HTML_color_to_RGB(flight_modes_table[10][1])},
        {'name': 'Stabilized', 'color': HTML_color_to_RGB(flight_modes_table[15][1])},
        {'name': 'Offboard', 'color': HTML_color_to_RGB(flight_modes_table[14][1])},
        {'name': 'Rattitude', 'color': HTML_color_to_RGB(flight_modes_table[16][1])},
        {'name': 'Auto (Mission, RTL, Follow, ...)',
         'color': HTML_color_to_RGB(flight_modes_table[3][1])}
        ]
    curdoc().template_variables['flight_modes'] = flight_modes
    vtol_modes = [
        {'name': 'Transition', 'color': HTML_color_to_RGB(vtol_modes_table[1][1])},
        {'name': 'Fixed-Wing', 'color': HTML_color_to_RGB(vtol_modes_table[2][1])},
        {'name': 'Multicopter', 'color': HTML_color_to_RGB(vtol_modes_table[3][1])},
        ]
    curdoc().template_variables['vtol_modes'] = vtol_modes

    link_to_3d_page = '3d?log='+log_id
    link_to_pid_analysis_page = '?plots=pid_analysis&log='+log_id

    plots = generate_plots(ulog, px4_ulog, db_data, vehicle_data,
                           link_to_3d_page, link_to_pid_analysis_page)
else:
    raise tornado.web.HTTPError(
        status_code=404,
        reason=f'Plots page not found: {plots_page}'
    )

layout = column(plots)
curdoc().add_root(layout)
curdoc().title = 'Flight Review - ' + px4_ulog.get_mav_type()

print_timing("Plotting", start_time)
