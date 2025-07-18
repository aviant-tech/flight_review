""" This contains the list of all drawn plots on the log plotting page """

from html import escape

from bokeh.layouts import column
from bokeh.models import Range1d
from bokeh.models.widgets import Button
from bokeh.io import curdoc

from config import *
from helper import *
from leaflet import ulog_to_polyline
from plotting import *
from plotted_tables import (
    get_logged_messages, get_changed_parameters,
    get_info_table_html, get_heading_html, get_error_labels_html,
    get_hardfault_html, get_corrupt_log_html
    )

#pylint: disable=cell-var-from-loop, undefined-loop-variable,
#pylint: disable=consider-using-enumerate,too-many-statements



def generate_plots(ulog, px4_ulog, db_data, vehicle_data, link_to_3d_page,
                   link_to_pid_analysis_page):
    """ create a list of bokeh plots (and widgets) to show """

    plots = []
    data = ulog.data_list

    # COMPATIBILITY support for old logs
    if any(elem.name in ('vehicle_air_data', 'vehicle_magnetometer')
           for elem in ulog.data_list):
        baro_alt_meter_topic = 'vehicle_air_data'
        magnetometer_ga_topic = 'vehicle_magnetometer'
    else: # old
        baro_alt_meter_topic = 'sensor_combined'
        magnetometer_ga_topic = 'sensor_combined'

    true_airspeed_sp_fieldname = 'true_airspeed_sp'
    voltage5v_fieldname = 'voltage5v_v'
    voltage3v_fieldname = 'sensors3v3[0]'
    for topic in ulog.data_list:
        if topic.name == 'system_power':
            # COMPATIBILITY: rename fields to new format
            # old (prior to PX4/Firmware:213aa93)
            if any(field.field_name == 'voltage5V_v' for field in topic.field_data):
                voltage5v_fieldname = 'voltage5V_v'

            # old (prior to PX4/Firmware:213aa93)
            if any(field.field_name == 'voltage3V3_v' for field in topic.field_data):
                voltage3v_fieldname = 'voltage3V3_v'
            elif any(field.field_name == 'voltage3v3_v' for field in topic.field_data):
                voltage3v_fieldname = 'voltage3v3_v'

        if topic.name == 'tecs_status' and any(field.field_name == 'airspeed_sp'
                                               for field in topic.field_data):
            true_airspeed_sp_fieldname = 'airspeed_sp'

    if any(elem.name == 'vehicle_angular_velocity' for elem in ulog.data_list):
        rate_estimated_topic_name = 'vehicle_angular_velocity'
        rate_groundtruth_topic_name = 'vehicle_angular_velocity_groundtruth'
        rate_field_names = ['xyz[0]', 'xyz[1]', 'xyz[2]']
    else: # old
        rate_estimated_topic_name = 'vehicle_attitude'
        rate_groundtruth_topic_name = 'vehicle_attitude_groundtruth'
        rate_field_names = ['rollspeed', 'pitchspeed', 'yawspeed']
    if any(elem.name == 'manual_control_switches' for elem in ulog.data_list):
        manual_control_switches_topic = 'manual_control_switches'
    else: # old
        manual_control_switches_topic = 'manual_control_setpoint'

    # initialize flight mode changes
    flight_mode_changes = get_flight_mode_changes(ulog)

    # initialize tecs mode changes
    tecs_mode_changes = get_tecs_mode_changes(ulog)

    # VTOL state changes & vehicle type
    vtol_states = None
    is_vtol = False
    try:
        cur_dataset = ulog.get_dataset('vehicle_status')
        if np.amax(cur_dataset.data['is_vtol']) == 1:
            is_vtol = True
            # find mode after transitions (states: 1=transition, 2=FW, 3=MC)
            if 'vehicle_type' in cur_dataset.data:
                vehicle_type_field = 'vehicle_type'
                vtol_state_mapping = {2: 2, 1: 3}
                vehicle_type = cur_dataset.data['vehicle_type']
                in_transition_mode = cur_dataset.data['in_transition_mode']
                is_armed = cur_dataset.data['arming_state'] == 2
                vtol_states = []
                for i in range(len(vehicle_type)):
                    # a VTOL can change state also w/o in_transition_mode set
                    # (e.g. in Manual mode)
                    if (len(vtol_states) == 0 or in_transition_mode[i-1] != in_transition_mode[i] or \
                        vehicle_type[i-1] != vehicle_type[i]) and is_armed[i]:
                        vtol_states.append((cur_dataset.data['timestamp'][i],
                                            in_transition_mode[i]))

            else: # COMPATIBILITY: old logs (https://github.com/PX4/Firmware/pull/11918)
                vtol_states = cur_dataset.list_value_changes('in_transition_mode')
                vehicle_type_field = 'is_rotary_wing'
                vtol_state_mapping = {0: 2, 1: 3}
            for i in range(len(vtol_states)):
                if vtol_states[i][1] == 0:
                    t = vtol_states[i][0]
                    idx = np.argmax(cur_dataset.data['timestamp'] >= t) + 1
                    vtol_states[i] = (t, vtol_state_mapping[
                        cur_dataset.data[vehicle_type_field][idx]])
            vtol_states.append((ulog.last_timestamp, -1))
    except (KeyError, IndexError) as error:
        vtol_states = None


    # Horizontal acceptance
    # Generates a list of tuples corresponding to changes in the
    # horizontal acceptance status of the upcoming waypoint. The first
    # item in the tuple is the change timestamp, and the second is a
    # boolean giving the acceptance status
    try:
        cur_dataset = ulog.get_dataset('position_controller_status')
        acceptance_radius = cur_dataset.data['acceptance_radius']
        dist_to_wp = cur_dataset.data['wp_dist']
        hor_acc = dist_to_wp <= acceptance_radius
        hor_acc_change = np.where(hor_acc[1:] != hor_acc[:-1])[0]
        hor_acc_idxs = np.append([0], hor_acc_change+1)
        horizontal_acceptance = [(cur_dataset.data['timestamp'][i], hor_acc[i]) for i in hor_acc_idxs]
        horizontal_acceptance.append((ulog.last_timestamp, False))
    except (KeyError, IndexError) as error:
        horizontal_acceptance = None

    # Vertical acceptance
    # Same as for horizontal acceptance, but using altitudes instead of
    # horizontal distance. Since we have to compare the continuous
    # altitude with the discrete next-waypoint altitude, the two lists are
    # interjoined in a "last-value" interpolation.
    try:
        alt_acc_rad = ulog.initial_parameters['NAV_FW_ALT_RAD']
        wp_dataset = ulog.get_dataset('navigator_mission_item')
        wp_alts = wp_dataset.data['altitude']
        wp_timestamps = wp_dataset.data['timestamp']

        gpos_dataset = ulog.get_dataset('vehicle_global_position')
        gpos_alts = gpos_dataset.data['alt']
        gpos_timestamps = gpos_dataset.data['timestamp']
        gpos_wp_alts = []

        # "Last-value" interpolation due to differing time axes
        cur_wp_idx = 0
        for gpos_idx in range(len(gpos_timestamps)):
            gpos_wp_alts.append(wp_alts[cur_wp_idx])
            if (gpos_timestamps[gpos_idx] >= wp_timestamps[cur_wp_idx]
                and cur_wp_idx < len(wp_timestamps)-1):
                cur_wp_idx += 1

        alt_acc = np.abs(gpos_alts-gpos_wp_alts) <= alt_acc_rad
        alt_acc_change = np.where(alt_acc[1:] != alt_acc[:-1])[0]
        alt_acc_idxs = np.append([0], alt_acc_change+1)
        vertical_acceptance = [(gpos_timestamps[i], alt_acc[i]) for i in alt_acc_idxs]
        vertical_acceptance.append((ulog.last_timestamp, False))
    except (KeyError, IndexError) as error:
        vertical_acceptance = None


    # Heading
    curdoc().template_variables['title_html'] = get_heading_html(
        ulog, px4_ulog, db_data, link_to_3d_page,
        additional_links=[("Open PID Analysis", link_to_pid_analysis_page)])

    # info text on top (logging duration, max speed, ...)
    curdoc().template_variables['info_table_html'] = \
        get_info_table_html(ulog, px4_ulog, db_data, vehicle_data, vtol_states)

    curdoc().template_variables['error_labels_html'] = get_error_labels_html()

    hardfault_html = get_hardfault_html(ulog)
    if hardfault_html is not None:
        curdoc().template_variables['hardfault_html'] = hardfault_html

    corrupt_log_html = get_corrupt_log_html(ulog)
    if corrupt_log_html:
        curdoc().template_variables['corrupt_log_html'] = corrupt_log_html

    # Position plot
    data_plot = DataPlot2D(ulog, plot_config, 'vehicle_local_position',
                           x_axis_label='[m]', y_axis_label='[m]', plot_height='large')
    data_plot.add_graph('y', 'x', colors2[0], 'Estimated',
                        check_if_all_zero=True)
    if not data_plot.had_error: # vehicle_local_position is required
        data_plot.change_dataset('vehicle_local_position_setpoint')
        data_plot.add_graph('y', 'x', colors2[1], 'Setpoint')
        # groundtruth (SITL only)
        data_plot.change_dataset('vehicle_local_position_groundtruth')
        data_plot.add_graph('y', 'x', color_gray, 'Groundtruth')
        # GPS + position setpoints
        plot_map(ulog, plot_config, map_type='plain', setpoints=True,
                 bokeh_plot=data_plot.bokeh_plot)
        if data_plot.finalize() is not None:
            plots.append(data_plot.bokeh_plot)

            # Leaflet Map
            try:
                pos_datas, flight_modes = ulog_to_polyline(ulog, flight_mode_changes)
                curdoc().template_variables['pos_datas'] = pos_datas
                curdoc().template_variables['pos_flight_modes'] = flight_modes
            except:
                pass
            curdoc().template_variables['has_position_data'] = True

    # initialize parameter changes
    changed_params = None
    if not 'replay' in ulog.msg_info_dict: # replay can have many param changes
        if len(ulog.changed_parameters) > 0:
            changed_params = ulog.changed_parameters
            plots.append(None) # save space for the param change button

    ### Add all data plots ###

    x_range_offset = (ulog.last_timestamp - ulog.start_timestamp) * 0.05
    x_range = Range1d(ulog.start_timestamp - x_range_offset, ulog.last_timestamp + x_range_offset)

    # Altitude estimate
    data_plot = DataPlot(ulog, plot_config, 'vehicle_gps_position',
                         y_axis_label='[m]', title='Altitude Estimate',
                         changed_params=changed_params, x_range=x_range)
    data_plot.add_graph([lambda data: ('alt', data['alt']*0.001)],
                        colors8[0:1], ['GPS Altitude'])
    data_plot.change_dataset(baro_alt_meter_topic)
    data_plot.add_graph(['baro_alt_meter'], colors8[1:2], ['Barometer Altitude'])
    data_plot.change_dataset('vehicle_global_position')
    data_plot.add_graph(['alt'], colors8[2:3], ['Fused Altitude Estimation'])
    data_plot.change_dataset('position_setpoint_triplet')
    data_plot.add_circle(['current.alt'], [plot_config['mission_setpoint_color']],
                         ['Altitude Setpoint'])
    data_plot.change_dataset('actuator_controls_0')
    data_plot.add_graph([lambda data: ('thrust', data['control[3]']*100)],
                        colors8[6:7], ['Thrust [0, 100]'])
    plot_flight_modes_background(data_plot, flight_mode_changes, vtol_states)

    if data_plot.finalize() is not None: plots.append(data_plot)

    # Waypoint tracking
    def below_or_none(limit, data):
        ''' Used to hide large outliers from plots '''
        data[data >= limit] = None
        return data

    data_plot = DataPlot(ulog, plot_config, 'position_controller_status',
                         title='Waypoint tracking', y_axis_label='[m]',
                         changed_params=changed_params, x_range=x_range)
    data_plot.add_graph([lambda data: ('wp_dist', below_or_none(9e5, data['wp_dist']))],
            colors8[0:1], ['Distance to next WP'])
    data_plot.add_graph(['acceptance_radius'], colors8[1:2], ['Acceptance radius'])
    data_plot.change_dataset('navigator_mission_item')
    data_plot.add_circle(['altitude'], [plot_config['mission_setpoint_color']], ['Waypoint altitude'])
    data_plot.change_dataset('tecs_status')
    data_plot.add_graph(['altitude_sp'], colors8[3:4], ['Altitude setpoint'])
    data_plot.change_dataset('vehicle_global_position')
    data_plot.add_graph(['alt'], colors8[4:5], ['Fused altitude'])
    plot_wp_acceptance_background(data_plot, horizontal_acceptance, vertical_acceptance)

    if data_plot.finalize() is not None: plots.append(data_plot)

    # QuadChute

    # PX4 uses a rolling average over the height rate and height rate setpoint
    # to figure out if it should QuadChute
    def quadchute_rolling_average(series):
        AVERAGING_FACTOR = 1/50 # Hard coded value, should apparently give 1 second window
        average_series = series.copy()
        for i in range(1, len(series)):
            average_series[i] = AVERAGING_FACTOR*series[i] + (1-AVERAGING_FACTOR)*average_series[i-1]
        return average_series

    data_plot = DataPlot(ulog, plot_config, 'tecs_status',
                         title='QuadChute',
                         changed_params=changed_params, x_range=x_range)
    data_plot.add_graph([lambda data: ('altitude_error', data['altitude_filtered'] - data['altitude_sp'])],
                        colors8[0:1], ['Altitude error [m]'])
    data_plot.add_graph(['height_rate'], colors8[1:2], ['Height rate [m/s]'])
    data_plot.add_graph([lambda data: ('height_rate_ra', quadchute_rolling_average(data['height_rate']))],
                        colors8[2:3], ['Height rate, rolling average [m/s]'])
    data_plot.add_graph(['height_rate_setpoint'], colors8[3:4], ['Height rate setpoint [m/s]'])
    data_plot.add_graph([lambda data: ('height_rate_setpoint_ra', quadchute_rolling_average(data['height_rate_setpoint']))],
                        colors8[4:5], ['Height rate setpoint, rolling average [m/s]'])
    data_plot.change_dataset('actuator_controls_1')
    data_plot.add_graph([lambda data: ('thrust', data['control[3]']*10)],
                        colors8[5:6], ['Thrust [0, 10]'])
    plot_flight_modes_background(data_plot, flight_mode_changes, vtol_states)

    if data_plot.finalize() is not None: plots.append(data_plot)

    # Icing
    data_plot = DataPlot(ulog, plot_config, 'actuator_controls_1',
                         y_start=0, title='Icing',
                         plot_height='small', changed_params=changed_params,
                         x_range=x_range)
    data_plot.add_graph([lambda data: ('thrust', data['control[3]']*10)],
                        colors8[0:1], ['Thrust control [0,10]'], mark_nan=True)

    data_plot.change_dataset('vehicle_attitude')
    data_plot.add_graph([lambda data: ('pitch_estimated', np.rad2deg(data['pitch']))],
                        colors8[1:2], ['Pitch estimated [deg]'])
    data_plot.change_dataset('tecs_status')
    data_plot.add_graph(['airspeed_filtered'], colors8[2:3], ['Airspeed filtered'])
    if data_plot.finalize() is not None: plots.append(data_plot)

    # Data link
    data_plot = DataPlot(ulog, plot_config, 'telemetry_status',
                         title='Data link',  changed_params=changed_params,
                         x_range=x_range)
    for link_id in range(3):
        data_plot.change_dataset('telemetry_status', topic_instance=link_id)
        if data_plot.dataset:
            data_plot.add_graph(['heartbeats[%d].timestamp' % i for i in range(4)],
                    colors8[link_id:link_id+1]*4,
                    ['Link %d heartbeats %d' % (link_id, i) for i in range(4)])

    plot_flight_modes_background(data_plot, flight_mode_changes, vtol_states)

    if data_plot.finalize() is not None: plots.append(data_plot)


    # Roll/Pitch/Yaw angle & angular rate
    angle_functions = [
        lambda data, s='': np.arctan2(2.0 * (data[f'q{s}[0]'] * data[f'q{s}[1]'] + data[f'q{s}[2]'] * data[f'q{s}[3]']), 1.0 - 2.0 * (data[f'q{s}[1]'] * data[f'q{s}[1]'] + data[f'q{s}[2]'] * data[f'q{s}[2]'])),
        lambda data, s='': np.arcsin(2.0 * (data[f'q{s}[0]'] * data[f'q{s}[2]'] - data[f'q{s}[3]'] * data[f'q{s}[1]'])),
        lambda data, s='': np.arctan2(2.0 * (data[f'q{s}[0]'] * data[f'q{s}[3]'] + data[f'q{s}[1]'] * data[f'q{s}[2]']), 1.0 - 2.0 * (data[f'q{s}[2]'] * data[f'q{s}[2]'] + data[f'q{s}[3]'] * data[f'q{s}[3]'])),

    ]
    for index, axis in enumerate(['roll', 'pitch', 'yaw']):

        # angle
        axis_name = axis.capitalize()
        data_plot = DataPlot(ulog, plot_config, 'vehicle_attitude',
                             y_axis_label='[deg]', title=axis_name+' Angle',
                             plot_height='small', changed_params=changed_params,
                             x_range=x_range)
        data_plot.add_graph([lambda data: (axis, np.rad2deg(angle_functions[index](data)))],
                            colors3[0:1], [axis_name+' Estimated'], mark_nan=True)
        data_plot.change_dataset('vehicle_attitude_setpoint')
        data_plot.add_graph([lambda data: (axis+'_d', np.rad2deg(angle_functions[index](data, s='_d')))],
                            colors3[1:2], [axis_name+' Setpoint'],
                            use_step_lines=True)
        if axis == 'yaw':
            data_plot.add_graph(
                [lambda data: ('yaw_sp_move_rate', np.rad2deg(data['yaw_sp_move_rate']))],
                colors3[2:3], [axis_name+' FF Setpoint [deg/s]'],
                use_step_lines=True)
        data_plot.change_dataset('vehicle_attitude_groundtruth')
        data_plot.add_graph([lambda data: (axis, np.rad2deg(data[axis]))],
                            [color_gray], [axis_name+' Groundtruth'])
        plot_flight_modes_background(data_plot, flight_mode_changes, vtol_states)

        if data_plot.finalize() is not None: plots.append(data_plot)

        # rate
        data_plot = DataPlot(ulog, plot_config, rate_estimated_topic_name,
                             y_axis_label='[deg/s]', title=axis_name+' Angular Rate',
                             plot_height='small', changed_params=changed_params,
                             x_range=x_range)
        data_plot.add_graph([lambda data: (axis+'speed',
                                           np.rad2deg(data[rate_field_names[index]]))],
                            colors3[0:1], [axis_name+' Rate Estimated'], mark_nan=True)
        data_plot.change_dataset('vehicle_rates_setpoint')
        data_plot.add_graph([lambda data: (axis, np.rad2deg(data[axis]))],
                            colors3[1:2], [axis_name+' Rate Setpoint'],
                            mark_nan=True, use_step_lines=True)
        axis_letter = axis[0].upper()
        rate_int_limit = '(*100)'
        # this param is MC/VTOL only (it will not exist on FW)
        rate_int_limit_param = 'MC_' + axis_letter + 'R_INT_LIM'
        if rate_int_limit_param in ulog.initial_parameters:
            rate_int_limit = '[-{0:.0f}, {0:.0f}]'.format(
                ulog.initial_parameters[rate_int_limit_param]*100)
        data_plot.change_dataset('rate_ctrl_status')
        data_plot.add_graph([lambda data: (axis, data[axis+'speed_integ']*100)],
                            colors3[2:3], [axis_name+' Rate Integral '+rate_int_limit])
        data_plot.change_dataset(rate_groundtruth_topic_name)
        data_plot.add_graph([lambda data: (axis+'speed',
                                           np.rad2deg(data[rate_field_names[index]]))],
                            [color_gray], [axis_name+' Rate Groundtruth'])
        plot_flight_modes_background(data_plot, flight_mode_changes, vtol_states)

        if data_plot.finalize() is not None: plots.append(data_plot)



    # Local position
    for axis in ['x', 'y', 'z']:
        data_plot = DataPlot(ulog, plot_config, 'vehicle_local_position',
                             y_axis_label='[m]', title='Local Position '+axis.upper(),
                             plot_height='small', changed_params=changed_params,
                             x_range=x_range)
        data_plot.add_graph([axis], colors2[0:1], [axis.upper()+' Estimated'], mark_nan=True)
        data_plot.change_dataset('vehicle_local_position_setpoint')
        data_plot.add_graph([axis], colors2[1:2], [axis.upper()+' Setpoint'],
                            use_step_lines=True)
        plot_flight_modes_background(data_plot, flight_mode_changes, vtol_states)

        if data_plot.finalize() is not None: plots.append(data_plot)



    # Velocity
    data_plot = DataPlot(ulog, plot_config, 'vehicle_local_position',
                         y_axis_label='[m/s]', title='Velocity',
                         plot_height='small', changed_params=changed_params,
                         x_range=x_range)
    data_plot.add_graph(['vx', 'vy', 'vz'], colors8[0:3], ['X', 'Y', 'Z'])
    data_plot.change_dataset('vehicle_local_position_setpoint')
    data_plot.add_graph(['vx', 'vy', 'vz'], [colors8[5], colors8[4], colors8[6]],
                        ['X Setpoint', 'Y Setpoint', 'Z Setpoint'], use_step_lines=True)
    plot_flight_modes_background(data_plot, flight_mode_changes, vtol_states)

    if data_plot.finalize() is not None: plots.append(data_plot)


    # Visual Odometry (only if topic found)
    if any(elem.name == 'vehicle_visual_odometry' for elem in ulog.data_list):
        # Vision position
        data_plot = DataPlot(ulog, plot_config, 'vehicle_visual_odometry',
                             y_axis_label='[m]', title='Visual Odometry Position',
                             plot_height='small', changed_params=changed_params,
                             x_range=x_range)
        data_plot.add_graph(['x', 'y', 'z'], colors3, ['X', 'Y', 'Z'], mark_nan=True)
        plot_flight_modes_background(data_plot, flight_mode_changes, vtol_states)

        data_plot.change_dataset('vehicle_local_position_groundtruth')
        data_plot.add_graph(['x', 'y', 'z'], colors8[2:5],
                            ['Groundtruth X', 'Groundtruth Y', 'Groundtruth Z'])

        if data_plot.finalize() is not None: plots.append(data_plot)


        # Vision velocity
        data_plot = DataPlot(ulog, plot_config, 'vehicle_visual_odometry',
                             y_axis_label='[m]', title='Visual Odometry Velocity',
                             plot_height='small', changed_params=changed_params,
                             x_range=x_range)
        data_plot.add_graph(['vx', 'vy', 'vz'], colors3, ['X', 'Y', 'Z'], mark_nan=True)
        plot_flight_modes_background(data_plot, flight_mode_changes, vtol_states)

        data_plot.change_dataset('vehicle_local_position_groundtruth')
        data_plot.add_graph(['vx', 'vy', 'vz'], colors8[2:5],
                            ['Groundtruth VX', 'Groundtruth VY', 'Groundtruth VZ'])
        if data_plot.finalize() is not None: plots.append(data_plot)


        # Vision attitude
        data_plot = DataPlot(ulog, plot_config, 'vehicle_visual_odometry',
                             y_axis_label='[deg]', title='Visual Odometry Attitude',
                             plot_height='small', changed_params=changed_params,
                             x_range=x_range)
        data_plot.add_graph([lambda data: ('roll', np.rad2deg(data['roll'])),
                             lambda data: ('pitch', np.rad2deg(data['pitch'])),
                             lambda data: ('yaw', np.rad2deg(data['yaw']))],
                            colors3, ['Roll', 'Pitch', 'Yaw'], mark_nan=True)
        plot_flight_modes_background(data_plot, flight_mode_changes, vtol_states)

        data_plot.change_dataset('vehicle_attitude_groundtruth')
        data_plot.add_graph([lambda data: ('roll', np.rad2deg(data['roll'])),
                             lambda data: ('pitch', np.rad2deg(data['pitch'])),
                             lambda data: ('yaw', np.rad2deg(data['yaw']))],
                            colors8[2:5],
                            ['Roll Groundtruth', 'Pitch Groundtruth', 'Yaw Groundtruth'])

        # Vision attitude rate
        data_plot = DataPlot(ulog, plot_config, 'vehicle_visual_odometry',
                             y_axis_label='[deg]', title='Visual Odometry Attitude Rate',
                             plot_height='small', changed_params=changed_params,
                             x_range=x_range)
        data_plot.add_graph([lambda data: ('rollspeed', np.rad2deg(data['rollspeed'])),
                             lambda data: ('pitchspeed', np.rad2deg(data['pitchspeed'])),
                             lambda data: ('yawspeed', np.rad2deg(data['yawspeed']))],
                            colors3, ['Roll Rate', 'Pitch Rate', 'Yaw Rate'], mark_nan=True)
        plot_flight_modes_background(data_plot, flight_mode_changes, vtol_states)

        data_plot.change_dataset(rate_groundtruth_topic_name)
        data_plot.add_graph([lambda data: ('rollspeed', np.rad2deg(data[rate_field_names[0]])),
                             lambda data: ('pitchspeed', np.rad2deg(data[rate_field_names[1]])),
                             lambda data: ('yawspeed', np.rad2deg(data[rate_field_names[2]]))],
                            colors8[2:5],
                            ['Roll Rate Groundtruth', 'Pitch Rate Groundtruth',
                             'Yaw Rate Groundtruth'])

        if data_plot.finalize() is not None: plots.append(data_plot)

    # Airspeed vs Ground speed: but only if there's valid airspeed data or a VTOL
    try:
        if is_vtol or ulog.get_dataset('airspeed') is not None:
            data_plot = DataPlot(ulog, plot_config, 'vehicle_global_position',
                                 y_axis_label='[m/s]', title='Airspeed',
                                 plot_height='small',
                                 changed_params=changed_params, x_range=x_range)
            data_plot.add_graph([lambda data: ('groundspeed_estimated',
                                               np.sqrt(data['vel_n']**2 + data['vel_e']**2))],
                                colors8[0:1], ['Ground Speed Estimated'])
            if any(elem.name == 'airspeed_validated' for elem in ulog.data_list):
                airspeed_validated = ulog.get_dataset('airspeed_validated')
                data_plot.change_dataset('airspeed_validated')
                if np.amax(airspeed_validated.data['airspeed_sensor_measurement_valid']) == 1:
                    data_plot.add_graph(['true_airspeed_m_s'], colors8[1:2],
                                        ['True Airspeed'])
                    data_plot.add_graph([lambda data: ('true_airspeed_m_s',
                                                       lpf(data['true_airspeed_m_s'], data['timestamp'], 3))],
                                        colors8[1:2], ['True Airspeed (3s average)'])
                else:
                    data_plot.add_graph(['calibrated_ground_minus_wind_m_s'], colors8[1:2],
                                        ['Ground Minus Wind'])
            else:
                data_plot.change_dataset('airspeed')
                data_plot.add_graph(['indicated_airspeed_m_s'], colors8[1:2],
                                    ['Indicated Airspeed'])
            data_plot.change_dataset('vehicle_gps_position')
            data_plot.add_graph(['vel_m_s'], colors8[2:3], ['Ground Speed (from GPS)'])
            data_plot.change_dataset('tecs_status')
            data_plot.add_graph([true_airspeed_sp_fieldname], colors8[3:4], ['True Airspeed Setpoint'])

            # Difference between airspeed and groundspeed
            data_plot.change_dataset('airspeed_validated')
            tas_timestamps = data_plot.dataset.data['timestamp']
            data_plot.change_dataset('vehicle_gps_position')
            gs_timestamps = data_plot.dataset.data['timestamp']
            gs_values = np.interp(tas_timestamps, gs_timestamps, data_plot.dataset.data['vel_m_s'])
            data_plot.change_dataset('airspeed_validated')
            data_plot.add_graph([lambda data: ('relative_wind',
                                np.abs(data['true_airspeed_m_s'] - gs_values))],
                                colors8[4:5], ['Relative wind'])

            # Various wind estimators
            data_plot.change_dataset('airspeed_wind')
            for estimator_id in range(2):
                data_plot.change_dataset('airspeed_wind', topic_instance=estimator_id)
                if data_plot.dataset:
                    data_plot.add_graph([lambda data: (f'airspeed_wind_{estimator_id}',
                                        np.sqrt(data['windspeed_north']**2 + data['windspeed_east']**2))],
                                        colors8[5:6], [f'Airspeed-based wind ({estimator_id})'])
            for estimator_id in range(6):
                data_plot.change_dataset('estimator_wind', topic_instance=estimator_id)
                if data_plot.dataset:
                    data_plot.add_graph([lambda data: (f'estimator_wind_{estimator_id}',
                                        np.sqrt(data['windspeed_north']**2 + data['windspeed_east']**2))],
                                        colors8[6:7], [f'Estimated wind ({estimator_id})'])
            data_plot.change_dataset('wind')
            if data_plot.dataset:
                data_plot.add_graph([lambda data: ('wind',
                                    np.sqrt(data['windspeed_north']**2 + data['windspeed_east']**2))],
                                    colors8[7:8], ['Estimated wind (selected)'])
            plot_flight_modes_background(data_plot, flight_mode_changes, vtol_states)

            if data_plot.finalize() is not None: plots.append(data_plot)
    except (KeyError, IndexError) as error:
        pass

    # Airspeed vs ground speed scatter
    try:
        data_plot = DataPlot(ulog, plot_config, 'airspeed_validated',
                             title=f'Airspeed vs groundspeed', plot_height='small',
                             x_axis_label='True airspeed [m/s]', y_axis_label='Apparent wind [m/s]')
        data_plot.set_use_time_formatter(False)
        tas_timestamps = data_plot.dataset.data['timestamp']
        tas_values = data_plot.dataset.data['true_airspeed_m_s']

        data_plot.change_dataset('vehicle_gps_position')
        gs_timestamps = data_plot.dataset.data['timestamp']
        gs_values = np.interp(tas_timestamps, gs_timestamps, data_plot.dataset.data['vel_m_s'])

        tas_values_fw = np.array([])
        gs_values_fw = np.array([])
        vtol_state_changes = zip(vtol_states[:-1], vtol_states[1:])
        for (change_timestamp, mode), (next_change_timestamp, _) in vtol_state_changes:
            # Mode 2 is fixed-wing
            if mode != 2:
                continue
            fw_indexes = (tas_timestamps > change_timestamp) * (tas_timestamps < next_change_timestamp)
            tas_values_fw = np.append(tas_values_fw, tas_values[fw_indexes])
            gs_values_fw = np.append(gs_values_fw, gs_values[fw_indexes])

        if gs_values_fw.shape[0] > 0 and tas_values_fw.shape[0] > 0:
            mean_gs = np.average(gs_values_fw)
            mean_tas = np.average(tas_values_fw)
            airspeed_correction_factor = mean_gs/mean_tas
            wind_values_fw = gs_values_fw - tas_values_fw
            mean_wind = np.average(wind_values_fw)

            aspd_primary = ulog.initial_parameters.get('ASPD_PRIMARY', -1)
            aspd_scale_param = 'ASPD_SCALE'
            if aspd_primary > -1:
                tmp_param = 'ASPD_SCALE_' + str(aspd_primary)
                if tmp_param in ulog.initial_parameters:
                    aspd_scale_param = tmp_param
            adsp_scale = ulog.initial_parameters.get(aspd_scale_param, 1)  # Use dafault if not found

            p = data_plot.bokeh_plot
            p.circle(tas_values_fw,
                     wind_values_fw,
                     color=colors8[0],
                     size=1,
                     legend_label='Measurements (suggested ASPD_SCALE: {sign:s}{pct:.1f}% = {scale:.2f})'.format(
                        sign="+" if airspeed_correction_factor >= 1 else "-",
                        pct=np.abs(airspeed_correction_factor*100-100),
                        scale=adsp_scale*airspeed_correction_factor,
                    ))
            p.line([min(tas_values_fw), max(tas_values_fw)],
                   [mean_wind, mean_wind],
                   legend_label=f'Mean apparent wind = {mean_wind:.1f} m/s')

        if data_plot.finalize() is not None: plots.append(data_plot)
    except (KeyError, IndexError) as error:
        pass

    # TECS (fixed-wing or VTOLs)

    data_plot = DataPlot(ulog, plot_config, 'tecs_status', y_start=0, title='TECS',
                         y_axis_label='', plot_height='small',
                         changed_params=changed_params, x_range=x_range)
    try:
        EAS2TAS = ulog.get_dataset("tecs_status").data["true_airspeed_sp"] / ulog.get_dataset("tecs_status").data["equivalent_airspeed_sp"]
        # From TECS::_detect_underspeed()
        TAS_min = ulog.initial_parameters["FW_AIRSPD_MIN"] * EAS2TAS
        tas_state_criticalness = TAS_min * 0.9 - ulog.get_dataset("tecs_status").data["true_airspeed_filtered"]
        throttle_state_criticalness = ulog.get_dataset("tecs_status").data["throttle_sp"] - ulog.initial_parameters["FW_THR_MAX"] * 0.95
        # From TECS:_detect_uncommanded_descent()
        ste_error_criticalness = ulog.get_dataset("tecs_status").data["total_energy_error"] - 200
        data_plot.add_graph([
                lambda data: ('Total energy rate', data['total_energy_rate']),
                lambda data: ('Total energy criticalness', ste_error_criticalness),
                lambda data: ('Airspeed criticalness', tas_state_criticalness),
                lambda data: ('Throttle criticalness scaled', throttle_state_criticalness * 10)
            ],
            colors8[:4],
            [
                'Total energy rate',
                'Total energy criticalness',
                'Airspeed criticalness',
                'Throttle criticalness scaled'
                ],
            mark_nan=True
            )

        plot_tecs_modes_background(data_plot, tecs_mode_changes, vtol_states)
    except KeyError:
        pass
    if data_plot.finalize() is not None: plots.append(data_plot)


    # manual control inputs
    # prefer the manual_control_setpoint topic. Old logs do not contain it
    if any(elem.name == 'manual_control_setpoint' for elem in ulog.data_list):
        data_plot = DataPlot(ulog, plot_config, 'manual_control_setpoint',
                             title='Manual Control Inputs (Radio or Joystick)',
                             plot_height='small', y_range=Range1d(-1.1, 1.1),
                             changed_params=changed_params, x_range=x_range)
        data_plot.add_graph(['y', 'x', 'r', 'z', 'aux1', 'aux2'], colors8[0:6],
                            ['Y / Roll', 'X / Pitch', 'Yaw', 'Throttle [0, 1]',
                             'Aux1', 'Aux2'])
        data_plot.change_dataset(manual_control_switches_topic)
        data_plot.add_graph([lambda data: ('mode_slot', data['mode_slot']/6),
                             lambda data: ('kill_switch', data['kill_switch'] == 1)],
                            colors8[6:8], ['Flight Mode', 'Kill Switch'])
        # TODO: add RTL switch and others? Look at params which functions are mapped?
        plot_flight_modes_background(data_plot, flight_mode_changes, vtol_states)

        if data_plot.finalize() is not None: plots.append(data_plot)

    else: # it's an old log (COMPATIBILITY)
        data_plot = DataPlot(ulog, plot_config, 'rc_channels',
                             title='Raw Radio Control Inputs',
                             plot_height='small', y_range=Range1d(-1.1, 1.1),
                             changed_params=changed_params, x_range=x_range)
        num_rc_channels = 8
        if data_plot.dataset:
            max_channels = np.amax(data_plot.dataset.data['channel_count'])
            if max_channels < num_rc_channels: num_rc_channels = max_channels
        legends = []
        for i in range(num_rc_channels):
            channel_names = px4_ulog.get_configured_rc_input_names(i)
            if channel_names is None:
                legends.append('Channel '+str(i))
            else:
                legends.append('Channel '+str(i)+' ('+', '.join(channel_names)+')')
        data_plot.add_graph(['channels['+str(i)+']' for i in range(num_rc_channels)],
                            colors8[0:num_rc_channels], legends, mark_nan=True)
        plot_flight_modes_background(data_plot, flight_mode_changes, vtol_states)

        if data_plot.finalize() is not None: plots.append(data_plot)

    has_dynamic_mixer = ulog.initial_parameters.get('SYS_CTRL_ALLOC', 0) == 1

    # Plot actuator_motors/servos and vehicle_torque/thrust_setpoints with CA, 
    # but plot actuator_controls without CA
    if has_dynamic_mixer:
        # Dynamic mixer MR setpoints
        data_plot = DataPlot(ulog, plot_config, 'vehicle_thrust_setpoint',
                             y_start=0, title=f'Motor dynamics setpoints', plot_height='small',
                             changed_params=changed_params, topic_instance=0,
                             x_range=x_range)
        if data_plot.dataset:
            data_plot.add_graph(['xyz[0]', 'xyz[1]', 'xyz[2]'],
                                colors8[:3],
                                ['X thrust', 'Y thrust', 'Z thrust'])
            data_plot.change_dataset('vehicle_torque_setpoint', topic_instance=0)
            data_plot.add_graph(['xyz[0]', 'xyz[1]', 'xyz[2]'],
                                colors8[3:6],
                                ['X torque', 'Y torque', 'Z torque'])
            plot_flight_modes_background(data_plot, flight_mode_changes, vtol_states)

        if data_plot.finalize() is not None: plots.append(data_plot)

        # Dynamic mixer unallocated thrust/torque
        data_plot = DataPlot(ulog, plot_config, 'control_allocator_status',
                             y_start=0, title=f'Unallocated motor dynamics', plot_height='small',
                             changed_params=changed_params, topic_instance=0,
                             x_range=x_range)
        if data_plot.dataset:
            data_plot.add_graph(['unallocated_thrust[0]', 'unallocated_thrust[1]', 'unallocated_thrust[2]'],
                                colors8[:3],
                                ['X thrust', 'Y thrust', 'Z thrust'])
            data_plot.add_graph(['unallocated_torque[0]', 'unallocated_torque[1]', 'unallocated_torque[2]'],
                                colors8[3:6],
                                ['X torque', 'Y torque', 'Z torque'])
            plot_flight_modes_background(data_plot, flight_mode_changes, vtol_states)

        if data_plot.finalize() is not None: plots.append(data_plot)

        # Dynamic mixer FW setpoints
        data_plot = DataPlot(ulog, plot_config, 'vehicle_thrust_setpoint',
                             y_start=0, title=f'Control surface dynamics setpoints', plot_height='small',
                             changed_params=changed_params, topic_instance=1,
                             x_range=x_range)
        if data_plot.dataset:
            data_plot.add_graph(['xyz[0]', 'xyz[1]', 'xyz[2]'],
                                colors8[:3],
                                ['X thrust', 'Y thrust', 'Z thrust'])
            data_plot.change_dataset('vehicle_torque_setpoint', topic_instance=1)
            data_plot.add_graph(['xyz[0]', 'xyz[1]', 'xyz[2]'],
                                colors8[3:6],
                                ['X torque', 'Y torque', 'Z torque'])
            plot_flight_modes_background(data_plot, flight_mode_changes, vtol_states)

        if data_plot.finalize() is not None: plots.append(data_plot)

        # Dynamic mixer motor controls
        data_plot = DataPlot(ulog, plot_config, 'actuator_motors',
                             y_start=0, title=f'Actuator Motors', plot_height='small',
                             changed_params=changed_params,
                             x_range=x_range)
        max_num_motors = 9  # Topic can have 12, but we only use up to 9, as of 250519
        for i in range(max_num_motors):
            field_name = f'control[{i}]'
            if field_name in data_plot.dataset.data:
                data_plot.add_graph([field_name],
                                    [colors8[i % 8]],
                                    [f'Motor {i+1}'])
        plot_flight_modes_background(data_plot, flight_mode_changes, vtol_states)
        if data_plot.finalize() is not None: plots.append(data_plot)

        # Dynamic mixer servo control
        data_plot = DataPlot(ulog, plot_config, 'actuator_servos',
                             y_start=0, title=f'Actuator Servos', plot_height='small',
                             changed_params=changed_params,
                             x_range=x_range)
        max_num_servos = 3  # Topic can have 8, but we only use 3, as of 250519
        for i in range(max_num_servos):
            field_name = f'control[{i}]'
            if field_name in data_plot.dataset.data:
                data_plot.add_graph([field_name],
                                    [colors8[i % 8]],
                                    [f'Servo {i+1}'])
        plot_flight_modes_background(data_plot, flight_mode_changes, vtol_states)
        if data_plot.finalize() is not None: plots.append(data_plot)

    else:  # static mixer
        # actuator controls 0
        data_plot = DataPlot(ulog, plot_config, 'actuator_controls_0',
                             y_start=0, title='Actuator Controls 0', plot_height='small',
                             changed_params=changed_params, x_range=x_range)
        data_plot.add_graph(['control[0]', 'control[1]', 'control[2]', 'control[3]'],
                            colors8[0:4], ['Roll', 'Pitch', 'Yaw', 'Thrust'], mark_nan=True)
        plot_flight_modes_background(data_plot, flight_mode_changes, vtol_states)
        if data_plot.finalize() is not None: plots.append(data_plot)

        # actuator controls (Main) FFT (for filter & output noise analysis)
        data_plot = DataPlotFFT(ulog, plot_config, 'actuator_controls_0',
                                title='Actuator Controls FFT', y_range = Range1d(0, 0.01))
        data_plot.add_graph(['control[0]', 'control[1]', 'control[2]'],
                            colors3, ['Roll', 'Pitch', 'Yaw'])
        if not data_plot.had_error:
            if 'MC_DTERM_CUTOFF' in ulog.initial_parameters: # COMPATIBILITY
                data_plot.mark_frequency(
                    ulog.initial_parameters['MC_DTERM_CUTOFF'],
                    'MC_DTERM_CUTOFF')
            if 'IMU_DGYRO_CUTOFF' in ulog.initial_parameters:
                data_plot.mark_frequency(
                    ulog.initial_parameters['IMU_DGYRO_CUTOFF'],
                    'IMU_DGYRO_CUTOFF')
            if 'IMU_GYRO_CUTOFF' in ulog.initial_parameters:
                data_plot.mark_frequency(
                    ulog.initial_parameters['IMU_GYRO_CUTOFF'],
                    'IMU_GYRO_CUTOFF', 20)

        if data_plot.finalize() is not None: plots.append(data_plot)


        # actuator controls 1
        # (only present on VTOL, Fixed-wing config)
        data_plot = DataPlot(ulog, plot_config, 'actuator_controls_1',
                             y_start=0, title='Actuator Controls 1 (VTOL in Fixed-Wing mode)',
                             plot_height='small', changed_params=changed_params,
                             x_range=x_range)
        data_plot.add_graph(['control[0]', 'control[1]', 'control[2]', 'control[3]'],
                            colors8[0:4], ['Roll', 'Pitch', 'Yaw', 'Thrust'], mark_nan=True)
        plot_flight_modes_background(data_plot, flight_mode_changes, vtol_states)
        if data_plot.finalize() is not None: plots.append(data_plot)

    # actuator outputs
    actuator_output_protocols = [
        (0, "UAVCAN 1", lambda val: np.where(val>8192, 0, val/8192), "UAVCAN_EC"),
        (1, "UAVCAN 2", lambda val: np.where(val>8192, 0, val/8192), "UAVCAN_EC"),
        (2, "MAIN PWM", lambda val: (val-1000)/1000, "PWM_MAIN"),
        (3, "AUX PWM", lambda val: (val-1000)/1000, "PWM_AUX"),
        (4, "AUX DSHOT", lambda val: (val-48)/1999, "PWM_AUX"),
    ]

    data_plot_effv = DataPlot(ulog, plot_config, 'battery_status',
                             y_start=0, title=f'Actuator Outputs Effective Voltage', plot_height='small',
                             changed_params=changed_params, topic_instance=0,
                             x_range=x_range)
    battery_voltages = data_plot_effv.dataset.data['voltage_v']
    battery_timestamps = data_plot_effv.dataset.data['timestamp']

    for ao_idx, port_name, throttle_func, param_key in actuator_output_protocols:
        data_plot_output = DataPlot(ulog, plot_config, 'actuator_outputs',
                             y_start=0, title=f'Actuator Outputs {ao_idx} ({port_name})', plot_height='small',
                             changed_params=changed_params, topic_instance=ao_idx,
                             x_range=x_range)
        num_actuator_outputs = 16
        if data_plot_output.dataset:
            # Output plot, one per output type
            max_outputs = np.amax(data_plot_output.dataset.data['noutputs'])
            if max_outputs < num_actuator_outputs:
                num_actuator_outputs = max_outputs

            data_plot_output.add_graph(['output['+str(i)+']' for i in range(num_actuator_outputs)],
                                [colors8[i % 8] for i in range(num_actuator_outputs)],
                                ['Output '+str(i) for i in range(num_actuator_outputs)], mark_nan=True)
            plot_flight_modes_background(data_plot_output, flight_mode_changes, vtol_states)

            # Effective voltage plot, one common for all motor outputs
            # TODO: Can use actuator_motors instead with manual THR_MDL_FAC
            # compensation, so that we benefit from the high-rate
            # actuator_motors topic
            data_plot_effv.change_dataset('actuator_outputs', topic_instance=ao_idx)
            output_timestamps = data_plot_output.dataset.data['timestamp']
            effv_func = lambda val: np.interp(output_timestamps, battery_timestamps, battery_voltages) * throttle_func(val)
            for i in range(num_actuator_outputs):
                MOTOR_FUNC_MIN = 100
                MAX_MOTOR_OUTPUTS = 12
                func_param_name = f'{param_key}_FUNC{i+1}'
                output_function = ulog.initial_parameters.get(func_param_name, None)
                if output_function is None or output_function <= MOTOR_FUNC_MIN or output_function >= MOTOR_FUNC_MIN + MAX_MOTOR_OUTPUTS:
                    # Not a motor output
                    print(f'{func_param_name}={output_function} is not a motor output')
                    continue
                # AUX ports report both DSHOT and PWM output when DSHOT is enabled, so we must
                # manually check the configured protocol and only show that one
                if port_name.startswith("AUX"):
                    TIM_IDX = 0 if i in (0, 1, 2, 3) else 1  # AUX1-4 or AUX5-6
                    configured_proto = ulog.initial_parameters.get(f'{param_key}_TIM{TIM_IDX}', None)
                    if configured_proto is None:
                        print(f'{param_key}_TIM{TIM_IDX} is not set')
                        continue
                    configured_proto_name = "PWM" if configured_proto > 0 else "DSHOT"
                    if not port_name.endswith(configured_proto_name):
                        print(f'{param_key}_TIM{TIM_IDX} is set to {configured_proto}, but {port_name} is not a {configured_proto_name} output')
                        continue

                motor_idx = output_function - MOTOR_FUNC_MIN
                data_plot_effv.add_graph(
                    [lambda data: (f'output[{i}]', effv_func(data[f'output[{i}]']))],
                    [colors8[i % 8]],
                    [f'Motor {motor_idx} ({port_name})'])

            if data_plot_output.finalize() is not None: plots.append(data_plot_output)
    if data_plot_effv.finalize() is not None: plots.append(data_plot_effv)

    # raw acceleration
    data_plot = DataPlot(ulog, plot_config, 'sensor_combined',
                         y_axis_label='[m/s^2]', title='Raw Acceleration',
                         plot_height='small', changed_params=changed_params,
                         x_range=x_range)
    data_plot.add_graph(['accelerometer_m_s2[0]', 'accelerometer_m_s2[1]',
                         'accelerometer_m_s2[2]'], colors3, ['X', 'Y', 'Z'])
    if data_plot.finalize() is not None: plots.append(data_plot)

    # Vibration Metrics
    data_plot = DataPlot(ulog, plot_config, 'vehicle_imu_status',
                         title='Vibration Metrics',
                         plot_height='small', changed_params=changed_params,
                         x_range=x_range, y_start=0, topic_instance=0)
    data_plot.add_graph(['accel_vibration_metric'], colors8[0:1],
                        ['Accel 0 Vibration Level [m/s^2]'])

    data_plot.change_dataset('vehicle_imu_status', 1)
    data_plot.add_graph(['accel_vibration_metric'], colors8[1:2],
                        ['Accel 1 Vibration Level [m/s^2]'])

    data_plot.change_dataset('vehicle_imu_status', 2)
    data_plot.add_graph(['accel_vibration_metric'], colors8[2:3],
                        ['Accel 2 Vibration Level [m/s^2]'])

    data_plot.change_dataset('vehicle_imu_status', 3)
    data_plot.add_graph(['accel_vibration_metric'], colors8[3:4],
                        ['Accel 3 Vibration Level [rad/s]'])

    data_plot.add_horizontal_background_boxes(
        ['green', 'orange', 'red'], [4.905, 9.81])

    if data_plot.finalize() is not None: plots.append(data_plot)

    # Acceleration Spectrogram
    data_plot = DataPlotSpec(ulog, plot_config, 'sensor_combined',
                             y_axis_label='[Hz]', title='Acceleration Power Spectral Density',
                             plot_height='small', x_range=x_range)
    data_plot.add_graph(['accelerometer_m_s2[0]', 'accelerometer_m_s2[1]', 'accelerometer_m_s2[2]'],
                        ['X', 'Y', 'Z'])
    if data_plot.finalize() is not None: plots.append(data_plot)

    # raw angular speed
    data_plot = DataPlot(ulog, plot_config, 'sensor_combined',
                         y_axis_label='[deg/s]', title='Raw Angular Speed (Gyroscope)',
                         plot_height='small', changed_params=changed_params,
                         x_range=x_range)
    data_plot.add_graph([
        lambda data: ('gyro_rad[0]', np.rad2deg(data['gyro_rad[0]'])),
        lambda data: ('gyro_rad[1]', np.rad2deg(data['gyro_rad[1]'])),
        lambda data: ('gyro_rad[2]', np.rad2deg(data['gyro_rad[2]']))],
                        colors3, ['X', 'Y', 'Z'])
    if data_plot.finalize() is not None: plots.append(data_plot)

    # FIFO accel
    if add_virtual_fifo_topic_data(ulog, 'sensor_accel_fifo'):
        # Raw data
        data_plot = DataPlot(ulog, plot_config, 'sensor_accel_fifo_virtual',
                             y_axis_label='[m/s^2]', title='Raw Acceleration (FIFO)',
                             plot_height='small', changed_params=changed_params,
                             x_range=x_range)
        data_plot.add_graph(['x', 'y', 'z'], colors3, ['X', 'Y', 'Z'])
        if data_plot.finalize() is not None: plots.append(data_plot)

        # power spectral density
        data_plot = DataPlotSpec(ulog, plot_config, 'sensor_accel_fifo_virtual',
                                 y_axis_label='[Hz]',
                                 title='Acceleration Power Spectral Density (FIFO)',
                                 plot_height='normal', x_range=x_range)
        data_plot.add_graph(['x', 'y', 'z'], ['X', 'Y', 'Z'])
        if data_plot.finalize() is not None: plots.append(data_plot)

        # sampling regularity
        data_plot = DataPlot(ulog, plot_config, 'sensor_accel_fifo', y_range=Range1d(0, 25e3),
                             y_axis_label='[us]',
                             title='Sampling Regularity of Sensor Data (FIFO)', plot_height='small',
                             changed_params=changed_params, x_range=x_range)
        sensor_accel_fifo = ulog.get_dataset('sensor_accel_fifo').data
        sampling_diff = np.diff(sensor_accel_fifo['timestamp'])
        min_sampling_diff = np.amin(sampling_diff)
        plot_dropouts(data_plot.bokeh_plot, ulog.dropouts, min_sampling_diff)
        data_plot.add_graph([lambda data: ('timediff', np.append(sampling_diff, 0))],
                            [colors3[2]], ['delta t (between 2 logged samples)'])
        if data_plot.finalize() is not None: plots.append(data_plot)

    # FIFO gyro
    if add_virtual_fifo_topic_data(ulog, 'sensor_gyro_fifo'):
        # Raw data
        data_plot = DataPlot(ulog, plot_config, 'sensor_gyro_fifo_virtual',
                             y_axis_label='[m/s^2]', title='Raw Gyro (FIFO)',
                             plot_height='small', changed_params=changed_params,
                             x_range=x_range)
        data_plot.add_graph(['x', 'y', 'z'], colors3, ['X', 'Y', 'Z'])
        data_plot.add_graph([
            lambda data: ('x', np.rad2deg(data['x'])),
            lambda data: ('y', np.rad2deg(data['y'])),
            lambda data: ('z', np.rad2deg(data['z']))],
                            colors3, ['X', 'Y', 'Z'])
        if data_plot.finalize() is not None: plots.append(data_plot)

        # power spectral density
        data_plot = DataPlotSpec(ulog, plot_config, 'sensor_gyro_fifo_virtual',
                                 y_axis_label='[Hz]', title='Gyro Power Spectral Density (FIFO)',
                                 plot_height='normal', x_range=x_range)
        data_plot.add_graph(['x', 'y', 'z'], ['X', 'Y', 'Z'])
        if data_plot.finalize() is not None: plots.append(data_plot)


    # magnetic field strength
    data_plot = DataPlot(ulog, plot_config, magnetometer_ga_topic,
                         y_axis_label='[gauss]', title='Raw Magnetic Field Strength',
                         plot_height='small', changed_params=changed_params,
                         x_range=x_range)
    for mag_instance in range(2):
        data_plot.change_dataset(magnetometer_ga_topic, topic_instance=mag_instance)
        data_plot.add_graph(['magnetometer_ga[0]', 'magnetometer_ga[1]',
            'magnetometer_ga[2]'], colors8[3*mag_instance:3*mag_instance+3],
                            [f'X (mag_{mag_instance})', f'Y (mag_{mag_instance})', f'Z (mag_{mag_instance})'])
    if data_plot.finalize() is not None: plots.append(data_plot)


    # distance sensor
    data_plot = DataPlot(ulog, plot_config, 'distance_sensor',
                         y_start=0, y_axis_label='[m]', title='Distance Sensor',
                         plot_height='small', changed_params=changed_params,
                         x_range=x_range)
    data_plot.add_graph(['current_distance', 'variance'], colors3[0:2],
                        ['Distance', 'Variance'])
    if data_plot.finalize() is not None: plots.append(data_plot)



    # gps uncertainty
    # the accuracy values can be really large if there is no fix, so we limit the
    # y axis range to some sane values
    data_plot = DataPlot(ulog, plot_config, 'vehicle_gps_position',
                         title='GPS Uncertainty', y_range=Range1d(0, 40),
                         plot_height='small', changed_params=changed_params,
                         x_range=x_range)
    data_plot.add_graph(['eph', 'epv', 'satellites_used', 'fix_type'], colors8[::2],
                        ['Horizontal position accuracy [m]', 'Vertical position accuracy [m]',
                         'Num Satellites used', 'GPS Fix'])
    if data_plot.finalize() is not None: plots.append(data_plot)


    # gps noise & jamming
    data_plot = DataPlot(ulog, plot_config, 'vehicle_gps_position',
                         y_start=0, title='GPS Noise & Jamming',
                         plot_height='small', changed_params=changed_params,
                         x_range=x_range)
    data_plot.add_graph(['noise_per_ms', 'jamming_indicator'], colors3[0:2],
                        ['Noise per ms', 'Jamming Indicator'])
    if data_plot.finalize() is not None: plots.append(data_plot)


    data_plot = DataPlot(ulog, plot_config, magnetometer_ga_topic,
                         y_start=0, title='Thrust and Magnetic Field', plot_height='small',
                         changed_params=changed_params, x_range=x_range, topic_instance=1)

    data_plot.change_dataset('actuator_controls_0')
    data_plot.add_graph([lambda data: ('thrust', data['control[3]'])],
                        colors8[0:1], ['Thrust'])
    if is_vtol:
        data_plot.change_dataset('actuator_controls_1')
        data_plot.add_graph([lambda data: ('thrust', data['control[3]'])],
                            colors8[1:2], ['Thrust (Fixed-wing)'])

    for mag_instance in range(2):
        data_plot.change_dataset(magnetometer_ga_topic, topic_instance=mag_instance)
        data_plot.add_graph(
            [lambda data: ('len_mag', np.sqrt(data['magnetometer_ga[0]']**2 +
                                            data['magnetometer_ga[1]']**2 +
                                            data['magnetometer_ga[2]']**2))],
            colors8[2+mag_instance:2+mag_instance+1], [f'Norm of Magnetic Field (mag_{mag_instance})'])

    if data_plot.finalize() is not None: plots.append(data_plot)

    # thrust and magentic field scatter plot
    min_thrust = 0.5
    data_plot = DataPlot(ulog, plot_config, 'actuator_controls_0',
                         title=f'Thrust and magnetic norm scatter plot (polyfit for thrust > {min_thrust})', plot_height='small',
                         x_axis_label='Thrust', y_axis_label='Magnetic field norm')
    data_plot.set_use_time_formatter(False)
    thrust_timestamps = data_plot.dataset.data['timestamp']
    thrust_values = data_plot.dataset.data['control[3]']

    max_mag_norm = 0
    for topic_instance in range(2):
        data_plot.change_dataset(magnetometer_ga_topic, topic_instance=topic_instance)
        if data_plot.dataset is None or not 'magnetometer_ga[0]' in data_plot.dataset.data:
            continue
        mag_norm_values = np.sqrt(
            data_plot.dataset.data['magnetometer_ga[0]']**2
            + data_plot.dataset.data['magnetometer_ga[1]']**2
            + data_plot.dataset.data['magnetometer_ga[2]']**2
        )
        mag_timestamps = data_plot.dataset.data['timestamp']

        # Bohek doesn't handle np.nan values, so we just remove them. Same as
        # line 791 in plot_app/plotting.py on commit
        # eba80b8d095ac85bbe2801b2583c78e984aa2ce4.
        not_nan_idxs = np.logical_not(np.logical_or(
            np.isnan(mag_timestamps),
            np.isnan(mag_norm_values)
        ))
        mag_timestamps = mag_timestamps[not_nan_idxs]
        mag_norm_values = mag_norm_values[not_nan_idxs]
        # Some logs might have a single magnetometer measurement with a np.nan
        # value. Not sure why, but in those cases we should just proceed to the
        # next magnetometer.
        if not np.any(not_nan_idxs):
            continue

        thrust_values_interp = np.interp(mag_timestamps, thrust_timestamps, thrust_values)

        p = data_plot.bokeh_plot
        p.circle(thrust_values_interp, mag_norm_values,
                color=colors8[topic_instance],
                legend_label=f'Magnetometer {topic_instance}')
        max_mag_norm = np.max([max_mag_norm, np.max(mag_norm_values)])

        filter_idxs = thrust_values_interp >= min_thrust
        if np.any(filter_idxs):
            mag_norm_filtered = mag_norm_values[filter_idxs]
            thrust_filtered = thrust_values_interp[filter_idxs]

            c1, c0 = np.polyfit(thrust_filtered, mag_norm_filtered, 1)
            fit_func = np.poly1d((c1, c0))

            polyfit_xs = np.linspace(0, 1, 100)
            polyfit_ys = fit_func(polyfit_xs)
            max_mag_norm = np.max([max_mag_norm, np.max(polyfit_ys)])
            p.line(polyfit_xs, fit_func(polyfit_xs),
                    color=colors8[topic_instance],
                    legend_label=f'Fit {topic_instance}: y={c0:.3f} (1 + {c1/c0:.3f}x)')


    p.x_range = Range1d(start=0, end=1)
    p.y_range = Range1d(start=0, end=max_mag_norm*1.1)

    if data_plot.finalize() is not None: plots.append(data_plot)



    # pusher power (instance 0)
    data_plot = DataPlot(ulog, plot_config, 'battery_status',
                         y_start=0, title='Pusher power',
                         plot_height='small', changed_params=changed_params,
                         x_range=x_range, topic_instance=0)
    if data_plot.dataset:
        data_plot.add_graph(['voltage_v', 'voltage_filtered_v',
                             'current_a', lambda data: ('discharged_mah', data['discharged_mah']/100),
                             lambda data: ('remaining', data['remaining']*10)],
                            colors8[::2]+colors8[1:2],
                            ['Voltage [V]', 'Voltage filtered [V]',
                             'Current [A]', 'Discharged [mAh / 100]',
                             'Remaining [0=empty, 10=full]'])
        if data_plot.finalize() is not None: plots.append(data_plot)

    # top power (instance 1)
    data_plot = DataPlot(ulog, plot_config, 'battery_status',
                         y_start=0, title='Top power',
                         plot_height='small', changed_params=changed_params,
                         x_range=x_range, topic_instance=1)
    if data_plot.dataset:
        data_plot.add_graph(['voltage_v', 'voltage_filtered_v',
                             'current_a', lambda data: ('discharged_mah', data['discharged_mah']/100),
                             lambda data: ('remaining', data['remaining']*10)],
                            colors8[::2]+colors8[1:2],
                            ['Voltage [V]', 'Voltage filtered [V]',
                             'Current [A]', 'Discharged [mAh / 100]',
                             'Remaining [0=empty, 10=full]'])
        if data_plot.finalize() is not None: plots.append(data_plot)

    # system power
    data_plot = DataPlot(ulog, plot_config, 'system_power',
                         y_start=0, title='System power',
                         plot_height='small', changed_params=changed_params,
                         x_range=x_range, topic_instance=0)
    if data_plot.dataset:
        if voltage5v_fieldname in data_plot.dataset.data and \
                        np.amax(data_plot.dataset.data[voltage5v_fieldname]) > 0.0001:
            data_plot.add_graph([voltage5v_fieldname], colors8[7:8], ['5 V'])
        if voltage3v_fieldname in data_plot.dataset.data and \
                        np.amax(data_plot.dataset.data[voltage3v_fieldname]) > 0.0001:
            data_plot.add_graph([voltage3v_fieldname], colors8[5:6], ['3.3 V'])
    if data_plot.finalize() is not None: plots.append(data_plot)


    #Temperature
    data_plot = DataPlot(ulog, plot_config, 'sensor_baro',
                         y_start=0, y_axis_label='[C]', title='Temperature',
                         plot_height='small', changed_params=changed_params,
                         x_range=x_range)
    data_plot.add_graph(['temperature'], colors8[0:1],
                        ['Baro temperature'])
    data_plot.change_dataset('sensor_accel')
    data_plot.add_graph(['temperature'], colors8[2:3],
                        ['Accel temperature'])
    data_plot.change_dataset('airspeed')
    data_plot.add_graph(['air_temperature_celsius'], colors8[4:5],
                        ['Airspeed temperature'])
    if data_plot.finalize() is not None: plots.append(data_plot)


    # estimator watchdog
    try:
        data_plot = DataPlot(ulog, plot_config, 'estimator_status',
                             y_start=0, title='Estimator Watchdog',
                             plot_height='small', changed_params=changed_params,
                             x_range=x_range)
        estimator_status = ulog.get_dataset('estimator_status').data
        plot_data = []
        plot_labels = []
        input_data = [
            ('Health Flags (vel, pos, hgt)', estimator_status['health_flags']),
            ('Timeout Flags (vel, pos, hgt)', estimator_status['timeout_flags']),
            ('Velocity Check Bit', (estimator_status['innovation_check_flags'])&0x1),
            ('Horizontal Position Check Bit', (estimator_status['innovation_check_flags']>>1)&1),
            ('Vertical Position Check Bit', (estimator_status['innovation_check_flags']>>2)&1),
            ('Mag X, Y, Z Check Bits', (estimator_status['innovation_check_flags']>>3)&0x7),
            ('Yaw Check Bit', (estimator_status['innovation_check_flags']>>6)&1),
            ('Airspeed Check Bit', (estimator_status['innovation_check_flags']>>7)&1),
            ('Synthetic Sideslip Check Bit', (estimator_status['innovation_check_flags']>>8)&1),
            ('Height to Ground Check Bit', (estimator_status['innovation_check_flags']>>9)&1),
            ('Optical Flow X, Y Check Bits', (estimator_status['innovation_check_flags']>>10)&0x3),
            ]
        # filter: show only the flags that have non-zero samples
        for cur_label, cur_data in input_data:
            if np.amax(cur_data) > 0.1:
                data_label = 'flags_'+str(len(plot_data)) # just some unique string
                plot_data.append(lambda d, data=cur_data, label=data_label: (label, data))
                plot_labels.append(cur_label)
                if len(plot_data) >= 8: # cannot add more than that
                    break

        if len(plot_data) == 0:
            # add the plot even in the absence of any problem, so that the user
            # can validate that (otherwise it's ambiguous: it could be that the
            # estimator_status topic is not logged)
            plot_data = [lambda d: ('flags', input_data[0][1])]
            plot_labels = [input_data[0][0]]
        data_plot.add_graph(plot_data, colors8[0:len(plot_data)], plot_labels)
        if data_plot.finalize() is not None: plots.append(data_plot)
    except (KeyError, IndexError) as error:
        print('Error in estimator plot: '+str(error))



    # RC Quality
    data_plot = DataPlot(ulog, plot_config, 'input_rc',
                         title='RC Quality', plot_height='small', y_range=Range1d(0, 1),
                         changed_params=changed_params, x_range=x_range)
    data_plot.add_graph([lambda data: ('rssi', data['rssi']/100), 'rc_lost'],
                        colors3[0:2], ['RSSI [0, 1]', 'RC Lost (Indicator)'])
    data_plot.change_dataset('vehicle_status')
    data_plot.add_graph(['rc_signal_lost'], colors3[2:3], ['RC Lost (Detected)'])
    if data_plot.finalize() is not None: plots.append(data_plot)



    # cpu load
    data_plot = DataPlot(ulog, plot_config, 'cpuload',
                         title='CPU & RAM', plot_height='small', y_range=Range1d(0, 1),
                         changed_params=changed_params, x_range=x_range)
    data_plot.add_graph(['ram_usage', 'load'], [colors3[1], colors3[2]],
                        ['RAM Usage', 'CPU Load'])
    data_plot.add_span('load', line_color=colors3[2])
    data_plot.add_span('ram_usage', line_color=colors3[1])
    plot_flight_modes_background(data_plot, flight_mode_changes, vtol_states)
    if data_plot.finalize() is not None: plots.append(data_plot)


    # sampling: time difference
    try:
        data_plot = DataPlot(ulog, plot_config, 'sensor_combined', y_range=Range1d(0, 25e3),
                             y_axis_label='[us]',
                             title='Sampling Regularity of Sensor Data', plot_height='small',
                             changed_params=changed_params, x_range=x_range)
        sensor_combined = ulog.get_dataset('sensor_combined').data
        sampling_diff = np.diff(sensor_combined['timestamp'])
        min_sampling_diff = np.amin(sampling_diff)

        plot_dropouts(data_plot.bokeh_plot, ulog.dropouts, min_sampling_diff)

        data_plot.add_graph([lambda data: ('timediff', np.append(sampling_diff, 0))],
                            [colors3[2]], ['delta t (between 2 logged samples)'])
        data_plot.change_dataset('estimator_status')
        data_plot.add_graph([lambda data: ('time_slip', data['time_slip']*1e6)],
                            [colors3[1]], ['Estimator time slip (cumulative)'])
        if data_plot.finalize() is not None: plots.append(data_plot)
    except:
        pass



    # exchange all DataPlot's with the bokeh_plot and handle parameter changes

    param_changes_button = Button(label="Hide Parameter Changes", width=170)
    param_change_labels = []
    # FIXME: this should be a CustomJS callback, not on the server. However this
    # did not work for me.
    def param_changes_button_clicked():
        """ callback to show/hide parameter changes """
        for label in param_change_labels:
            if label.visible:
                param_changes_button.label = 'Show Parameter Changes'
                label.visible = False
                label.text_alpha = 0 # label.visible does not work, so we use this instead
            else:
                param_changes_button.label = 'Hide Parameter Changes'
                label.visible = True
                label.text_alpha = 1
    param_changes_button.on_click(param_changes_button_clicked)


    jinja_plot_data = []
    for i in range(len(plots)):
        if plots[i] is None:
            plots[i] = column(param_changes_button, width=int(plot_width * 0.99))
        if isinstance(plots[i], DataPlot):
            if plots[i].param_change_label is not None:
                param_change_labels.append(plots[i].param_change_label)

            plot_title = plots[i].title
            plots[i] = plots[i].bokeh_plot

            fragment = 'Nav-'+plot_title.replace(' ', '-') \
                .replace('&', '_').replace('(', '').replace(')', '')
            jinja_plot_data.append({
                'model_id': plots[i].ref['id'],
                'fragment': fragment,
                'title': plot_title
                })


    # changed parameters
    plots.append(get_changed_parameters(ulog.initial_parameters, plot_width))



    # information about which messages are contained in the log
# TODO: need to load all topics for this (-> log loading will take longer)
#       but if we load all topics and the log contains some (external) topics
#       with buggy timestamps, it will affect the plotting.
#    data_list_sorted = sorted(ulog.data_list, key=lambda d: d.name + str(d.multi_id))
#    table_text = []
#    for d in data_list_sorted:
#        message_size = sum([ULog.get_field_size(f.type_str) for f in d.field_data])
#        num_data_points = len(d.data['timestamp'])
#        table_text.append((d.name, str(d.multi_id), str(message_size), str(num_data_points),
#           str(message_size * num_data_points)))
#    topics_info = '<table><tr><th>Name</th><th>Topic instance</th><th>Message Size</th>' \
#            '<th>Number of data points</th><th>Total bytes</th></tr>' + ''.join(
#            ['<tr><td>'+'</td><td>'.join(list(x))+'</td></tr>' for x in table_text]) + '</table>'
#    topics_div = Div(text=topics_info, width=int(plot_width*0.9))
#    plots.append(column(topics_div, width=int(plot_width*0.9)))


    # log messages
    plots.append(get_logged_messages(ulog.logged_messages, plot_width))


    # console messages, perf & top output
    top_data = ''
    perf_data = ''
    console_messages = ''
    if 'boot_console_output' in ulog.msg_info_multiple_dict:
        console_output = ulog.msg_info_multiple_dict['boot_console_output'][0]
        console_output = escape(''.join(console_output))
        console_messages = '<p><pre>'+console_output+'</pre></p>'

    for state in ['pre', 'post']:
        if 'perf_top_'+state+'flight' in ulog.msg_info_multiple_dict:
            current_top_data = ulog.msg_info_multiple_dict['perf_top_'+state+'flight'][0]
            flight_data = escape('\n'.join(current_top_data))
            top_data += '<p>'+state.capitalize()+' Flight:<br/><pre>'+flight_data+'</pre></p>'
        if 'perf_counter_'+state+'flight' in ulog.msg_info_multiple_dict:
            current_perf_data = ulog.msg_info_multiple_dict['perf_counter_'+state+'flight'][0]
            flight_data = escape('\n'.join(current_perf_data))
            perf_data += '<p>'+state.capitalize()+' Flight:<br/><pre>'+flight_data+'</pre></p>'
    if 'perf_top_watchdog' in ulog.msg_info_multiple_dict:
        current_top_data = ulog.msg_info_multiple_dict['perf_top_watchdog'][0]
        flight_data = escape('\n'.join(current_top_data))
        top_data += '<p>Watchdog:<br/><pre>'+flight_data+'</pre></p>'

    additional_data_html = ''
    if len(console_messages) > 0:
        additional_data_html += '<h5>Console Output</h5>'+console_messages
    if len(top_data) > 0:
        additional_data_html += '<h5>Processes</h5>'+top_data
    if len(perf_data) > 0:
        additional_data_html += '<h5>Performance Counters</h5>'+perf_data
    if len(additional_data_html) > 0:
        # hide by default & use a button to expand
        additional_data_html = '''
<button id="show-additional-data-btn" class="btn btn-secondary" data-toggle="collapse" style="min-width:0;"
 data-target="#show-additional-data">Show additional Data</button>
<div id="show-additional-data" class="collapse">
{:}
</div>
'''.format(additional_data_html)
        curdoc().template_variables['additional_info'] = additional_data_html


    curdoc().template_variables['plots'] = jinja_plot_data

    return plots
