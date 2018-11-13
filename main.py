#!/usr/bin/python3 -u
######################################################################
#
#   This file is part of SimControl.
#
#   SimControl is free software: you can redistribute it and/or modify it under
#   the terms of the GNU General Public License as published by the Free
#   Software Foundation, either version 3 of the License, or (at your option)
#   any later version.
#
#   SimControl is distributed in the hope that it will be useful, but WITHOUT
#   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
#   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
#   more details.
#
#   You should have received a copy of the GNU General Public License along
#   with SimControl.  If not, see <https://www.gnu.org/licenses/>.
#
######################################################################

'''
Модуль, обеспечивающий управление имитатором ВЧ навигационного сигнала.
Обеспечивает выполнение следующих задач
1. Установка времени начала симуляции
2. Установка начального вектора состояния
3. Запуск симуляции

Время начала симуляции - Т0.
Для его записи необходимо знать Т0 в условном времени.
T0 задается через специально предусмотренный графический интерфейс.

В качестве начального значения вектора состояния берется текущий вектор
состояния из динамики.
До пуска динамики там хранится выбранный вектор состояния на T0.

Запуск симуляции осуществляется в условное время T0.
ДМВ, соответствующее условному времени T0, вычисляется из T0 и величины
поправки ко времени.
Величина поправки ко времени является разницей от условного до ДМВ и задается
по локальной сети.
'''

#import pyximport; pyximport.install()

from simdispatcher import *
from simcontroller import *
from taskdispatcher import *
import datareceiver as dr
import vectorprocessor
import util
import timedeltasvc
import remotecontrol

import threading
import logging
from datetime import datetime
import configparser
import signal


DEFAULT_CONFIG = {
        'SimGEN': {
            'ip': '127.0.0.1',
            'port': 15650,
            'active_sc_id': 1,
            'passive_sc_id': 2,
            'status_request_period': 1.0,
            'armed_max_wait_time': 25.0,
            'running_max_wait_time': 1.5,
            'arming_time': 30.0,
            'average_run_time': 1.0,
        },
        'dynamic': {
            'ip': '127.0.0.1',
            'port': 5251,
            'start_time': datetime( 2000, 1, 1 ),
            'dt': 0.0,
            'vector_processor_caller_period': 0.25,
            'x_scale': 1.0,
            'v_scale': 1.0,
            'omega_scale': 1.0,
        },
        'systems': {
            'ip': '127.0.0.1',
            'port': 5257,
        },
        'TDsrv': {
            'srv_host': '127.0.0.1',
            'srv_port': 5258,
            'clt_host': '127.0.0.1',
            'clt_port': 5261,
        },
        'control': {
            'cmd_host': '127.0.0.1',
            'cmd_port': 8001,
            'ctrl_host': '127.0.0.1',
            'msgs_port': 9000,
            'vars_port': 9001,
            'msgs_period': 0.01,
            'vars_period': 0.01,
        },
        'main': {
            'period': 1e-3,
            'mov_filter_accuracy': '1e-1 1e-2 1e-3',
            'quat_filter_accuracy': '1e-2',
            'predict_dt': '0.2',
            'dt_GPS': '-10800',
        },
    }


def setup_log( log_fname ):
    logging.basicConfig( filename=log_fname, level=logging.DEBUG, \
        format='%(asctime)s %(message)s' )

    consoleHandler = logging.StreamHandler()
    consoleHandler.setLevel( logging.INFO )

    logging.getLogger().addHandler( consoleHandler )

    logging.info( 'See full log in file \'{}\''.format( log_fname ) )


class InterruptWithSignal( Exception ):
    def __init__( self, signum, *args, **kwargs ):
        self.signum = signum
        super( InterruptWithSignal, self ).__init__( *args, **kwargs )


def term_handler( signum, frame ):
    raise InterruptWithSignal( signum )


if __name__ == '__main__':

    # имя файла лога по умолчанию
    log_fname = 'nav_sim_{}.log'.format( datetime.now().strftime( '%y_%m_%d_%H_%M_%S' ) )

    import argparse
    # считываем имя файла лога и имя файла конфигурации
    parser = argparse.ArgumentParser( description='SimREMOTE control software' )
    parser.add_argument( '--logfname', help='Log file name', \
            default=log_fname )
    parser.add_argument( '--cfgname', help='Config file name', \
            default='main.config' )

    args = parser.parse_args()
    log_fname = args.logfname
    config_fname = args.cfgname

    setup_log( log_fname )

    logging.info( '===================================' )
    logging.info( 'Starting simulator control software' )
    logging.info( '===================================' )

    # перехватываем сигнал SIGTERM
    signal.signal( signal.SIGTERM, term_handler )
    # перехватываем сигнал SIGINT
    signal.signal( signal.SIGINT, term_handler )

    config = configparser.ConfigParser()
    config.read_dict( DEFAULT_CONFIG )
    res = config.read( config_fname )
    logging.info( 'Configuration read from : {}'.format( str( res ) ) )

    # объект для удаленного управления
    ctrl = remotecontrol.RemoteControl(
            config.get( 'control', 'cmd_host' ),
            config.getint( 'control', 'cmd_port' ),
            config.get( 'control', 'ctrl_host' ),
            config.getint( 'control', 'msgs_port' ),
            config.getint( 'control', 'vars_port' ),
        )
    # запускаем тред удаленного управления
    logging.info( 'main: listening for remote commands at address ({}:{})'.\
            format( config.get( 'control', 'cmd_host' ), \
                    config.getint( 'control', 'cmd_port' ) ) )
    logging.info( 'main: Starting remote control thread' )
    threading.Thread( target=ctrl.run_server, daemon=True ).start()

    # разбираем строки с точностью фильтров
    mfa = [ float( x ) \
        for x in config.get( 'main', 'mov_filter_accuracy' ).split( ' ' ) ]
    qfa = [ float( x ) \
        for x in config.get( 'main', 'quat_filter_accuracy' ).split( ' ' ) ]

    # объект для вычисления контрольной суммы
    control_sum_checker = util.ControlSumCalculator()
    # объект - очередь для передачи ВС между тредами
    q = util.DataQueue()
    q.packer = vectorprocessor.VectorProcessor.packer
    # объект - очередь для передачи признаков между тредами
    sq = util.DataQueue()
    sq.packer = vectorprocessor.VectorProcessor.signal_packer

    # прием признаков из модели
    model_addr = (
            config.get( 'systems', 'ip' ), \
            config.getint( 'systems', 'port' ), \
        )
    logging.info( 'main: listening for model data at address ({}:{})'.\
            format( *model_addr ) )
    logging.debug( 'main: Creating vector receiver object' )
    # приемник признаков
    sinp = dr.DataReceiver( *model_addr )
    sinp.add_parameter( 'num', 0, 'B' )
    sinp.add_parameter( 'type', 1, 'B' )
    sinp.add_parameter( 'sum', 2, 'H' )
    # признаки работы НМ
    sinp.add_parameter( 'work_nm1', 4, 'H' )
    sinp.add_parameter( 'work_nm2', 6, 'H' )
    # признак разрешения работы имитатора
    sinp.add_parameter( 'work_sim', 8, 'H' )
    # обработка буфера...
    sinp.add_buffer_handler( control_sum_checker )
    sinp.add_data_handler( sq.insert_params, priority=100 )


    # прием параметров движения
    dyn_addr = (
            config.get( 'dynamic', 'ip' ), \
            config.getint( 'dynamic', 'port' ), \
        )
    logging.info( 'main: listening for vector at address ({}:{})'.format( *dyn_addr ) )
    logging.debug( 'main: Creating vector receiver object' )
    # приемник параметров
    inp = dr.DataReceiver( *dyn_addr )
    inp.add_parameter( 'num', 0, 'B' )
    inp.add_parameter( 'type', 1, 'B' )
    inp.add_parameter( 'sum', 2, 'H' )
    # RVA
    inp.add_parameter( 'XA', 4, 'd' )
    inp.add_parameter( 'YA', 12, 'd' )
    inp.add_parameter( 'ZA', 20, 'd' )
    inp.add_parameter( 'VXA', 28, 'd' )
    inp.add_parameter( 'VYA', 36, 'd' )
    inp.add_parameter( 'VZA', 44, 'd' )
    # ICA
    inp.add_parameter( 'ICA', 52, 'ddddddddd' )
    # OMGXA, OMGYA, OMGZA
    inp.add_parameter( 'wXA', 124, 'd' )
    inp.add_parameter( 'wYA', 132, 'd' )
    inp.add_parameter( 'wZA', 140, 'd' )
    # RVP
    inp.add_parameter( 'XP', 148, 'd' )
    inp.add_parameter( 'YP', 156, 'd' )
    inp.add_parameter( 'ZP', 164, 'd' )
    inp.add_parameter( 'VXP', 172, 'd' )
    inp.add_parameter( 'VYP', 180, 'd' )
    inp.add_parameter( 'VZP', 188, 'd' )
    # ICP
    inp.add_parameter( 'ICP', 196, 'ddddddddd' )
    # OMGXP, OMGYP, OMGZP
    inp.add_parameter( 'wXP', 268, 'd' )
    inp.add_parameter( 'wYP', 276, 'd' )
    inp.add_parameter( 'wZP', 284, 'd' )
    # SLNI
    inp.add_parameter( 'SLNI', 292, 'ddd' )
    # t_Fly
    inp.add_parameter( 't_Fly', 316, 'd' )
    # t_zv
    inp.add_parameter( 't_zv', 324, 'd' )

    coord_sc = util.CoordinateScaler()
    coord_sc.set_scale( ( 'XA', 'YA', 'ZA', 'XP', 'YP', 'ZP' ), \
            config.getfloat( 'dynamic', 'x_scale' ) )
    coord_sc.set_scale( ( 'VXA', 'VYA', 'VZA', 'VXP', 'VYP', 'VZP' ), \
            config.getfloat( 'dynamic', 'v_scale' ) )
    coord_sc.set_scale( ( 'wXA', 'wYA', 'wZA', 'wXP', 'wYP', 'wZP' ), \
            config.getfloat( 'dynamic', 'omega_scale' ) )

    inp.add_buffer_handler( control_sum_checker )
    inp.add_data_handler( coord_sc, priority=1 )
    inp.add_data_handler( q.insert_params, priority=100 )

    logging.info( 'main: Starting vector reciever thread' )
    # запускаем тред, принимающий сообщения от динамики
    threading.Thread( target=inp.run, daemon=True ).start()
    threading.Thread( target=sinp.run, daemon=True ).start()

    sim_addr = (
            config.get( 'SimGEN', 'ip' ), \
            config.getint( 'SimGEN', 'port' )
        )
    logging.info( 'main: SimGEN address ({}:{})'.format( *sim_addr ) )
    sd = SimDispatcher( *( sim_addr + ( \
            config.getfloat( 'SimGEN', 'armed_max_wait_time' ),
            config.getfloat( 'SimGEN', 'running_max_wait_time' ),
        ) ) )
    sc = SimController( \
            config.getfloat( 'SimGEN', 'arming_time' ),
            config.getfloat( 'SimGEN', 'average_run_time' ),
        )
    td = TaskDispatcher( config.getfloat( 'main', 'period' ) )

    ctrl.bind_task_dispatcher( td )

    sc.bind_sim_dispatcher( sd )
    sc.bind_task_dispatcher( td )

    sc.set_dt_Tng_Decree( config.getfloat( 'dynamic', 'dt' ) )
    sc.set_dt_Decree_GPS( config.getfloat( 'main', 'dt_GPS' ) )

    scwc = util.SimControllerWorkerCaller( sc )

    logging.info( 'main: Starting SimGEN controller worker' )
    scwc.start()

    vp = vectorprocessor.VectorProcessor( \
            vec_reliable_condition=mfa, quat_reliable_condition=qfa, \
            predict_dt = config.getfloat( 'main', 'predict_dt' ), \
         )
    vp.bind_signal_queue_item_getter( sq.get_params )
    vp.bind_queue_item_getter( q.get_params )
    vp.bind_sim_controller( sc )
    vp.bind_remode_control( ctrl )
    vp.set_active_spacecraft_id( config.getint( 'SimGEN', 'active_sc_id' ) )
    vp.set_passive_spacecraft_id( config.getint( 'SimGEN', 'passive_sc_id' ) )
    vp.set_train_time_offset( config.getfloat( 'dynamic', 'dt' ) )
    logging.info( 'main: Set time offset to {}'.\
        format( config.getfloat( 'dynamic', 'dt' ) ) )

    vpc = util.VectorProcessorCaller( vp )
    vpc.set_train_time( 0.0 )

    td_srv = timedeltasvc.TimeDeltaSrv( \
            config.get( 'TDsrv', 'srv_host' ), \
            config.getint( 'TDsrv', 'srv_port' ), \
            config.get( 'TDsrv', 'clt_host' ), \
            config.getint( 'TDsrv', 'clt_port' ), \
        )
    td_srv.bind_dt_setter( vp.set_train_time_offset )
    td_srv.run()
    logging.info( 'main: Time delta service started' )

    td.add_periodic_task( util.GetStatusCaller( sd ),\
        config.getfloat( 'SimGEN', 'status_request_period' ) )
    td.add_periodic_task( vpc, \
        config.getfloat( 'dynamic', 'vector_processor_caller_period' ) )

    td.add_periodic_task( ctrl.send_messages, \
            config.getfloat( 'control', 'msgs_period' ) )
    td.add_periodic_task( ctrl.send_variables, \
            config.getfloat( 'control', 'vars_period' ) )

    try:
        logging.info( 'main: Run task dispatcher' )
        td.run()
    except KeyboardInterrupt:
        logging.info( 'main: Ctrl+C caught' )
    except InterruptWithSignal as e:
        logging.info( 'main: Signal {} caught'.format( e.signum ) )
    finally:
        sc.stop()
        scwc.stop()
        td_srv.stop()

    logging.info( '==================================' )
    logging.info( 'Simulator control software stopped' )
    logging.info( '==================================' )

