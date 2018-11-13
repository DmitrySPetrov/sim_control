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
Здесь описан класс SimController, осуществляющий управление имитатором
навигационного сигнала
'''

__all__ = [
        'SimController',
        'InappropriateSimStatus',
        'SimControllerInternalError',
        'SimControllerInitializationError'
    ]


from simqueue import SimQueue

from datetime import timedelta
from math import ceil
from threading import Lock
import logging


class InappropriateSimStatus( Exception ):
    def __init__( self, value ):
        self.value = value
    def __str__( self ):
        return repr( self.value )

class SimControllerInternalError( Exception ):
    def __init__( self, value ):
        self.value = value
    def __str__( self ):
        return repr( self.value )

class SimControllerInitializationError( Exception ):
    def __init__( self, value ):
        self.value = value
    def __str__( self ):
        return repr( self.value )


class SimController:
    '''
    Класс - контроллер имитатора ВЧ навигационного сигнала.
    Принимает команды на подготовку, запуск, перезапуск имитатора, задание
    времени начала имитации и задание параметров движения моделируемых КА.

    Выполнение всех команд, кроме команды на запуск имитации, должно
    производиться в одном треде.
    Перед началом работы необходимо указать ссылки на объекты классов
    SimDispatcher и TaskDispatcher.
    '''

    def __init__( self, sim_armed_time=30.0, sim_run_time=1.0 ):
        self.__starting = False    # признак запуска процедуры старта имитатора
        self.__work = False        # признак работы имитатора
        self.__sd = None        # диспетчер имитатора
        self.__td = None        # диспетчер задач
        # поправка к декретному времени для вычисления условного
        # должна быть положительной если условное больше декретного
        self.__dt_tng_decree = timedelta( seconds=0.0 )
        # разница между шкалами декретного времени и GPS
        self.__dt_decree_gps = timedelta( hours=-3.0 )
        # время, которое отводится имитатору для перехода
        # из статуса ARMING в статус ARMED
        self.__sim_start_dt = sim_armed_time
        # время, которое проходит от команды RU до действительного запуска
        self.__sim_run_dt = sim_run_time
        # очередь сообщений в имитатор навигационного сигнала
        self.__sim_queue = SimQueue()
        # блокировка функции worker
        self.__worker_lock = Lock()
        # декретное время запуска имитатора
        self.__sim_start_time = None
        # флаг несоответствия статуса имитатора и запрашиваемых действий
        self.__inappropriate_status = False
        # блокировка запуска и останова
        self.__startstop_lock = Lock()
        # точность округления времени при прогнозе
        self.__predict_accuracy = 2

        logging.debug( 'SimController object created' )


    def bind_sim_dispatcher( self, sd ):
        self.__sd = sd

    def bind_task_dispatcher( self, td ):
        self.__td = td

    def set_inappropriate_status_flag( self ):
        self.__inappropriate_status = True
    @property
    def inappropriate_status( self ):
        return self.__inappropriate_status

    def __validate_init( self ):
        if self.__sd == None:
            logging.error( 'SimController: SimDispatcher not binded' )
            raise SimControllerInitializationError( 'SimDispatcher not specified' )
        if self.__td == None:
            logging.error( 'SimController: TaskDispatcher not binded' )
            raise SimControllerInitializationError( 'TaskDispatcher not specified' )
        

    def __start_simulation( self ):
        self.__sd.start_simulation()
        self.__work = True

    def __start_simulation_and_notify( self ):
        self.__start_simulation()
        logging.info( 'SimController: Simulation started' )

    def start_at_nearest_time( self, local_time, mot_time: float, vector_predictors ):
        '''
        Запустить имитатор навигационного сигнала в ближайшее время
        local_time - текущее декретное время
        mot_time - текущее время в шкале времени параметров движения
        vector_predictors - словарь объектов для прогнозирования векторов
            различных КА вида:
                { идентификатор_КА: функция_для_прогнозирования_ВС_КА }
            функция для прогнозирования вектора КА должна возвращать данные
            в формате, подходящем для SimDispatcher.send_vector

        Выполняется долго, т.к. внутри осуществляются вызовы по сети
        Добавляет задачу на запуск имитатора в диспетчер задач

        При неожиданном статусе имитатора бросает исключение
            InappropriateSimStatus
        '''
        with self.__startstop_lock:
            if self.__starting or self.__work:
                logging.debug( 'SimController: no need to start SimGEN twice' )
                return

            logging.debug( 'SimController: trying to start at nearest time' )
            self.__validate_init()
            if not self.__sd.has_connecton:
                logging.debug( 'SimController: There is no connection to SimGEN, could not start' )
                return

            self.__sim_start_time = local_time + timedelta( seconds=self.__sim_start_dt )
            start_tng_time = self.__sim_start_time \
                    + self.__dt_tng_decree + self.__dt_decree_gps
            # Время в шкале вещ. чисел для предсказания вектора
            predict_time = mot_time + self.__sim_start_dt \
                    + self.__dt_tng_decree.total_seconds()

            # Приводим имитацию к началу 6-секундного интервала
            seconds = start_tng_time.second + 1e-6*start_tng_time.microsecond
            dt6 = 6.0 - seconds % 6.0
            start_tng_time += timedelta( seconds=dt6 )
            self.__sim_start_time += timedelta( seconds=dt6 )
            predict_time += dt6

            logging.debug( 'SimController: will start at {}, tng time is {}'.\
                format( self.__sim_start_time, start_tng_time ) )

            sd = self.__sd
            # Завершенный сценарий необходимо вернуть назад командой RW
            if sd.status == sd.SimStatus.ENDED:
                logging.warning( 'SimController: Status is ENDED. Let\'s REWIND' )
                sd.do_rewind()
            # Проверяем, можно ли запустить имитатор
            if sd.status > sd.SimStatus.READY:
                logging.warning( 'SimController: Trying to start simulator but status is {}. Let\'s do RESTART'.\
                    format( sd.status ) )
                self.set_inappropriate_status_flag()
                raise InappropriateSimStatus( \
                    'Trying to start simulator at nearest time with status {}'.\
                        format( sd.status ) )

            # Добавляем задачу запуска имитатора навигационного сигнала в
            # вычисленный момент времени
            self.__td.add_once_task(\
                    self.__start_simulation_and_notify,\
                    self.__sim_start_dt - self.__sim_run_dt + dt6\
                )

            # Устанавливаем время начала моделирования
            sd.set_start_time( start_tng_time )

            # Прогнозируем вектор состояния и отсылаем его
            for k,v in vector_predictors.items():
                sd.send_initial_vector( 0, k, v( predict_time ), 0 )
            # Выставляем флаг начала запуска имитатора
            self.__starting = True

            # Выполняем подготовку имитатора к запуску
            sd.prepare()

    def predict_and_send_vectors( self, local_time, start_time,
            predict_dt: float, vector_predictors ):
        '''
        Используя функцию vector_predictors, спрогнозировать вектор состояния
        на момент, соответствующий декретному времени local_time.
        start_time - начало ШВ для вектора состояния из модели движения
        predict_dt - время, требуемое имитатору на подготовку
        vector_predictors - функции, которые выполняют прогноз
        Выполняется долго, т.к. внутри осуществляются вызовы по сети

        При неожиданном статусе имитатора бросает исключение
            InappropriateSimStatus
        '''
        logging.debug( 'SimController: starting to predict and send vectors at time {}'.\
            format( local_time ) )
        self.__validate_init()
        sd = self.__sd
        if sd.status != sd.SimStatus.RUNNING:
            logging.warning( 'SimController: Trying to send vector but status is {}. Let\' do RESTART'.\
                format( sd.status ) )
            self.set_inappropriate_status_flag()
            raise InappropriateSimStatus( \
                'Trying to send vector with status {}'.\
                    format( sd.status ) )
        # тренировочное время, соответствующее local_time
        tng_time = local_time + self.__dt_tng_decree
        # время в шкале времен модели движения
        _mot_time = ( tng_time - start_time ).total_seconds()
        # время, на которое прогнозируем вектор
        _predict_time = _mot_time + predict_dt
        # округляем с точностью до сотых долей секунд
        _predict_time = round( _predict_time, self.__predict_accuracy )
        # время от запуска имитатора навигационного сигнала
        _sim_time = ( local_time - self.__sim_start_time ).total_seconds() \
            + predict_dt
        # Прогнозируем вектор состояния и отсылаем его
        for k,v in vector_predictors.items():
            self.__sd.send_vector( _sim_time, k, v( _predict_time ), self.__predict_accuracy )

    def stop( self ):
        with self.__startstop_lock:
            logging.debug( 'SimController: stopping simulation' )
            self.__validate_init()
            self.__inappropriate_status = False
            self.__starting = False
            self.__work = False
            if self.__sd.has_connecton:
                try:
                    # Игнорируем исключения
                    self.__sd.finish()
                except Exception:
                    pass

    @property
    def starting( self ) -> bool:
        return self.__starting

    @property
    def work( self ) -> bool:
        return self.__work

    @property
    def simulator_ready( self ) -> bool:
        if self.__sd == None:
            return False
        return self.__sd.status >= self.__sd.SimStatus.READY

    def set_dt_Tng_Decree( self, td_in_seconds: float ) -> None:
        '''
        Установить разницу между условным временем и текущим декретным
        '''
        self.__dt_tng_decree = timedelta( seconds=td_in_seconds )

    def set_dt_Decree_GPS( self, td_in_seconds: float ) -> None:
        '''
        Установить разницу между декретным временем и GPS
        '''
        self.__dt_decree_gps = timedelta( seconds=td_in_seconds )

    @property
    def dt_Decree_GPS( self ) -> float:
        return self.__dt_decree_gps.total_seconds()

    @property
    def dt_Tng_Decree( self ) -> float:
        return self.__dt_tng_decree.total_seconds()

    @property
    def prepare_to_run_time( self ) -> float:
        return self.__sim_start_dt


    def worker( self ) -> bool:
        '''
        Взять очередь сообщений и выполнить их
        Возвращает True если была выполнена какая-либо команда
        '''
        self.__validate_init()

        if not self.__worker_lock.acquire( blocking=False ):
            return False
        do = False
        try:
            while True:
                msg = self.__sim_queue.get_command()
                if msg == None:
                    break
                if hasattr( msg, 'call' ):
                    msg.call( self )
                    do = True
        finally:
            self.__worker_lock.release()
        return do

    def add_command( self, cmd ):
        logging.debug( 'SimController: message with type {} inserted to queue'.\
            format( str( type( cmd ) ) ) )
        self.__sim_queue.insert_command( cmd )

    @property
    def armed_time( self ) -> float:
        return self.__sim_start_dt
    @armed_time.setter
    def armed_time( self, value: float ):
        self.__sim_start_dt = value

    @property
    def run_time( self ) -> float:
        return self.__sim_run_dt
    @run_time.setter
    def run_time( self, value: float ):
        self.__sim_run_dt = value

