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
Здесь описан класс SimDispatcher, являющийся средством управления имитатором
навигационного сигнала
'''

__all__ = [
        'SimDispatcher',
        'CouldNotConnectToSimulator',
        'InvalidSimulatorResponse',
        'SimulatorError',
        'SimulatorFatalError',
        'SimulatorInappropriateState',
    ]


from lxml import etree
import socket
import datetime
import enum
from time import sleep
import logging
from threading import Lock


class CouldNotConnectToSimulator( Exception ):
    def __init__( self, value ):
        self.value = value
    def __str__( self ):
        return repr( self.value )

class InvalidSimulatorResponse( Exception ):
    def __init__( self, value ):
        self.value = value
    def __str__( self ):
        return repr( self.value )

class SimulatorError( Exception ):
    def __init__( self, value ):
        self.value = value
    def __str__( self ):
        return repr( self.value )

class SimulatorFatalError( Exception ):
    def __init__( self, value ):
        self.value = value
    def __str__( self ):
        return repr( self.value )

class SimulatorInappropriateState( Exception ):
    def __init__( self, value ):
        self.value = value
    def __str__( self ):
        return repr( self.value )


class SimDispatcher:
    '''
    Класс - диспетчер для работы с имитатором ВЧ навигационного сигнала.
    Принимает команды от других модулей, формирует запросы к имитатору ВЧ
    навигационного сигнала, анализирует ответы и изменяет собственные свойства.
    Большинство функций данного класса выполняют сетевые запросы и поэтому
    работают долго.
    '''

    class Response:
        def __init__( self, status=-1, data='', error='', fatal='' ):
            self.status = status
            self.data = data
            self.error = error
            self.fatal = fatal

    MONTH_NAME = [
            'JAN', 'FEB', 'MAR', 'APR', 'MAY', 'JUN',
             'JUL', 'AUG', 'SEP', 'OCT', 'NOV', 'DEC' ]

    VEC_FMT = ( '{:0.6f},'*3 + '{:0.9f},'*3*3 + '{:0.9f},'*4 + '{:0.9f},'*3*3 )[:-1]

    class SimStatus( enum.IntEnum ):
        LOADING = 1
        READY = 2
        ARMING = 3
        ARMED = 4
        RUNNING = 5
        PAUSED = 6
        ENDED = 7
    

    def __init__( self, \
                host, port, \
                armed_wait_time=25.0, running_wait_time=2.0, \
            ):
        self.__net_addr = ( host, port )
        self.__status = -1
        self.__error = {}
        self.__fatal = {}
        self.__internal = {}
        self.__has_connection = True
        # блокировка статуса
        self.__status_locker = Lock()
        # блокировка обмена по сети
        self.__request_locker = Lock()
        # времена установки статусов имитатора
        self.__armed_wait_time = armed_wait_time
        self.__running_wait_time = running_wait_time

        logging.debug( 'SimDispatcher object constructed' )

    @property
    def has_connecton( self ):
        '''
        Возвращает флаг наличия соединения с имитатором навигационного сигнала,
        полученный по результатам запросов статуса.
        Не выполняет сетевые запросы, работает быстро.
        '''
        return self.__has_connection

    @property
    def status( self ):
        '''
        Возвращает последний принятый код статуса имитатора нав сигнала
        Не выполняет сетевые запросы, но ждет пока не снимется блокировка с
        параметра с кодом статуса. Может работать долго.
        '''
        with self.__status_locker:
            return self.__status
    def __set_status( self, value ):
        with self.__status_locker:
            self.__status = value

    def get_status( self ):
        '''
        Запрашивает код статуса имитатора ВЧ навигационного сигнала, записывает
        его во внутреннее свойство объекта и возвращает.
        Выполняет сетевые запросы, работает долго.
        '''
        logging.debug( 'SimDispatcher: requesting for SimGEN status...' )

        res = self.__request_n_validate( 'NULL', datetime.datetime.now() )
        # выполнение self.__request_n_validate автоматически записывает статус
        # в переменную self.__status

        return self.status

    def set_start_time( self, start_datetime ):
        '''
        Устанавливает время начала сценария в имитаторе ВЧ навигационного
        сигнала.
        Выполняет сетевые запросы, работает долго.
        start_datetime - объект класса datetime, условное время начала сценария
        '''
        logging.debug( 'SimDispatcher: Request for initial time set: {}'.\
            format( start_datetime ) )
        timestr = start_datetime.strftime( '%d-{}-%Y %H:%M:%S' )
        timestr = timestr.format( self.MONTH_NAME[ start_datetime.month-1 ] )
        req = '-,START_TIME,{}'.format( timestr )
        res = self.__request_n_validate( req, datetime.datetime.now() )
        return self.status

    def prepare( self ):
        '''
        Выполняет подготовительные действия в имитаторе ВЧ навигационного
        сигнала: TR, AR
        Выполняет сетевые запросы, работает долго, порядка 26 секунд.
        '''
        logging.debug( 'SimDispatcher: starting SimGEN preparation...' )
        self.get_status()
        # Подготавливаться можно только из статуса READY
        if( self.status != self.SimStatus.READY ):
            logging.warning( 'SimDispatcher: before simulator preparing: expected state {}, got {}'.\
                format( self.SimStatus.READY, self.status ) )
            raise SimulatorInappropriateState( \
                'Could not prepare simulator with state \'{}\''.\
                    format( self.status ) )

        res = self.__request_n_validate( '-,TR,0', datetime.datetime.now() )
        if( self.status != self.SimStatus.READY ):
            logging.warning( 'SimDispatcher: trying to set TRIGGER mode: expected state {}, got {}'.\
                format( self.SimStatus.READY, self.status ) )
            raise SimulatorInappropriateState( \
                'Failed to set trigger mode with status \'{}\''.\
                    format( self.status ) )

        res = self.__request_n_validate( '-,AR', datetime.datetime.now() )
        if self.status != self.SimStatus.ARMING and self.status != self.SimStatus.ARMED:
            logging.warning( 'SimDispatcher: trying to set ARMING mode: expected state {}, got {}'.\
                format( self.SimStatus.ARMING, self.status ) )
            raise SimulatorInappropriateState( \
                'Failed to set arming mode with status \'{}\''.\
                    format( self.status ) )

        # Ожидаем установки статуса ARMED
        COUNTER_INIT_VAL = self.__armed_wait_time / 0.5
        COUNTER_WAIT_TIME = 0.5
        counter = COUNTER_INIT_VAL
        while counter > 0 and self.status != self.SimStatus.ARMED:
            sleep( COUNTER_WAIT_TIME )
            self.get_status()
            counter -= 1
        # В течение этого времени должен установиться статус ARMED
        if( self.status != self.SimStatus.ARMED ):        
            logging.warning( 'SimDispatcher: ARMED mode not set during specified timeout; status is {}'.\
                format( self.status ) )
            raise SimulatorInappropriateState( \
                'Failed to set armed mode with status \'{}\''.\
                    format( self.status ) )

        logging.debug( 'SimDispatcher: SimGEN prepared at {} seconds'.\
            format( COUNTER_WAIT_TIME*( COUNTER_INIT_VAL-counter ) ) )
        return

    def start_simulation( self ):
        '''
        Выполняет запуск моделирования ВЧ сигнала.
        Выполняет сетевые запросы, работает долго.
        '''
        logging.debug( 'SimDispatcher: starting simulation...' )
        self.get_status()
        # Проверяем что статус ARMED
        if( self.status != self.SimStatus.ARMED ):
            logging.warning( 'SimDispatcher: trying to set RUNNING mode: expected state {}, got {}'.\
                format( self.SimStatus.ARMED, self.status ) )
            raise SimulatorInappropriateState( \
                'Could not start simulation with state \'{}\''.\
                    format( self.status ) )    

        # Отправляем команду на запуск имитации
        res = self.__request_n_validate( '-,RU', datetime.datetime.now() )
        # Ожидаем установки статуса RUNNING
        COUNTER_WAIT_TIME = 0.01
        COUNTER_INIT_VAL = self.__running_wait_time / COUNTER_WAIT_TIME
        counter = COUNTER_INIT_VAL
        while counter > 0 and self.status != self.SimStatus.RUNNING:
            sleep( COUNTER_WAIT_TIME )
            self.get_status()
            counter -= 1
        if( self.status != self.SimStatus.RUNNING ):
            logging.warning( 'SimDispatcher: send RU command: expected state {}, got {}'.\
                format( self.SimStatus.RUNNING, self.status ) )
            raise SimulatorInappropriateState( \
                'Failed to set simulation mode with status \'{}\''.\
                    format( self.status ) )

        return

    def do_rewind( self ):
        '''
        Отсылает в имитатор ВЧ навигационного сигнала команду RW
        '''
        res = self.__request_n_validate( '-,RW', datetime.datetime.now() )
        return

    def __send_vector_wo_status_validation( self, \
            time: float, vehicle_id, vector: tuple, time_accuracy: int ):
        # Формируем запрос на задание положения КА и отправляем,
        # не делаем никаких проверок
        req = '{:0.{}f},MOTQ,v{}_m1,'.format( time, time_accuracy, vehicle_id ) \
                + self.VEC_FMT.format( *vector )
        res = self.__request_n_validate( req, datetime.datetime.now() )

    def send_vector( self, \
            time: float, vehicle_id: int, vector: tuple, time_accuracy: int ):
        '''
        Отсылает в имитатор ВЧ навигационного сигнала вектор состояния КА с
        идентификатором vehicle_id на момент времени time
        vector - массив чисел, соответствующий координатам:
            x,y,z,vx,vy,vz, ax,ay,az,jx,jy,jz,
            q0,q1,q2,q3,wx,wy,wz,w'x,w'y,w'z,w"x,w"y,w"z
        Выполняет сетевые запросы, работает долго.
        '''
        logging.debug( 'SimDispatcher: send state vector for vehicle with id {}'.\
            format( vehicle_id ) )
        if( self.status != self.SimStatus.RUNNING ):
            logging.warning( 'SimDispatcher: when sending motion command: expected state {}, have {}'.\
                format( self.SimStatus.RUNNING, self.status ) )
            raise SimulatorInappropriateState( \
                'Inappropriate status after vector definition \'{}\''.\
                    format( self.status ) )
        self.__send_vector_wo_status_validation\
                ( time, vehicle_id, vector, time_accuracy )
        return

    def send_initial_vector( self, \
            time: float, vehicle_id: int, vector: tuple, time_accuracy: int ):
        '''
        Отличается от send_vector отсутствием проверки статуса имитатора
        Выполняет сетевые запросы, работает долго.
        '''
        logging.debug( 'SimDispatcher: send initial state vector for vehicle with id {}'.\
            format( vehicle_id ) )
        self.__send_vector_wo_status_validation\
                ( time, vehicle_id, vector, time_accuracy )
        return

    def finish( self ):
        '''
        Останавливает моделирование ВЧ сигнала если статус RUNNING или ARMED
        Выполняет сетевые запросы, работает долго.
        '''
        logging.debug( 'SimDispatcher: trying to finish simulation' )
        self.get_status()
        # если статус ARMED, необходимо сначала запустить на исполнение
        if self.status == self.SimStatus.ARMED:
            self.start_simulation()

        if self.status != self.SimStatus.RUNNING:
            logging.warning( 'SimDispatcher: could not stop simulation with status {}'.\
                format( self.status ) )
            raise SimulatorInappropriateState( \
                'Could not finish simulation with state \'{}\''.\
                    format( self.status ) )    

        res = self.__request_n_validate( '-,EN,1', datetime.datetime.now() )
        if( self.status != self.SimStatus.READY ):
            logging.warning( 'SimDispatcher: trying to RESET simulation: expected state {}, got {}'.\
                format( self.SimStatus.READY, self.status ) )
            raise SimulatorInappropriateState( \
                'Failed to set simulation mode with status \'{}\''.\
                    format( self.status ) )

        return

    def __request( self, body, time ):
        '''
        Отсылает запрос на имитатор ВЧ навигационного сигнала, ожидает и
        получает ответ.
        На время ожидания ответа исполнение блокируется.
        '''
        logging.debug( 'SimDispatcher: request: \'{}\' at {}'.\
            format( body, time ) )
        with self.__request_locker:
            try:
                s = socket.socket( socket.AF_INET, socket.SOCK_STREAM )
                s.connect( self.__net_addr )
                s.send( body.encode( 'ascii' ) )
                resp = s.recv( 16384 )
                logging.debug( 'SimDispatcher: response: \'{}\''.format( resp ) )
                if not self.__has_connection:
                    self.__has_connection = True
                    logging.info( 'SimDispatcher: Connecton to SimGEN restored' )
            except ConnectionError as e:
                if self.__has_connection:
                    self.__has_connection = False
                    logging.warning( 'SimDispatcher: Connection to SimGEN lost' )
                raise CouldNotConnectToSimulator( '{}'.format( e ) )
            s.close()

            resp_string = resp.decode( 'ascii' )
            try:
                root = etree.fromstring( resp_string )
            except etree.XMLSyntaxError as e:
                self.__internal[ time ] = ( body, resp_string )
                logging.warning( 'SimDispatcher: Could not parse XML response' )
                raise InvalidSimulatorResponse( 'Invalid XML structure recieved at {}'.format( time ) )

            if root.tag != 'msg':
                self.__internal[ time ] = ( body, resp_string )
                logging.warning( 'SimDispatcher: \'msg\' node is not present in XML structure' )
                raise InvalidSimulatorResponse( '\'msg\' node is not present in XML structure recieved at {}'.format( time ) )

            res = self.Response()
            for child in root:
                if child.tag == 'status':
                    res.status = child.text
                elif child.tag == 'data':
                    res.data = child.text
                elif child.tag == 'error':
                    res.error = child.text
                    self.__error[ time ] = ( body, res.error )
                elif child.tag == 'fatal':
                    res.fatal = child.text
                    self.__fatal[ time ] = ( body, res.fatal )
        
        new_status = int( res.status )
        if new_status != self.status:
            logging.info( 'SimDispatcher: SimGEN status changed to {}'.\
                format( new_status ) )
            self.__set_status( new_status )
        return res

    def __validate_response( self, res ):
        if res.fatal:
            logging.warning( 'SimDispatcher: SimGEN status is \'FATAL ERROR\': {}'.\
                format( res.fatal ) )
            raise SimulatorFatalError( 'Fatal error in simulator: {}'.\
                format( res.fatal ) )
        if res.error:
            logging.warning( 'SimDispatcher: SimGEN status is \'ERROR\': {}'\
                .format( res.error ) )
            raise SimulatorError( 'Error in simulator: {}'.\
                format( res.error ) )
        logging.debug( 'SimDispatcher: SimGEN status is \'OK\': {}'\
            .format( res.status ) )

    def __request_n_validate( self, *args, **kwargs ):
        res = self.__request( *args, **kwargs )
        self.__validate_response( res )
        return res

if __name__ == '__main__':
    d = SimDispatcher()
    d.get_status()

