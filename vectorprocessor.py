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
Здесь описан класс VectorProcessor, выполняющий следующие функции:
1) Осуществляющий контроль неразрывности времени, приходящего вместе с
   параметрами движения.
2) При скачках времени осуществляющий перезапуск имитатора навигационного
   сигнала.
3) Осуществляющий прогнозирование параметров движения и передачу их в имитатор
   навигационного сигнала.

Использование класса VectorProcessor:
    vp = VectorProcessor()
    # Функция qg извлекает следующий пришедщий вектор состояния из очереди
    vp.bind_queue_item_getter( qg )
    # Объект sc - контроллер имитатора навигационного сигнала
    vp.bind_sim_controller( sc )
    # Устанавливаем идентификатор активного КА
    vp.set_active_spacecraft_id( 1 )
    # Устанавливаем идентификатор пассивного КА
    vp.set_passive_spacecraft_id( 2 )
    ...
    ...
    # ftime - вещественное количество секунд, прошедшее от начала моделирования
    #        до текущего момента
    # time - текущее время, экземпляр класса datetime
    vp.worker( ftime, time )

Метод VectorProcessor.worker забирает из очереди сообщения, упакованные с
помощью статического метода VectorProcessor.packer
'''

import mathutils
import variablepredictor as vp
import timewatcher as tw
from simqueue import StopCmd, StartAtNearestTimeCmd, PredictAndSendVectorsCmd

import numpy as np
import math
import copy

from datetime import datetime, timedelta
import logging

class InvalidParameterDict( Exception ):
    pass

class VectorProcessor:
    '''
    Класс, осуществляющий прогнозирование вектора состояния и упаковку
    его в формат, годный к отправке с помощью класса SimDispatcher
    '''

    # сжатие Земли
    kappa = 1/298.25784
    # коэффициент сжатия Земли
    k = 1 - kappa

    # кватернион перехода из ССК в ССК имитатора
    m_sim = np.array( [ [-1,0,0],[0,0,-1],[0,-1,0] ] )
    q_sim = mathutils.Matrix( m_sim.tolist() ).to_quaternion()

    @staticmethod
    def signal_packer( param: dict ):
        '''
        Выполняет упаковку признаков из словаря в структуру данных, понятную
        для этого класса
        '''
        try:
            # разрешение работы имитатора навигационного сигнала
            do_work = param['work_sim']
            item = ( \
                    ( do_work[0], ),
                )
            return item
        except KeyError as e:
            # отсутствует ключ для массива param
            raise InvalidParameterDict( e )


    @staticmethod
    def packer( param: dict ):
        '''
        Выполняет упаковку параметров из словаря в структуру данных, понятную
        для данного класса
        '''
        try:
            # вычисляем кватернион разворота АКА
            A_A = mathutils.Matrix( \
                tuple( tuple( param['ICA'][i+3*j] for i in range(3) )
                    for j in range(3) )
                )
            qA = A_A.to_quaternion()

            # вычисляем кватернион разворота ПКА
            A_P = mathutils.Matrix( \
                tuple( tuple( param['ICP'][i+3*j] for i in range(3) )
                    for j in range(3) )
                )
            qP = A_P.to_quaternion()

            # формируем элемент очереди
            # данные сгруппированы по виду величины-производные
            item = ( \
                    # условное время вектора состояния
                    ( param['t_Fly'][0], ),
                    # звездное время
                    ( param['t_zv'][0], ),

                    # координаты АКА
                    ( param['XA'][0], param['YA'][0], param['ZA'][0], \
                        param['VXA'][0],param['VYA'][0],  param['VZA'][0] ),
                    # компоненты угловой скорости и кватерниона разворота АКА
                    ( param['wXA'][0], ),
                    ( param['wYA'][0], ),
                    ( param['wZA'][0], ),
                    ( qA.w, qA.x, qA.y, qA.z ),

                    # координаты ПКА
                    ( param['XP'][0], param['YP'][0], param['ZP'][0], \
                        param['VXP'][0],param['VYP'][0],  param['VZP'][0] ),
                    # компоненты кватерниона разворота и угловой скорости ПКА
                    ( param['wXP'][0], ),
                    ( param['wYP'][0], ),
                    ( param['wZP'][0], ),
                    ( qP.w, qP.x, qP.y, qP.z ), 
                )

            return item
        except KeyError as e:
            # отсутствует ключ для массива param
            raise InvalidParameterDict( e )


    # Буферы актива и пассива должны нести о разных объектах
    # строго одну и ту же информацию!
    ACTIVE_SC_BUFFER = (
            ( 2,  0 ), ( 2,  1 ), ( 2,  2 ),    # X, Y, Z
            ( 2,  3 ), ( 2,  4 ), ( 2,  5 ),    # VX, VY, VZ
            ( 2,  6 ), ( 2,  7 ), ( 2,  8 ),    # AX, AY, AZ
            ( 2,  9 ), ( 2, 10 ), ( 2, 11 ),    # JX, JY, JZ
            ( 6, 0 ), ( 6, 1 ), ( 6, 2 ), ( 6, 3 ), # Q0, Q1, Q2, Q6
            ( 3, 0 ), ( 4, 0 ), ( 5, 0 ),    # wX, wY, wZ
            ( 3, 1 ), ( 4, 1 ), ( 5, 1 ),    # w'X, w'Y, w'Z
            ( 3, 2 ), ( 4, 2 ), ( 5, 2 ),    # w"X, w"Y, w"Z
        )

    PASSIVE_SC_BUFFER = (
            ( 7,  0 ), ( 7,  1 ), ( 7,  2 ),    # X, Y, Z
            ( 7,  3 ), ( 7,  4 ), ( 7,  5 ),    # VX, VY, VZ
            ( 7,  6 ), ( 7,  7 ), ( 7,  8 ),    # AX, AY, AZ
            ( 7,  9 ), ( 7, 10 ), ( 7, 11 ),    # JX, JY, JZ
            ( 11, 0 ), ( 11, 1 ), ( 11, 2 ), ( 11, 3 ), # Q0, Q1, Q2, Q3
            ( 8, 0 ), ( 9, 0 ), ( 10, 0 ),    # wX, wY, wZ
            ( 8, 1 ), ( 9, 1 ), ( 10, 1 ),    # w'X, w'Y, w'Z
            ( 8, 2 ), ( 9, 2 ), ( 10, 2 ),    # w"X, w"Y, w"Z
        )

    def __init__( self, \
                start_time=None, \
                vec_reliable_condition=None, quat_reliable_condition=None, \
                predict_dt=0.05, \
                max_time_error = 30.0, \
            ):
        logging.debug( 'Creating VectorProcessor instance' )
        self.__signal_queue_item_getter = None
        self.__queue_item_getter = None
        self.__sim_controller = None
        self.__remote_control = None
        self.__time_offset = 0.0
        self.__time_offset_reliable = False
        self.__time_offset_changed = False
        self.__predict_dt = predict_dt
        self.__max_time_error = max_time_error
        # идентификатор АКА в имитаторе навигационного сигнала
        self.__asc_id = None
        # идентификатор ПКА в имитаторе навигационного сигнала
        self.__psc_id = None

        self.__vec_reliable_condition = vec_reliable_condition
        self.__q_reliable_condition = quat_reliable_condition

        # разрешение симуляции от компьютеров комплекса
        self.__external_enable_work = False

        # фильтр для принимаемых параметров
        self.__reset_filter()
        # начало шкалы времени для векторов
        if start_time:
            self.__vector_start_time = start_time
        else:
            self.__vector_start_time = datetime( 2000, 1, 1 )
        logging.debug( 'VectorProcessor: motion time is zero at {}'.\
            format( self.__vector_start_time ) )

    def __reset_filter( self ):
        # классы для прогноза кватернионов разворота АКА и ПКА
        p_qA = vp.QuaternionPredictor()
        p_qP = vp.QuaternionPredictor()
        # принимаемые параметры
        self.__params = [
                None,    # time
                vp.LinearPredictor( der=7.2921158553e-5 ),    # t_zv 

                vp.OrbitalMovementPredictor( 4, 1e-15, 100.0 ),
                    # XA, YA, ZA, VXA, VYA, VZA
                p_qA.wX,    # wXA
                p_qA.wY,    # wYA
                p_qA.wZ,    # wZA
                p_qA,        # Q0A...Q3A

                vp.OrbitalMovementPredictor( 4, 1e-15, 100.0 ),
                    # XP, YP, ZP, VXP, VYP, VZP
                p_qP.wX,    # wXP
                p_qP.wY,    # wYP
                p_qP.wZ,    # wZP
                p_qP,        # Q0P...Q3P
            ]

        # устанавливаем пороги достоверности координат и скоростей
        if self.__vec_reliable_condition:
            for i in ( 2, 7 ):
                self.__params[ i ].set_reliable_condition\
                    ( self.__vec_reliable_condition )
        else:
            for i in ( 2, 7 ):
                self.__params[ i ].set_reliable_condition\
                    ( [ 1e-1, 1e-2, 1e-3 ] )
        # устанавливаем пороги достоверности угловых скоростей
        if self.__q_reliable_condition:
            for i in ( 3, 4, 5, 8, 9, 10 ):
                self.__params[ i ].set_reliable_condition\
                    ( self.__q_reliable_condition )
        else:
            for i in ( 3, 4, 5, 8, 9, 10 ):
                self.__params[ i ].set_reliable_condition\
                    ( [ 1e-2 ] )

        # класс для отслеживания изменения времени
        self.__time_watcher = tw.TimeWatcher()

        self.__vec_reliable = False


    def bind_signal_queue_item_getter( self, getter: callable ) -> None:
        '''
        Запоминает функцию для получения информации из очереди
        '''
        self.__signal_queue_item_getter = getter

    def bind_queue_item_getter( self, getter: callable ) -> None:
        '''
        Запоминает функцию для получения информации из очереди
        '''
        self.__queue_item_getter = getter

    def bind_sim_controller( self, controller ) -> None:
        '''
        Запоминает функцию для отправки вектора состояния в имитатор
        '''
        self.__sim_controller = controller

    def bind_remode_control( self, rc ) -> None:
        '''
        Запоминает объект удаленного контроля
        '''
        self.__remote_control = rc

    def send_vars2rc( self, data ) -> None:
        '''
        Отсылает значения переменных в объект удаленного управления
        '''
        if self.__remote_control:
            self.__remote_control.add_variables( data )
    def send_msg2r( self, msg ) -> None:
        '''
        Отсылает строку диагностики в объект удаленного управления
        '''
        pass

    def set_train_time_offset( self, offset: float, rel:bool=False ) -> None:
        '''
        Устанавливает поправку условного к текущему времени в секуднах
        '''
        if self.__time_offset != offset:
            self.__time_offset = offset
            self.__time_offset_changed = True
            self.__sim_controller.set_dt_Tng_Decree( offset )
        if self.__time_offset_reliable != rel:
            self.__time_offset_reliable = rel
            self.__time_offset_changed = True

    @property
    def train_time_offset( self ):
        return self.__time_offset
    @property
    def train_time_reliable( self ):
        return self.__time_offset_reliable

    @property
    def start_time( self ):
        return self.__vector_start_time

    def set_active_spacecraft_id( self, id: int ) -> None:
        '''
        Устанавливает идентификатор сообщений для АКА
        '''
        self.__asc_id = id

    def set_passive_spacecraft_id( self, id: int ) -> None:
        '''
        Устанавливает идентификатор сообщений для АКА
        '''
        self.__psc_id = id

    def worker( self, ftime: float, now ):
        '''
        Достает векторы состояния из очереди, обновляет фильтр,
        выполняет аппроксимацию к текущему условному времени и
        отправляет вектор состояния в имитатор навигационного сигнала.
        Если модельное время динамики изменилось, это значит что динамика
        находится в стопе или введен новый вектор состояния, вызывает команду
        остановки имитации ВЧ сигнала.
        После восстановления нормального хода модельного времени необходимо
        спрогнозировать ВС на определенное время T1 и затем в условное время T1
        запустить имитатор навигационного сигнала.

        ftime - количество секунд от начала тренировки
        now - текущее время

        ftime и now соответствуют одному и тому же времени по различным шкалам
        '''
        item = None
        while True:
            # забираем сигналы
            _next = self.__signal_queue_item_getter()
            if _next == None:
                # сигнал не приходил
                break
            item = _next
        if item:
            self.__external_enable_work = bool( item[0][0] )
            self.send_vars2rc( { 'сигНС': self.__external_enable_work } )

        # флаг сброса имитатора навигационного сигнала
        item = None
        while True:
            # забираем вектор состояния из очереди
            _next = self.__queue_item_getter()
            if _next == None:
                # вектор не приходил
                break

            item = _next

        self.send_vars2rc( {
                'SimGEN_status': self.__sim_controller.sd.status,
                'векторДостоверен': self.__vec_reliable,
            } )

        # берем последний вектор, если такой есть
        if item:
            _time = item[0][0]
            self.send_vars2rc( { 't_Fly': _time } )
            # Шаг 1. Проверяем, не тот же самый ли этот вектор
            # Если да, то модель движения находится в СТОПе
            if _time == self.__time_watcher.value:
                logging.debug( 'VectorProcessor: Duplicate vector recieved. Skipping...' )
                return

            # Шаг 2. Распаковываем время и проверяем, не было ли скачка
            try:
                self.__time_watcher.validate_time( ftime, _time )
                if not self.__time_watcher.reliable:
                    return
            except tw.TimeChanged as e:
                ( logging.debug, logging.warning )[ self.__external_enable_work ]\
                    ( 'VectorProcessor: Big time step...' )
                self.__restart_sim( not self.__external_enable_work )
                return

            # Шаг 3. Проверяем, не слишком ли большая разница по времени
            _nowtime = ( now - self.__vector_start_time ).total_seconds()
            t_err = _nowtime - _time + self.__time_offset
            self.send_vars2rc( { 'dt_err': t_err } )
            if abs( t_err ) > self.__max_time_error:
                ( logging.debug, logging.warning )[ self.__external_enable_work ]\
                    ( 'VectorProcessor: Too big time error: {}'.\
                        format( t_err ) )
                self.__restart_sim( not self.__external_enable_work )
                return

            # Шаг 4. Регистрируем новые значения вектора состояния
            self.__vec_reliable = True
            for s,d in zip( item, self.__params ):
                logging.debug( 'Registering value: {}'.format( s ) )
                if d:
                    d.register_value( _time, s )
                    # Вычисляем, а достоверен ли вектор состояния
                    if not d.reliable:
                        self.__vec_reliable = False
                        logging.debug( 'Non reliable' )
        # end if item

        # Проверяем, был ли скачок по времени
        if self.__time_offset_changed:
            self.__time_offset_changed = False
            logging.info( 'VectorProcessor: restart due to change of time offset' )
            # Рестартуем имитатор
            self.__restart_sim()
            return

        #if self.__sim_controller.simulator_ready and self.__external_enable_work:
        if self.__sim_controller.simulator_ready:
            # Если статус имитатора неподходящий - рестартуем
            if self.__sim_controller.inappropriate_status:
                logging.debug( 'VectorProcessor: restart due to inappropriate status of SimGEN' )
                self.__restart_sim()
                return

            # Если нет запуска имитатора, есть достоверность ВС и времени
            # и разрешение работы,
            # вычисляем и записываем прогноз, запускаем имитатор
            if not self.__sim_controller.starting:
                if self.__vec_reliable \
                        and self.__time_offset_reliable \
                        and self.__external_enable_work:
                    self.__start_sim( now )
                return

            # Если есть запуск имитатора и нет работы, ждем T1 и запускаем
            # имитатор (делается через диспетчер задач)
            if not self.__sim_controller.work:
                return

            # Если каким-то чудом мы здесь, а достоверности времени
            # или разрешения работы нет,
            # рестартуем
            if not ( self.__time_offset_reliable and self.__external_enable_work ):
                self.__restart_sim()
                return

            # Если есть запуск имитатора и есть работа, посылаем ВС
            self.__send_vectors_to_sim( now, ftime )


    def __restart_sim( self, quiet=False ):
        '''
        Рестартует генератор навигационного сигнала,
        сбрасывает фильтр движения,
        обнуляет флаг наличия скачка по времени.
        quiet - надо ли рестартовать генератор навигационного сигнала
        '''
        if not quiet:
            self.__sim_controller.add_command( StopCmd() )
        self.__reset_filter()
        self.__time_offset_changed = False
#        for p in self.__params:
#            if p:
#                p.restart()

    def __start_sim( self, now ):
        logging.info( 'VectorProcessor: Vector reliable, starting...' )
        ftime = ( now - self.__vector_start_time ).total_seconds()
        self.__sim_controller.add_command(
                StartAtNearestTimeCmd( now, ftime, {
                    self.__asc_id: self.__make_predictor\
                            ( self.__params, self.ACTIVE_SC_BUFFER ),
                    self.__psc_id: self.__make_predictor\
                            ( self.__params, self.PASSIVE_SC_BUFFER ),
                } )
            )

    def __send_vectors_to_sim( self, now, ftime ):
        logging.debug( 'VectorProcessor: real time: {}({}) tng time: {}'.\
                format( now, now + timedelta( seconds=self.__time_offset ),\
                    self.__vector_start_time + timedelta(\
                        seconds=self.__time_watcher.extrapolate( ftime )[0] ),\
                ) )
        self.__sim_controller.add_command(
            PredictAndSendVectorsCmd( now, self.__vector_start_time, \
                self.__predict_dt, {
                    self.__asc_id: self.__make_predictor\
                            ( self.__params, self.ACTIVE_SC_BUFFER ),
                    self.__psc_id: self.__make_predictor\
                            ( self.__params, self.PASSIVE_SC_BUFFER ),
                } ) )


    @staticmethod
    def predict_sc_vector( params, ftime: float, buffer_description ):
        '''
        Спрогнозировать вектор состояния на указанный момент времени
        ftime - время, на которое рассчитывается вектор состояния
        buffer_description - массив с описанием буфера выходных данных
        Выходные данные: список с данными, соответствующий описанию из 
        buffer_description, спрогнозированный на основе params на время ftime
        '''
        _sc_pred = [ None ] * len( params )

        sc_params = []
        for p in buffer_description:
            if params[ p[0] ]:
                # экстраполяцию вызываем только один раз для каждого параметра
                if _sc_pred[ p[0] ] is None:
                    _sc_pred[ p[0] ] = \
                        params[ p[0] ].extrapolate( ftime )
                # добавляем в список параметров КА, отправляемых в имитатор
                sc_params.append( _sc_pred[ p[0] ][ p[1] ] )
            else:
                sc_params.append( 0.0 )

        return sc_params

    @staticmethod
    def rotate_quaternion( r_ECI, v_ECI, q_IC ):
        '''
        Преобразуем кватернион перехода из инерциальной в связанную СК q_IC
        в кватернион перехода из локальной геодезической в связанную SimGEN СК
        '''
        x = r_ECI[0]
        y = r_ECI[1]
        z = r_ECI[2]
        rad = np.linalg.norm( r_ECI[0:2] )
        # вычисляем кватернион из ИСК в ЛГСК
        # север, восток, низ
        e1 = mathutils.Vector( ( -z*x/rad, -z*y/rad, rad*VectorProcessor.k**2 ) ).normalized()
        e2 = e1.cross( mathutils.Vector( r_ECI ) ).normalized()
        e3 = e1.cross( e2 )
        q_lgsk = mathutils.Matrix( ( e1, e2, e3 ) ).to_quaternion()
        # результат
        q_res = VectorProcessor.q_sim*q_IC*q_lgsk.inverted()
        #print( '{:10.4f} {:10.4f} {:10.4f}'.format( *tuple( a*180/math.pi for a in q_res.to_euler( 'ZYX' )[:] ) ) )
        return q_res

    @staticmethod
    def predict_and_rotate_sc_vector( params, ftime: float, buffer_description ):
        '''
        Спрогнозировать вектор состояния на указанный момент времени и
        перевести его в ГСК WGS-84
        Поскольку буферы актива и пассива одинаковые, можно универсальным
        образом извлекать параметры движения
        '''
        # угловая скорость вращения Земли
        w_earth = np.array( [ 0.0, 0.0, params[1].diff(1) ] )
        # звездное время
        t_sta = params[1].extrapolate( ftime )[0]
        # кватернион разворота в ГСК
        # звездное время берется со знаком "минус", поскольку переводим
        # из ИСК в ГСК, а не наоборот
        c,s = math.cos( t_sta ), math.sin( t_sta )
        m_sta = np.array( [[c,s,0],[-s,c,0],[0,0,1]] )
        q_sta = mathutils.Quaternion( [ 0, 0, 1 ], -t_sta )
        # прогнозируем вектор на нужное время
        res = VectorProcessor.predict_sc_vector\
            ( params, ftime, buffer_description )
        logging.debug( 'Rotate (at time {}) following vector: {}'.format( ftime, res ) )

        # координаты и производные...
        r_ECI = np.array( res[0:3] ).reshape( (3) )
        v_ECI = np.array( res[3:6] ).reshape( (3) )
        a_ECI = np.array( res[6:9] ).reshape( (3) )
        j_ECI = np.array( res[9:12] ).reshape( (3) )
        # разворачиваем параметры движения
        r_ECEF = m_sta.dot( r_ECI )
        wr = np.cross( w_earth, r_ECI )
        v_ECEF = m_sta.dot( v_ECI - wr )
        wv = np.cross( w_earth, v_ECI )
        a_ECEF = m_sta.dot( a_ECI - 2 * wv + np.cross( w_earth, wr ) )
        j_ECEF = m_sta.dot( j_ECI - 3 * np.cross( w_earth, a_ECI ) \
                + 3 * np.cross( w_earth, wv ) + w_earth.dot(w_earth) * wr )

        # кватернион и угловые скорости...
        q_ECI = mathutils.Quaternion( res[ 12:16 ] )
        w_JCS = np.array( res[ 16:19 ] )
        w1_JCS = np.array( res[ 19:22 ] )
        w2_JCS = np.array( res[ 22:25 ] )
        # разворачиваем параметры вращения
        q_SIM = VectorProcessor.rotate_quaternion( r_ECI, v_ECI, q_ECI )
        w_SIM = VectorProcessor.m_sim.dot( w_JCS )
        w1_SIM = VectorProcessor.m_sim.dot( w1_JCS )
        w2_SIM = VectorProcessor.m_sim.dot( w2_JCS )

        # упаковываем обратно
        res = r_ECEF.tolist() + v_ECEF.tolist() \
            + a_ECEF.tolist() + j_ECEF.tolist() \
            + [ q_SIM.w, q_SIM.x, q_SIM.y, q_SIM.z ] \
            + w_SIM.tolist() + w1_SIM.tolist() + w2_SIM.tolist()
        logging.debug( 'Result of rotation: {}'.format( res ) )
        return res

    class __make_predictor:
        def __init__( self, params, buffer_desc ):
            self.params = copy.deepcopy( params )
            self.buffer_desc = buffer_desc

        def __call__( self, ftime ):
            return VectorProcessor.predict_and_rotate_sc_vector\
                    ( self.params, ftime, self.buffer_desc )


if __name__ == '__main__':
    # Простые модульные тесты
    import unittest

    # Тестируем упаковку и распаковку данных
    class TestDataPack( unittest.TestCase ):

        class ItemGetterStub:
            # Заглушка для "приема" векторов состояния
            def __init__( self, data: dict ):
                self.data = data
                self.len = len( data )
                self.index = 0
            def __call__( self ):
                if self.index < self.len:
                    x = VectorProcessor.packer( self.data[ self.index ] )
                    self.index += 1
                    return x
                else:
                    return None

        class SimSenderStub:
            # Заглушка для "отправки" сообщения: проверяет его корректность
            def __init__( self, validator, time: float, result: dict ):
                self.result = result
                self.time = time
                self.validator = validator
                self.num_calls = 0

            def __call__( self, time: float, sc_id: int, params: list ):
                self.validator.assertAlmostEqual( time, self.time )
                self.validator.assertIn( sc_id, self.result )
                _p = self.result[ sc_id ]
                self.validator.assertEqual( len( params ), len( _p ) )
                self.num_calls += 1
                # не проверяем значения

            @property
            def calls( self ):
                return self.num_calls

        def setUp( self ):
            self.proc = VectorProcessor()
            self.proc.set_active_spacecraft_id( 1 )
            self.proc.set_passive_spacecraft_id( 2 )

        def test_packer_empty( self ):
            try:
                VectorProcessor.packer( dict() )
            except InvalidParameterDict:
                pass

        def test_packer_nonempty( self ):
            item = {
                    't_Fly': ( 10.0, ), 't_zv': ( 11.0, ),
                    'XA': ( 12.0, ), 'YA': ( 13.0, ), 'ZA': ( 14.0, ),
                    'VXA': ( 15.0, ), 'VYA': ( 16.0, ), 'VZA': ( 17.0, ),
                    'ICA': ( 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 ),
                    'wXA': ( 18.0, ), 'wYA': ( 19.0, ), 'wZA': ( 20.0, ),
                    'XP': ( 21.0, ), 'YP': ( 22.0, ), 'ZP': ( 23.0, ),
                    'VXP': ( 24.0, ), 'VYP': ( 25.0, ), 'VZP': ( 26.0, ),
                    'ICP': ( -1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0 ),
                    'wXP': ( 27.0, ), 'wYP': ( 28.0, ), 'wZP': ( 29.0, ),
                }
            res = VectorProcessor.packer( item )
            res2 = ( \
                    ( 10.0, ), ( 11.0, ),
                    ( 12.0, 15.0 ), ( 13.0, 16.0 ), ( 14.0, 17.0 ),
                    ( 18.0, ), ( 19.0, ), ( 20.0, ),
                    ( 1.0, 0.0, 0.0, 0.0 ),
                    ( 21.0, 24.0 ), ( 22.0, 25.0 ), ( 23.0, 26.0 ),
                    ( 27.0, ), ( 28.0, ), ( 29.0, ),
                    ( 0.0, 0.0, 0.0, 1.0 ),
                )
            self.assertEqual( len( res ), len( res2 ) )
            for a, b in zip( res, res2 ):
                self.assertEqual( len( a ), len( b ) )
                for c, d in zip( a, b ):
                    self.assertAlmostEqual( c, d, places=5 )

        def test_worker( self ):
            item = [ {
                        't_Fly': ( 10.0, ), 't_zv': ( 11.0, ),
                        'XA': ( 12.0, ), 'YA': ( 13.0, ), 'ZA': ( 14.0, ),
                        'VXA': ( 15.0, ), 'VYA': ( 16.0, ), 'VZA': ( 17.0, ),
                        'ICA': ( 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 ),
                        'wXA': ( 18.0, ), 'wYA': ( 19.0, ), 'wZA': ( 20.0, ),
                        'XP': ( 21.0, ), 'YP': ( 22.0, ), 'ZP': ( 23.0, ),
                        'VXP': ( 24.0, ), 'VYP': ( 25.0, ), 'VZP': ( 26.0, ),
                        'ICP': ( -1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0 ),
                        'wXP': ( 27.0, ), 'wYP': ( 28.0, ), 'wZP': ( 29.0, ),
                    } ] * 10

            self.proc.bind_queue_item_getter( self.ItemGetterStub( item ) )
            sender = self.SimSenderStub( self, 10.0, {
                    1: (    12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 18.0, 19.0, 20.0, 1.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ),
                    2: (    12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 18.0, 19.0, 20.0, 1.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ),
                } )
            self.proc.bind_sim_vector_sender( sender )
            self.proc.worker( 10.0 )
            self.assertEqual( sender.calls, 2 )

    unittest.main()

